/**
 * ============================================================================
 * 项目名称: IMSLAS - 自主化学源定位系统 (核心控制与采集单元)
 * 硬件平台: ESP32-S3 (N16R8) + ST7796U (TFT) + FT6336 (Touch) + ADS8681 (ADC)
 *
 * 核心功能:
 * 1. 产生高精度离子门控制时序 (LEDC 硬件 PWM)
 * 2. 高速 ADC 数据采集 (SPI DMA/Burst)
 * 3. 硬件中断级同步采集 (消除 Jitter)
 * 4. 信号累加平均算法 (降噪)
 * 5. 双核多任务架构 (Core1 采集 / Core0 显示)
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// --- ESP32 底层硬件库 ---
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#include "driver/ledc.h"

// --- 第三方库 ---
#include <TFT_eSPI.h>
#include <FT6336.h>
#include <lvgl.h>

// --- 本地头文件 ---
#include "ui/ui.h"
#include "IMS_ADC.h"
#include "UI_Manager.h"
#include "IMS_RemoteWeb.h"

IMS_RemoteWeb remote;
// ============================================================================
// [SECTION 1] 硬件引脚定义 (Hardware Pin Definitions)
// ============================================================================

// --- I2C 总线 (触摸屏 & 其他传感器) ---
#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9

// --- 触摸屏中断与复位 ---
#define CTP_INT_PIN         4
#define CTP_RST_PIN         13

// --- IMS 核心控制引脚 ---
#define ION_GATE_PIN        48      // 离子门控制信号 (兼 LEDC 输出与 GPIO 中断输入)

// ============================================================================
// [SECTION 2] IMS 系统参数配置 (System Configuration)
// ============================================================================

// --- 时序与频率参数 ---
#define IMS_SAMPLE_RATE     1000000 // ADC采样率: 1MSPS (1us/点)
#define IMS_DURATION_MS     24      // 单次采样窗口: 24ms (适配 UI X轴)

#define IMS_CYCLE_FREQ      33      // 工作频率: 33Hz (周期约 30.3ms)
#define IMS_PULSE_WIDTH_US  500    // 离子门开启脉宽: 250us (0.25ms)

// --- 信号处理参数 ---
#define IMS_AVG_COUNT        24     // 平均次数: 累加 16 次后更新显示 (平衡流畅度与信噪比)
#define RAW_DATA_LEN        (IMS_SAMPLE_RATE * IMS_DURATION_MS / 1000) // 缓冲区长度: 24000 点

// --- UI 显示参数 ---
#define SCREEN_WIDTH        480
#define SCREEN_HEIGHT       320
#define TOUCH_RAW_WIDTH     320
#define TOUCH_RAW_HEIGHT    480
#define CHART_POINTS        200     // 图表分辨率 (X轴点数)

// ============================================================================
// [SECTION 3] 全局变量与对象 (Global Variables & Objects)
// ============================================================================

// --- 硬件对象 ---
TFT_eSPI tft = TFT_eSPI();
FT6336   ts  = FT6336(I2C_SDA_PIN, I2C_SCL_PIN, CTP_INT_PIN, CTP_RST_PIN, TOUCH_RAW_WIDTH, TOUCH_RAW_HEIGHT);

// --- LVGL 显示缓冲 ---
static lv_disp_draw_buf_t draw_buf;
static lv_color_t        *buf;

// --- 数据缓冲区 (动态分配) ---
uint16_t *big_raw_buffer     = NULL; // 1. 原始数据缓存 (单次采集, uint16)
uint32_t *accumulator_buffer = NULL; // 2. 累加缓存 (用于平均算法, uint32 防止溢出)
int16_t   waveform_buffer[CHART_POINTS]; // 3. 显示波形缓存 (压缩后)

// --- RTOS 句柄 ---
SemaphoreHandle_t dataMutex;         // 数据互斥锁 (保护 UI 显示数据)
SemaphoreHandle_t syncSemaphore;     // 同步信号量 (中断 -> 采集任务)
TaskHandle_t      TaskHandle_UI;     // UI 任务句柄

// --- 远程命令队列（网页按钮 -> UI线程执行）---
enum RemoteCmd : uint8_t {
  CMD_START = 1,
  CMD_PAUSE = 2,
  CMD_SAVE  = 3,
};
QueueHandle_t remoteCmdQueue = nullptr;


// --- 业务状态变量 ---
volatile bool isScanning         = false; // 扫描开关
float         detected_peak_time = 0.0;   // 检测到的峰值时间 (ms)
float         detected_peak_amp  = 0.0;     // 检测到的峰值幅度
bool          ui_update_needed   = false; // UI 刷新标志位
float         temp_captured_time = 0.0;

// --- UI 对象引用 ---
lv_chart_series_t *ui_SignalSeries;

// ============================================================================
// [SECTION 4] 函数原型声明 (Function Prototypes)
// ============================================================================

void IRAM_ATTR onIonGateTrigger(void *arg);
void           IMS_HW_Init();
void           Task_Acquisition(void *pvParameters);
void           Task_UI_Handler(void *pvParameters);
void           OnScanClick(lv_event_t *e);
void           my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);
void           my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static void    SetScanning(bool on);



static void postCmd(RemoteCmd cmd) {
  if (!remoteCmdQueue) return;
  xQueueSend(remoteCmdQueue, &cmd, 0);
}

static void cbStart() {
  Serial.println("[REMOTE] START pressed");
  postCmd(CMD_START);
}

static void cbPause() {
  Serial.println("[REMOTE] PAUSE pressed");
  postCmd(CMD_PAUSE);
}

static void cbSave() {
  Serial.println("[REMOTE] SAVE pressed");
  postCmd(CMD_SAVE);
}





// ============================================================================
// [SECTION 5] IMS 硬件驱动与中断 (Hardware Drivers & ISR)
// ============================================================================

/**
 * @brief [修改] 离子门触发中断服务函数
 * @note  现在的逻辑是：硬件 LEDC 自动拉高引脚 -> 触发此中断 -> 通知任务采集
 * 彻底消除了 delay 阻塞和时序抖动。
 */
void IRAM_ATTR onIonGateTrigger(void *arg) {
    // 仅在扫描状态下通知采集任务 (虽然 LEDC 可能会一直发波，但我们只在需要时处理)
    if (isScanning) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 唤醒采集任务 (Zero Latency)
        xSemaphoreGiveFromISR(syncSemaphore, &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

/**
 * @brief 初始化 IMS 专用硬件 (LEDC PWM + GPIO Interrupt)
 */
void IMS_HW_Init() {
    // 1. 创建同步信号量
    syncSemaphore = xSemaphoreCreateBinary();

    // ===================================================
    // 2. 配置 LEDC (负责产生脉冲)
    // ===================================================

    // 计算占空比
    uint32_t duty_reg_val = (uint32_t)((float)IMS_PULSE_WIDTH_US * IMS_CYCLE_FREQ * 8192.0 / 1000000.0);

    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = IMS_CYCLE_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num   = ION_GATE_PIN,         // 这里填 48 (或你实际用的引脚)
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = duty_reg_val,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 }
    };
    ledc_channel_config(&ledc_channel);

    // 强制启动更新
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    // ===================================================
    // 3. 【关键】开启内部输入回环 (修复输出为0和中断不触发的问题)
    // ===================================================

    // 这行代码直接操作 IO MUX 寄存器，打开 Input Enable 位
    // 无论引脚号是多少(48也行)，都不会溢出，因为它是数组索引操作
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[ION_GATE_PIN]);

    // ===================================================
    // 4. 配置 GPIO 中断 (监听同一个引脚)
    // ===================================================

    // 安装中断服务
    gpio_install_isr_service(0);

    // 设置上升沿触发
    gpio_set_intr_type((gpio_num_t)ION_GATE_PIN, GPIO_INTR_POSEDGE);

    // 添加中断回调
    gpio_isr_handler_add((gpio_num_t)ION_GATE_PIN, onIonGateTrigger, NULL);

    Serial.printf(">> IMS Init Done. Pin %d Mode: LEDC Output + Internal Input\n", ION_GATE_PIN);
}

// ============================================================================
// [SECTION 6] RTOS 任务定义 (RTOS Tasks)
// ============================================================================

/**
 * @brief 任务 1: ADC 采集与信号处理 (运行于 Core 1)
 */
void Task_Acquisition(void *pvParameters) {
    (void)pvParameters;
    int average_counter = 0;

    Serial.println("DEBUG: Acquisition Task Started!");
    while (true) {
        if (big_raw_buffer == NULL || accumulator_buffer == NULL) {
            vTaskDelay(100);
            continue;
        }

        // [BLOCKING] 等待信号量
        // 这里现在由 GPIO 中断触发，且完全同步于 LEDC 脉冲上升沿
        // 延迟固定，波形不再漂移
        if (xSemaphoreTake(syncSemaphore, portMAX_DELAY) == pdTRUE) {

            // 停止状态下的清理逻辑
            if (!isScanning) {
                average_counter = 0;
                memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
                continue;
            }

            // --- 阶段 1: 高速采集 ---
            // 此时 T ≈ 10us (中断延迟)，非常接近物理 T=0
            IMS_ADC_ReadBurst(big_raw_buffer, RAW_DATA_LEN);
            Serial.println("DEBUG: ADC Read Done.");

            // --- 阶段 2: 数据累加 ---
            for (int i = 0; i < RAW_DATA_LEN; i++) {
                uint16_t raw_val = big_raw_buffer[i];
                // 大小端转换
                raw_val = (raw_val << 8) | (raw_val >> 8);
                accumulator_buffer[i] += raw_val;
            }
            average_counter++;

            // --- 阶段 3: 平均与压缩 ---
            if (average_counter >= IMS_AVG_COUNT) {

                if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {

                    int ratio          = RAW_DATA_LEN / CHART_POINTS;
                    int global_max_val = 0;
                    // [修正建议] 这里为了精确时间，可以记录精确索引，暂保持原样以免改动太多
                    int global_max_idx = 0;

                    for (int i = 0; i < CHART_POINTS; i++) {
                        int local_max_avg = 0;

                        for (int j = 0; j < ratio; j++) {
                            int idx = i * ratio + j;
                            if (idx >= RAW_DATA_LEN) break;

                            int avg_val = accumulator_buffer[idx] / IMS_AVG_COUNT;
                            if (avg_val > local_max_avg) local_max_avg = avg_val;
                            
                        }

                        waveform_buffer[i] = local_max_avg / 128;

                        if (local_max_avg > global_max_val) {
                            global_max_val = local_max_avg;
                            global_max_idx = i * ratio;
                        }
                    }

                    detected_peak_time = (float)global_max_idx / 1000.0;
                    detected_peak_amp  = global_max_val / 12800.0;

                    ui_update_needed = true;
                    xSemaphoreGive(dataMutex);
                }

                memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
                average_counter = 0;
            }
        }
    }
}



/**
 * @brief 任务 2: UI 界面刷新 (运行于 Core 0)
 * 注：此任务代码未修改，保持原样
 */
void Task_UI_Handler(void *pvParameters) {
    (void)pvParameters;

    while (true) {

        // ===========================
        // [新增] 处理网页端命令（UI线程执行，避免 LVGL 线程不安全）
        // ===========================
        RemoteCmd cmd;
        while (remoteCmdQueue && xQueueReceive(remoteCmdQueue, &cmd, 0) == pdTRUE) {
            if (cmd == CMD_START) {
                // 1) 先切到谱图界面（Screen2）
                if (ui_Screen2) {
                    lv_scr_load(ui_Screen2);
                    // 让切屏立刻生效（可选但推荐，视觉上更“立马”）
                    lv_timer_handler();
                }
                // 2) 再开始采集
                SetScanning(true);
            } else if (cmd == CMD_PAUSE) {
                SetScanning(false);
            } else if (cmd == CMD_SAVE) {
                // 等价于本地“保存物质”：暂停 + 搬运数据 + 切到 Screen3
                SetScanning(false);
                OnSaveSubstance(nullptr);
                if (ui_Screen3) lv_scr_load(ui_Screen3);
            }
        }

        if (ui_update_needed) {
            if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {

                if (ui_SignalSeries != NULL) {
                    lv_chart_set_ext_y_array(ui_Chart1, ui_SignalSeries, (lv_coord_t *)waveform_buffer);
                    lv_chart_refresh(ui_Chart1);
                }

                if (isScanning && detected_peak_amp > 0) {
                    static char buf_time[16];
                    static char buf_amp[16];
                    sprintf(buf_time, "%.2f ms", detected_peak_time);
                    sprintf(buf_amp, "%.2f V", detected_peak_amp);

                    if (ui_Label18) lv_label_set_text(ui_Label18, buf_time);
                    if (ui_Label19) lv_label_set_text(ui_Label19, buf_amp);
                } else {
                    if (ui_Label18) lv_label_set_text(ui_Label18, "--.--");
                }

                // ===========================
                // [新增] 推送同一帧谱图到网页（保证网页与本地显示一致）
                // ===========================
                remote.pushWaveform(waveform_buffer, CHART_POINTS,
                                    detected_peak_time, detected_peak_amp,
                                    isScanning);

                ui_update_needed = false;
                xSemaphoreGive(dataMutex);
            }
        }

        lv_timer_handler();
        lv_tick_inc(5);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


// ============================================================================
// [SECTION 7] 回调与辅助函数 (Callbacks & Helpers)
// ============================================================================

static void SetScanning(bool on) {
  isScanning = on;

  // 同步本地 UI 按钮文案/颜色（ui_Button9 是 SquareLine 导出的全局对象）
  if (ui_Button9) {
    lv_obj_t *label = lv_obj_get_child(ui_Button9, 0);
    if (label) lv_label_set_text(label, on ? "暂停扫描" : "开始扫描");
    lv_obj_set_style_bg_color(ui_Button9,
                              lv_color_hex(on ? 0x00AA00 : 0x0869B4),
                              LV_PART_MAIN);
  }

  Serial.println(on ? "Action: IMS Start Scanning" : "Action: IMS Stop Scanning");

  // 推送一次状态到网页端（峰值信息尽量带上，抢不到锁也没关系）
  float pt = 0.0f, pa = 0.0f;
  if (dataMutex && xSemaphoreTake(dataMutex, 0) == pdTRUE) {
    pt = detected_peak_time;
    pa = detected_peak_amp;
    xSemaphoreGive(dataMutex);
  }
  remote.pushStatus(pt, pa, isScanning);
}




void OnScanClick(lv_event_t *e) {
  (void)e;
  SetScanning(!isScanning);
}


void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();
    lv_disp_flush_ready(disp_drv);
}

void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    ts.read();
    if (ts.touches > 0) {
        data->state   = LV_INDEV_STATE_PR;
        data->point.x = ts.points[0].y;
        data->point.y = 319 - ts.points[0].x;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// ============================================================================
// [SECTION 8] 系统初始化 (Setup)
// ============================================================================

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    remoteCmdQueue = xQueueCreate(8, sizeof(RemoteCmd));


    remote.onStart(cbStart);
    remote.onPause(cbPause);
    remote.onSave(cbSave);

    bool ok = remote.beginAP("IMS_CTRL", "12345678");
    if(!ok){
      Serial.println("AP start failed!");
      while(true) delay(1000);
    }



    
    Serial.println("\n\n==================================");
    Serial.println("IMSLAS System Booting...");
    Serial.println("==================================");

    dataMutex = xSemaphoreCreateMutex();

    IMS_ADC_Init();
    Serial.println("[OK] ADC Hardware Initialized.");

    size_t raw_size_bytes = RAW_DATA_LEN * sizeof(uint16_t);
    size_t acc_size_bytes = RAW_DATA_LEN * sizeof(uint32_t);

    Serial.printf("[INFO] Memory Req: Raw=%.2f KB, Acc=%.2f KB\n", raw_size_bytes / 1024.0, acc_size_bytes / 1024.0);

    big_raw_buffer     = (uint16_t *)heap_caps_malloc(raw_size_bytes, MALLOC_CAP_SPIRAM);
    accumulator_buffer = (uint32_t *)heap_caps_malloc(acc_size_bytes, MALLOC_CAP_SPIRAM);

    if (big_raw_buffer == NULL) big_raw_buffer = (uint16_t *)malloc(raw_size_bytes);
    if (accumulator_buffer == NULL) accumulator_buffer = (uint32_t *)malloc(acc_size_bytes);

    if (big_raw_buffer == NULL || accumulator_buffer == NULL) {
        Serial.println("[CRITICAL] Memory Alloc Failed! Halted.");
        while (1);
    }

    memset(big_raw_buffer, 0, raw_size_bytes);
    memset(accumulator_buffer, 0, acc_size_bytes);
    Serial.println("[OK] Memory Allocated & Cleared.");

    // 5. [修改] 启动 IMS 核心控制 (现在是 LEDC + GPIO Sync)
    IMS_HW_Init();

    // 6. UI 子系统初始化 (保持不变)
    tft.init();
    tft.setRotation(1);
    tft.invertDisplay(true);
    tft.fillScreen(TFT_BLACK);
    ts.begin();

    lv_init();
    size_t buffer_size = SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(lv_color_t);
    buf = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buf == NULL) buf = (lv_color_t *)malloc(SCREEN_WIDTH * 32 * sizeof(lv_color_t));
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_WIDTH * (buf == NULL ? 32 : SCREEN_HEIGHT));

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();

    Init_Library();        // 从 Flash 读取历史数据
    Refresh_Library_UI();  // 将数据绘制到 Screen 3 的列表中


    ui_SignalSeries = lv_chart_get_series_next(ui_Chart1, NULL);
    lv_chart_set_update_mode(ui_Chart1, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_point_count(ui_Chart1, CHART_POINTS);

    Serial.println("[OK] UI System Initialized.");

    // 7. 启动多任务处理
    Serial.println("[INFO] Starting Tasks...");

    xTaskCreatePinnedToCore(Task_Acquisition, "IMS_ADC_Core1", 8192, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(Task_UI_Handler, "IMS_UI_Core0", 8192, NULL, 5, &TaskHandle_UI, 0);

    Serial.println(">> IMS System Ready & Running. Waiting for user command.");
}

// ============================================================================
// [SECTION 9] 主循环 (Main Loop)
// ============================================================================

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}