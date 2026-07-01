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

/**
 * @file main.cpp
 * @brief IMS_CS 系统主控入口。
 *
 * 本文件把硬件驱动、采集任务、谱图处理、LVGL 显示、远程 Web 和物质库连接起来。
 *
 * 核心数据流：
 * 离子门 LEDC 脉冲 -> GPIO 上升沿中断 -> 释放 syncSemaphore ->
 * Task_Acquisition 调用 IMS_ADC_ReadBurst() 采集 24ms 原始数据 ->
 * 多帧累加平均 -> IMS_SpectrumProcessing 做基线/噪声/峰检测 ->
 * IMS_Features 构建 current_features ->
 * Task_UI_Handler 刷新 TFT、特征显示和 WebSocket 谱图。
 *
 * 线程原则：
 * - ADC 高速采样在 Core1 的 Task_Acquisition 中进行；
 * - LVGL 操作集中在 Core0 的 Task_UI_Handler 中进行；
 * - 远程 Web 命令通过队列交给 UI 任务执行，避免 WebServer 回调直接操作 LVGL。
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// --- ESP32 底层硬件库 ---
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#include "driver/ledc.h"
#include "xtensa/core-macros.h"
#include "esp_system.h"
#include "esp_heap_caps.h"

// --- 第三方库 ---
#include <TFT_eSPI.h>
#include <FT6336.h>
#include <lvgl.h>

// --- 本地头文件 ---
#include "ui/ui.h"
#include "IMS_ADC.h"
#include "IMS_Features.h"
#include "IMS_SpectrumProcessing.h"
#include "UI_Manager.h"
#include "UI_FeatureDisplay.h"
#include "IMS_SubstanceDB.h"
#include "IMS_Identifier.h"
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

#define IMS_CYCLE_FREQ      25      // 工作频率: 25Hz (周期约 40ms，24ms 采集后约 16ms 处理余量)
#define IMS_PULSE_WIDTH_US  500     // 离子门开启脉宽: 500us (0.5ms)
#define IMS_MAX_TRIGGER_LATENCY_US 1500

// --- 信号处理参数 ---
#define IMS_AVG_COUNT        24     // 平均次数: 累加 24 次后更新显示 (平衡流畅度与信噪比)
#define RAW_DATA_LEN        (IMS_SAMPLE_RATE * IMS_DURATION_MS / 1000) // 缓冲区长度: 24000 点

// --- IMS 谱图分析参数 ---
#define IMS_ADC_FULL_SCALE_V      5.12f
#define IMS_DISPLAY_SCALE_DIV     4     // 65535 / 16 ~= 4096, matches the chart Y axis
#define IMS_FRONT_BLANK_US        (IMS_PULSE_WIDTH_US + 300) // Ignore gate feedthrough around T0
#define IMS_BASELINE_START_US     20000
#define IMS_BASELINE_END_US       24000
#define IMS_NOISE_START_US        18000
#define IMS_NOISE_END_US          24000
#define IMS_MIN_PEAK_WIDTH_US     40
#define IMS_THRESHOLD_FLOOR_COUNTS 80
#define IMS_THRESHOLD_SIGMA_MULT  6
#ifndef IMS_MAX_PEAKS
#define IMS_MAX_PEAKS             6
#endif

// --- 实时性诊断参数 ---
#define IMS_ENABLE_TIMING_DIAGNOSTICS 1
#define IMS_TIMING_PRINT_LIMIT        10
#define IMS_PROCESS_WARN_US           12000UL
#define IMS_PUBLISH_WAIT_MS           2

// --- UI 显示参数 ---
#define SCREEN_WIDTH        480
#define SCREEN_HEIGHT       320
#define TOUCH_RAW_WIDTH     320
#define TOUCH_RAW_HEIGHT    480
#define CHART_POINTS        200     // 图表分辨率 (X轴点数)

// 关键时序参数说明：
// IMS_SAMPLE_RATE：ADC 采样率，1MSPS 表示每 1us 采集 1 点。
// IMS_DURATION_MS：每次离子门触发后读取的时间窗口，当前为 24ms。
// IMS_CYCLE_FREQ：离子门重复频率，LEDC 硬件按该频率输出脉冲。
// IMS_PULSE_WIDTH_US：离子门打开时间，决定离子包注入宽度。
// IMS_AVG_COUNT：累加平均次数，达到该次数后才处理并刷新一帧谱图。
// RAW_DATA_LEN：单次采集原始点数，1MSPS * 24ms = 24000 点。
// CHART_POINTS：TFT/Web 显示谱图点数，原始谱图会压缩到 200 点。

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
float     web_voltage_buffer[CHART_POINTS]; // Web-only spectrum in baseline-corrected volts.
static uint16_t baseline_corrected_buffer[RAW_DATA_LEN]; // 4. 基线扣除后的临时谱图
static uint16_t median_filtered_buffer[RAW_DATA_LEN];    // 5. 3点中值滤波后的临时谱图

// 缓冲区数据流说明：
// big_raw_buffer：一次离子门触发后的完整原始 ADC 数据。
// accumulator_buffer：把多次 big_raw_buffer 累加，用于平均降噪。
// baseline_corrected_buffer：平均谱图扣除基线后的工作区。
// median_filtered_buffer：对基线扣除谱图做 3 点中值滤波后的工作区。
// waveform_buffer：压缩到 200 点的显示谱图，供 LVGL 图表和 WebSocket 使用。

static lv_coord_t ui_chart_y[CHART_POINTS] = {0};
static volatile int average_counter = 0;

volatile uint32_t g_trigger_edges = 0;
volatile uint32_t g_busy_edges = 0;
volatile uint32_t g_stale_triggers = 0;
volatile uint32_t g_last_trigger_ccount = 0;
volatile bool g_acq_busy = false;
volatile uint32_t g_processing_overruns = 0;
volatile uint32_t g_publish_drops = 0;
volatile uint32_t g_ui_snapshot_misses = 0;

static const IMSSpectrumConfig spectrum_config = {
    IMS_SAMPLE_RATE,
    RAW_DATA_LEN,
    CHART_POINTS,
    IMS_ADC_FULL_SCALE_V,
    IMS_DISPLAY_SCALE_DIV,
    IMS_FRONT_BLANK_US,
    IMS_BASELINE_START_US,
    IMS_BASELINE_END_US,
    IMS_NOISE_START_US,
    IMS_NOISE_END_US,
    IMS_MIN_PEAK_WIDTH_US,
    IMS_THRESHOLD_FLOOR_COUNTS,
    IMS_THRESHOLD_SIGMA_MULT,
    IMS_MAX_PEAKS,
    5000,
    1000,
    80
};

// --- RTOS 句柄 ---
SemaphoreHandle_t dataMutex;         // 数据互斥锁 (保护 UI 显示数据)
SemaphoreHandle_t syncSemaphore;     // 同步信号量 (中断 -> 采集任务)
TaskHandle_t      TaskHandle_UI;     // UI 任务句柄

// 任务同步说明：
// syncSemaphore 只负责“离子门上升沿到了”的实时通知，由 ISR 释放、采集任务等待。
// dataMutex 负责保护采集任务和 UI 任务之间共享的谱图、峰值和特征向量快照。
// LVGL 对象只在 UI 任务里操作，避免跨任务直接刷新界面导致线程安全问题。

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
volatile bool ui_update_needed   = false; // UI 刷新标志位
static volatile bool pending_web_snapshot_push = false;
float         temp_captured_time = 0.0;
IMSSpectrumPeak detected_peaks[IMS_MAX_PEAKS];
int           detected_peak_count = 0;
float         detected_baseline_v = 0.0f;
uint32_t      detected_noise_counts = 0;
float         detected_total_frame_response_au = 0.0f;
uint32_t      detected_frame_id = 0;
IMSFeatureVector current_features;
IMSIdentificationResult current_id_result;

// --- UI 对象引用 ---
lv_chart_series_t *ui_SignalSeries;
lv_obj_t          *ui_Label_IMS_Status = NULL;

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

static float IMS_CountsToVoltage(float counts) {
    return (counts * IMS_ADC_FULL_SCALE_V) / 65535.0f;
}

static float IMS_CountsAreaToVoltageMs(float area_counts) {
    const float sample_period_ms = 1000.0f / (float)IMS_SAMPLE_RATE;
    return IMS_CountsToVoltage(area_counts) * sample_period_ms;
}

static void BuildWebVoltageWaveform(const uint16_t *filtered_spectrum,
                                    float *web_waveform_v,
                                    int point_count) {
    if (!web_waveform_v || point_count <= 0) return;

    for (int i = 0; i < point_count; i++) {
        web_waveform_v[i] = 0.0f;
    }

    if (!filtered_spectrum) return;

    int ratio = RAW_DATA_LEN / point_count;
    if (ratio <= 0) ratio = 1;

    const int blank_samples = IMS_SP_SampleFromUs(IMS_FRONT_BLANK_US,
                                                  IMS_SAMPLE_RATE,
                                                  RAW_DATA_LEN);

    for (int i = 0; i < point_count; i++) {
        uint16_t local_max = 0;

        for (int j = 0; j < ratio; j++) {
            int idx = i * ratio + j;
            if (idx >= RAW_DATA_LEN) break;
            if (idx < blank_samples) continue;

            if (filtered_spectrum[idx] > local_max) {
                local_max = filtered_spectrum[idx];
            }
        }

        web_waveform_v[i] = IMS_CountsToVoltage((float)local_max);
    }
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
    // 离子门由 LEDC 硬件自动输出脉冲。这里监听同一个 GPIO 的上升沿，
    // 把“离子门刚打开”这个硬件事件转换为 FreeRTOS 信号量，唤醒 ADC 采集任务。
    // ISR 中只做 Give 信号量，不做 ADC、Serial、文件、TFT 等耗时操作。
    // 仅在扫描状态下通知采集任务 (虽然 LEDC 可能会一直发波，但我们只在需要时处理)
    if (isScanning) {
        g_trigger_edges++;

        if (g_acq_busy) {
            g_busy_edges++;
            return;
        }

        g_last_trigger_ccount = XTHAL_GET_CCOUNT();

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
    // 硬件同步设计：
    // 1. LEDC 用硬件定时输出稳定的离子门控制脉冲；
    // 2. 同一 GPIO 开启输入回读；
    // 3. GPIO 上升沿中断在离子门打开瞬间释放 syncSemaphore；
    // 4. ADC 采集任务由该信号量触发，读取固定 24ms 时间窗口。
    //
    // 这样 ADC 采集起点与离子门脉冲边沿绑定，减少软件 delay 带来的抖动。
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
    // 采集任务运行在 Core1，是实时性最敏感的任务。
    // 这里避免 TFT 刷屏、WiFi 发送、SPIFFS 写入和大量 Serial 打印，
    // 只做离子门同步后的 ADC 读取、累加平均、谱图处理和特征构建。
    (void)pvParameters;
    // 特征向量调试输出限频计数：每生成 10 次平均谱图特征后才打印一次，避免串口影响实时性。
    uint8_t feature_debug_frame_count = 0;
#if IMS_ENABLE_TIMING_DIAGNOSTICS
    uint8_t timing_print_count = 0;
#endif

    Serial.println("DEBUG: Acquisition Task Started!");
    while (true) {
        if (big_raw_buffer == NULL || accumulator_buffer == NULL) {
            g_acq_busy = false;
            vTaskDelay(100);
            continue;
        }

        // [BLOCKING] 等待信号量
        // 这里现在由 GPIO 中断触发，且完全同步于 LEDC 脉冲上升沿
        // 延迟固定，波形不再漂移
        // 等待离子门上升沿 ISR 释放同步信号量。
        // 只有收到这个信号，ADC 才开始读取 24ms 的数据窗口。
        if (xSemaphoreTake(syncSemaphore, portMAX_DELAY) == pdTRUE) {
            g_acq_busy = true;

            // 停止状态下的清理逻辑
            if (!isScanning) {
                average_counter = 0;
                memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
                g_acq_busy = false;
                continue;
            }

            uint32_t trigger_latency_us =
                (XTHAL_GET_CCOUNT() - g_last_trigger_ccount) / (uint32_t)getCpuFrequencyMhz();

            if (trigger_latency_us > IMS_MAX_TRIGGER_LATENCY_US) {
                g_stale_triggers++;
                average_counter = 0;
                memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
                g_acq_busy = false;
                continue;
            }

            // --- 阶段 1: 高速采集 ---
            // 此时 T ≈ 10us (中断延迟)，非常接近物理 T=0
#if IMS_ENABLE_TIMING_DIAGNOSTICS
            uint32_t adc_read_start_us = micros();
#endif
            // 高速采集原始 ADC 数据。该函数直接操作 SPI 寄存器，是采集路径核心。
            IMS_ADC_ReadBurst(big_raw_buffer, RAW_DATA_LEN);
#if IMS_ENABLE_TIMING_DIAGNOSTICS
            uint32_t adc_read_us = micros() - adc_read_start_us;
#endif

            // --- 阶段 2: 数据累加 ---
            // 将本次原始数据累加到 accumulator_buffer。
            // ADS8681 输出字节序在这里做交换，然后进入平均谱图累加。
            for (int i = 0; i < RAW_DATA_LEN; i++) {
                uint16_t raw_val = big_raw_buffer[i];
                // 大小端转换
                raw_val = (raw_val << 8) | (raw_val >> 8);
                accumulator_buffer[i] += raw_val;
            }
            average_counter++;

            // --- 阶段 3: 平均与压缩 ---
            // 累加到指定次数后，生成一帧平均谱图并更新 UI/Web 所需数据。
            if (average_counter >= IMS_AVG_COUNT) {

#if IMS_ENABLE_TIMING_DIAGNOSTICS
                uint32_t processing_stage_start_us = micros();
                uint32_t spectrum_process_us = 0;
#endif

                static int16_t processed_waveform[CHART_POINTS];
                static float processed_web_voltage_waveform[CHART_POINTS];
                static IMSSpectrumPeak processed_peaks[IMS_MAX_PEAKS];
                static IMSPeakInput processed_feature_input_peaks[IMS_MAX_PEAKS];
                static IMSFeatureVector processed_features;

                IMSSpectrumProcessResult spectrum_result;
                float publish_peak_time = 0.0f;
                float publish_peak_amp = 0.0f;
                int publish_peak_count = 0;
                float publish_baseline_v = 0.0f;
                uint32_t publish_noise_counts = 0;
                float publish_total_frame_response_au = 0.0f;
                bool published = false;

#if IMS_ENABLE_TIMING_DIAGNOSTICS
                uint32_t spectrum_process_start_us = micros();
#endif
                bool spectrum_ok = IMS_SP_ProcessAveragedSpectrum(accumulator_buffer,
                                                                  IMS_AVG_COUNT,
                                                                  &spectrum_config,
                                                                  baseline_corrected_buffer,
                                                                  median_filtered_buffer,
                                                                  RAW_DATA_LEN,
                                                                  processed_waveform,
                                                                  CHART_POINTS,
                                                                  processed_peaks,
                                                                  IMS_MAX_PEAKS,
                                                                  &spectrum_result);
#if IMS_ENABLE_TIMING_DIAGNOSTICS
                spectrum_process_us = micros() - spectrum_process_start_us;
#endif

                if (spectrum_ok) {
                    publish_peak_count = spectrum_result.peak_count;
                    publish_baseline_v = spectrum_result.baseline_v;
                    publish_noise_counts = spectrum_result.noise_counts;
                    publish_peak_time = spectrum_result.main_peak_time_ms;
                    publish_peak_amp  = spectrum_result.main_peak_amp_v;
                    publish_total_frame_response_au = spectrum_result.total_frame_response_au;
                    BuildWebVoltageWaveform(median_filtered_buffer,
                                            processed_web_voltage_waveform,
                                            CHART_POINTS);
                } else {
                    memset(processed_waveform, 0, sizeof(processed_waveform));
                    memset(processed_web_voltage_waveform, 0, sizeof(processed_web_voltage_waveform));
                    memset(processed_peaks, 0, sizeof(processed_peaks));
                }

                int feature_peak_count = publish_peak_count;
                if (feature_peak_count > IMS_MAX_PEAKS) feature_peak_count = IMS_MAX_PEAKS;
                for (int i = 0; i < feature_peak_count; i++) {
                    processed_feature_input_peaks[i].start_idx = processed_peaks[i].start_idx;
                    processed_feature_input_peaks[i].peak_idx = processed_peaks[i].peak_idx;
                    processed_feature_input_peaks[i].end_idx = processed_peaks[i].end_idx;
                    processed_feature_input_peaks[i].time_ms = processed_peaks[i].time_ms;
                    processed_feature_input_peaks[i].amp_v = processed_peaks[i].amp_v;
                    processed_feature_input_peaks[i].area_counts = processed_peaks[i].area_counts;
                    processed_feature_input_peaks[i].width_ms = processed_peaks[i].width_ms;
                    processed_feature_input_peaks[i].fwhm_ms = processed_peaks[i].fwhm_ms;
                    processed_feature_input_peaks[i].prominence_counts = processed_peaks[i].prominence_counts;
                    processed_feature_input_peaks[i].snr_est = processed_peaks[i].snr_est;
                }

                if (spectrum_ok) {
                    IMS_BuildFeatureVectorEx(processed_feature_input_peaks,
                                             feature_peak_count,
                                             median_filtered_buffer,
                                             RAW_DATA_LEN,
                                             processed_waveform,
                                             CHART_POINTS,
                                             publish_baseline_v,
                                             publish_noise_counts,
                                             spectrum_result.threshold_counts,
                                             spectrum_result.baseline_valid,
                                             spectrum_result.baseline_span_counts,
                                             IMS_ADC_FULL_SCALE_V,
                                             IMS_SAMPLE_RATE,
                                             &processed_features);
                } else {
                    IMS_ClearFeatureVector(&processed_features);
                }

                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(IMS_PUBLISH_WAIT_MS)) == pdTRUE) {
                    memcpy(waveform_buffer, processed_waveform, sizeof(waveform_buffer));
                    memcpy(web_voltage_buffer, processed_web_voltage_waveform, sizeof(web_voltage_buffer));
                    memcpy(detected_peaks, processed_peaks, sizeof(detected_peaks));
                    memcpy(&current_features, &processed_features, sizeof(current_features));
                    detected_peak_count = publish_peak_count;
                    detected_baseline_v = publish_baseline_v;
                    detected_noise_counts = publish_noise_counts;
                    detected_peak_time = publish_peak_time;
                    detected_peak_amp = publish_peak_amp;
                    detected_total_frame_response_au = publish_total_frame_response_au;
                    detected_frame_id++;
                    ui_update_needed = true;
                    published = true;
                    xSemaphoreGive(dataMutex);
                } else {
                    g_publish_drops++;
                }

                feature_debug_frame_count++;
                if (feature_debug_frame_count >= 10) {
                    feature_debug_frame_count = 0;
                    Serial.printf("[IMS Features] valid=%d usable=%d peaks=%d main=%.2fms amp=%.2fV snr=%.1f quality=%.2f reason=%u published=%d\n",
                                  processed_features.valid ? 1 : 0,
                                  processed_features.usable_for_identification ? 1 : 0,
                                  processed_features.peak_count,
                                  processed_features.main_peak_time_ms,
                                  processed_features.main_peak_amp_v,
                                  processed_features.snr,
                                  processed_features.quality_score,
                                  (unsigned)processed_features.reject_reason,
                                  published ? 1 : 0);
                }

                memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
#if IMS_ENABLE_TIMING_DIAGNOSTICS
                uint32_t total_process_us = micros() - processing_stage_start_us;
                if (total_process_us > IMS_PROCESS_WARN_US) {
                    g_processing_overruns++;
                }
                if (timing_print_count < IMS_TIMING_PRINT_LIMIT) {
                    Serial.printf("[IMS Timing] ADC read us=%lu, Spectrum us=%lu, Publish/feature total us=%lu, overrun=%lu, publish_drop=%lu\n",
                                  (unsigned long)adc_read_us,
                                  (unsigned long)spectrum_process_us,
                                  (unsigned long)total_process_us,
                                  (unsigned long)g_processing_overruns,
                                  (unsigned long)g_publish_drops);
                    timing_print_count++;
                }
#endif
                average_counter = 0;
            }
            g_acq_busy = false;
        }
    }
}



/**
 * @brief 任务 2: UI 界面刷新 (运行于 Core 0)
 * 注：此任务代码未修改，保持原样
 */
void Task_UI_Handler(void *pvParameters) {
    // UI 任务运行在 Core0，集中处理 LVGL、远程命令和 WebSocket 推送。
    // LVGL 不是线程安全的，因此切屏、标签刷新、图表刷新都放在此任务中执行。
    (void)pvParameters;
    static int16_t local_waveform[CHART_POINTS];
    static float local_web_voltage_waveform[CHART_POINTS];
    static IMSFeatureVector local_features;
    static IMSIdentificationResult local_id_result;
    uint8_t id_debug_frame_count = 0;
    static uint32_t last_web_status_ms = 0;
    static uint32_t last_web_diag_ms = 0;

    while (true) {

        // ===========================
        // [新增] 处理网页端命令（UI线程执行，避免 LVGL 线程不安全）
        // ===========================
        RemoteCmd cmd;
        // 远程网页按钮不会直接操作 LVGL，而是先进入队列，再由 UI 任务安全执行。
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
                pending_web_snapshot_push = false;
            } else if (cmd == CMD_PAUSE) {
                SetScanning(false);
                pending_web_snapshot_push = true;
                ui_update_needed = true;
            } else if (cmd == CMD_SAVE) {
                // 等价于本地“保存物质”：暂停 + 搬运数据 + 切到 Screen3
                SetScanning(false);
                OnSaveSubstance(nullptr);
                if (ui_Screen3) lv_scr_load(ui_Screen3);
            }
        }

        if (ui_update_needed) {
            // 采集任务准备好新一帧平均谱图后设置 ui_update_needed。
            // UI 任务在持有 dataMutex 的短时间内复制快照，然后释放锁再刷新 LVGL/Web。
            float local_peak_time = 0.0f;
            float local_peak_amp = 0.0f;
            float local_total_frame_response_au = 0.0f;
            uint32_t local_frame_id = 0;
            bool local_is_scanning = false;
            bool has_snapshot = false;
            bool force_web_snapshot = false;
            bool skip_web_push = false;

            if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
                memcpy(local_waveform, waveform_buffer, sizeof(local_waveform));
                memcpy(local_web_voltage_waveform, web_voltage_buffer, sizeof(local_web_voltage_waveform));
                memcpy(&local_features, &current_features, sizeof(local_features));
                local_peak_time = detected_peak_time;
                local_peak_amp = detected_peak_amp;
                local_total_frame_response_au = detected_total_frame_response_au;
                local_frame_id = detected_frame_id;
                local_is_scanning = isScanning;
                force_web_snapshot = pending_web_snapshot_push;
                if (force_web_snapshot && local_frame_id == 0) {
                    pending_web_snapshot_push = false;
                    force_web_snapshot = false;
                    skip_web_push = true;
                }
                ui_update_needed = false;
                has_snapshot = true;
                xSemaphoreGive(dataMutex);
            } else {
                g_ui_snapshot_misses++;
            }

            if (has_snapshot) {
                IMS_ID_Identify(&local_features, &local_id_result);
                memcpy(&current_id_result, &local_id_result, sizeof(current_id_result));

                id_debug_frame_count++;
                if (id_debug_frame_count >= 10) {
                    id_debug_frame_count = 0;
                    if (local_id_result.status == IMS_ID_MATCHED ||
                        local_id_result.status == IMS_ID_LOW_CONFIDENCE ||
                        local_id_result.status == IMS_ID_UNKNOWN) {
                        Serial.printf("[IMS ID] result=%s conf=%.2f score=%.2f second=%.2f margin=%.2f tpl=%.2f best1=%.2f best2=%.2f drift=%.2f template=%.2f peak=%.2f shape=%.2f q=%.2f\n",
                                      local_id_result.name[0] ? local_id_result.name : IMS_ID_StatusText(local_id_result.status),
                                      local_id_result.confidence,
                                      local_id_result.final_score,
                                      local_id_result.second_score,
                                      local_id_result.score_margin,
                                      local_id_result.template_feature_score,
                                      local_id_result.best_sample_1_score,
                                      local_id_result.best_sample_2_score,
                                      local_id_result.best_breakdown.drift_score,
                                      local_id_result.best_breakdown.template_score,
                                      local_id_result.best_breakdown.peak_pattern_score,
                                      local_id_result.best_breakdown.peak_shape_score,
                                      local_id_result.best_breakdown.quality_factor);
                    } else {
                        Serial.printf("[IMS ID] result=%s\n",
                                      IMS_ID_StatusText(local_id_result.status));
                    }
                }

                if (ui_SignalSeries != NULL && ui_Chart1 != NULL && lv_scr_act() == ui_Screen2) {
                    for (int i = 0; i < CHART_POINTS; i++) {
                        ui_chart_y[i] = (lv_coord_t)local_waveform[i];
                    }
                    lv_chart_refresh(ui_Chart1);
                }

                UI_FeatureDisplay_UpdateScreen2WithID(&local_features, &local_id_result);

                // ===========================
                // [新增] 推送同一帧谱图到网页（保证网页与本地显示一致）
                // ===========================
                const char *id_name_for_web = IMS_ID_StatusText(local_id_result.status);
                if (local_id_result.status == IMS_ID_MATCHED ||
                    local_id_result.status == IMS_ID_LOW_CONFIDENCE) {
                    if (local_id_result.name[0] != '\0') {
                        id_name_for_web = local_id_result.name;
                    }
                }

                bool local_has_second_peak = false;
                float local_main_second_height_ratio = 0.0f;
                float local_main_second_time_diff_ms = 0.0f;
                if (local_features.main_peak_amp_v > 0.0f &&
                    local_features.second_peak_amp_v > 0.0f) {
                    local_has_second_peak = true;
                    local_main_second_height_ratio =
                        local_features.main_peak_amp_v / local_features.second_peak_amp_v;
                    local_main_second_time_diff_ms =
                        local_features.main_peak_time_ms - local_features.second_peak_time_ms;
                    if (local_main_second_time_diff_ms < 0.0f) {
                        local_main_second_time_diff_ms = -local_main_second_time_diff_ms;
                    }
                }

                if (skip_web_push) {
                    // No completed spectrum frame is available yet.
                } else if (local_is_scanning && !force_web_snapshot) {
                    uint32_t now_ms = millis();
                    if (now_ms - last_web_status_ms >= 1000) {
                        last_web_status_ms = now_ms;
                        remote.pushStatus(local_peak_time, local_peak_amp, true);
                    }
                } else {
                    remote.pushWaveform(local_web_voltage_waveform, CHART_POINTS,
                                        local_peak_time, local_peak_amp,
                                        false,
                                        id_name_for_web,
                                        (int)local_id_result.status,
                                        local_features.quality_score,
                                        local_frame_id,
                                        (float)IMS_DURATION_MS,
                                        local_features.main_peak_fwhm_ms,
                                        IMS_CountsAreaToVoltageMs(local_features.main_peak_area),
                                        IMS_CountsAreaToVoltageMs(local_total_frame_response_au),
                                        local_has_second_peak,
                                        local_main_second_height_ratio,
                                        local_main_second_time_diff_ms);
                    if (force_web_snapshot) {
                        pending_web_snapshot_push = false;
                    }
                }
            }
        }

        uint32_t now_ms = millis();
        if (now_ms - last_web_diag_ms >= 5000) {
            last_web_diag_ms = now_ms;
            bool scanning_snapshot = isScanning;
            Serial.printf("[WEB] ws=%u scanning=%d full_waveform_push=%d internal_free=%u min_heap=%u psram=%u\n",
                          (unsigned)remote.clientCount(),
                          scanning_snapshot ? 1 : 0,
                          scanning_snapshot ? 0 : 1,
                          (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                          (unsigned)ESP.getMinFreeHeap(),
                          (unsigned)ESP.getFreePsram());
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
  // 统一控制扫描状态。本地按钮和远程网页命令最终都走这里，
  // 以保证 UI 按钮文字、颜色和 Web 状态推送保持一致。
  isScanning = false;
  if (syncSemaphore) {
    while (xSemaphoreTake(syncSemaphore, 0) == pdTRUE) {
      // Clear old triggers before changing scan state.
    }
  }

  if (on) {
    g_trigger_edges = 0;
    g_busy_edges = 0;
    g_stale_triggers = 0;
    g_last_trigger_ccount = 0;
    g_acq_busy = false;
    average_counter = 0;
    if (accumulator_buffer) {
      memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
    }
    isScanning = true;
  } else {
    g_acq_busy = false;
  }

  // 同步本地 UI 按钮文案/颜色（ui_Button9 是 SquareLine 导出的全局对象）
  if (ui_Button9) {
    lv_obj_t *label = lv_obj_get_child(ui_Button9, 0);
    if (label) lv_label_set_text(label, on ? "Pause Analysis" : "Start Analysis");
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
  bool next_state = !isScanning;
  SetScanning(next_state);
  if (next_state) {
    pending_web_snapshot_push = false;
  } else {
    pending_web_snapshot_push = true;
    ui_update_needed = true;
  }
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
    // 系统启动顺序：
    // 1. 串口/I2C/WiFi AP；
    // 2. 创建互斥锁和命令队列；
    // 3. 初始化 ADC、分配采集缓冲；
    // 4. 初始化离子门 LEDC/GPIO 同步；
    // 5. 初始化 TFT/Touch/LVGL 和 SquareLine UI；
    // 6. 初始化物质库并刷新 UI；
    // 7. 创建采集任务和 UI 任务。
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
    Serial.printf("[RESET] reason=%d\n", (int)esp_reset_reason());
    Serial.printf("[MEM] internal free=%u heap=%u min_heap=%u psram=%u\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getMinFreeHeap(),
                  (unsigned)ESP.getFreePsram());

    dataMutex = xSemaphoreCreateMutex();

    IMS_ADC_Init();
    Serial.println("[OK] ADC Hardware Initialized.");

    size_t raw_size_bytes = RAW_DATA_LEN * sizeof(uint16_t);
    size_t acc_size_bytes = RAW_DATA_LEN * sizeof(uint32_t);

    Serial.printf("[INFO] Memory Req: Raw=%.2f KB, Acc=%.2f KB\n", raw_size_bytes / 1024.0, acc_size_bytes / 1024.0);

    big_raw_buffer = (uint16_t *)heap_caps_malloc(
        raw_size_bytes,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    if (big_raw_buffer) {
        Serial.printf("[MEM] raw buffer allocated in INTERNAL SRAM (DMA): %u bytes\n",
                      (unsigned)raw_size_bytes);
    }

    if (big_raw_buffer == NULL) {
        big_raw_buffer = (uint16_t *)heap_caps_malloc(
            raw_size_bytes,
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (big_raw_buffer) {
            Serial.printf("[MEM] raw buffer allocated in INTERNAL SRAM: %u bytes\n",
                          (unsigned)raw_size_bytes);
        }
    }

    if (big_raw_buffer == NULL) {
        big_raw_buffer = (uint16_t *)heap_caps_malloc(
            raw_size_bytes,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (big_raw_buffer) {
            Serial.println("[WARN] big_raw_buffer fallback to PSRAM; ADC timing may be less stable.");
        }
    }

    accumulator_buffer = (uint32_t *)heap_caps_malloc(
        acc_size_bytes,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (accumulator_buffer) {
        Serial.printf("[MEM] accumulator buffer allocated in PSRAM: %u bytes\n",
                      (unsigned)acc_size_bytes);
    }

    if (accumulator_buffer == NULL) accumulator_buffer = (uint32_t *)malloc(acc_size_bytes);

    if (big_raw_buffer == NULL || accumulator_buffer == NULL) {
        Serial.println("[CRITICAL] Memory Alloc Failed! Halted.");
        while (1);
    }

    memset(big_raw_buffer, 0, raw_size_bytes);
    memset(accumulator_buffer, 0, acc_size_bytes);
    Serial.println("[OK] Memory Allocated & Cleared.");
    Serial.printf("[MEM] free internal=%u free heap=%u free psram=%u\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getFreePsram());

    // 5. [修改] 启动 IMS 核心控制 (现在是 LEDC + GPIO Sync)
    IMS_HW_Init();

    // 6. UI 子系统初始化 (保持不变)
    tft.init();
    tft.setRotation(1);
    tft.invertDisplay(true);
    tft.fillScreen(TFT_BLACK);
    ts.begin();

    lv_init();
    size_t draw_lines = SCREEN_HEIGHT;
    size_t buffer_size = SCREEN_WIDTH * draw_lines * sizeof(lv_color_t);
    buf = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buf == NULL) {
        draw_lines = 32;
        buffer_size = SCREEN_WIDTH * draw_lines * sizeof(lv_color_t);
        buf = (lv_color_t *)malloc(buffer_size);
    }
    if (buf == NULL) {
        Serial.println("[CRITICAL] LVGL Buffer Alloc Failed! Halted.");
        while (1);
    }
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_WIDTH * draw_lines);

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
    UI_FeatureDisplay_Init();

    IMS_DB_Init();
    Refresh_AnalyteLibrary_UI();
    UI_SetSelectedAnalyteText(nullptr);


    ui_SignalSeries = lv_chart_get_series_next(ui_Chart1, NULL);
    lv_chart_set_update_mode(ui_Chart1, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_point_count(ui_Chart1, CHART_POINTS);
    memset(ui_chart_y, 0, sizeof(ui_chart_y));
    if (ui_SignalSeries != NULL) {
        lv_chart_set_ext_y_array(ui_Chart1, ui_SignalSeries, ui_chart_y);
        lv_chart_refresh(ui_Chart1);
    }

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
    static uint32_t lastSyncDebugMs = 0;
    uint32_t now = millis();
    if (now - lastSyncDebugMs >= 2000) {
        lastSyncDebugMs = now;
        Serial.printf("[SYNC] trig=%lu busy_drop=%lu stale_drop=%lu proc_overrun=%lu publish_drop=%lu ui_miss=%lu busy=%d\n",
                      (unsigned long)g_trigger_edges,
                      (unsigned long)g_busy_edges,
                      (unsigned long)g_stale_triggers,
                      (unsigned long)g_processing_overruns,
                      (unsigned long)g_publish_drops,
                      (unsigned long)g_ui_snapshot_misses,
                      g_acq_busy ? 1 : 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
}
