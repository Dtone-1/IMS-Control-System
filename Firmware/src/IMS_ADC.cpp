/**
 * @file IMS_ADC.cpp
 * @brief ADS8681 外置 ADC 的初始化、单点读取和高速突发读取实现。
 *
 * 本模块是 IMS 原始信号进入软件系统的入口。ADS8681 通过独立 FSPI 总线连接
 * ESP32-S3，用于读取外部跨阻放大器输出的电压信号。普通读数函数适合调试，
 * 采集任务实际使用 IMS_ADC_ReadBurst() 在离子门触发后读取完整时间窗口。
 *
 * 关键点：
 * - ADS8681 的 CS 每次采样都要产生一次低-高脉冲，满足芯片转换/读数时序；
 * - 高速读取路径直接操作 ESP32-S3 SPI/GPIO 寄存器，减少 Arduino SPI 封装开销；
 * - 当前快速 GPIO 写法使用 GPIO.out_w1tc/out_w1ts，适用于 GPIO 0~31。
 */
#include "IMS_ADC.h"
#include <SPI.h>

// 引入 ESP32-S3 底层硬件定义 (这是实现"核弹级"优化的关键)
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "hal/spi_types.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"

// ================= 配置区域 =================
// 请根据实际接线修改，确保不与屏幕引脚冲突
// ADS_RST：ADS8681 复位脚。启动时拉低再拉高，保证 ADC 处于已知状态。
// ADS_CS：片选脚。每次读取一个采样点时都会拉低/拉高一次，形成 ADC 采样时序边界。
// ADS_MOSI：ESP32-S3 -> ADS8681 的配置命令输入线。
// ADS_MISO：ADS8681 -> ESP32-S3 的转换结果输出线。
// ADS_SCLK：SPI 时钟线。注意这些引脚应避免与 TFT/Touch 引脚冲突。
#define ADS_RST   14
#define ADS_CS    15  // 片选信号 (Chip Select)
#define ADS_MOSI  16  // SDI (数据输入)
#define ADS_MISO  17  // SDO (数据输出)
#define ADS_SCLK  18  // SCLK (时钟)

// SPI 时钟频率：40MHz
// 40MHz 下传输 16bit 物理耗时 0.4us，这是 1MSPS 的物理基础
// 40MHz SPI 传输 16bit 的理想耗时约 0.4us，是接近 1MSPS 采样率的硬件基础。
#define ADS_SPI_SPEED 40000000 

// 创建 SPI 实例，使用 FSPI (对应硬件 SPI2)
SPIClass adcSPI(FSPI);

// 获取 FSPI (SPI2) 的硬件寄存器基地址指针
// volatile 关键字防止编译器优化掉对寄存器的读写
volatile spi_dev_t *hw_spi = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);

static inline uint16_t IRAM_ATTR adc_read_nop16(volatile uint32_t *spi_cmd_reg,
                                                volatile uint32_t *spi_w0_reg,
                                                uint32_t spi_usr_mask,
                                                uint32_t cs_mask) {
    GPIO.out_w1tc = cs_mask;
    // data_buf is reused for TX/RX; force NOP instead of echoing ADC data as commands.
    *spi_w0_reg = 0x00000000;
    *spi_cmd_reg = spi_usr_mask;
    while (*spi_cmd_reg & spi_usr_mask) {
    }
    uint16_t value = (uint16_t)*spi_w0_reg;
    GPIO.out_w1ts = cs_mask;
    return value;
}

// ================= 内部辅助函数 =================
// 用于发送配置命令 (低速、标准模式，用于初始化)
// 初始化阶段使用标准 SPI 事务发送 ADS8681 配置命令；速度不是瓶颈，优先保证模式清晰可靠。
uint16_t _adc_transfer(uint32_t cmd) {
    adcSPI.beginTransaction(SPISettings(ADS_SPI_SPEED, MSBFIRST, SPI_MODE0));
    digitalWrite(ADS_CS, LOW);
    // ADS8681 需要 32个时钟周期来完成配置 (16位命令 + 16位数据/填充)
    uint16_t high = adcSPI.transfer16((cmd >> 16) & 0xFFFF);
    uint16_t low = adcSPI.transfer16(cmd & 0xFFFF);
    digitalWrite(ADS_CS, HIGH);
    adcSPI.endTransaction();
    return high; // 配置命令通常不关心返回值，或者是高16位状态
}

// ================= 对外公开函数 =================

/**
 * @brief 初始化 ADS8681 和专用 FSPI 总线。
 *
 * 该函数完成 GPIO 默认电平、ADC 硬复位、SPI 总线启动以及输入量程配置。
 * 它只在 setup() 阶段调用一次，高速采集任务启动前必须已经完成。
 */
void IMS_ADC_Init() {
    // 1. GPIO 模式配置
    pinMode(ADS_CS, OUTPUT);
    pinMode(ADS_RST, OUTPUT);
    digitalWrite(ADS_CS, HIGH); // 默认拉高 (不选中)

    // 2. 硬件复位 (RST脉冲)
    digitalWrite(ADS_RST, LOW);
    delay(1);
    digitalWrite(ADS_RST, HIGH);
    delay(20); // 等待芯片内部电路稳定

    // 3. SPI 总线初始化
    adcSPI.begin(ADS_SCLK, ADS_MISO, ADS_MOSI, ADS_CS);

    // 4. 发送 ADS8681 配置指令
    // 设定输入量程为 0 ~ 5.12V (单极性)
    // 命令: Write(0xD0) | RegAddr(0x14) | Data(0x000B)
    _adc_transfer(0xD014000B);
    delay(10);
    
    // 5. 发送空指令刷新管道
    _adc_transfer(0x00000000);
}

// 单次慢速读取 (调试用)
// 单次低速读取，主要用于调试 ADC 是否有响应，不用于主采集路径。
uint16_t IMS_ADC_ReadRaw() {
    return _adc_transfer(0x00000000);
}

// 电压转换辅助函数
// 将 16 位 ADC 计数值按 0~5.12V 量程换算为电压。
float IMS_ADC_ReadVoltage() {
    return (IMS_ADC_ReadRaw() / 65536.0f) * 5.12f;
}

// =================================================================
// 🚀 极速突发读取 (Direct Register Access Version)
// 核心优化：绕过 Arduino 库，直接操作寄存器，配合循环展开
// =================================================================
// IRAM_ATTR: 强制将函数加载到 RAM 中运行，避免 Flash 读取延迟 (Cache Miss)
/**
 * @brief 高速读取 count 个 ADC 原始点。
 *
 * 采集任务在收到离子门同步信号后调用此函数。这里绕过 Arduino SPI.transfer16
 * 的重复配置开销，直接触发 SPI 硬件寄存器，并使用 10 点一组的循环展开降低
 * 循环控制开销。函数放入 IRAM，减少 Flash cache miss 对采样时序的影响。
 */
void IRAM_ATTR IMS_ADC_ReadBurst(uint16_t *buffer, size_t count) {
    
    // 1. 使用库函数完成基础配置 (时钟、模式等)
    // 虽然这一步有开销，但只在开始时执行一次，为了安全起见保留
    adcSPI.beginTransaction(SPISettings(ADS_SPI_SPEED, MSBFIRST, SPI_MODE0));

    // 2. 准备 GPIO 快速操作掩码
    // 将 CS 引脚号转换为位掩码 (例如 GPIO 15 -> 0x00008000)
    // 注意：GPIO.out_w1tc 寄存器仅支持 GPIO 0-31。若用 GPIO 32+ 需改代码
    // 快速 GPIO 写寄存器只覆盖 GPIO 0~31；当前 ADS_CS=15，满足该限制。
    const uint32_t cs_mask = (1 << ADS_CS);

    // 3. 【核心优化】直接配置 SPI 寄存器为 16位 模式
    // 标准库 transfer16 每次都会重新配置长度，这里我们在循环外一次性配好
    // ms_data_bitlen = 传输位数 - 1 (16位 -> 15)
    // 直接配置 SPI 为 16bit 传输。ms_data_bitlen = 传输位数 - 1，所以 16bit 对应 15。
    hw_spi->ms_dlen.ms_data_bitlen = 15; 
    hw_spi->ms_dlen.ms_data_bitlen = 15;
    
    // 确保启用 MOSI/MISO/USR 功能位
    hw_spi->user.usr_mosi = 1;
    hw_spi->user.usr_miso = 1;

    // 4. 准备循环展开变量
    // 将总次数分为 "10次一组" 的主循环 和 "不足10次" 的尾数
    // 将总点数拆为 10 点一组的主循环和尾数，减少每个采样点上的循环判断开销。
    size_t main_loop = count / 10;
    size_t remainder = count % 10;
    uint16_t *ptr = buffer;

    // 5. 缓存寄存器地址，减少指针解引用开销
    // cmd.val: 写入 1 触发传输
    // 缓存寄存器地址，避免在高速循环中反复计算结构体字段地址。
    volatile uint32_t *spi_cmd_reg = &(hw_spi->cmd.val);
    // data_buf[0]: 读写数据缓冲 (FIFO)
    volatile uint32_t *spi_w0_reg  = &(hw_spi->data_buf[0]);
    // 触发掩码 (SPI_USR 位)
    const uint32_t     spi_usr_mask = SPI_USR; 
    *spi_w0_reg = 0x00000000;

    // ------------------ 主循环 (Unrolled Loop) ------------------
    // 原理：减少了 90% 的 "i++" 和 "i < count" 判断指令
    // 手动翻转 CS：满足 ADS8681 "每次采样必须有 CS 下降沿" 的要求
    // 每个采样点都执行：CS 拉低 -> 触发 SPI -> 等待完成 -> 读取 FIFO -> CS 拉高。
    for (size_t i = 0; i < main_loop; i++) {
        
        // --- 第 1 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);

        // --- 第 2 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);

        // --- 第 3 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);

        // --- 第 4 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);

        // --- 第 5 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);

        // --- 第 6 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);

        // --- 第 7 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);

        // --- 第 8 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);

        // --- 第 9 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);

        // --- 第 10 个点 ---
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);
    }

    // ------------------ 尾数处理 ------------------
    for (size_t i = 0; i < remainder; i++) {
        *ptr++ = adc_read_nop16(spi_cmd_reg, spi_w0_reg, spi_usr_mask, cs_mask);
    }

    // 6. 结束事务，恢复 SPI 总线状态给其他设备使用
    adcSPI.endTransaction();
}
