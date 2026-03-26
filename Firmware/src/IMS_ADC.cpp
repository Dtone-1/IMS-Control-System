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
#define ADS_RST   14
#define ADS_CS    15  // 片选信号 (Chip Select)
#define ADS_MOSI  16  // SDI (数据输入)
#define ADS_MISO  17  // SDO (数据输出)
#define ADS_SCLK  18  // SCLK (时钟)

// SPI 时钟频率：40MHz
// 40MHz 下传输 16bit 物理耗时 0.4us，这是 1MSPS 的物理基础
#define ADS_SPI_SPEED 40000000 

// 创建 SPI 实例，使用 FSPI (对应硬件 SPI2)
SPIClass adcSPI(FSPI);

// 获取 FSPI (SPI2) 的硬件寄存器基地址指针
// volatile 关键字防止编译器优化掉对寄存器的读写
volatile spi_dev_t *hw_spi = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);

// ================= 内部辅助函数 =================
// 用于发送配置命令 (低速、标准模式，用于初始化)
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
uint16_t IMS_ADC_ReadRaw() {
    return _adc_transfer(0x00000000);
}

// 电压转换辅助函数
float IMS_ADC_ReadVoltage() {
    return (IMS_ADC_ReadRaw() / 65536.0f) * 5.12f;
}

// =================================================================
// 🚀 极速突发读取 (Direct Register Access Version)
// 核心优化：绕过 Arduino 库，直接操作寄存器，配合循环展开
// =================================================================
// IRAM_ATTR: 强制将函数加载到 RAM 中运行，避免 Flash 读取延迟 (Cache Miss)
void IRAM_ATTR IMS_ADC_ReadBurst(uint16_t *buffer, size_t count) {
    
    // 1. 使用库函数完成基础配置 (时钟、模式等)
    // 虽然这一步有开销，但只在开始时执行一次，为了安全起见保留
    adcSPI.beginTransaction(SPISettings(ADS_SPI_SPEED, MSBFIRST, SPI_MODE0));

    // 2. 准备 GPIO 快速操作掩码
    // 将 CS 引脚号转换为位掩码 (例如 GPIO 15 -> 0x00008000)
    // 注意：GPIO.out_w1tc 寄存器仅支持 GPIO 0-31。若用 GPIO 32+ 需改代码
    const uint32_t cs_mask = (1 << ADS_CS);

    // 3. 【核心优化】直接配置 SPI 寄存器为 16位 模式
    // 标准库 transfer16 每次都会重新配置长度，这里我们在循环外一次性配好
    // ms_data_bitlen = 传输位数 - 1 (16位 -> 15)
    hw_spi->ms_dlen.ms_data_bitlen = 15; 
    hw_spi->ms_dlen.ms_data_bitlen = 15;
    
    // 确保启用 MOSI/MISO/USR 功能位
    hw_spi->user.usr_mosi = 1;
    hw_spi->user.usr_miso = 1;

    // 4. 准备循环展开变量
    // 将总次数分为 "10次一组" 的主循环 和 "不足10次" 的尾数
    size_t main_loop = count / 10;
    size_t remainder = count % 10;
    uint16_t *ptr = buffer;

    // 5. 缓存寄存器地址，减少指针解引用开销
    // cmd.val: 写入 1 触发传输
    volatile uint32_t *spi_cmd_reg = &(hw_spi->cmd.val);
    // data_buf[0]: 读写数据缓冲 (FIFO)
    volatile uint32_t *spi_w0_reg  = &(hw_spi->data_buf[0]);
    // 触发掩码 (SPI_USR 位)
    const uint32_t     spi_usr_mask = SPI_USR; 

    // ------------------ 主循环 (Unrolled Loop) ------------------
    // 原理：减少了 90% 的 "i++" 和 "i < count" 判断指令
    // 手动翻转 CS：满足 ADS8681 "每次采样必须有 CS 下降沿" 的要求
    for (size_t i = 0; i < main_loop; i++) {
        
        // --- 第 1 个点 ---
        GPIO.out_w1tc = cs_mask;             // [极速] CS 拉低 (Clear)
        *spi_cmd_reg = spi_usr_mask;         // [极速] 触发 SPI 传输
        while (*spi_cmd_reg & spi_usr_mask); // [等待] 等待传输完成 (这是唯一的物理耗时 0.4us)
        *ptr++ = (uint16_t)*spi_w0_reg;      // [极速] 直接读 FIFO
        GPIO.out_w1ts = cs_mask;             // [极速] CS 拉高 (Set)

        // --- 第 2 个点 ---
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;

        // --- 第 3 个点 ---
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;

        // --- 第 4 个点 ---
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;

        // --- 第 5 个点 ---
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;

        // --- 第 6 个点 ---
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;

        // --- 第 7 个点 ---
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;

        // --- 第 8 个点 ---
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;

        // --- 第 9 个点 ---
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;

        // --- 第 10 个点 ---
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;
    }

    // ------------------ 尾数处理 ------------------
    for (size_t i = 0; i < remainder; i++) {
        GPIO.out_w1tc = cs_mask;
        *spi_cmd_reg = spi_usr_mask;
        while (*spi_cmd_reg & spi_usr_mask);
        *ptr++ = (uint16_t)*spi_w0_reg;
        GPIO.out_w1ts = cs_mask;
    }

    // 6. 结束事务，恢复 SPI 总线状态给其他设备使用
    adcSPI.endTransaction();
}