/**
 * @file IMS_ADC.h
 * @brief ADS8681 外置 ADC 驱动接口
 *
 * 本文件只声明 ADC 驱动对外暴露的接口。主程序通过这些接口完成：
 * 1. 初始化 ADS8681、SPI 总线和相关 GPIO；
 * 2. 读取单点原始 ADC 数据或换算后的电压；
 * 3. 在离子门触发后高速读取一个完整采样窗口的数据。
 *
 * 数据流位置：
 * 离子门同步触发 -> IMS_ADC_ReadBurst() 采集原始 ADC 数据 -> 谱图处理模块
 */
#ifndef IMS_ADC_H
#define IMS_ADC_H

#include <Arduino.h>

// 对外公开的 ADC 功能接口。高速采样路径主要使用 IMS_ADC_ReadBurst()。

/**
 * @brief 初始化 ADS8681 外置 ADC。
 *
 * 包括 ADC 复位、SPI 总线启动和输入量程配置。
 * 该函数必须在 ADC 采集任务启动前完成，否则后续高速读取的数据不可靠。
 */
void IMS_ADC_Init();

/**
 * @brief 读取一次 ADC 原始值，主要用于调试或低速检查。
 * @return 16 位无符号 ADC 计数值，范围通常为 0 ~ 65535。
 */
uint16_t IMS_ADC_ReadRaw();

/**
 * @brief 读取一次 ADC 并换算为电压。
 * @return 按当前 0~5.12V 量程换算后的电压值。
 */
float IMS_ADC_ReadVoltage();

/**
 * @brief 高速突发读取一段 ADC 数据。
 * @param buffer 接收原始 ADC 数据的数组，调用方必须保证空间足够。
 * @param count 要读取的点数，例如 24ms @ 1MSPS 时约 24000 点。
 *
 * 这是采集任务的核心读数函数。它会绕过 Arduino SPI.transfer 的部分开销，
 * 直接操作 ESP32-S3 SPI 寄存器，以便尽量接近 1MSPS 的采样节奏。
 */
void IMS_ADC_ReadBurst(uint16_t *buffer, size_t count);

#endif
