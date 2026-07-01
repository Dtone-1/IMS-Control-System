/**
 * @file IMS_SpectrumProcessing.h
 * @brief IMS 平均谱图预处理与峰检测接口。
 *
 * 本模块的输入不是 ADC 单次原始数据 big_raw_buffer，而是 main.cpp 中
 * 多次采集累加得到的 accumulator_buffer。处理函数通过 avg_count 把累加值
 * 还原为平均谱图，然后完成基线估计、噪声估计、阈值计算、基线扣除、滤波、
 * 200 点显示压缩和峰检测。
 *
 * 输出数据用途：
 * - baseline_corrected_buffer：原始长度的基线扣除谱图；
 * - median_filtered_buffer：原始长度的滤波后谱图；
 * - display_waveform：TFT/Web 使用的 200 点显示谱图；
 * - peaks：传给 IMS_Features 构建 IMSFeatureVector 的峰特征输入；
 * - result：本帧谱图处理摘要和诊断指标。
 *
 * 本模块只做“预处理 + 峰检测”，不做物质识别。
 */
#pragma once

#include <Arduino.h>
#include <stdint.h>

#ifndef IMS_SP_MAX_PEAKS
#define IMS_SP_MAX_PEAKS 6
#endif

/**
 * @brief 单个检测到的 IMS 峰。
 *
 * start_idx/end_idx/peak_idx 都是原始采样数组中的索引。
 * time_ms 使用面积加权中心换算，amp_v 使用峰顶 counts 换算。
 * width_ms、fwhm_ms、prominence_counts 和 snr_est 为后续物质分辨提供更稳定的峰形信息。
 */
struct IMSSpectrumPeak {
    uint16_t start_idx;          // 峰开始索引。
    uint16_t peak_idx;           // 峰顶索引。
    uint16_t end_idx;            // 峰结束索引。
    float    time_ms;            // 峰漂移时间，单位 ms。
    float    amp_v;              // 峰顶电压，单位 V。
    uint32_t area_counts;        // 峰面积，基线扣除后的 ADC counts 累加。
    float    width_ms;           // 峰宽，end_idx - start_idx + 1 换算为 ms。
    float    fwhm_ms;            // 半峰宽，基于滤波谱图搜索半峰高交点。
    uint32_t prominence_counts;  // 峰突出度：峰顶相对左右边界较高者的高度差。
    float    snr_est;            // 当前峰的简易 SNR 估计：峰顶 counts / noise_counts。
};

/**
 * @brief 谱图处理参数。
 *
 * 新增的 max_peak_width_us、min_peak_area_counts 和 min_prominence_counts 用于
 * 降低宽基线漂移、窄噪声尖峰和低突出度伪峰的误检概率。
 */
struct IMSSpectrumConfig {
    uint32_t sample_rate_hz;         // ADC 采样率，例如 1,000,000 Hz。
    int raw_data_len;               // 原始采样窗口点数，例如 24000。
    int chart_points;               // UI 目标显示点数，当前为 200。

    float adc_full_scale_v;         // ADC 满量程电压，用于 counts -> V。
    int display_scale_div;          // 显示压缩比例。

    int front_blank_us;             // 忽略离子门馈通/T0 干扰的前端空白时间。

    int baseline_start_us;          // 基线估计窗口起点。
    int baseline_end_us;            // 基线估计窗口终点。

    int noise_start_us;             // 噪声估计窗口起点。
    int noise_end_us;               // 噪声估计窗口终点。

    int min_peak_width_us;          // 最小峰宽，过滤单点/窄脉冲噪声。
    uint32_t threshold_floor_counts; // 阈值下限，避免噪声估计偏低时误检。
    uint32_t threshold_sigma_mult;   // 噪声倍数阈值，threshold=max(floor, noise*mult)。

    int max_peaks;                  // 最多输出的峰数量。
    int max_peak_width_us;          // 最大峰宽，<=0 表示不启用。
    uint32_t min_peak_area_counts;  // 最小峰面积，过滤能量过低的伪峰。
    uint32_t min_prominence_counts; // 最小突出度，过滤贴近边界/基线的伪峰。
};

/**
 * @brief 谱图处理结果摘要和诊断信息。
 */
struct IMSSpectrumProcessResult {
    int peak_count;                 // 检测到的峰数量。
    float main_peak_time_ms;        // 主峰漂移时间。
    float main_peak_amp_v;          // 主峰峰高电压。
    float main_peak_area;           // 主峰面积，单位 counts。
    float main_peak_width_ms;       // 主峰峰宽。
    float main_peak_fwhm_ms;        // 主峰半峰宽。
    float total_frame_response_au;  // 前端空白后完整滤波谱图总响应。
    float baseline_v;               // 基线电压。
    uint32_t noise_counts;          // 噪声估计，当前为平均绝对偏差，不是严格 RMS/标准差。
    uint32_t threshold_counts;      // 本帧峰检测阈值。
    bool baseline_valid;            // 基线窗口是否相对稳定。
    uint32_t baseline_span_counts;  // 基线窗口 max-min；越大表示基线越不稳定。
};

void IMS_SP_ClearResult(IMSSpectrumProcessResult *result);

int IMS_SP_SampleFromUs(int us, uint32_t sample_rate_hz, int raw_data_len);
float IMS_SP_SampleToMs(float sample_idx, uint32_t sample_rate_hz);

/**
 * @brief 对累加平均谱图做预处理、显示压缩和峰检测。
 *
 * 流程：
 * 1. 校验输入；
 * 2. 计算各时间窗口对应的采样索引；
 * 3. 估计基线和基线稳定性；
 * 4. 用平均绝对偏差估计噪声；
 * 5. 计算峰检测阈值；
 * 6. 构建基线扣除谱图；
 * 7. 做 3 点中值滤波；
 * 8. 用区间最大值策略生成 200 点显示谱图；
 * 9. 检测峰并计算峰宽、真实 FWHM、突出度和简易 SNR；
 * 10. 填充结果摘要。
 */
bool IMS_SP_ProcessAveragedSpectrum(const uint32_t *accumulator,
                                    int avg_count,
                                    const IMSSpectrumConfig *config,
                                    uint16_t *baseline_corrected_buffer,
                                    uint16_t *median_filtered_buffer,
                                    int work_buffer_len,
                                    int16_t *display_waveform,
                                    int display_waveform_len,
                                    IMSSpectrumPeak *peaks,
                                    int peaks_len,
                                    IMSSpectrumProcessResult *result);
