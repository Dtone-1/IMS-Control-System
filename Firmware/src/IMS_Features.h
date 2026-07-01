/**
 * @file IMS_Features.h
 * @brief IMS 谱图特征向量定义与构建接口。
 *
 * 本模块不是峰检测模块。峰检测由 IMS_SpectrumProcessing 完成，本模块只把已有峰检测结果、
 * 显示谱图、滤波谱图、基线和噪声信息整理成统一的 IMSFeatureVector。
 *
 * IMSFeatureVector 后续会被：
 * - UI_FeatureDisplay 用于屏幕显示；
 * - IMS_SubstanceDB 用于物质库样本保存；
 * - 后续物质识别算法用于规则匹配、多峰指纹、模板相似度和传统机器学习输入。
 *
 * 数据流位置：
 * ADC采集 -> 谱图预处理/峰检测 -> 本文件构建特征 -> UI显示/物质库/识别模块
 */
#pragma once

#include <Arduino.h>
#include <stdint.h>

#ifndef IMS_MAX_PEAKS
#define IMS_MAX_PEAKS 6
#endif

#ifndef IMS_FEATURE_TEMPLATE_POINTS
#define IMS_FEATURE_TEMPLATE_POINTS 200
#endif

#ifndef IMS_RECOGNITION_TEMPLATE_POINTS
#define IMS_RECOGNITION_TEMPLATE_POINTS 128
#endif

/**
 * @brief 特征拒绝原因。
 *
 * reject_reason 不是最终识别结果，只说明当前帧是否适合进入后续匹配流程。
 * 质量差应由上层显示 Poor Signal / No Peak；质量可用但匹配不到物质时才显示 Unknown。
 */
enum IMSFeatureRejectReason : uint8_t {
    IMS_FEATURE_OK = 0,
    IMS_FEATURE_NO_PEAK = 1,
    IMS_FEATURE_LOW_SNR = 2,
    IMS_FEATURE_BAD_BASELINE = 3,
    IMS_FEATURE_BAD_PEAK_SHAPE = 4,
    IMS_FEATURE_SATURATED = 5,
    IMS_FEATURE_TOO_MANY_PEAKS = 6
};

/**
 * @brief 单个峰的识别特征。
 *
 * 这些字段来自峰检测结果，并补充了相对主峰/总面积/主峰 FWHM 的派生比例。
 * 后续识别不应只依赖单个峰高，而应综合漂移时间、峰形、峰面积比例和模板形状。
 */
struct IMSPeakFeature {
    uint16_t start_idx;       // 峰开始点在原始采样数组中的索引。
    uint16_t peak_idx;        // 峰顶点索引。
    uint16_t end_idx;         // 峰结束点索引。

    float time_ms;            // 峰漂移时间，单位 ms。
    float amp_v;              // 峰高电压。
    uint32_t area_counts;     // 峰面积，单位为 ADC counts 累加值。

    float width_ms;           // 峰起止点对应的宽度，单位 ms。
    float fwhm_ms;            // 半峰宽，优先来自谱图处理模块的半峰高搜索结果。
    uint32_t prominence_counts; // 峰突出度，峰顶相对左右边界较高者的高度差。
    float snr_est;            // 单峰简易 SNR 估计，峰顶 counts / noise_counts。

    float amp_ratio;          // 当前峰 amp_v / 主峰 amp_v。
    float area_ratio;         // 当前峰 area_counts / total_peak_area。
    float fwhm_ratio;         // 当前峰 fwhm_ms / 主峰 fwhm_ms。
    float time_delta_to_main_ms; // 当前峰 time_ms - main_peak_time_ms。
};

/**
 * @brief 一帧平均谱图对应的完整特征向量。
 *
 * valid 表示当前有有效峰；usable_for_identification 表示质量足够进入后续识别。
 * 保存样本和物质识别应优先参考 usable_for_identification/reject_reason，而不是只看 valid。
 */
struct IMSFeatureVector {
    int peak_count;                                   // 本帧检测到的有效峰数量。

    float main_peak_time_ms;                          // 主峰漂移时间。
    float main_peak_amp_v;                            // 主峰峰高电压。
    float main_peak_area;                             // 主峰面积。
    float main_peak_width_ms;                         // 主峰宽度。
    float main_peak_fwhm_ms;                          // 主峰近似半峰宽。

    float second_peak_time_ms;                        // 次强峰漂移时间，用于后续多峰识别。
    float second_peak_amp_v;                          // 次强峰峰高。

    float baseline_v;                                 // 本帧估计基线电压。
    uint32_t noise_counts;                            // 本帧噪声估计，单位 ADC counts。
    uint32_t threshold_counts;                        // 本帧峰检测阈值。
    bool baseline_valid;                              // 基线窗口是否稳定。
    uint32_t baseline_span_counts;                    // 基线窗口 max-min；越大表示越不稳定。

    float snr;                                        // 主峰信噪比。
    float total_peak_area;                            // 所有保留峰的总面积。

    float quality_score;                              // 0~1 综合质量评分。
    bool usable_for_identification;                   // 是否适合进入后续识别算法。
    uint8_t reject_reason;                            // IMSFeatureRejectReason。

    float normalized_template[IMS_FEATURE_TEMPLATE_POINTS];       // 200 点显示模板，和 UI/Web 显示一致。
    float recognition_template[IMS_RECOGNITION_TEMPLATE_POINTS];  // 128 点识别模板，来自原始长度滤波谱图。

    float peak_time_deltas_ms[IMS_MAX_PEAKS];          // 多峰漂移时间组合特征。
    float peak_amp_ratios[IMS_MAX_PEAKS];              // 多峰峰高比例特征。
    float peak_area_ratios[IMS_MAX_PEAKS];             // 多峰面积比例特征。
    float peak_fwhm_ratios[IMS_MAX_PEAKS];             // 多峰半峰宽比例特征。

    IMSPeakFeature peaks[IMS_MAX_PEAKS];               // 最多保存 IMS_MAX_PEAKS 个峰的详细特征。

    bool valid;                                        // 当前是否有有效峰。
};

/**
 * @brief 峰检测模块传给特征模块的轻量输入结构。
 *
 * main.cpp 将 IMSSpectrumPeak 中需要的字段复制到这里，避免特征模块依赖谱图处理模块内部实现。
 */
struct IMSPeakInput {
    uint16_t start_idx;
    uint16_t peak_idx;
    uint16_t end_idx;
    float time_ms;
    float amp_v;
    uint32_t area_counts;
    float width_ms;
    float fwhm_ms;
    uint32_t prominence_counts;
    float snr_est;
};

void IMS_ClearFeatureVector(IMSFeatureVector *feature);

float IMS_CalcSNR(float main_peak_amp_v, uint32_t noise_counts, float adc_full_scale_v);

void IMS_NormalizeTemplate(const int16_t *display_waveform,
                           int point_count,
                           float *normalized_template,
                           int template_points);

void IMS_NormalizeRecognitionTemplate(const uint16_t *filtered_spectrum,
                                      int filtered_len,
                                      float *recognition_template,
                                      int template_points);

/**
 * @brief 增强特征构建接口。
 *
 * filtered_spectrum/filtered_len 用于构建更适合识别的 128 点模板；
 * display_waveform/display_point_count 仍用于构建 200 点显示模板。
 */
void IMS_BuildFeatureVectorEx(const IMSPeakInput *input_peaks,
                              int input_peak_count,
                              const uint16_t *filtered_spectrum,
                              int filtered_len,
                              const int16_t *display_waveform,
                              int display_point_count,
                              float baseline_v,
                              uint32_t noise_counts,
                              uint32_t threshold_counts,
                              bool baseline_valid,
                              uint32_t baseline_span_counts,
                              float adc_full_scale_v,
                              uint32_t sample_rate_hz,
                              IMSFeatureVector *out_feature);

/**
 * @brief 兼容旧调用的特征构建接口。
 *
 * 旧接口内部调用 IMS_BuildFeatureVectorEx()，但没有原始长度滤波谱图和诊断字段。
 */
void IMS_BuildFeatureVector(const IMSPeakInput *input_peaks,
                            int input_peak_count,
                            const int16_t *display_waveform,
                            int display_point_count,
                            float baseline_v,
                            uint32_t noise_counts,
                            float adc_full_scale_v,
                            uint32_t sample_rate_hz,
                            IMSFeatureVector *out_feature);
