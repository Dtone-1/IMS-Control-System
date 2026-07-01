/**
 * @file IMS_Features.cpp
 * @brief IMS 谱图特征构建模块。
 *
 * 本文件把峰检测结果、显示谱图、滤波谱图、基线和噪声信息整理成 IMSFeatureVector。
 * 它不负责寻找峰，也不直接访问 ADC；它的职责是把“检测结果”转换成更稳定、
 * 更适合 UI 展示、物质库保存和后续识别算法使用的特征描述。
 */
#include "IMS_Features.h"
#include <string.h>

#define IMS_RECOGNITION_SKIP_FRONT_RATIO 0.05f
#define IMS_MAIN_PEAK_MIN_SNR 5.0f
#define IMS_MAIN_PEAK_SATURATION_RATIO 0.95f
#define IMS_MIN_REASONABLE_FWHM_MS 0.02f
#define IMS_MAX_REASONABLE_WIDTH_MS 5.0f

static float clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

static float absf_local(float v) {
    return (v < 0.0f) ? -v : v;
}

void IMS_ClearFeatureVector(IMSFeatureVector *feature) {
    if (feature == nullptr) return;
    memset(feature, 0, sizeof(IMSFeatureVector));
    feature->valid = false;
    feature->usable_for_identification = false;
    feature->reject_reason = IMS_FEATURE_NO_PEAK;
}

float IMS_CalcSNR(float main_peak_amp_v, uint32_t noise_counts, float adc_full_scale_v) {
    if (noise_counts == 0 || main_peak_amp_v <= 0.0f || adc_full_scale_v <= 0.0f) {
        return 0.0f;
    }

    float noise_v = ((float)noise_counts * adc_full_scale_v) / 65535.0f;
    if (noise_v <= 0.0f) return 0.0f;

    return main_peak_amp_v / noise_v;
}

void IMS_NormalizeTemplate(const int16_t *display_waveform,
                           int point_count,
                           float *normalized_template,
                           int template_points) {
    // 200 点 display template 与 TFT/Web 显示一致，用于轻量模板比较和调试显示一致性。
    if (normalized_template == nullptr || template_points <= 0) return;

    for (int i = 0; i < template_points; i++) {
        normalized_template[i] = 0.0f;
    }

    if (display_waveform == nullptr || point_count <= 0) return;

    int16_t max_y = 0;
    for (int i = 0; i < point_count; i++) {
        if (display_waveform[i] > max_y) max_y = display_waveform[i];
    }
    if (max_y <= 0) return;

    for (int i = 0; i < template_points; i++) {
        int src_idx = i;
        if (point_count != template_points) {
            src_idx = (int)(((int64_t)i * point_count) / template_points);
            if (src_idx >= point_count) src_idx = point_count - 1;
        }

        normalized_template[i] = clamp01((float)display_waveform[src_idx] / (float)max_y);
    }
}

void IMS_NormalizeRecognitionTemplate(const uint16_t *filtered_spectrum,
                                      int filtered_len,
                                      float *recognition_template,
                                      int template_points) {
    // 识别模板来自原始长度滤波谱图，而不是 200 点显示压缩结果，以保留更多时间分辨率。
    // 当前简单跳过前 5% 点，降低离子门馈通/T0 尖峰主导归一化的概率。
    // TODO: 后续可把 front_blank_samples 从 main.cpp 传入，使跳过区域与实际门控参数严格一致。
    if (recognition_template == nullptr || template_points <= 0) return;

    for (int i = 0; i < template_points; i++) {
        recognition_template[i] = 0.0f;
    }

    if (filtered_spectrum == nullptr || filtered_len <= 0) return;

    int start_idx = (int)((float)filtered_len * IMS_RECOGNITION_SKIP_FRONT_RATIO);
    if (start_idx < 0) start_idx = 0;
    if (start_idx >= filtered_len) start_idx = 0;

    int usable_len = filtered_len - start_idx;
    if (usable_len <= 0) return;

    uint16_t max_y = 0;
    for (int i = start_idx; i < filtered_len; i++) {
        if (filtered_spectrum[i] > max_y) max_y = filtered_spectrum[i];
    }
    if (max_y == 0) return;

    for (int i = 0; i < template_points; i++) {
        int src_idx = start_idx + (int)(((int64_t)i * usable_len) / template_points);
        if (src_idx >= filtered_len) src_idx = filtered_len - 1;
        recognition_template[i] = clamp01((float)filtered_spectrum[src_idx] / (float)max_y);
    }
}

static float calcSnrScore(float snr) {
    if (snr >= 20.0f) return 1.0f;
    if (snr >= 10.0f) return 0.8f;
    if (snr >= 5.0f) return 0.6f;
    if (snr >= 3.0f) return 0.4f;
    if (snr > 0.0f) return 0.2f;
    return 0.0f;
}

static float calcBaselineScore(bool baseline_valid,
                               uint32_t baseline_span_counts,
                               uint32_t threshold_counts) {
    if (!baseline_valid) return 0.2f;
    if (threshold_counts == 0) return 0.8f;

    float span_ratio = (float)baseline_span_counts / (float)threshold_counts;
    if (span_ratio <= 0.25f) return 1.0f;
    if (span_ratio <= 0.50f) return 0.9f;
    if (span_ratio <= 0.75f) return 0.8f;
    if (span_ratio <= 1.00f) return 0.7f;
    return 0.4f;
}

static float calcPeakShapeScore(const IMSFeatureVector *feature) {
    if (!feature || feature->peak_count <= 0) return 0.0f;
    if (feature->main_peak_width_ms <= 0.0f || feature->main_peak_fwhm_ms <= 0.0f) return 0.0f;

    float score = 1.0f;
    if (feature->main_peak_fwhm_ms < IMS_MIN_REASONABLE_FWHM_MS) score -= 0.4f;
    if (feature->main_peak_width_ms > IMS_MAX_REASONABLE_WIDTH_MS) score -= 0.3f;
    uint32_t main_prominence = 0;
    for (int i = 0; i < feature->peak_count && i < IMS_MAX_PEAKS; i++) {
        if (feature->peaks[i].amp_v == feature->main_peak_amp_v) {
            main_prominence = feature->peaks[i].prominence_counts;
            break;
        }
    }
    if (feature->threshold_counts > 0 && main_prominence < feature->threshold_counts) {
        score -= 0.3f;
    }

    return clamp01(score);
}

static float calcPeakCountScore(int peak_count, int input_peak_count) {
    if (peak_count <= 0) return 0.0f;
    if (input_peak_count > IMS_MAX_PEAKS) return 0.65f;
    if (peak_count <= 3) return 1.0f;
    if (peak_count <= IMS_MAX_PEAKS) return 0.8f;
    return 0.6f;
}

static float calcSaturationScore(float main_peak_amp_v, float adc_full_scale_v) {
    if (adc_full_scale_v <= 0.0f) return 0.8f;
    if (main_peak_amp_v > adc_full_scale_v * IMS_MAIN_PEAK_SATURATION_RATIO) return 0.1f;
    return 1.0f;
}

static float calcQualityScore(const IMSFeatureVector *feature,
                              int input_peak_count,
                              float adc_full_scale_v) {
    if (!feature || feature->peak_count <= 0) return 0.0f;

    float snr_score = calcSnrScore(feature->snr);
    float baseline_score = calcBaselineScore(feature->baseline_valid,
                                             feature->baseline_span_counts,
                                             feature->threshold_counts);
    float peak_shape_score = calcPeakShapeScore(feature);
    float peak_count_score = calcPeakCountScore(feature->peak_count, input_peak_count);
    float saturation_score = calcSaturationScore(feature->main_peak_amp_v, adc_full_scale_v);

    float score = 0.35f * snr_score +
                  0.20f * baseline_score +
                  0.20f * peak_shape_score +
                  0.15f * peak_count_score +
                  0.10f * saturation_score;
    return clamp01(score);
}

static void updateUsability(IMSFeatureVector *feature,
                            int input_peak_count,
                            float adc_full_scale_v) {
    if (!feature) return;

    feature->usable_for_identification = false;
    feature->reject_reason = IMS_FEATURE_OK;

    if (feature->peak_count <= 0 || !feature->valid) {
        feature->reject_reason = IMS_FEATURE_NO_PEAK;
        return;
    }
    if (feature->snr < IMS_MAIN_PEAK_MIN_SNR) {
        feature->reject_reason = IMS_FEATURE_LOW_SNR;
        return;
    }
    if (!feature->baseline_valid) {
        feature->reject_reason = IMS_FEATURE_BAD_BASELINE;
        return;
    }
    if (feature->main_peak_fwhm_ms <= 0.0f || feature->main_peak_width_ms <= 0.0f) {
        feature->reject_reason = IMS_FEATURE_BAD_PEAK_SHAPE;
        return;
    }
    if (adc_full_scale_v > 0.0f &&
        feature->main_peak_amp_v > adc_full_scale_v * IMS_MAIN_PEAK_SATURATION_RATIO) {
        feature->reject_reason = IMS_FEATURE_SATURATED;
        return;
    }
    if (input_peak_count > IMS_MAX_PEAKS) {
        feature->reject_reason = IMS_FEATURE_TOO_MANY_PEAKS;
        return;
    }

    feature->usable_for_identification = true;
    feature->reject_reason = IMS_FEATURE_OK;
}

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
                              IMSFeatureVector *out_feature) {
    if (out_feature == nullptr) return;

    IMS_ClearFeatureVector(out_feature);

    out_feature->baseline_v = baseline_v;
    out_feature->noise_counts = noise_counts;
    out_feature->threshold_counts = threshold_counts;
    out_feature->baseline_valid = baseline_valid;
    out_feature->baseline_span_counts = baseline_span_counts;

    IMS_NormalizeTemplate(display_waveform,
                          display_point_count,
                          out_feature->normalized_template,
                          IMS_FEATURE_TEMPLATE_POINTS);
    IMS_NormalizeRecognitionTemplate(filtered_spectrum,
                                     filtered_len,
                                     out_feature->recognition_template,
                                     IMS_RECOGNITION_TEMPLATE_POINTS);

    if (input_peaks == nullptr || input_peak_count <= 0 || sample_rate_hz == 0) {
        out_feature->quality_score = 0.0f;
        out_feature->reject_reason = IMS_FEATURE_NO_PEAK;
        out_feature->valid = false;
        out_feature->usable_for_identification = false;
        return;
    }

    int peak_count = input_peak_count;
    if (peak_count > IMS_MAX_PEAKS) peak_count = IMS_MAX_PEAKS;
    out_feature->peak_count = peak_count;

    int main_peak_index = 0;
    uint64_t total_area_counts = 0;

    for (int i = 0; i < peak_count; i++) {
        IMSPeakFeature *dst = &out_feature->peaks[i];
        const IMSPeakInput *src = &input_peaks[i];

        dst->start_idx = src->start_idx;
        dst->peak_idx = src->peak_idx;
        dst->end_idx = src->end_idx;
        dst->time_ms = src->time_ms;
        dst->amp_v = src->amp_v;
        dst->area_counts = src->area_counts;

        uint32_t width_samples = 0;
        if (src->end_idx >= src->start_idx) {
            width_samples = (uint32_t)src->end_idx - (uint32_t)src->start_idx + 1U;
        }
        float fallback_width_ms = ((float)width_samples * 1000.0f) / (float)sample_rate_hz;
        dst->width_ms = (src->width_ms > 0.0f) ? src->width_ms : fallback_width_ms;
        dst->fwhm_ms = (src->fwhm_ms > 0.0f) ? src->fwhm_ms : (dst->width_ms * 0.5f);
        dst->prominence_counts = src->prominence_counts;
        dst->snr_est = src->snr_est;

        total_area_counts += src->area_counts;
        if (src->amp_v > input_peaks[main_peak_index].amp_v) {
            main_peak_index = i;
        }
    }

    IMSPeakFeature *main_peak = &out_feature->peaks[main_peak_index];
    out_feature->main_peak_time_ms = main_peak->time_ms;
    out_feature->main_peak_amp_v = main_peak->amp_v;
    out_feature->main_peak_area = (float)main_peak->area_counts;
    out_feature->main_peak_width_ms = main_peak->width_ms;
    out_feature->main_peak_fwhm_ms = main_peak->fwhm_ms;
    out_feature->total_peak_area = (float)total_area_counts;

    int second_peak_index = -1;
    for (int i = 0; i < peak_count; i++) {
        if (i == main_peak_index) continue;
        if (second_peak_index < 0 ||
            out_feature->peaks[i].amp_v > out_feature->peaks[second_peak_index].amp_v) {
            second_peak_index = i;
        }
    }
    if (second_peak_index >= 0) {
        out_feature->second_peak_time_ms = out_feature->peaks[second_peak_index].time_ms;
        out_feature->second_peak_amp_v = out_feature->peaks[second_peak_index].amp_v;
    }

    for (int i = 0; i < peak_count; i++) {
        IMSPeakFeature *peak = &out_feature->peaks[i];

        if (out_feature->main_peak_amp_v > 0.0f) {
            peak->amp_ratio = peak->amp_v / out_feature->main_peak_amp_v;
        }
        if (total_area_counts > 0) {
            peak->area_ratio = (float)peak->area_counts / (float)total_area_counts;
        }
        if (out_feature->main_peak_fwhm_ms > 0.0f) {
            peak->fwhm_ratio = peak->fwhm_ms / out_feature->main_peak_fwhm_ms;
        }
        peak->time_delta_to_main_ms = peak->time_ms - out_feature->main_peak_time_ms;

        out_feature->peak_time_deltas_ms[i] = peak->time_delta_to_main_ms;
        out_feature->peak_amp_ratios[i] = peak->amp_ratio;
        out_feature->peak_area_ratios[i] = peak->area_ratio;
        out_feature->peak_fwhm_ratios[i] = peak->fwhm_ratio;

        // 避免极小浮点误差在主峰 delta 上显示成 -0.00。
        if (absf_local(out_feature->peak_time_deltas_ms[i]) < 0.0001f) {
            out_feature->peak_time_deltas_ms[i] = 0.0f;
            peak->time_delta_to_main_ms = 0.0f;
        }
    }

    out_feature->snr = IMS_CalcSNR(out_feature->main_peak_amp_v, noise_counts, adc_full_scale_v);
    out_feature->valid = true;
    out_feature->quality_score = calcQualityScore(out_feature, input_peak_count, adc_full_scale_v);
    updateUsability(out_feature, input_peak_count, adc_full_scale_v);
}

void IMS_BuildFeatureVector(const IMSPeakInput *input_peaks,
                            int input_peak_count,
                            const int16_t *display_waveform,
                            int display_point_count,
                            float baseline_v,
                            uint32_t noise_counts,
                            float adc_full_scale_v,
                            uint32_t sample_rate_hz,
                            IMSFeatureVector *out_feature) {
    IMS_BuildFeatureVectorEx(input_peaks,
                             input_peak_count,
                             nullptr,
                             0,
                             display_waveform,
                             display_point_count,
                             baseline_v,
                             noise_counts,
                             0,
                             true,
                             0,
                             adc_full_scale_v,
                             sample_rate_hz,
                             out_feature);
}
