/**
 * @file IMS_SpectrumProcessing.cpp
 * @brief IMS 平均谱图预处理与峰检测实现。
 *
 * 本文件处理 main.cpp 中 accumulator_buffer 保存的多次采样累加值。
 * 通过 avg_count 得到平均谱图后，本模块依次完成：
 * 基线估计、噪声估计、阈值计算、基线扣除、3 点中值滤波、200 点显示压缩、
 * 峰检测、峰宽/FWHM/突出度/SNR 估计。
 *
 * 这里仍然是轻量级实时算法，没有引入小波、拟合、机器学习或复杂基线校正。
 */
#include "IMS_SpectrumProcessing.h"

#include <string.h>

struct IMSWindowIndices {
    int baseline_start;
    int baseline_end;
    int noise_start;
    int noise_end;
    int blank_samples;
};

struct IMSBaselineStats {
    uint16_t mean_counts;
    uint32_t span_counts;
};

void IMS_SP_ClearResult(IMSSpectrumProcessResult *result) {
    if (!result) return;
    memset(result, 0, sizeof(IMSSpectrumProcessResult));
}

int IMS_SP_SampleFromUs(int us, uint32_t sample_rate_hz, int raw_data_len) {
    int sample = (int)(((uint64_t)us * sample_rate_hz) / 1000000ULL);
    if (sample < 0) return 0;
    if (sample > raw_data_len) return raw_data_len;
    return sample;
}

float IMS_SP_SampleToMs(float sample_idx, uint32_t sample_rate_hz) {
    return (sample_idx * 1000.0f) / (float)sample_rate_hz;
}

static inline uint16_t avgSampleAt(const uint32_t *accumulator, int idx, int avg_count) {
    return (uint16_t)(accumulator[idx] / (uint32_t)avg_count);
}

static bool validateInputs(const uint32_t *accumulator,
                           int avg_count,
                           const IMSSpectrumConfig *config,
                           uint16_t *baseline_corrected_buffer,
                           uint16_t *median_filtered_buffer,
                           int work_buffer_len,
                           int16_t *display_waveform,
                           int display_waveform_len,
                           IMSSpectrumPeak *peaks,
                           int peaks_len,
                           IMSSpectrumProcessResult *result) {
    if (!accumulator || !config || !baseline_corrected_buffer || !median_filtered_buffer ||
        !display_waveform || !peaks || !result) {
        return false;
    }

    if (avg_count <= 0 || config->sample_rate_hz == 0 || config->raw_data_len <= 0 ||
        config->display_scale_div <= 0 || display_waveform_len <= 0 ||
        work_buffer_len < config->raw_data_len || peaks_len < 0) {
        return false;
    }

    return true;
}

static IMSWindowIndices calculateWindowIndices(const IMSSpectrumConfig *config) {
    IMSWindowIndices idx;
    idx.baseline_start = IMS_SP_SampleFromUs(config->baseline_start_us,
                                             config->sample_rate_hz,
                                             config->raw_data_len);
    idx.baseline_end = IMS_SP_SampleFromUs(config->baseline_end_us,
                                           config->sample_rate_hz,
                                           config->raw_data_len);
    idx.noise_start = IMS_SP_SampleFromUs(config->noise_start_us,
                                          config->sample_rate_hz,
                                          config->raw_data_len);
    idx.noise_end = IMS_SP_SampleFromUs(config->noise_end_us,
                                        config->sample_rate_hz,
                                        config->raw_data_len);
    idx.blank_samples = IMS_SP_SampleFromUs(config->front_blank_us,
                                            config->sample_rate_hz,
                                            config->raw_data_len);
    return idx;
}

static IMSBaselineStats estimateBaselineStats(const uint32_t *accumulator,
                                              int avg_count,
                                              int start_idx,
                                              int end_idx,
                                              int raw_data_len) {
    start_idx = constrain(start_idx, 0, raw_data_len - 1);
    end_idx = constrain(end_idx, start_idx + 1, raw_data_len);

    uint64_t sum = 0;
    uint16_t min_value = UINT16_MAX;
    uint16_t max_value = 0;
    int count = 0;

    for (int i = start_idx; i < end_idx; i++) {
        uint16_t value = avgSampleAt(accumulator, i, avg_count);
        sum += value;
        if (value < min_value) min_value = value;
        if (value > max_value) max_value = value;
        count++;
    }

    IMSBaselineStats stats;
    memset(&stats, 0, sizeof(stats));
    if (count > 0) {
        stats.mean_counts = (uint16_t)(sum / (uint64_t)count);
        stats.span_counts = (uint32_t)max_value - (uint32_t)min_value;
    }
    return stats;
}

static uint32_t estimateNoiseCounts(const uint32_t *accumulator,
                                    int avg_count,
                                    uint16_t baseline,
                                    int start_idx,
                                    int end_idx,
                                    int raw_data_len) {
    start_idx = constrain(start_idx, 0, raw_data_len - 1);
    end_idx = constrain(end_idx, start_idx + 1, raw_data_len);

    // 当前 noise_counts 是相对基线的平均绝对偏差，不是严格 RMS 或标准差。
    // 这样做计算量低，适合 ESP32 实时处理；后续如需 RMS 可另加可选诊断，不默认替换。
    uint64_t sum_abs = 0;
    int count = 0;
    for (int i = start_idx; i < end_idx; i++) {
        int32_t v = (int32_t)avgSampleAt(accumulator, i, avg_count) - (int32_t)baseline;
        sum_abs += (uint32_t)abs(v);
        count++;
    }

    if (count == 0) return 0;
    return (uint32_t)(sum_abs / (uint64_t)count);
}

static uint32_t calculateThreshold(const IMSSpectrumConfig *config, uint32_t noise_counts) {
    uint64_t dynamic_threshold = (uint64_t)noise_counts * (uint64_t)config->threshold_sigma_mult;
    if (dynamic_threshold > UINT32_MAX) dynamic_threshold = UINT32_MAX;
    return max(config->threshold_floor_counts, (uint32_t)dynamic_threshold);
}

static void buildBaselineCorrectedSpectrum(const uint32_t *accumulator,
                                           int avg_count,
                                           uint16_t baseline,
                                           uint16_t *corrected,
                                           int raw_data_len) {
    for (int i = 0; i < raw_data_len; i++) {
        int value = (int)avgSampleAt(accumulator, i, avg_count) - (int)baseline;
        corrected[i] = (value > 0) ? (uint16_t)value : 0;
    }
}

static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) {
        uint16_t tmp = a;
        a = b;
        b = tmp;
    }
    if (b > c) {
        uint16_t tmp = b;
        b = c;
        c = tmp;
    }
    if (a > b) {
        b = a;
    }
    return b;
}

static void applyMedian3Filter(const uint16_t *input, uint16_t *output, int raw_data_len) {
    if (raw_data_len <= 0) return;

    output[0] = input[0];
    for (int i = 1; i < raw_data_len - 1; i++) {
        output[i] = median3(input[i - 1], input[i], input[i + 1]);
    }
    output[raw_data_len - 1] = input[raw_data_len - 1];
}

static void buildDisplayWaveform(const uint16_t *filtered_spectrum,
                                 int blank_samples,
                                 const IMSSpectrumConfig *config,
                                 int16_t *display_waveform,
                                 int display_waveform_len) {
    // 保持“区间最大值”压缩策略，不改成平均值。
    // IMS 峰可能很窄，取最大值能保留窄峰在 200 点屏幕谱图中的可见性。
    int ratio = config->raw_data_len / display_waveform_len;
    if (ratio <= 0) ratio = 1;

    for (int i = 0; i < display_waveform_len; i++) {
        int local_max = 0;

        for (int j = 0; j < ratio; j++) {
            int idx = i * ratio + j;
            if (idx >= config->raw_data_len) break;
            if (idx < blank_samples) continue;

            if ((int)filtered_spectrum[idx] > local_max) {
                local_max = filtered_spectrum[idx];
            }
        }

        int scaled = local_max / config->display_scale_div;
        if (scaled > 512) scaled = 512;
        display_waveform[i] = (int16_t)scaled;
    }
}

static float calculateTotalFrameResponse(const uint16_t *filtered_spectrum,
                                         int blank_samples,
                                         int raw_data_len) {
    if (!filtered_spectrum || raw_data_len <= 0) return 0.0f;
    blank_samples = constrain(blank_samples, 0, raw_data_len);

    uint64_t total = 0;
    for (int i = blank_samples; i < raw_data_len; i++) {
        total += filtered_spectrum[i];
    }

    return (float)total;
}

static float interpolateCrossing(int low_idx,
                                 int high_idx,
                                 uint16_t low_value,
                                 uint16_t high_value,
                                 float half_height) {
    if (high_value == low_value) return (float)high_idx;
    float ratio = (half_height - (float)low_value) / ((float)high_value - (float)low_value);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    return (float)low_idx + ratio * (float)(high_idx - low_idx);
}

static float calculateFWHM(const uint16_t *filtered_spectrum,
                           int start,
                           int peak_idx,
                           int end,
                           uint16_t peak_height,
                           uint32_t sample_rate_hz,
                           float fallback_width_ms) {
    if (peak_height == 0 || peak_idx < start || peak_idx > end) return 0.0f;

    float half_height = (float)peak_height * 0.5f;
    int left = peak_idx;
    while (left > start && filtered_spectrum[left] >= half_height) {
        left--;
    }

    int right = peak_idx;
    while (right < end && filtered_spectrum[right] >= half_height) {
        right++;
    }

    bool has_left = filtered_spectrum[left] < half_height && left < peak_idx;
    bool has_right = filtered_spectrum[right] < half_height && right > peak_idx;
    if (!has_left || !has_right) {
        // 如果半峰高交点不在本峰区间内，说明峰被截断或边界仍高于半峰高。
        // 此时使用 width_ms 作为保守 fallback，避免输出误导性的负值或随机值。
        return fallback_width_ms;
    }

    float left_cross = interpolateCrossing(left,
                                           left + 1,
                                           filtered_spectrum[left],
                                           filtered_spectrum[left + 1],
                                           half_height);
    float right_cross = interpolateCrossing(right,
                                            right - 1,
                                            filtered_spectrum[right],
                                            filtered_spectrum[right - 1],
                                            half_height);

    float fwhm_samples = right_cross - left_cross;
    if (fwhm_samples <= 0.0f) return fallback_width_ms;
    return IMS_SP_SampleToMs(fwhm_samples, sample_rate_hz);
}

static void insertPeakByAmplitude(IMSSpectrumPeak *peaks,
                                  int *peak_count,
                                  int peak_capacity,
                                  const IMSSpectrumPeak &candidate) {
    int count = *peak_count;

    if (count < peak_capacity) {
        peaks[count++] = candidate;
    } else if (peak_capacity <= 0 || candidate.amp_v <= peaks[count - 1].amp_v) {
        return;
    } else {
        peaks[count - 1] = candidate;
    }

    for (int i = count - 1; i > 0 && peaks[i].amp_v > peaks[i - 1].amp_v; i--) {
        IMSSpectrumPeak tmp = peaks[i - 1];
        peaks[i - 1] = peaks[i];
        peaks[i] = tmp;
    }

    *peak_count = count;
}

static int detectPeaks(const uint16_t *filtered_spectrum,
                       uint32_t threshold,
                       uint32_t noise_counts,
                       int blank_samples,
                       const IMSSpectrumConfig *config,
                       IMSSpectrumPeak *peaks,
                       int peaks_len) {
    int peak_count = 0;
    int peak_capacity = peaks_len;
    if (config->max_peaks < peak_capacity) peak_capacity = config->max_peaks;
    if (peak_capacity < 0) peak_capacity = 0;

    int min_width_samples = IMS_SP_SampleFromUs(config->min_peak_width_us,
                                                config->sample_rate_hz,
                                                config->raw_data_len);
    if (min_width_samples < 1) min_width_samples = 1;

    int max_width_samples = 0;
    if (config->max_peak_width_us > 0) {
        max_width_samples = IMS_SP_SampleFromUs(config->max_peak_width_us,
                                                config->sample_rate_hz,
                                                config->raw_data_len);
    }

    int i = blank_samples;
    while (i < config->raw_data_len) {
        int corrected = filtered_spectrum[i];
        if (corrected < (int)threshold) {
            i++;
            continue;
        }

        int start = i;
        int max_idx = i;
        int max_counts = corrected;
        uint64_t area = 0;
        uint64_t weighted_sum = 0;
        uint64_t weight = 0;

        while (i < config->raw_data_len) {
            int value = filtered_spectrum[i];
            if (value < (int)threshold) break;

            if (value > max_counts) {
                max_counts = value;
                max_idx = i;
            }

            area += (uint32_t)value;
            weighted_sum += (uint64_t)i * (uint64_t)value;
            weight += (uint64_t)value;
            i++;
        }

        int end = i - 1;
        if (end >= config->raw_data_len - 1) {
            // 峰延伸到采样窗口末尾时，无法确认右边界，暂不输出。
            continue;
        }

        int width_samples = end - start + 1;
        if (width_samples < min_width_samples) continue;
        if (max_width_samples > 0 && width_samples > max_width_samples) continue;
        if (area < config->min_peak_area_counts) continue;
        if (weight == 0 || max_counts <= (int)threshold) continue;

        int left_edge_value = filtered_spectrum[start];
        if (start > 0) left_edge_value = filtered_spectrum[start - 1];
        int right_edge_value = filtered_spectrum[end];
        if (end + 1 < config->raw_data_len) right_edge_value = filtered_spectrum[end + 1];
        int edge_value = max(left_edge_value, right_edge_value);
        uint32_t prominence = (max_counts > edge_value) ? (uint32_t)(max_counts - edge_value) : 0U;
        if (prominence < config->min_prominence_counts) continue;

        float width_ms = IMS_SP_SampleToMs((float)width_samples, config->sample_rate_hz);
        float fwhm_ms = calculateFWHM(filtered_spectrum,
                                      start,
                                      max_idx,
                                      end,
                                      (uint16_t)max_counts,
                                      config->sample_rate_hz,
                                      width_ms);

        float centroid = (float)((double)weighted_sum / (double)weight);
        IMSSpectrumPeak candidate;
        memset(&candidate, 0, sizeof(candidate));
        candidate.start_idx = (uint16_t)start;
        candidate.peak_idx = (uint16_t)max_idx;
        candidate.end_idx = (uint16_t)end;
        candidate.time_ms = IMS_SP_SampleToMs(centroid, config->sample_rate_hz);
        candidate.amp_v = ((float)max_counts * config->adc_full_scale_v) / 65535.0f;
        candidate.area_counts = (area > UINT32_MAX) ? UINT32_MAX : (uint32_t)area;
        candidate.width_ms = width_ms;
        candidate.fwhm_ms = fwhm_ms;
        candidate.prominence_counts = prominence;
        candidate.snr_est = (noise_counts > 0) ? ((float)max_counts / (float)noise_counts) : 0.0f;
        insertPeakByAmplitude(peaks, &peak_count, peak_capacity, candidate);
    }

    return peak_count;
}

static void fillResult(IMSSpectrumProcessResult *result,
                       const IMSSpectrumPeak *peaks,
                       int peak_count,
                       uint16_t baseline,
                       uint32_t baseline_span,
                       uint32_t noise,
                       uint32_t threshold,
                       const IMSSpectrumConfig *config) {
    result->peak_count = peak_count;
    if (peak_count > 0) {
        result->main_peak_time_ms = peaks[0].time_ms;
        result->main_peak_amp_v = peaks[0].amp_v;
        result->main_peak_area = (float)peaks[0].area_counts;
        result->main_peak_width_ms = peaks[0].width_ms;
        result->main_peak_fwhm_ms = peaks[0].fwhm_ms;
    }
    result->baseline_v = ((float)baseline * config->adc_full_scale_v) / 65535.0f;
    result->noise_counts = noise;
    result->threshold_counts = threshold;
    result->baseline_span_counts = baseline_span;
    result->baseline_valid = (baseline_span <= threshold);
}

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
                                    IMSSpectrumProcessResult *result) {
    IMS_SP_ClearResult(result);

    if (!validateInputs(accumulator,
                        avg_count,
                        config,
                        baseline_corrected_buffer,
                        median_filtered_buffer,
                        work_buffer_len,
                        display_waveform,
                        display_waveform_len,
                        peaks,
                        peaks_len,
                        result)) {
        return false;
    }

    IMSWindowIndices idx = calculateWindowIndices(config);

    IMSBaselineStats baseline_stats = estimateBaselineStats(accumulator,
                                                            avg_count,
                                                            idx.baseline_start,
                                                            idx.baseline_end,
                                                            config->raw_data_len);
    uint32_t noise = estimateNoiseCounts(accumulator,
                                         avg_count,
                                         baseline_stats.mean_counts,
                                         idx.noise_start,
                                         idx.noise_end,
                                         config->raw_data_len);
    uint32_t threshold = calculateThreshold(config, noise);

    buildBaselineCorrectedSpectrum(accumulator,
                                   avg_count,
                                   baseline_stats.mean_counts,
                                   baseline_corrected_buffer,
                                   config->raw_data_len);
    applyMedian3Filter(baseline_corrected_buffer,
                       median_filtered_buffer,
                       config->raw_data_len);

    result->total_frame_response_au = calculateTotalFrameResponse(median_filtered_buffer,
                                                                  idx.blank_samples,
                                                                  config->raw_data_len);

    buildDisplayWaveform(median_filtered_buffer,
                         idx.blank_samples,
                         config,
                         display_waveform,
                         display_waveform_len);

    memset(peaks, 0, sizeof(IMSSpectrumPeak) * peaks_len);
    int peak_count = detectPeaks(median_filtered_buffer,
                                 threshold,
                                 noise,
                                 idx.blank_samples,
                                 config,
                                 peaks,
                                 peaks_len);

    fillResult(result,
               peaks,
               peak_count,
               baseline_stats.mean_counts,
               baseline_stats.span_counts,
               noise,
               threshold,
               config);

    return true;
}
