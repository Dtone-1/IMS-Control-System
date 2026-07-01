#include "IMS_Identifier.h"

#include <math.h>
#include <string.h>

#define IMS_ID_WEIGHT_DRIFT        0.35f
#define IMS_ID_WEIGHT_TEMPLATE     0.35f
#define IMS_ID_WEIGHT_PEAK_PATTERN 0.20f
#define IMS_ID_WEIGHT_SHAPE        0.10f

#define IMS_ID_EVIDENCE_TEMPLATE   0.55f
#define IMS_ID_EVIDENCE_BEST1      0.30f
#define IMS_ID_EVIDENCE_BEST2      0.15f

#define IMS_ID_TEMPLATE_ONLY_WEIGHT 1.0f

#define IMS_ID_MIN_MATCH_SCORE     0.58f
#define IMS_ID_STRONG_SCORE        0.72f
#define IMS_ID_MIN_MARGIN          0.06f

#define IMS_ID_DRIFT_TOL_MIN_MS    0.25f
#define IMS_ID_DRIFT_TOL_MAX_MS    0.80f
#define IMS_ID_PEAK_DELTA_TOL_MS   0.40f
#define IMS_ID_EPS                 1e-6f

static float IMS_ID_Clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

static float IMS_ID_Abs(float v) {
    return (v < 0.0f) ? -v : v;
}

static float IMS_ID_MinFloat(float a, float b) {
    return (a < b) ? a : b;
}

static float IMS_ID_MaxFloat(float a, float b) {
    return (a > b) ? a : b;
}

static float IMS_ID_MinMaxRatioScore(float a, float b) {
    if (a <= 0.0f || b <= 0.0f) return 0.5f;
    return IMS_ID_Clamp01(IMS_ID_MinFloat(a, b) / IMS_ID_MaxFloat(a, b));
}

static bool IMS_ID_FeatureUsable(const IMSFeatureVector *feature) {
    return (feature != nullptr && feature->valid);
}

static bool IMS_ID_SampleUsable(const IMSSampleRecord *sample) {
    return (sample != nullptr && sample->valid && sample->feature.valid);
}

static float IMS_ID_CalcDriftScore(const IMSFeatureVector *current,
                                   const IMSFeatureVector *ref) {
    float delta = IMS_ID_Abs(current->main_peak_time_ms - ref->main_peak_time_ms);
    float tol = 0.35f;

    if (current->main_peak_fwhm_ms > 0.0f && ref->main_peak_fwhm_ms > 0.0f) {
        tol = 0.5f * (current->main_peak_fwhm_ms + ref->main_peak_fwhm_ms);
        if (tol < IMS_ID_DRIFT_TOL_MIN_MS) tol = IMS_ID_DRIFT_TOL_MIN_MS;
        if (tol > IMS_ID_DRIFT_TOL_MAX_MS) tol = IMS_ID_DRIFT_TOL_MAX_MS;
    }

    float ratio = delta / tol;
    return IMS_ID_Clamp01(1.0f / (1.0f + ratio * ratio));
}

static float IMS_ID_CalcCosine01(const float *a,
                                 const float *b,
                                 int len) {
    if (a == nullptr || b == nullptr || len <= 0) return 0.0f;

    float dot = 0.0f;
    float norm_a = 0.0f;
    float norm_b = 0.0f;

    for (int i = 0; i < len; i++) {
        float av = a[i];
        float bv = b[i];
        dot += av * bv;
        norm_a += av * av;
        norm_b += bv * bv;
    }

    float denom = sqrtf(norm_a * norm_b);
    if (denom <= IMS_ID_EPS) return 0.0f;
    return IMS_ID_Clamp01(dot / denom);
}

static float IMS_ID_CalcPearson01(const float *a,
                                  const float *b,
                                  int len) {
    if (a == nullptr || b == nullptr || len <= 1) return 0.0f;

    float mean_a = 0.0f;
    float mean_b = 0.0f;
    for (int i = 0; i < len; i++) {
        mean_a += a[i];
        mean_b += b[i];
    }
    mean_a /= (float)len;
    mean_b /= (float)len;

    float cov = 0.0f;
    float var_a = 0.0f;
    float var_b = 0.0f;
    for (int i = 0; i < len; i++) {
        float da = a[i] - mean_a;
        float db = b[i] - mean_b;
        cov += da * db;
        var_a += da * da;
        var_b += db * db;
    }

    float denom = sqrtf(var_a * var_b);
    if (denom <= IMS_ID_EPS) return 0.0f;

    float pearson = cov / denom;
    if (pearson < -1.0f) pearson = -1.0f;
    if (pearson > 1.0f) pearson = 1.0f;
    return IMS_ID_Clamp01((pearson + 1.0f) * 0.5f);
}

static float IMS_ID_CalcTemplateScore(const IMSFeatureVector *current,
                                      const IMSFeatureVector *ref) {
    float cosine_score = IMS_ID_CalcCosine01(current->recognition_template,
                                            ref->recognition_template,
                                            IMS_RECOGNITION_TEMPLATE_POINTS);
    float pearson_score = IMS_ID_CalcPearson01(current->recognition_template,
                                              ref->recognition_template,
                                              IMS_RECOGNITION_TEMPLATE_POINTS);
    return IMS_ID_Clamp01(0.70f * cosine_score + 0.30f * pearson_score);
}

static float IMS_ID_CalcPeakPatternScore(const IMSFeatureVector *current,
                                         const IMSFeatureVector *ref) {
    int n = current->peak_count;
    if (ref->peak_count < n) n = ref->peak_count;
    if (n > 3) n = 3;

    if (n <= 1) return 0.65f;

    float total = 0.0f;
    for (int i = 0; i < n; i++) {
        float delta_time = IMS_ID_Abs(current->peak_time_deltas_ms[i] -
                                      ref->peak_time_deltas_ms[i]);
        float ratio = delta_time / IMS_ID_PEAK_DELTA_TOL_MS;
        float time_delta_score = 1.0f / (1.0f + ratio * ratio);

        float cur_area_ratio = current->peak_area_ratios[i];
        float ref_area_ratio = ref->peak_area_ratios[i];
        float area_denom = IMS_ID_MaxFloat(cur_area_ratio + ref_area_ratio, IMS_ID_EPS);
        float area_ratio_score = 1.0f - IMS_ID_Abs(cur_area_ratio - ref_area_ratio) / area_denom;

        float cur_amp_ratio = current->peak_amp_ratios[i];
        float ref_amp_ratio = ref->peak_amp_ratios[i];
        float amp_denom = IMS_ID_MaxFloat(cur_amp_ratio + ref_amp_ratio, IMS_ID_EPS);
        float amp_ratio_score = 1.0f - IMS_ID_Abs(cur_amp_ratio - ref_amp_ratio) / amp_denom;

        float single_peak_score = 0.60f * IMS_ID_Clamp01(time_delta_score) +
                                  0.25f * IMS_ID_Clamp01(area_ratio_score) +
                                  0.15f * IMS_ID_Clamp01(amp_ratio_score);
        total += single_peak_score;
    }

    return IMS_ID_Clamp01(total / (float)n);
}

static float IMS_ID_CalcPeakShapeScore(const IMSFeatureVector *current,
                                       const IMSFeatureVector *ref) {
    float fwhm_score = IMS_ID_MinMaxRatioScore(current->main_peak_fwhm_ms,
                                               ref->main_peak_fwhm_ms);
    float width_score = IMS_ID_MinMaxRatioScore(current->main_peak_width_ms,
                                                ref->main_peak_width_ms);
    return IMS_ID_Clamp01(0.70f * fwhm_score + 0.30f * width_score);
}

static float IMS_ID_CalcQualityFactor(const IMSFeatureVector *current,
                                      float ref_quality) {
    float current_q = IMS_ID_Clamp01(current->quality_score);
    float ref_q = IMS_ID_Clamp01(ref_quality);

    float current_factor = 0.75f + 0.25f * current_q;
    float ref_factor = 0.85f + 0.15f * ref_q;
    return IMS_ID_Clamp01(current_factor * ref_factor);
}

static float IMS_ID_CalcFeaturePairScore(const IMSFeatureVector *current,
                                         const IMSFeatureVector *ref,
                                         float ref_quality,
                                         IMSIDScoreBreakdown *breakdown) {
    IMSIDScoreBreakdown local_breakdown;
    memset(&local_breakdown, 0, sizeof(local_breakdown));

    if (!IMS_ID_FeatureUsable(current) || !IMS_ID_FeatureUsable(ref)) {
        if (breakdown != nullptr) *breakdown = local_breakdown;
        return 0.0f;
    }

    local_breakdown.drift_score = IMS_ID_CalcDriftScore(current, ref);
    local_breakdown.template_score = IMS_ID_CalcTemplateScore(current, ref);
    local_breakdown.peak_pattern_score = IMS_ID_CalcPeakPatternScore(current, ref);
    local_breakdown.peak_shape_score = IMS_ID_CalcPeakShapeScore(current, ref);
    local_breakdown.quality_factor = IMS_ID_CalcQualityFactor(current, ref_quality);

    float raw = IMS_ID_WEIGHT_DRIFT * local_breakdown.drift_score +
                IMS_ID_WEIGHT_TEMPLATE * local_breakdown.template_score +
                IMS_ID_WEIGHT_PEAK_PATTERN * local_breakdown.peak_pattern_score +
                IMS_ID_WEIGHT_SHAPE * local_breakdown.peak_shape_score;

    local_breakdown.pair_score = IMS_ID_Clamp01(raw * local_breakdown.quality_factor);

    if (breakdown != nullptr) *breakdown = local_breakdown;
    return local_breakdown.pair_score;
}

static float IMS_ID_TemplateRefQuality(const IMSSubstanceRecord *record) {
    if (record->avg_quality_score > 0.0f) return record->avg_quality_score;
    return record->template_feature.quality_score;
}

static float IMS_ID_CalcRecordScore(const IMSFeatureVector *current,
                                    const IMSSubstanceRecord *record,
                                    IMSIdentificationResult *candidate_debug) {
    if (candidate_debug != nullptr) {
        IMS_ID_ClearResult(candidate_debug);
    }

    if (record == nullptr || !record->valid) return 0.0f;

    IMSIDScoreBreakdown template_breakdown;
    float score_template = IMS_ID_CalcFeaturePairScore(current,
                                                       &record->template_feature,
                                                       IMS_ID_TemplateRefQuality(record),
                                                       &template_breakdown);
    float score_best1 = 0.0f;
    float score_best2 = 0.0f;
    bool best1_valid = IMS_ID_SampleUsable(&record->samples[0]);
    bool best2_valid = IMS_ID_SampleUsable(&record->samples[1]);

    if (best1_valid) {
        score_best1 = IMS_ID_CalcFeaturePairScore(current,
                                                  &record->samples[0].feature,
                                                  record->samples[0].feature.quality_score,
                                                  nullptr);
    }

    if (best2_valid) {
        score_best2 = IMS_ID_CalcFeaturePairScore(current,
                                                  &record->samples[1].feature,
                                                  record->samples[1].feature.quality_score,
                                                  nullptr);
    }

    float record_score = score_template * IMS_ID_TEMPLATE_ONLY_WEIGHT;
    if (best1_valid && best2_valid) {
        record_score = IMS_ID_EVIDENCE_TEMPLATE * score_template +
                       IMS_ID_EVIDENCE_BEST1 * score_best1 +
                       IMS_ID_EVIDENCE_BEST2 * score_best2;
    } else if (best1_valid) {
        record_score = 0.65f * score_template + 0.35f * score_best1;
    }

    if (candidate_debug != nullptr) {
        candidate_debug->template_feature_score = score_template;
        candidate_debug->best_sample_1_score = score_best1;
        candidate_debug->best_sample_2_score = score_best2;
        candidate_debug->best_breakdown = template_breakdown;
    }

    return IMS_ID_Clamp01(record_score);
}

void IMS_ID_ClearResult(IMSIdentificationResult *result) {
    if (result == nullptr) return;
    memset(result, 0, sizeof(IMSIdentificationResult));
    result->status = IMS_ID_NO_SIGNAL;
    result->best_index = -1;
    result->second_index = -1;
    result->name[0] = '\0';
}

const char* IMS_ID_StatusText(IMSIdentificationStatus status) {
    switch (status) {
        case IMS_ID_NO_SIGNAL: return "No Signal";
        case IMS_ID_DB_EMPTY: return "No Library";
        case IMS_ID_UNKNOWN: return "Unknown";
        case IMS_ID_LOW_CONFIDENCE: return "Low Confidence";
        case IMS_ID_MATCHED: return "Matched";
        default: return "Invalid";
    }
}

bool IMS_ID_Identify(const IMSFeatureVector *current,
                     IMSIdentificationResult *out) {
    if (out == nullptr) return false;
    IMS_ID_ClearResult(out);

    if (!IMS_ID_FeatureUsable(current)) {
        out->status = IMS_ID_NO_SIGNAL;
        out->valid = true;
        return true;
    }

    int count = IMS_DB_GetSubstanceCount();
    if (count <= 0) {
        out->status = IMS_ID_DB_EMPTY;
        out->valid = true;
        return true;
    }

    float best_score = -1.0f;
    float second_score = -1.0f;
    int best_index = -1;
    int second_index = -1;
    int valid_record_count = 0;
    IMSIdentificationResult best_debug;
    IMS_ID_ClearResult(&best_debug);

    for (int i = 0; i < IMS_DB_MAX_SUBSTANCES; i++) {
        const IMSSubstanceRecord *record = IMS_DB_GetSubstance(i);
        if (record == nullptr || !record->valid) continue;

        valid_record_count++;

        IMSIdentificationResult candidate_debug;
        float score = IMS_ID_CalcRecordScore(current, record, &candidate_debug);

        if (score > best_score) {
            second_score = best_score;
            second_index = best_index;
            best_score = score;
            best_index = i;
            best_debug = candidate_debug;
        } else if (score > second_score) {
            second_score = score;
            second_index = i;
        }
    }

    if (valid_record_count <= 0 || best_index < 0) {
        out->status = IMS_ID_DB_EMPTY;
        out->valid = true;
        return true;
    }

    if (second_score < 0.0f) second_score = 0.0f;
    float margin = best_score - second_score;

    if (best_score < IMS_ID_MIN_MATCH_SCORE) {
        out->status = IMS_ID_UNKNOWN;
    } else if (margin < IMS_ID_MIN_MARGIN && valid_record_count > 1) {
        out->status = IMS_ID_LOW_CONFIDENCE;
    } else {
        out->status = IMS_ID_MATCHED;
    }

    float margin_score = IMS_ID_Clamp01(margin / 0.20f);
    float confidence = IMS_ID_Clamp01(0.75f * best_score + 0.25f * margin_score);

    const IMSSubstanceRecord *best_record = IMS_DB_GetSubstance(best_index);

    out->best_index = best_index;
    out->second_index = second_index;
    if (best_record != nullptr) {
        strncpy(out->name, best_record->name, IMS_DB_NAME_LEN - 1);
        out->name[IMS_DB_NAME_LEN - 1] = '\0';
        out->best_main_time_ms = best_record->template_feature.main_peak_time_ms;
    }
    out->confidence = confidence;
    out->final_score = IMS_ID_Clamp01(best_score);
    out->second_score = IMS_ID_Clamp01(second_score);
    out->score_margin = margin;
    out->template_feature_score = best_debug.template_feature_score;
    out->best_sample_1_score = best_debug.best_sample_1_score;
    out->best_sample_2_score = best_debug.best_sample_2_score;
    out->best_breakdown = best_debug.best_breakdown;
    out->current_main_time_ms = current->main_peak_time_ms;
    out->valid = true;

    return true;
}
