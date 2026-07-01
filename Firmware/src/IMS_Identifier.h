#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "IMS_Features.h"
#include "IMS_SubstanceDB.h"

enum IMSIdentificationStatus : uint8_t {
    IMS_ID_NO_SIGNAL = 0,
    IMS_ID_DB_EMPTY = 1,
    IMS_ID_UNKNOWN = 2,
    IMS_ID_LOW_CONFIDENCE = 3,
    IMS_ID_MATCHED = 4
};

struct IMSIDScoreBreakdown {
    float drift_score;
    float template_score;
    float peak_pattern_score;
    float peak_shape_score;
    float quality_factor;
    float pair_score;
};

struct IMSIdentificationResult {
    IMSIdentificationStatus status;

    int best_index;
    int second_index;

    char name[IMS_DB_NAME_LEN];

    float confidence;
    float final_score;
    float second_score;
    float score_margin;

    float template_feature_score;
    float best_sample_1_score;
    float best_sample_2_score;

    IMSIDScoreBreakdown best_breakdown;

    float current_main_time_ms;
    float best_main_time_ms;

    bool valid;
};

void IMS_ID_ClearResult(IMSIdentificationResult *result);

const char* IMS_ID_StatusText(IMSIdentificationStatus status);

bool IMS_ID_Identify(const IMSFeatureVector *current,
                     IMSIdentificationResult *out);
