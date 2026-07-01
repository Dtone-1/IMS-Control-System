#pragma once

#include <Arduino.h>
#include <lvgl.h>

#include "IMS_Features.h"
#include "IMS_Identifier.h"
#include "ui/ui.h"

void UI_FeatureDisplay_Init();

void UI_FeatureDisplay_UpdateScreen2WithID(const IMSFeatureVector *feature,
                                           const IMSIdentificationResult *id);

void UI_FeatureDisplay_UpdateScreen2(const IMSFeatureVector *feature);

void UI_FeatureDisplay_UpdateScreen3Snapshot(const IMSFeatureVector *feature,
                                             const IMSIdentificationResult *id);

const char* UI_FeatureDisplay_QualityText(float quality_score);
