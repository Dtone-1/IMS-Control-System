#include "UI_FeatureDisplay.h"

static lv_obj_t *s2_label_result = nullptr;
static lv_obj_t *s2_label_main = nullptr;
static lv_obj_t *s2_label_quality = nullptr;

static lv_obj_t *s3_label_result = nullptr;
static lv_obj_t *s3_label_main = nullptr;
static lv_obj_t *s3_label_quality = nullptr;

const char* UI_FeatureDisplay_QualityText(float quality_score) {
    if (quality_score >= 0.8f) return "Good";
    if (quality_score >= 0.5f) return "Normal";
    return "Poor";
}

static lv_obj_t *createLabel(lv_obj_t *parent, const lv_font_t *font) {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_align(label, LV_ALIGN_TOP_LEFT);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(label, lv_color_hex(0x202020), LV_PART_MAIN);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    return label;
}

static void setLabelLayout(lv_obj_t *label, int x, int y, int width, int height) {
    if (!label) return;
    lv_obj_set_pos(label, x, y);
    lv_obj_set_size(label, width, height);
}

static void layoutScreen2Labels() {
    setLabelLayout(s2_label_result, 12, -4, 358, 30);
    setLabelLayout(s2_label_main, 12, 30, 170, 20);
    setLabelLayout(s2_label_quality, 205, 30, 165, 20);
}

static void layoutScreen3Labels() {
    setLabelLayout(s3_label_result, 14, 0, 198, 22);
    setLabelLayout(s3_label_main, 14, 45, 198, 22);
    setLabelLayout(s3_label_quality, 14, 90, 198, 22);
}

static void formatResultText(char *buffer,
                             size_t buffer_size,
                             const IMSFeatureVector *feature,
                             const IMSIdentificationResult *id) {
    if (!feature || !feature->valid) {
        snprintf(buffer, buffer_size, "Result: No Signal");
        return;
    }

    if (id == nullptr || !id->valid) {
        snprintf(buffer, buffer_size, "Result: Feature Ready");
    } else if (id->status == IMS_ID_NO_SIGNAL) {
        snprintf(buffer, buffer_size, "Result: No Signal");
    } else if (id->status == IMS_ID_DB_EMPTY) {
        snprintf(buffer, buffer_size, "Result: No Library");
    } else if (id->status == IMS_ID_UNKNOWN) {
        snprintf(buffer, buffer_size, "Result: Unknown");
    } else if (id->status == IMS_ID_LOW_CONFIDENCE) {
        snprintf(buffer, buffer_size, "Result: Maybe %s", id->name);
    } else if (id->status == IMS_ID_MATCHED) {
        snprintf(buffer, buffer_size, "Result: %s", id->name);
    } else {
        snprintf(buffer, buffer_size, "Result: Unknown");
    }
}

static void formatFeatureText(const IMSFeatureVector *feature,
                              const IMSIdentificationResult *id,
                              char *result_buf,
                              size_t result_buf_size,
                              char *main_buf,
                              size_t main_buf_size,
                              char *quality_buf,
                              size_t quality_buf_size) {
    formatResultText(result_buf, result_buf_size, feature, id);

    if (!feature || !feature->valid) {
        snprintf(main_buf, main_buf_size, "Main: --.-- ms");
        snprintf(quality_buf, quality_buf_size, "Quality: Poor");
        return;
    }

    snprintf(main_buf, main_buf_size, "Main: %.2f ms", feature->main_peak_time_ms);
    snprintf(quality_buf, quality_buf_size, "Quality: %s",
             UI_FeatureDisplay_QualityText(feature->quality_score));
}

void UI_FeatureDisplay_Init() {
    if (ui_Panel5 && s2_label_result == nullptr) {
        s2_label_result = createLabel(ui_Panel5, &lv_font_montserrat_12);
        s2_label_main = createLabel(ui_Panel5, &lv_font_montserrat_12);
        s2_label_quality = createLabel(ui_Panel5, &lv_font_montserrat_12);
        layoutScreen2Labels();
    }

    if (ui_Panel3 && s3_label_result == nullptr) {
        s3_label_result = createLabel(ui_Panel3, &lv_font_montserrat_12);
        s3_label_main = createLabel(ui_Panel3, &lv_font_montserrat_12);
        s3_label_quality = createLabel(ui_Panel3, &lv_font_montserrat_12);
        layoutScreen3Labels();
    }

    UI_FeatureDisplay_UpdateScreen2(nullptr);
    UI_FeatureDisplay_UpdateScreen3Snapshot(nullptr, nullptr);
}

void UI_FeatureDisplay_UpdateScreen2WithID(const IMSFeatureVector *feature,
                                           const IMSIdentificationResult *id) {
    if (!s2_label_result || !s2_label_main || !s2_label_quality) {
        return;
    }

    static char result_buf[64];
    static char main_buf[32];
    static char quality_buf[32];

    formatFeatureText(feature, id,
                      result_buf, sizeof(result_buf),
                      main_buf, sizeof(main_buf),
                      quality_buf, sizeof(quality_buf));

    lv_label_set_text(s2_label_result, result_buf);
    lv_label_set_text(s2_label_main, main_buf);
    lv_label_set_text(s2_label_quality, quality_buf);
}

void UI_FeatureDisplay_UpdateScreen2(const IMSFeatureVector *feature) {
    UI_FeatureDisplay_UpdateScreen2WithID(feature, nullptr);
}

void UI_FeatureDisplay_UpdateScreen3Snapshot(const IMSFeatureVector *feature,
                                             const IMSIdentificationResult *id) {
    if (!s3_label_result || !s3_label_main || !s3_label_quality) {
        return;
    }

    static char result_buf[64];
    static char main_buf[32];
    static char quality_buf[32];

    formatFeatureText(feature, id,
                      result_buf, sizeof(result_buf),
                      main_buf, sizeof(main_buf),
                      quality_buf, sizeof(quality_buf));

    lv_label_set_text(s3_label_result, result_buf);
    lv_label_set_text(s3_label_main, main_buf);
    lv_label_set_text(s3_label_quality, quality_buf);
}
