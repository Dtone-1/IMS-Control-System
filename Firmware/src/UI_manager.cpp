/**
 * @file UI_manager.cpp
 * @brief 物质保存界面、物质库列表和保存状态机实现。
 *
 * 本文件不做 ADC 采集和峰检测，只处理 UI 层的保存流程：
 * - 从 current_features 复制 pending_save_feature 快照；
 * - New Analyte 模式下创建新物质；
 * - Add Sample 模式下选择已有物质并追加样本；
 * - 删除物质时显示确认弹窗；
 * - 动态刷新右侧物质库列表。
 *
 * 新版物质库数据来自 IMS_SubstanceDB，持久化走 SPIFFS 文件。
 * 旧版 myLibrary/Preferences 函数仍保留为遗留接口，新版流程不依赖它。
 */
#include "UI_Manager.h"
#include "UI_FeatureDisplay.h"
#include "IMS_SubstanceDB.h"
#include "IMS_Identifier.h"
#include <Preferences.h>

// Global state owned by this module.
SubstanceData myLibrary[MAX_LIBRARY_SIZE];
Preferences preferences;
IMSFeatureVector pending_save_feature;
IMSIdentificationResult pending_save_id_result;
bool pending_save_valid = false;

// External runtime state.
extern SemaphoreHandle_t dataMutex;
extern IMSFeatureVector current_features;
extern IMSIdentificationResult current_id_result;

// SquareLine exported objects used by this module.
extern lv_obj_t * ui_TextArea1;
extern lv_obj_t * ui_Panel13;
extern lv_obj_t * ui_Label10;
extern lv_obj_t * ui_Keyboard2;

enum IMSSaveMode {
    IMS_SAVE_MODE_NONE = 0,        // 默认状态：当前没有等待执行的新建或追加操作。
    IMS_SAVE_MODE_NEW_ANALYTE,     // New Analyte：输入名称后 Save as New 创建新物质。
    IMS_SAVE_MODE_APPEND_SAMPLE    // Add Sample：选择已有物质后 Confirm Save 追加样本。
};

static IMSSaveMode current_save_mode = IMS_SAVE_MODE_NONE;
// Add Sample 模式下当前选中的物质索引。-1 表示尚未选择。
static int selected_analyte_index = -1;
// 删除确认弹窗对象和待删除索引，只在 UI 线程内使用。
static lv_obj_t *delete_confirm_overlay = nullptr;
static int pending_delete_index = -1;

static void ShowDeleteConfirmDialog(int index);
static void CloseDeleteConfirmDialog();
static void OnDeleteCancelClicked(lv_event_t *e);
static void OnDeleteConfirmClicked(lv_event_t *e);

static bool HasNonWhitespaceText(const char *text) {
    // 保存物质名称时只要求至少包含一个非空白字符，真正的裁剪在 IMS_DB 内完成。
    if (!text) return false;

    while (*text != '\0') {
        if (*text != ' ' && *text != '\t' && *text != '\r' && *text != '\n') {
            return true;
        }
        text++;
    }

    return false;
}

static void OnDeleteBtnClicked(lv_event_t * e) {
    int index = (intptr_t)lv_event_get_user_data(e);
    Delete_Substance_ByIndex(index);
}

static void ConfigureAnalyteLibraryPanel() {
    if (!ui_Panel13) return;

    lv_obj_add_flag(ui_Panel13, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(ui_Panel13, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(ui_Panel13, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(ui_Panel13, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_Panel13, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(ui_Panel13, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_row(ui_Panel13, 8, LV_PART_MAIN);
}

static void ConfigureSelectedLabel() {
    if (!ui_Label10) return;

    lv_obj_set_width(ui_Label10, 240);
    lv_obj_set_height(ui_Label10, 24);
    lv_obj_set_align(ui_Label10, LV_ALIGN_CENTER);
    lv_obj_set_x(ui_Label10, 116);
    lv_obj_set_y(ui_Label10, 125);
    lv_label_set_long_mode(ui_Label10, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(ui_Label10, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
}

static void OnDeleteAnalyteClicked(lv_event_t *e) {
    // 点击列表项右侧 X 后不直接删除，而是先弹确认框，避免误触。
    int index = (int)(intptr_t)lv_event_get_user_data(e);

    const char *name = IMS_DB_GetSubstanceName(index);
    if (!name || name[0] == '\0') {
        Serial.println("[DB] Delete ignored: invalid index");
        return;
    }

    ShowDeleteConfirmDialog(index);
}

static void CloseDeleteConfirmDialog() {
    if (delete_confirm_overlay) {
        lv_obj_del(delete_confirm_overlay);
        delete_confirm_overlay = nullptr;
    }

    pending_delete_index = -1;
}

static void OnDeleteCancelClicked(lv_event_t *e) {
    (void)e;
    Serial.println("[DB] Delete cancelled");
    CloseDeleteConfirmDialog();
}

static void OnDeleteConfirmClicked(lv_event_t *e) {
    (void)e;

    int index = pending_delete_index;
    if (index < 0) {
        CloseDeleteConfirmDialog();
        return;
    }

    if (IMS_DB_DeleteSubstance(index)) {
        // RAM 删除成功后立即保存 SPIFFS。保存失败只影响掉电持久化，不重启系统。
        Serial.printf("[DB] Deleted analyte index=%d\n", index);
        if (!IMS_DB_SaveToFile()) {
            Serial.println("[DB] Delete saved in RAM only: file save failed");
        }
        selected_analyte_index = -1;
        UI_SetSelectedAnalyteText(nullptr);
        Refresh_AnalyteLibrary_UI();
    } else {
        Serial.printf("[DB] Delete analyte failed index=%d\n", index);
    }

    CloseDeleteConfirmDialog();
}

static void ShowDeleteConfirmDialog(int index) {
    // 删除确认弹窗放在 lv_layer_top()，覆盖当前界面但不切换 Screen。
    const char *name = IMS_DB_GetSubstanceName(index);
    if (!name || name[0] == '\0') {
        Serial.println("[DB] Delete ignored: invalid index");
        return;
    }

    char name_buf[IMS_DB_NAME_LEN];
    strncpy(name_buf, name, sizeof(name_buf) - 1);
    name_buf[sizeof(name_buf) - 1] = '\0';

    int sample_count = IMS_DB_GetSampleCount(index);
    CloseDeleteConfirmDialog();
    pending_delete_index = index;

    delete_confirm_overlay = lv_obj_create(lv_layer_top());
    lv_obj_set_size(delete_confirm_overlay, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(delete_confirm_overlay, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(delete_confirm_overlay, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_border_width(delete_confirm_overlay, 0, LV_PART_MAIN);
    lv_obj_clear_flag(delete_confirm_overlay, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *box = lv_obj_create(delete_confirm_overlay);
    lv_obj_set_size(box, 260, 145);
    lv_obj_center(box);
    lv_obj_set_style_radius(box, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(box, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(box, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(box, 14, LV_PART_MAIN);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    char title[IMS_DB_NAME_LEN + 16];
    snprintf(title, sizeof(title), "Delete %s?", name_buf);
    lv_obj_t *title_label = lv_label_create(box);
    lv_obj_set_width(title_label, lv_pct(100));
    lv_label_set_long_mode(title_label, LV_LABEL_LONG_DOT);
    lv_label_set_text(title_label, title);
    lv_obj_set_style_text_color(title_label, lv_color_black(), LV_PART_MAIN);
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);

    char sample_text[24];
    snprintf(sample_text, sizeof(sample_text), "Samples: %d", sample_count);
    lv_obj_t *sample_label = lv_label_create(box);
    lv_label_set_text(sample_label, sample_text);
    lv_obj_set_style_text_color(sample_label, lv_color_black(), LV_PART_MAIN);
    lv_obj_align(sample_label, LV_ALIGN_TOP_LEFT, 0, 34);

    lv_obj_t *btn_cancel = lv_btn_create(box);
    lv_obj_set_size(btn_cancel, 86, 34);
    lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_radius(btn_cancel, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(0x6C757D), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(btn_cancel, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_cancel, OnDeleteCancelClicked, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *lbl_cancel = lv_label_create(btn_cancel);
    lv_label_set_text(lbl_cancel, "Cancel");
    lv_obj_center(lbl_cancel);

    lv_obj_t *btn_delete = lv_btn_create(box);
    lv_obj_set_size(btn_delete, 86, 34);
    lv_obj_align(btn_delete, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_set_style_radius(btn_delete, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn_delete, lv_color_hex(0xC93434), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(btn_delete, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_delete, OnDeleteConfirmClicked, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *lbl_delete = lv_label_create(btn_delete);
    lv_label_set_text(lbl_delete, "Delete");
    lv_obj_center(lbl_delete);
}

static void OnAnalyteItemClicked(lv_event_t *e) {
    // 只有 Add Sample 模式允许点击物质项进行选择。
    // 普通浏览状态下点击列表项不会改变保存目标，避免误操作。
    int index = (int)(intptr_t)lv_event_get_user_data(e);

    if (current_save_mode != IMS_SAVE_MODE_APPEND_SAMPLE) {
        Serial.println("[DB] Select ignored: not in append mode");
        return;
    }

    const char *name = IMS_DB_GetSubstanceName(index);
    if (!name || name[0] == '\0') {
        Serial.println("[DB] Select failed: invalid analyte");
        return;
    }

    selected_analyte_index = index;
    UI_SetSelectedAnalyteText(name);

    Serial.printf("[DB] Selected analyte: %s\n", name);
}

void Save_Library_To_Flash() {
    // 旧版物质库遗留保存接口：保存 myLibrary 到 Preferences。
    // 新版物质库保存不走这里，而是 UI 成功操作后调用 IMS_DB_SaveToFile()。
    preferences.begin("ims_lib", false);
    preferences.putBytes("lib_data", myLibrary, sizeof(myLibrary));
    preferences.end();
    Serial.println("[Manager] Library saved to Flash.");
}

void Init_Library() {
    // 旧版物质库遗留加载接口。当前 setup() 使用 IMS_DB_Init() 加载新版 SPIFFS 数据库。
    preferences.begin("ims_lib", true);

    if (preferences.isKey("lib_data")) {
        preferences.getBytes("lib_data", myLibrary, sizeof(myLibrary));
        Serial.println("[Manager] Library loaded success.");
    } else {
        Serial.println("[Manager] No library found, initialized empty.");
    }

    preferences.end();
}

void Add_Substance(const char* name, float time) {
    // 旧版添加接口，保留用于兼容。新版 Save as New 使用 IMS_DB_CreateSubstanceWithSample()。
    int slot = -1;

    for (int i = 0; i < MAX_LIBRARY_SIZE; i++) {
        if (!myLibrary[i].isValid) {
            slot = i;
            break;
        }
    }

    if (slot != -1) {
        strncpy(myLibrary[slot].name, name, 31);
        myLibrary[slot].name[31] = '\0';
        myLibrary[slot].peakTime = time;
        myLibrary[slot].isValid = true;

        Save_Library_To_Flash();
        Refresh_Library_UI();

        Serial.printf("[Manager] Added: %s at %.2f\n", name, time);
    } else {
        Serial.println("[Manager] Library Full!");
    }
}

void Delete_Substance_ByIndex(int index) {
    // 旧版删除接口，当前新版列表删除走 OnDeleteConfirmClicked() -> IMS_DB_DeleteSubstance()。
    if (index >= 0 && index < MAX_LIBRARY_SIZE) {
        myLibrary[index].isValid = false;
        Save_Library_To_Flash();
        Refresh_Library_UI();
        Serial.printf("[Manager] Deleted index: %d\n", index);
    }
}

void Refresh_Library_UI() {
    Refresh_AnalyteLibrary_UI();
}

void Refresh_AnalyteLibrary_UI() {
    // 根据 IMS_SubstanceDB 当前 RAM 数据库动态重建物质库列表。
    // 注意该函数会操作 LVGL 对象，因此必须在 ui_init() 之后、UI 线程上下文中调用。
    if (ui_Panel13 == NULL) return;

    ConfigureAnalyteLibraryPanel();
    lv_obj_clean(ui_Panel13);

    int visible_count = 0;

    for (int i = 0; i < IMS_DB_MAX_SUBSTANCES; i++) {
        const IMSSubstanceRecord *record = IMS_DB_GetSubstance(i);
        if (record) {
            lv_obj_t * item = lv_obj_create(ui_Panel13);
            lv_obj_set_width(item, lv_pct(100));
            lv_obj_set_height(item, 42);
            lv_obj_set_style_bg_color(item, lv_color_white(), LV_PART_MAIN);
            lv_obj_set_style_radius(item, 5, LV_PART_MAIN);
            lv_obj_set_style_border_width(item, 0, LV_PART_MAIN);
            lv_obj_clear_flag(item, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_set_flex_flow(item, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(item, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_set_style_pad_left(item, 10, LV_PART_MAIN);
            lv_obj_set_style_pad_right(item, 10, LV_PART_MAIN);
            lv_obj_set_style_pad_top(item, 6, LV_PART_MAIN);
            lv_obj_set_style_pad_bottom(item, 6, LV_PART_MAIN);
            lv_obj_set_style_pad_column(item, 10, LV_PART_MAIN);
            lv_obj_add_event_cb(item, OnAnalyteItemClicked, LV_EVENT_CLICKED, (void *)(intptr_t)i);

            lv_obj_t * lbl_name = lv_label_create(item);
            lv_obj_set_width(lbl_name, 120);
            lv_label_set_long_mode(lbl_name, LV_LABEL_LONG_CLIP);
            lv_label_set_text(lbl_name, record->name);
            lv_obj_set_style_text_color(lbl_name, lv_color_black(), LV_PART_MAIN);
            lv_obj_set_flex_grow(lbl_name, 1);

            lv_obj_t * lbl_count = lv_label_create(item);
            lv_obj_set_width(lbl_count, 34);
            char buf[16];
            snprintf(buf, sizeof(buf), "n=%u", record->sample_count);
            lv_label_set_text(lbl_count, buf);
            lv_obj_set_style_text_color(lbl_count, lv_color_black(), LV_PART_MAIN);

            lv_obj_t * btn_delete = lv_btn_create(item);
            lv_obj_set_size(btn_delete, 28, 28);
            lv_obj_set_style_radius(btn_delete, 4, LV_PART_MAIN);
            lv_obj_set_style_bg_color(btn_delete, lv_color_hex(0xC93434), LV_PART_MAIN);
            lv_obj_set_style_shadow_width(btn_delete, 0, LV_PART_MAIN);
            lv_obj_clear_flag(btn_delete, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(btn_delete, LV_OBJ_FLAG_EVENT_BUBBLE);
            lv_obj_add_event_cb(btn_delete, OnDeleteAnalyteClicked, LV_EVENT_CLICKED, (void *)(intptr_t)i);

            lv_obj_t * lbl_delete = lv_label_create(btn_delete);
            lv_label_set_text(lbl_delete, "X");
            lv_obj_center(lbl_delete);

            visible_count++;
        }
    }

    if (visible_count == 0) {
        lv_obj_t * empty = lv_label_create(ui_Panel13);
        lv_label_set_text(empty, "No analytes");
        lv_obj_set_style_text_color(empty, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(empty, &ui_font_Font3, LV_PART_MAIN);
        lv_obj_set_style_pad_left(empty, 10, LV_PART_MAIN);
        lv_obj_set_style_pad_top(empty, 8, LV_PART_MAIN);
    }
}

void UI_SetSelectedAnalyteText(const char *name) {
    // Screen3 上显示当前 Add Sample 目标物质，或 New Analyte/None 状态提示。
    if (!ui_Label10) return;

    ConfigureSelectedLabel();

    if (name && name[0] != '\0') {
        char buf[IMS_DB_NAME_LEN + 12];
        snprintf(buf, sizeof(buf), "Selected: %s", name);
        lv_label_set_text(ui_Label10, buf);
    } else {
        lv_label_set_text(ui_Label10, "Selected: None");
    }
}

void OnConfirmSave(lv_event_t * e) {
    // Add Sample 模式的确认保存：
    // 1. 检查当前模式和 pending_save_feature 快照；
    // 2. 检查是否已选择物质；
    // 3. 调用 IMS_DB_AddSampleToSubstance() 更新 RAM 数据库；
    // 4. 调用 IMS_DB_SaveToFile() 持久化；
    // 5. 刷新 UI 列表和选中标签。
    (void)e;

    Serial.println("[UI] Confirm Save clicked");

    if (current_save_mode != IMS_SAVE_MODE_APPEND_SAMPLE) {
        Serial.println("[DB] Confirm ignored: not in append mode");
        return;
    }

    if (!pending_save_valid || !pending_save_feature.valid) {
        Serial.println("[DB] Cannot add sample: pending sample invalid");
        return;
    }

    if (selected_analyte_index < 0) {
        Serial.println("[DB] Cannot add sample: no analyte selected");
        return;
    }

    const char *name = IMS_DB_GetSubstanceName(selected_analyte_index);
    if (!name || name[0] == '\0') {
        Serial.println("[DB] Cannot add sample: selected analyte invalid");
        selected_analyte_index = -1;
        UI_SetSelectedAnalyteText(nullptr);
        return;
    }
    char selected_name[IMS_DB_NAME_LEN];
    strncpy(selected_name, name, sizeof(selected_name) - 1);
    selected_name[sizeof(selected_name) - 1] = '\0';

    if (IMS_DB_GetSampleCount(selected_analyte_index) >= IMS_DB_MAX_SAMPLES_PER_SUBSTANCE) {
        Serial.println("[DB] Cannot add sample: sample list full");
        return;
    }

    bool ok = IMS_DB_AddSampleToSubstance(selected_analyte_index,
                                          &pending_save_feature);

    if (!ok) {
        Serial.println("[DB] Cannot add sample: add failed");
        return;
    }

    int n = IMS_DB_GetSampleCount(selected_analyte_index);
    Serial.printf("[DB] Added sample to %s n=%d\n", selected_name, n);
    if (!IMS_DB_SaveToFile()) {
        Serial.println("[DB] Added sample in RAM only: file save failed");
    }

    Refresh_AnalyteLibrary_UI();
    UI_SetSelectedAnalyteText(selected_name);
}

void OnAddSampleClick(lv_event_t * e) {
    // 进入追加样本模式。此时用户需要在右侧物质库列表中选择目标物质。
    (void)e;

    current_save_mode = IMS_SAVE_MODE_APPEND_SAMPLE;
    selected_analyte_index = -1;

    UI_SetSelectedAnalyteText(nullptr);
    Serial.println("[DB] Add Sample mode");
}

void OnNewAnalyteClick(lv_event_t * e) {
    // 进入新建物质模式。清空输入框，等待用户输入物质名称。
    (void)e;

    current_save_mode = IMS_SAVE_MODE_NEW_ANALYTE;
    selected_analyte_index = -1;

    if (ui_TextArea1) {
        lv_textarea_set_text(ui_TextArea1, "");
        lv_obj_clear_state(ui_TextArea1, LV_STATE_DISABLED);
    }

    UI_SetSelectedAnalyteText("New Analyte");
    Serial.println("[DB] New Analyte mode");
}

void OnSaveAsNewClick(lv_event_t * e) {
    // New Analyte 模式的保存流程：
    // 1. 检查当前快照是否有效；
    // 2. 读取并校验输入名称；
    // 3. 确认名称不重复；
    // 4. 创建物质并写入第一个样本；
    // 5. 保存 SPIFFS 文件并刷新 UI。
    (void)e;
    Serial.println("[UI] Save as New clicked");

    if (ui_Keyboard2) {
        lv_obj_add_flag(ui_Keyboard2, LV_OBJ_FLAG_HIDDEN);
    }

    if (!pending_save_valid || !pending_save_feature.valid) {
        Serial.println("[DB] Cannot save: pending sample invalid");
        return;
    }

    if (!ui_TextArea1) {
        Serial.println("[DB] Cannot save: text area object is null");
        return;
    }

    const char *name = lv_textarea_get_text(ui_TextArea1);
    if (!HasNonWhitespaceText(name)) {
        Serial.println("[DB] Cannot save: empty name");
        return;
    }

    if (IMS_DB_FindSubstanceByName(name) >= 0) {
        Serial.println("[DB] Cannot save: analyte already exists");
        return;
    }

    if (IMS_DB_CreateSubstanceWithSample(name, &pending_save_feature)) {
        int index = IMS_DB_FindSubstanceByName(name);
        const char *saved_name = (index >= 0) ? IMS_DB_GetSubstanceName(index) : name;
        char saved_name_buf[IMS_DB_NAME_LEN];
        strncpy(saved_name_buf, saved_name ? saved_name : "", sizeof(saved_name_buf) - 1);
        saved_name_buf[sizeof(saved_name_buf) - 1] = '\0';
        int sample_count = (index >= 0) ? IMS_DB_GetSampleCount(index) : 1;

        Serial.printf("[DB] Created analyte: %s n=%d\n", saved_name_buf, sample_count);
        if (!IMS_DB_SaveToFile()) {
            Serial.println("[DB] Created analyte in RAM only: file save failed");
        }
        selected_analyte_index = index;
        Refresh_AnalyteLibrary_UI();
        UI_SetSelectedAnalyteText(saved_name_buf);
        lv_textarea_set_text(ui_TextArea1, "");
        current_save_mode = IMS_SAVE_MODE_NONE;
    } else {
        Serial.println("[DB] Cannot save: create analyte failed");
    }
}

void OnSaveSubstance(lv_event_t * e) {
    // 从 Screen2 点击“保存物质”进入保存界面前调用。
    // 这里从实时 current_features 复制一份快照，防止保存过程中采集任务更新数据。
    (void)e;

    bool ok = CapturePendingFeatureSnapshot();

    if (ok) {
        Serial.printf("[Save Snapshot] valid=1 main=%.2fms peaks=%d snr=%.1f quality=%.2f\n",
                      pending_save_feature.main_peak_time_ms,
                      pending_save_feature.peak_count,
                      pending_save_feature.snr,
                      pending_save_feature.quality_score);
        UI_FeatureDisplay_UpdateScreen3Snapshot(&pending_save_feature, &pending_save_id_result);
    } else {
        Serial.println("[Save Snapshot] invalid");
        UI_FeatureDisplay_UpdateScreen3Snapshot(nullptr, nullptr);
    }

    if (ui_TextArea1) {
        lv_textarea_set_text(ui_TextArea1, "");
    }
}

bool CapturePendingFeatureSnapshot() {
    // current_features 由采集任务更新，复制时必须短时间持有 dataMutex。
    // 只复制结构体，不在这里写文件、不刷新 UI，避免影响采集实时性。
    bool ok = false;

    if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        memcpy(&pending_save_feature, &current_features, sizeof(pending_save_feature));
        memcpy(&pending_save_id_result, &current_id_result, sizeof(pending_save_id_result));
        pending_save_valid = current_features.valid;
        ok = pending_save_valid;
        xSemaphoreGive(dataMutex);
    } else {
        pending_save_valid = false;
        IMS_ClearFeatureVector(&pending_save_feature);
        IMS_ID_ClearResult(&pending_save_id_result);
    }

    if (!pending_save_valid) {
        IMS_ClearFeatureVector(&pending_save_feature);
        IMS_ID_ClearResult(&pending_save_id_result);
    }

    return ok;
}
