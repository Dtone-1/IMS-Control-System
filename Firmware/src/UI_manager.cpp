#include "UI_Manager.h"
#include <Preferences.h> // ESP32 NVS 存储库

// =================================================================================
// 全局变量与外部引用
// =================================================================================

// --- 本地全局变量 ---
SubstanceData myLibrary[MAX_LIBRARY_SIZE];
Preferences preferences;

// --- 外部全局变量引用 ---
extern bool isScanning;            // 扫描状态标志位
extern float temp_captured_time;   // 临时捕获的时间（跨屏传递用）
extern float detected_peak_time;   // 实时检测到的峰值时间

// --- UI 组件引用 (需确保变量名与 SquareLine 导出一致) ---

// [Screen 2] 源数据界面
extern lv_obj_t * ui_Label18;      // 漂移时间 Label
extern lv_obj_t * ui_Label19;      // 信号幅值 Label
// extern lv_obj_t * ui_Label_Mobility_S2; // 迁移率 Label (预留)
extern lv_obj_t * ui_Button9;      // "开始扫描" 按钮

// [Screen 3] 目的地/保存界面
extern lv_obj_t * ui_Screen3;      // Screen 3 对象
extern lv_obj_t * ui_Label25;      // 漂移时间 Label (回显)
extern lv_obj_t * ui_Label38;      // 信号幅值 Label (回显)
// extern lv_obj_t * ui_Label_Mobility_S3; // 迁移率 Label (预留)
extern lv_obj_t * ui_TextArea1;    // 物质名称输入框

// =================================================================================
// 辅助功能函数
// =================================================================================

/**
 * @brief 删除按钮回调函数 (内部静态函数)
 * 当列表项中的红色 "X" 按钮被点击时触发
 */
static void OnDeleteBtnClicked(lv_event_t * e) {
    // 1. 获取传递过来的索引 (User Data)
    // 使用 (intptr_t) 安全地将指针转换为整数索引
    int index = (intptr_t)lv_event_get_user_data(e);

    // 2. 调用核心删除逻辑
    Delete_Substance_ByIndex(index);
}

/**
 * @brief 将内存中的库数据保存到 Flash (NVS)
 */
void Save_Library_To_Flash() {
    preferences.begin("ims_lib", false);
    preferences.putBytes("lib_data", myLibrary, sizeof(myLibrary));
    preferences.end();
    Serial.println("[Manager] Library saved to Flash.");
}

/**
 * @brief 初始化：从 Flash 加载数据
 */
void Init_Library() {
    preferences.begin("ims_lib", true); // 只读模式打开
    
    if (preferences.isKey("lib_data")) {
        preferences.getBytes("lib_data", myLibrary, sizeof(myLibrary));
        Serial.println("[Manager] Library loaded success.");
    } else {
        Serial.println("[Manager] No library found, initialized empty.");
    }
    
    preferences.end();
}

// =================================================================================
// 核心业务逻辑
// =================================================================================

/**
 * @brief 添加新物质到库中
 */
void Add_Substance(const char* name, float time) {
    int slot = -1;

    // 1. 寻找空位
    for (int i = 0; i < MAX_LIBRARY_SIZE; i++) {
        if (!myLibrary[i].isValid) {
            slot = i;
            break;
        }
    }
    
    // 2. 执行添加操作
    if (slot != -1) {
        // 更新内存数据
        strncpy(myLibrary[slot].name, name, 31);
        myLibrary[slot].peakTime = time;
        myLibrary[slot].isValid = true;
        
        // 保存并刷新
        Save_Library_To_Flash();
        Refresh_Library_UI(); 
        
        Serial.printf("[Manager] Added: %s at %.2f\n", name, time);
    } else {
        Serial.println("[Manager] Library Full!");
    }
}

/**
 * @brief 根据索引删除物质
 */
void Delete_Substance_ByIndex(int index) {
    if (index >= 0 && index < MAX_LIBRARY_SIZE) {
        myLibrary[index].isValid = false; // 标记无效
        Save_Library_To_Flash();          // 保存更新
        Refresh_Library_UI();             // 刷新列表显示
        Serial.printf("[Manager] Deleted index: %d\n", index);
    }
}

/**
 * @brief [UI核心] 刷新已存物质列表
 * 根据 myLibrary 数组动态绘制 UI 列表项
 */
void Refresh_Library_UI() {
    // 安全检查：确保父容器已初始化
    if (ui_Panel13 == NULL) return;

    // 1. 清空旧列表对象
    lv_obj_clean(ui_Panel13);

    // 2. 遍历数组重绘列表
    for (int i = 0; i < MAX_LIBRARY_SIZE; i++) {
        if (myLibrary[i].isValid) {
            
            // --- 创建横条容器 (Item) ---
            lv_obj_t * item = lv_obj_create(ui_Panel13);
            lv_obj_set_width(item, lv_pct(97));
            lv_obj_set_height(item, 40);
            lv_obj_set_style_bg_color(item, lv_color_white(), LV_PART_MAIN);
            lv_obj_set_style_radius(item, 5, LV_PART_MAIN);
            lv_obj_clear_flag(item, LV_OBJ_FLAG_SCROLLABLE);

            // --- 设置 Flex 布局 (横向排列) ---
            lv_obj_set_flex_flow(item, LV_FLEX_FLOW_ROW); 
            // 左对齐开始，但在交叉轴(垂直)上居中
            lv_obj_set_flex_align(item, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

            // --- 设置间距与边距 ---
            lv_obj_set_style_pad_all(item, 10, LV_PART_MAIN);       // 内部边距
            lv_obj_set_style_pad_column(item, 10, LV_PART_MAIN);    // 元素间隙 (Gap)

            // --- A. 物质名称 Label ---
            lv_obj_t * lbl_name = lv_label_create(item);
            lv_label_set_text(lbl_name, myLibrary[i].name);
            lv_obj_set_style_text_color(lbl_name, lv_color_black(), LV_PART_MAIN);
            // 关键：让名字占据剩余空间，将后续元素推向右侧
            lv_obj_set_flex_grow(lbl_name, 1);

            // --- B. 漂移时间 Label ---
            lv_obj_t * lbl_time = lv_label_create(item);
            char buf[16];
            sprintf(buf, "%.2f ms", myLibrary[i].peakTime);
            lv_label_set_text(lbl_time, buf);
            lv_obj_set_style_text_color(lbl_time, lv_color_black(), LV_PART_MAIN);

            // --- C. 红色删除按钮 ---
            lv_obj_t * btn_del = lv_btn_create(item);
            lv_obj_set_size(btn_del, 30, 30);
            lv_obj_set_style_bg_color(btn_del, lv_color_hex(0xFF0000), LV_PART_MAIN);
            lv_obj_set_style_radius(btn_del, 3, LV_PART_MAIN);
            
            // 按钮上的 "X" 文字
            lv_obj_t * lbl_x = lv_label_create(btn_del);
            lv_label_set_text(lbl_x, "X");
            lv_obj_center(lbl_x);
            
            // --- D. 绑定删除事件 ---
            // 将循环索引 i 作为 UserData 传递
            lv_obj_add_event_cb(btn_del, OnDeleteBtnClicked, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        }
    }
}

// =================================================================================
// UI 事件回调函数
// =================================================================================

/**
 * @brief 点击 "确认保存" 按钮触发
 * 执行数据入库操作
 */
void OnConfirmSave(lv_event_t * e) {
    // 1. 安全检查
    if (ui_TextArea1 == NULL) return; 
    
    // 2. 获取输入内容并校验
    const char * input_name = lv_textarea_get_text(ui_TextArea1);
    if (strlen(input_name) == 0) {
        Serial.println("[Manager] Name is empty, ignored.");
        return; 
    }

    // 3. 添加到库
    // 从 Screen3 的 Label 中读取时间文本并转回 float
    const char* text = lv_label_get_text(ui_Label25);
    Add_Substance(input_name, atof(text));
    
    // 4. 清理 UI 状态
    lv_textarea_set_text(ui_TextArea1, "");
    
    // (可选) 隐藏键盘
    // if(ui_Keyboard_Input) lv_obj_add_flag(ui_Keyboard_Input, LV_OBJ_FLAG_HIDDEN);

    Serial.println("[Manager] Save event handled completely.");
}

/**
 * @brief 点击 "保存物质" (Screen 2 -> Screen 3) 按钮触发
 * 负责暂停扫描、搬运数据并切换屏幕
 */
void OnSaveSubstance(lv_event_t * e) {
    
    // 1. 暂停扫描任务 (Stop Scanning)
    if (isScanning) {
        isScanning = false;
        Serial.println("Scan Paused by Save Action.");
        
        // (可选) 同步按钮状态
        // lv_obj_clear_state(ui_Button_Scan, LV_STATE_CHECKED); 
    }

    // 2. 数据搬运 (Screen 2 -> Screen 3)
    
    // 2.1 搬运漂移时间
    if (ui_Label18 && ui_Label25) {
        const char * text = lv_label_get_text(ui_Label18);
        lv_label_set_text(ui_Label25, text);
    }

    // 2.2 搬运信号幅值
    if (ui_Label19 && ui_Label38) {
        lv_label_set_text(ui_Label38, lv_label_get_text(ui_Label19));
    }

    // 2.3 搬运迁移率 (如有)
    // if(ui_Label_Mobility_S2 && ui_Label_Mobility_S3) { ... }
    
    // 3. 预处理 Screen 3
    if (ui_TextArea1) {
         lv_textarea_set_text(ui_TextArea1, ""); // 清空上次残留的输入
    }
}