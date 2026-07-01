/**
 * @file UI_manager.h
 * @brief 物质保存界面与物质库 UI 交互接口。
 *
 * 本模块连接 SquareLine 生成的 LVGL 界面和 IMS_SubstanceDB 新版物质库。
 * 它负责保存前的特征快照、New Analyte / Add Sample 状态切换、
 * Save as New / Confirm Save 按钮处理，以及物质库列表刷新。
 */
#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include <Arduino.h>
#include "ui/ui.h" 
#include "IMS_Features.h"

// =================================================================================
// 宏定义与常量
// =================================================================================

#define MAX_LIBRARY_SIZE 20  // 物质库最大存储容量

// =================================================================================
// 数据结构定义
// =================================================================================

/**
 * @brief 旧版物质库遗留结构。
 *
 * 当前新版物质库主要走 IMS_SubstanceDB 和 SPIFFS 持久化。
 * 该结构和 myLibrary/Preferences 相关接口保留是为了兼容旧代码和避免编译破坏。
 */
struct SubstanceData {
    char name[32];      // 物质名称
    float peakTime;     // 漂移时间 (ms)
    bool isValid;       // 有效标志位 (true: 有效数据, false: 空位/已删除)
};

// =================================================================================
// 全局变量声明 (Extern)
// =================================================================================

/**
 * @brief 全局物质库数组
 * 定义在 .cpp 文件中，此处声明以便其他模块引用
 */
extern SubstanceData myLibrary[MAX_LIBRARY_SIZE];
// pending_save_feature 是点击“保存物质”时从 current_features 复制出来的快照。
// 这样即使采集任务继续刷新 current_features，保存界面看到的仍是同一帧数据。
extern IMSFeatureVector pending_save_feature;
// pending_save_valid 表示当前快照是否可保存。没有有效峰或没有成功复制时为 false。
extern bool pending_save_valid;

// =================================================================================
// 功能函数声明 - 核心逻辑
// =================================================================================

/**
 * @brief 初始化物质库
 * 系统启动时调用，从 Flash (NVS) 读取历史数据到内存
 */
void Init_Library();

/**
 * @brief 将当前库保存到 Flash
 * 当发生添加或删除操作时调用，确保数据断电不丢失
 */
void Save_Library_To_Flash();

/**
 * @brief 添加新物质
 * @param name 物质名称
 * @param time 漂移时间
 */
void Add_Substance(const char* name, float time);

/**
 * @brief 删除指定物质
 * @param index 数组索引 (0 ~ MAX_LIBRARY_SIZE-1)
 */
void Delete_Substance_ByIndex(int index);

// =================================================================================
// 功能函数声明 - UI 交互
// =================================================================================

/**
 * @brief 刷新 UI 列表
 * 清空右侧列表区并根据 myLibrary 数据重新绘制
 */
void Refresh_Library_UI();
void Refresh_AnalyteLibrary_UI();
void UI_SetSelectedAnalyteText(const char *name);
bool CapturePendingFeatureSnapshot();

/**
 * @brief [Screen 3] "确认保存" 按钮回调
 * 读取输入框内容并写入库
 * 注意：函数名必须与 SquareLine Studio 设置完全一致
 */
void OnConfirmSave(lv_event_t * e);
void OnAddSampleClick(lv_event_t * e);
void OnNewAnalyteClick(lv_event_t * e);
void OnSaveAsNewClick(lv_event_t * e);

/**
 * @brief [Screen 2] "保存物质" 按钮回调
 * 负责暂停扫描、跨屏搬运数据并跳转到 Screen 3
 */
void OnSaveSubstance(lv_event_t * e);

#endif // UI_MANAGER_H
