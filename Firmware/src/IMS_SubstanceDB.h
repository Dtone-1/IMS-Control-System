/**
 * @file IMS_SubstanceDB.h
 * @brief 新版 IMS 物质库接口与公开数据结构。
 *
 * 物质库保存的是每个物质的 IMSFeatureVector 样本和模板特征。
 * UI_manager 通过本文件声明的 API 新建物质、追加样本、删除物质和刷新 UI。
 *
 * 运行时数据库在 RAM/PSRAM 中维护；持久化由 IMS_SubstanceDB.cpp 使用 SPIFFS 文件完成。
 */
#pragma once

#include <Arduino.h>
#include "IMS_Features.h"

#define IMS_DB_MAX_SUBSTANCES 10              // 当前最多保存的物质数量。
#define IMS_DB_MAX_SAMPLES_PER_SUBSTANCE 10   // 每个物质最多累计的样本数量。
#define IMS_DB_NAME_LEN 32                    // 物质名称固定长度，便于二进制持久化。

enum IMSSampleNote : uint8_t {
    IMS_SAMPLE_NOTE_GOOD = 0,
    IMS_SAMPLE_NOTE_NORMAL = 1,
    IMS_SAMPLE_NOTE_LOW_QUALITY = 2,
    IMS_SAMPLE_NOTE_LOW_SNR = 3,
    IMS_SAMPLE_NOTE_BAD_BASELINE = 4,
    IMS_SAMPLE_NOTE_BAD_PEAK_SHAPE = 5,
    IMS_SAMPLE_NOTE_SATURATED = 6,
    IMS_SAMPLE_NOTE_UNKNOWN_QUALITY = 7
};

/**
 * @brief 单个样本记录。
 *
 * 一个样本对应一次有效采集后构建出的 IMSFeatureVector。
 * sample_id 从 1 开始递增，用于表示该物质下第几个样本。
 */
struct IMSSampleRecord {
    IMSFeatureVector feature;  // 本次样本的完整 IMS 特征向量。
    uint32_t sample_id;        // 样本序号。
    bool valid;                // 该样本槽是否有效。
};

/**
 * @brief 对外公开的物质记录结构。
 *
 * UI 通过 IMS_DB_GetSubstance() 获取该结构，用于显示物质名和样本数。
 * 当前内部实现为 compact record，只持久保存 best_sample_1、best_sample_2、
 * latest_sample 和 template_feature。
 * 因此 samples 数组不是完整历史样本列表，而是代表样本视图：
 * - samples[0] = best_sample_1；
 * - samples[1] = best_sample_2；
 * - samples[sample_count - 1] = latest_sample，索引未越界时填入。
 */
struct IMSSubstanceRecord {
    char name[IMS_DB_NAME_LEN];                            // 物质名称。
    uint8_t sample_count;                                  // 已累计样本数。
    IMSSampleRecord samples[IMS_DB_MAX_SAMPLES_PER_SUBSTANCE]; // 公开兼容字段。
    IMSFeatureVector template_feature;                     // 当前物质的平均模板特征。
    float avg_quality_score;                               // 所有保存样本的平均质量。
    float best_quality_score;                              // 历史最高样本质量。
    float worst_quality_score;                             // 历史最低样本质量。
    float latest_quality_score;                            // 最新样本质量。
    uint8_t usable_sample_count;                           // 可进入识别的样本数量。
    uint8_t low_quality_sample_count;                      // quality_score < 0.5 的样本数量。
    uint8_t latest_reject_reason;                          // 最新样本的 IMSFeatureRejectReason。
    uint8_t latest_sample_note;                            // 最新样本质量备注 IMSSampleNote。
    bool valid;                                            // 该物质槽是否有效。
};

void IMS_DB_Init();

int IMS_DB_GetSubstanceCount();

const IMSSubstanceRecord* IMS_DB_GetSubstance(int index);

const char* IMS_DB_GetSubstanceName(int index);

int IMS_DB_GetSampleCount(int index);

int IMS_DB_FindSubstanceByName(const char *name);

bool IMS_DB_CreateSubstanceWithSample(const char *name,
                                      const IMSFeatureVector *feature);

bool IMS_DB_AddSampleToSubstance(int index,
                                 const IMSFeatureVector *feature);

bool IMS_DB_UpdateTemplate(int index);

bool IMS_DB_DeleteSubstance(int index);

void IMS_DB_Clear();

// SPIFFS 文件持久化接口。主文件、临时文件和备份文件由 .cpp 内部管理。
bool IMS_DB_SaveToFile();
bool IMS_DB_LoadFromFile();
bool IMS_DB_ClearFile();

// 兼容旧命名的包装函数。实际不再使用 NVS 保存数据库 blob，而是转到 SPIFFS 文件接口。
bool IMS_DB_SaveToNVS();
bool IMS_DB_LoadFromNVS();
bool IMS_DB_ClearNVS();
