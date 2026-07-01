/**
 * @file IMS_SubstanceDB.cpp
 * @brief 新版 IMS 物质库运行时管理与 SPIFFS 持久化实现。
 *
 * 本模块负责：
 * - 在 RAM 中维护物质、样本计数和模板特征；
 * - Save as New 时创建新物质；
 * - Add Sample / Confirm Save 时追加样本并更新模板；
 * - Delete 时删除物质；
 * - 启动时从 SPIFFS 加载数据库文件；
 * - 数据变化后安全写入 /ims_db_v1.bin。
 *
 * 当前内部使用 compact record：只保存首个样本、最新样本和平均模板。
 * 这样可以降低 RAM/Flash 占用，但限制是不能回看每一个历史样本的完整列表。
 */
#include "IMS_SubstanceDB.h"

#include <esp_heap_caps.h>
#include <SPIFFS.h>
#include <stddef.h>
#include <string.h>

#define IMS_DB_FILE_MAGIC 0x494D5344UL
#define IMS_DB_FILE_VERSION 4
#define IMS_DB_LOW_QUALITY_THRESHOLD 0.5f

// SPIFFS 持久化文件路径：
// main：正常加载的主数据库文件；
// tmp：保存时先写临时文件并校验，避免写一半破坏主文件；
// bak：覆盖主文件前保存的旧版本，用于主文件损坏时回退。
static const char *IMS_DB_MAIN_PATH = "/ims_db_v1.bin";
static const char *IMS_DB_TMP_PATH = "/ims_db_v1.tmp";
static const char *IMS_DB_BAK_PATH = "/ims_db_v1.bak";

/**
 * @brief 内部压缩物质记录。
 *
 * 设计目的：保留 UI 和后续识别最需要的信息，同时避免把 10 个完整样本全部持久化。
 * 限制：当前只能直接访问两个最高质量样本、latest_sample 和 template_feature，
 * 不能列出该物质的全部历史样本详情。
 */
struct IMSSubstanceCompactRecord {
    char name[IMS_DB_NAME_LEN];
    uint8_t sample_count;
    IMSSampleRecord best_sample_1;
    IMSSampleRecord best_sample_2;
    IMSSampleRecord latest_sample;
    IMSFeatureVector template_feature;
    float template_weight_sum;
    float quality_sum;
    float avg_quality_score;
    float best_quality_score;
    float worst_quality_score;
    float latest_quality_score;
    uint8_t usable_sample_count;
    uint8_t low_quality_sample_count;
    uint8_t latest_reject_reason;
    uint8_t latest_sample_note;
    bool valid;
};

static IMSSubstanceCompactRecord *g_substance_db = nullptr;
static IMSSubstanceRecord *g_public_record = nullptr;
static bool g_spiffs_mounted = false;

/**
 * @brief SPIFFS 文件中的二进制数据库镜像。
 *
 * magic/version 用于识别文件格式；payload_size 防止结构体大小不匹配；
 * crc32 覆盖 crc32 字段之前的所有字节，用于发现断电、短写或格式损坏。
 */
struct IMSPersistentDB {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t payload_size;
    IMSSubstanceCompactRecord substances[IMS_DB_MAX_SUBSTANCES];
    uint32_t crc32;
};

static IMSPersistentDB *g_file_image = nullptr;

static void* IMS_DB_AllocLargeBlock(size_t size) {
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!ptr) ptr = malloc(size);
    if (ptr) memset(ptr, 0, size);
    return ptr;
}

static bool IMS_DB_EnsureStorage() {
    // IMSFeatureVector 增加 recognition_template 后结构体变大。
    // 这些数据库镜像不适合继续放在静态 BSS 中，否则会挤爆 ESP32-S3 内部 DRAM。
    if (!g_substance_db) {
        g_substance_db = (IMSSubstanceCompactRecord *)IMS_DB_AllocLargeBlock(sizeof(IMSSubstanceCompactRecord) * IMS_DB_MAX_SUBSTANCES);
    }
    if (!g_public_record) {
        g_public_record = (IMSSubstanceRecord *)IMS_DB_AllocLargeBlock(sizeof(IMSSubstanceRecord));
    }
    if (!g_file_image) {
        g_file_image = (IMSPersistentDB *)IMS_DB_AllocLargeBlock(sizeof(IMSPersistentDB));
    }

    if (!g_substance_db || !g_public_record || !g_file_image) {
        Serial.println("[DB] Storage alloc failed, start empty");
        return false;
    }
    return true;
}

static uint32_t IMS_DB_CRC32(const uint8_t *data, size_t len) {
    // 标准 CRC32 多项式 0xEDB88320。这里不使用简单累加校验，避免弱校验漏检错误。
    uint32_t crc = 0xFFFFFFFFUL;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 1UL) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

static uint32_t IMS_DB_FileCRC(const IMSPersistentDB *db) {
    return IMS_DB_CRC32((const uint8_t *)db, offsetof(IMSPersistentDB, crc32));
}

static int IMS_DB_CountValidRecords() {
    if (!g_substance_db) return 0;
    int count = 0;
    for (int i = 0; i < IMS_DB_MAX_SUBSTANCES; i++) {
        if (g_substance_db[i].valid) count++;
    }
    return count;
}

static float IMS_DB_Clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

static float IMS_DB_WeightedFloat(float old_value,
                                  float new_value,
                                  float old_weight_sum,
                                  float new_weight,
                                  float new_weight_sum) {
    if (new_weight_sum <= 0.0f) return new_value;
    return (old_value * old_weight_sum + new_value * new_weight) / new_weight_sum;
}

static uint32_t IMS_DB_WeightedU32(uint32_t old_value,
                                   uint32_t new_value,
                                   float old_weight_sum,
                                   float new_weight,
                                   float new_weight_sum) {
    float v = IMS_DB_WeightedFloat((float)old_value,
                                   (float)new_value,
                                   old_weight_sum,
                                   new_weight,
                                   new_weight_sum);
    if (v < 0.0f) v = 0.0f;
    return (uint32_t)(v + 0.5f);
}

static uint16_t IMS_DB_WeightedU16(uint16_t old_value,
                                   uint16_t new_value,
                                   float old_weight_sum,
                                   float new_weight,
                                   float new_weight_sum) {
    float v = IMS_DB_WeightedFloat((float)old_value,
                                   (float)new_value,
                                   old_weight_sum,
                                   new_weight,
                                   new_weight_sum);
    if (v < 0.0f) v = 0.0f;
    if (v > 65535.0f) v = 65535.0f;
    return (uint16_t)(v + 0.5f);
}

static uint8_t IMS_DB_BuildSampleNote(const IMSFeatureVector *feature) {
    if (!feature || !feature->valid) return IMS_SAMPLE_NOTE_UNKNOWN_QUALITY;

    if (feature->reject_reason == IMS_FEATURE_LOW_SNR) return IMS_SAMPLE_NOTE_LOW_SNR;
    if (feature->reject_reason == IMS_FEATURE_BAD_BASELINE) return IMS_SAMPLE_NOTE_BAD_BASELINE;
    if (feature->reject_reason == IMS_FEATURE_BAD_PEAK_SHAPE) return IMS_SAMPLE_NOTE_BAD_PEAK_SHAPE;
    if (feature->reject_reason == IMS_FEATURE_SATURATED) return IMS_SAMPLE_NOTE_SATURATED;

    float q = IMS_DB_Clamp01(feature->quality_score);
    if (q >= 0.8f) return IMS_SAMPLE_NOTE_GOOD;
    if (q >= 0.5f) return IMS_SAMPLE_NOTE_NORMAL;
    return IMS_SAMPLE_NOTE_LOW_QUALITY;
}

static const char* IMS_DB_SampleNoteText(uint8_t note) {
    switch (note) {
        case IMS_SAMPLE_NOTE_GOOD: return "Good";
        case IMS_SAMPLE_NOTE_NORMAL: return "Normal";
        case IMS_SAMPLE_NOTE_LOW_QUALITY: return "LowQuality";
        case IMS_SAMPLE_NOTE_LOW_SNR: return "LowSNR";
        case IMS_SAMPLE_NOTE_BAD_BASELINE: return "BadBaseline";
        case IMS_SAMPLE_NOTE_BAD_PEAK_SHAPE: return "BadPeakShape";
        case IMS_SAMPLE_NOTE_SATURATED: return "Saturated";
        default: return "UnknownQuality";
    }
}

static float IMS_DB_FeatureWeight(const IMSFeatureVector *feature) {
    if (!feature || !feature->valid) return 0.0f;
    return 0.3f + 0.7f * IMS_DB_Clamp01(feature->quality_score);
}

static float IMS_DB_SampleQuality(const IMSSampleRecord *sample) {
    if (!sample || !sample->valid) return -1.0f;
    return IMS_DB_Clamp01(sample->feature.quality_score);
}

static void IMS_DB_UpdateBestSamples(IMSSubstanceCompactRecord *record,
                                     const IMSSampleRecord *new_sample) {
    if (!record || !new_sample || !new_sample->valid) return;

    float new_q = IMS_DB_SampleQuality(new_sample);
    float best1_q = IMS_DB_SampleQuality(&record->best_sample_1);
    float best2_q = IMS_DB_SampleQuality(&record->best_sample_2);

    if (!record->best_sample_1.valid) {
        record->best_sample_1 = *new_sample;
        return;
    }

    if (new_q > best1_q) {
        record->best_sample_2 = record->best_sample_1;
        record->best_sample_1 = *new_sample;
        return;
    }

    if (!record->best_sample_2.valid) {
        record->best_sample_2 = *new_sample;
        return;
    }

    if (new_q > best2_q) {
        record->best_sample_2 = *new_sample;
    }
}

static void IMS_DB_InitQualityStats(IMSSubstanceCompactRecord *record,
                                    const IMSFeatureVector *feature) {
    if (!record || !feature) return;

    float q = IMS_DB_Clamp01(feature->quality_score);
    record->template_weight_sum = IMS_DB_FeatureWeight(feature);
    record->quality_sum = q;
    record->avg_quality_score = q;
    record->best_quality_score = q;
    record->worst_quality_score = q;
    record->latest_quality_score = q;
    record->usable_sample_count = feature->usable_for_identification ? 1 : 0;
    record->low_quality_sample_count = (q < IMS_DB_LOW_QUALITY_THRESHOLD) ? 1 : 0;
    record->latest_reject_reason = feature->reject_reason;
    record->latest_sample_note = IMS_DB_BuildSampleNote(feature);
}

static void IMS_DB_UpdateQualityStatsOnAdd(IMSSubstanceCompactRecord *record,
                                           const IMSFeatureVector *feature) {
    if (!record || !feature || record->sample_count == 0) return;

    float q = IMS_DB_Clamp01(feature->quality_score);
    record->quality_sum += q;
    record->avg_quality_score = record->quality_sum / (float)record->sample_count;
    if (q > record->best_quality_score) record->best_quality_score = q;
    if (q < record->worst_quality_score) record->worst_quality_score = q;
    record->latest_quality_score = q;

    if (feature->usable_for_identification && record->usable_sample_count < 255) {
        record->usable_sample_count++;
    }
    if (q < IMS_DB_LOW_QUALITY_THRESHOLD && record->low_quality_sample_count < 255) {
        record->low_quality_sample_count++;
    }

    record->latest_reject_reason = feature->reject_reason;
    record->latest_sample_note = IMS_DB_BuildSampleNote(feature);
}

static bool IMS_DB_EnsureMounted() {
    // SPIFFS 挂载失败不能影响系统进入 UI；失败时数据库为空，只打印日志。
    if (g_spiffs_mounted) return true;

    g_spiffs_mounted = SPIFFS.begin(true);
    if (g_spiffs_mounted) {
        Serial.println("[DB] SPIFFS mounted");
    } else {
        Serial.println("[DB] SPIFFS mount failed, start empty");
    }

    return g_spiffs_mounted;
}

static bool IMS_DB_ValidateImage(const IMSPersistentDB *image) {
    // 只有 magic、version、payload_size 和 crc32 全部通过后，才能把文件内容复制到运行时数据库。
    if (!image) return false;

    if (image->magic != IMS_DB_FILE_MAGIC) {
        Serial.println("[DB] Load failed: bad magic");
        return false;
    }

    if (image->version != IMS_DB_FILE_VERSION) {
        Serial.println("[DB] Version mismatch, start empty");
        return false;
    }

    if (image->payload_size != sizeof(image->substances)) {
        Serial.println("[DB] Load failed: bad size");
        return false;
    }

    uint32_t crc = IMS_DB_FileCRC(image);
    if (crc != image->crc32) {
        Serial.println("[DB] Load failed: bad crc");
        return false;
    }

    return true;
}

static bool IMS_DB_LoadOneFile(const char *path) {
    Serial.printf("[DB] Try load %s\n", path);

    if (!IMS_DB_EnsureStorage()) return false;

    if (!g_spiffs_mounted) {
        Serial.println("[DB] Load failed: SPIFFS not mounted");
        return false;
    }

    // 文件不存在、大小不对、短读或 CRC 错误都视为加载失败，但不会重启系统。
    if (!SPIFFS.exists(path)) {
        Serial.println("[DB] Load failed: file not found");
        return false;
    }

    File file = SPIFFS.open(path, FILE_READ);
    if (!file) {
        Serial.println("[DB] Load failed: open");
        return false;
    }

    size_t size = file.size();
    if (size != sizeof(IMSPersistentDB)) {
        file.close();
        Serial.println("[DB] Version mismatch, start empty");
        return false;
    }

    memset(g_file_image, 0, sizeof(*g_file_image));
    size_t bytes = file.read((uint8_t *)g_file_image, sizeof(*g_file_image));
    file.close();

    if (bytes != sizeof(*g_file_image)) {
        Serial.println("[DB] Load failed: short read");
        return false;
    }

    if (!IMS_DB_ValidateImage(g_file_image)) {
        return false;
    }

    memset(g_substance_db, 0, sizeof(IMSSubstanceCompactRecord) * IMS_DB_MAX_SUBSTANCES);
    memcpy(g_substance_db, g_file_image->substances, sizeof(IMSSubstanceCompactRecord) * IMS_DB_MAX_SUBSTANCES);
    Serial.printf("[DB] Load ok, substances=%d\n", IMS_DB_CountValidRecords());
    return true;
}

static bool IMS_DB_VerifyFileOnly(const char *path) {
    // 保存流程写完 tmp 后会重新读回校验，确认临时文件完整再替换主文件。
    if (!IMS_DB_EnsureStorage()) return false;
    if (!g_spiffs_mounted || !SPIFFS.exists(path)) return false;

    File file = SPIFFS.open(path, FILE_READ);
    if (!file) return false;

    if (file.size() != sizeof(IMSPersistentDB)) {
        file.close();
        return false;
    }

    memset(g_file_image, 0, sizeof(*g_file_image));
    size_t bytes = file.read((uint8_t *)g_file_image, sizeof(*g_file_image));
    file.close();

    return bytes == sizeof(*g_file_image) && IMS_DB_ValidateImage(g_file_image);
}

static bool buildCleanName(const char *name, char *out, size_t out_len) {
    if (!name || !out || out_len == 0) return false;

    while (*name == ' ' || *name == '\t' || *name == '\r' || *name == '\n') {
        name++;
    }

    size_t len = 0;
    while (name[len] != '\0') {
        len++;
    }

    while (len > 0) {
        char c = name[len - 1];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            len--;
        } else {
            break;
        }
    }

    if (len == 0) return false;

    size_t copy_len = len;
    if (copy_len >= out_len) copy_len = out_len - 1;
    memcpy(out, name, copy_len);
    out[copy_len] = '\0';
    return true;
}

void IMS_DB_Clear() {
    // 清空 RAM 中的数据库。启动加载失败或用户清库时必须把所有槽位归零。
    if (!IMS_DB_EnsureStorage()) return;
    memset(g_substance_db, 0, sizeof(IMSSubstanceCompactRecord) * IMS_DB_MAX_SUBSTANCES);
}

void IMS_DB_Init() {
    // 启动流程：先清空 RAM，再挂载 SPIFFS，然后按主文件 -> 备份文件顺序加载。
    // 本函数不依赖 LVGL，也不操作任何 UI 对象，因此可安全放在 ui_init() 前或后。
    if (!IMS_DB_EnsureStorage()) {
        return;
    }

    IMS_DB_Clear();

    if (!IMS_DB_EnsureMounted()) {
        return;
    }

    if (IMS_DB_LoadFromFile()) {
        return;
    }

    IMS_DB_Clear();
    Serial.println("[DB] No valid DB file, start empty");
}

int IMS_DB_GetSubstanceCount() {
    return IMS_DB_CountValidRecords();
}

const IMSSubstanceRecord* IMS_DB_GetSubstance(int index) {
    if (!IMS_DB_EnsureStorage()) return nullptr;
    if (index < 0 || index >= IMS_DB_MAX_SUBSTANCES) return nullptr;
    if (!g_substance_db[index].valid) return nullptr;

    // 将内部 compact record 展开成公开结构。
    // samples[] 不是完整历史列表，只是代表样本视图：best1、best2 和 latest。
    memset(g_public_record, 0, sizeof(*g_public_record));
    strncpy(g_public_record->name, g_substance_db[index].name, IMS_DB_NAME_LEN - 1);
    g_public_record->name[IMS_DB_NAME_LEN - 1] = '\0';
    g_public_record->sample_count = g_substance_db[index].sample_count;

    if (g_substance_db[index].best_sample_1.valid) {
        g_public_record->samples[0] = g_substance_db[index].best_sample_1;
    }
    if (IMS_DB_MAX_SAMPLES_PER_SUBSTANCE > 1 && g_substance_db[index].best_sample_2.valid) {
        g_public_record->samples[1] = g_substance_db[index].best_sample_2;
    }

    uint8_t latest_index = (g_substance_db[index].sample_count > 0) ?
                           (uint8_t)(g_substance_db[index].sample_count - 1) : 0;
    if (g_substance_db[index].latest_sample.valid &&
        latest_index < IMS_DB_MAX_SAMPLES_PER_SUBSTANCE) {
        g_public_record->samples[latest_index] = g_substance_db[index].latest_sample;
    }
    g_public_record->template_feature = g_substance_db[index].template_feature;
    g_public_record->avg_quality_score = g_substance_db[index].avg_quality_score;
    g_public_record->best_quality_score = g_substance_db[index].best_quality_score;
    g_public_record->worst_quality_score = g_substance_db[index].worst_quality_score;
    g_public_record->latest_quality_score = g_substance_db[index].latest_quality_score;
    g_public_record->usable_sample_count = g_substance_db[index].usable_sample_count;
    g_public_record->low_quality_sample_count = g_substance_db[index].low_quality_sample_count;
    g_public_record->latest_reject_reason = g_substance_db[index].latest_reject_reason;
    g_public_record->latest_sample_note = g_substance_db[index].latest_sample_note;
    g_public_record->valid = true;

    return g_public_record;
}

const char* IMS_DB_GetSubstanceName(int index) {
    const IMSSubstanceRecord *record = IMS_DB_GetSubstance(index);
    if (!record) return nullptr;
    return record->name;
}

int IMS_DB_GetSampleCount(int index) {
    const IMSSubstanceRecord *record = IMS_DB_GetSubstance(index);
    if (!record) return 0;
    return record->sample_count;
}

int IMS_DB_FindSubstanceByName(const char *name) {
    if (!IMS_DB_EnsureStorage()) return -1;
    char clean_name[IMS_DB_NAME_LEN];
    if (!buildCleanName(name, clean_name, sizeof(clean_name))) return -1;

    for (int i = 0; i < IMS_DB_MAX_SUBSTANCES; i++) {
        if (g_substance_db[i].valid &&
            strncmp(g_substance_db[i].name, clean_name, IMS_DB_NAME_LEN) == 0) {
            return i;
        }
    }

    return -1;
}

bool IMS_DB_CreateSubstanceWithSample(const char *name,
                                      const IMSFeatureVector *feature) {
    // Save as New 的核心入口：校验名称和特征，找空槽，写入第一个样本和初始模板。
    if (!IMS_DB_EnsureStorage()) return false;
    char clean_name[IMS_DB_NAME_LEN];
    if (!buildCleanName(name, clean_name, sizeof(clean_name))) return false;
    if (!feature || !feature->valid) return false;
    if (IMS_DB_FindSubstanceByName(clean_name) >= 0) return false;

    int slot = -1;
    for (int i = 0; i < IMS_DB_MAX_SUBSTANCES; i++) {
        if (!g_substance_db[i].valid) {
            slot = i;
            break;
        }
    }

    if (slot < 0) return false;

    float weight = IMS_DB_FeatureWeight(feature);
    if (weight <= 0.0f) return false;

    IMSSubstanceCompactRecord *record = &g_substance_db[slot];
    memset(record, 0, sizeof(*record));
    strncpy(record->name, clean_name, IMS_DB_NAME_LEN - 1);
    record->name[IMS_DB_NAME_LEN - 1] = '\0';
    record->sample_count = 1;
    record->best_sample_1.feature = *feature;
    record->best_sample_1.sample_id = 1;
    record->best_sample_1.valid = true;
    memset(&record->best_sample_2, 0, sizeof(record->best_sample_2));
    record->latest_sample = record->best_sample_1;
    record->template_feature = *feature;
    record->template_feature.valid = true;
    IMS_DB_InitQualityStats(record, feature);
    record->valid = true;

    Serial.printf("[DB] Create substance name=%s samples=1 q=%.2f w=%.2f note=%s usable=%d best1=%.2f best2=--\n",
                  record->name,
                  IMS_DB_Clamp01(feature->quality_score),
                  weight,
                  IMS_DB_SampleNoteText(record->latest_sample_note),
                  feature->usable_for_identification ? 1 : 0,
                  IMS_DB_SampleQuality(&record->best_sample_1));

    return true;
}

static bool IMS_DB_AccumulateTemplate(IMSSubstanceCompactRecord *record,
                                      const IMSFeatureVector *feature) {
    // 追加样本时用 quality_score 加权平均更新模板。
    // 低质量样本仍然保存，也仍然有基础权重，但不会像高质量样本一样强烈污染模板。
    if (!record || !feature || !feature->valid) return false;
    if (!record->template_feature.valid || record->sample_count == 0) return false;

    float old_weight_sum = record->template_weight_sum;
    if (old_weight_sum <= 0.0f) {
        old_weight_sum = (record->sample_count > 0) ? (float)record->sample_count : 1.0f;
    }

    float new_weight = IMS_DB_FeatureWeight(feature);
    if (new_weight <= 0.0f) return false;

    float new_weight_sum = old_weight_sum + new_weight;
    if (new_weight_sum <= 0.0f) return false;

    float q = IMS_DB_Clamp01(feature->quality_score);
    float estimated_avg_quality =
        (record->quality_sum + q) / (float)((int)record->sample_count + 1);

    IMSFeatureVector average = record->template_feature;

    average.peak_count = (int)(IMS_DB_WeightedFloat((float)record->template_feature.peak_count,
                                                    (float)feature->peak_count,
                                                    old_weight_sum,
                                                    new_weight,
                                                    new_weight_sum) + 0.5f);
    average.main_peak_time_ms = IMS_DB_WeightedFloat(record->template_feature.main_peak_time_ms,
                                                     feature->main_peak_time_ms,
                                                     old_weight_sum, new_weight, new_weight_sum);
    average.main_peak_amp_v = IMS_DB_WeightedFloat(record->template_feature.main_peak_amp_v,
                                                   feature->main_peak_amp_v,
                                                   old_weight_sum, new_weight, new_weight_sum);
    average.main_peak_area = IMS_DB_WeightedFloat(record->template_feature.main_peak_area,
                                                  feature->main_peak_area,
                                                  old_weight_sum, new_weight, new_weight_sum);
    average.main_peak_width_ms = IMS_DB_WeightedFloat(record->template_feature.main_peak_width_ms,
                                                      feature->main_peak_width_ms,
                                                      old_weight_sum, new_weight, new_weight_sum);
    average.main_peak_fwhm_ms = IMS_DB_WeightedFloat(record->template_feature.main_peak_fwhm_ms,
                                                     feature->main_peak_fwhm_ms,
                                                     old_weight_sum, new_weight, new_weight_sum);
    average.second_peak_time_ms = IMS_DB_WeightedFloat(record->template_feature.second_peak_time_ms,
                                                       feature->second_peak_time_ms,
                                                       old_weight_sum, new_weight, new_weight_sum);
    average.second_peak_amp_v = IMS_DB_WeightedFloat(record->template_feature.second_peak_amp_v,
                                                     feature->second_peak_amp_v,
                                                     old_weight_sum, new_weight, new_weight_sum);
    average.baseline_v = IMS_DB_WeightedFloat(record->template_feature.baseline_v,
                                              feature->baseline_v,
                                              old_weight_sum, new_weight, new_weight_sum);
    average.noise_counts = IMS_DB_WeightedU32(record->template_feature.noise_counts,
                                              feature->noise_counts,
                                              old_weight_sum, new_weight, new_weight_sum);
    average.threshold_counts = IMS_DB_WeightedU32(record->template_feature.threshold_counts,
                                                  feature->threshold_counts,
                                                  old_weight_sum, new_weight, new_weight_sum);
    average.baseline_valid = record->template_feature.baseline_valid && feature->baseline_valid;
    average.baseline_span_counts = IMS_DB_WeightedU32(record->template_feature.baseline_span_counts,
                                                      feature->baseline_span_counts,
                                                      old_weight_sum, new_weight, new_weight_sum);
    average.snr = IMS_DB_WeightedFloat(record->template_feature.snr,
                                       feature->snr,
                                       old_weight_sum, new_weight, new_weight_sum);
    average.total_peak_area = IMS_DB_WeightedFloat(record->template_feature.total_peak_area,
                                                   feature->total_peak_area,
                                                   old_weight_sum, new_weight, new_weight_sum);
    average.quality_score = IMS_DB_WeightedFloat(record->template_feature.quality_score,
                                                 feature->quality_score,
                                                 old_weight_sum, new_weight, new_weight_sum);
    average.usable_for_identification =
        (estimated_avg_quality >= IMS_DB_LOW_QUALITY_THRESHOLD) ||
        feature->usable_for_identification;
    average.reject_reason = average.usable_for_identification ? IMS_FEATURE_OK : feature->reject_reason;

    for (int i = 0; i < IMS_FEATURE_TEMPLATE_POINTS; i++) {
        average.normalized_template[i] =
            IMS_DB_WeightedFloat(record->template_feature.normalized_template[i],
                                 feature->normalized_template[i],
                                 old_weight_sum, new_weight, new_weight_sum);
    }

    for (int i = 0; i < IMS_RECOGNITION_TEMPLATE_POINTS; i++) {
        average.recognition_template[i] =
            IMS_DB_WeightedFloat(record->template_feature.recognition_template[i],
                                 feature->recognition_template[i],
                                 old_weight_sum, new_weight, new_weight_sum);
    }

    for (int i = 0; i < IMS_MAX_PEAKS; i++) {
        average.peak_time_deltas_ms[i] =
            IMS_DB_WeightedFloat(record->template_feature.peak_time_deltas_ms[i],
                                 feature->peak_time_deltas_ms[i],
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peak_amp_ratios[i] =
            IMS_DB_WeightedFloat(record->template_feature.peak_amp_ratios[i],
                                 feature->peak_amp_ratios[i],
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peak_area_ratios[i] =
            IMS_DB_WeightedFloat(record->template_feature.peak_area_ratios[i],
                                 feature->peak_area_ratios[i],
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peak_fwhm_ratios[i] =
            IMS_DB_WeightedFloat(record->template_feature.peak_fwhm_ratios[i],
                                 feature->peak_fwhm_ratios[i],
                                 old_weight_sum, new_weight, new_weight_sum);

        average.peaks[i].start_idx = IMS_DB_WeightedU16(record->template_feature.peaks[i].start_idx,
                                                        feature->peaks[i].start_idx,
                                                        old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].peak_idx = IMS_DB_WeightedU16(record->template_feature.peaks[i].peak_idx,
                                                       feature->peaks[i].peak_idx,
                                                       old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].end_idx = IMS_DB_WeightedU16(record->template_feature.peaks[i].end_idx,
                                                      feature->peaks[i].end_idx,
                                                      old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].time_ms =
            IMS_DB_WeightedFloat(record->template_feature.peaks[i].time_ms,
                                 feature->peaks[i].time_ms,
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].amp_v =
            IMS_DB_WeightedFloat(record->template_feature.peaks[i].amp_v,
                                 feature->peaks[i].amp_v,
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].area_counts = IMS_DB_WeightedU32(record->template_feature.peaks[i].area_counts,
                                                          feature->peaks[i].area_counts,
                                                          old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].width_ms =
            IMS_DB_WeightedFloat(record->template_feature.peaks[i].width_ms,
                                 feature->peaks[i].width_ms,
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].fwhm_ms =
            IMS_DB_WeightedFloat(record->template_feature.peaks[i].fwhm_ms,
                                 feature->peaks[i].fwhm_ms,
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].prominence_counts = IMS_DB_WeightedU32(record->template_feature.peaks[i].prominence_counts,
                                                                feature->peaks[i].prominence_counts,
                                                                old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].snr_est =
            IMS_DB_WeightedFloat(record->template_feature.peaks[i].snr_est,
                                 feature->peaks[i].snr_est,
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].amp_ratio =
            IMS_DB_WeightedFloat(record->template_feature.peaks[i].amp_ratio,
                                 feature->peaks[i].amp_ratio,
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].area_ratio =
            IMS_DB_WeightedFloat(record->template_feature.peaks[i].area_ratio,
                                 feature->peaks[i].area_ratio,
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].fwhm_ratio =
            IMS_DB_WeightedFloat(record->template_feature.peaks[i].fwhm_ratio,
                                 feature->peaks[i].fwhm_ratio,
                                 old_weight_sum, new_weight, new_weight_sum);
        average.peaks[i].time_delta_to_main_ms =
            IMS_DB_WeightedFloat(record->template_feature.peaks[i].time_delta_to_main_ms,
                                 feature->peaks[i].time_delta_to_main_ms,
                                 old_weight_sum, new_weight, new_weight_sum);
    }

    average.valid = true;
    record->template_feature = average;
    record->template_weight_sum = new_weight_sum;
    return true;
}

bool IMS_DB_AddSampleToSubstance(int index,
                                 const IMSFeatureVector *feature) {
    // Confirm Save 的核心入口：追加一个样本，并把模板特征更新为所有样本的平均近似。
    if (!IMS_DB_EnsureStorage()) return false;
    if (index < 0 || index >= IMS_DB_MAX_SUBSTANCES) return false;
    if (!feature || !feature->valid) return false;

    IMSSubstanceCompactRecord *record = &g_substance_db[index];
    if (!record->valid) return false;
    if (record->sample_count >= IMS_DB_MAX_SAMPLES_PER_SUBSTANCE) return false;

    float weight = IMS_DB_FeatureWeight(feature);
    if (!IMS_DB_AccumulateTemplate(record, feature)) return false;

    uint8_t sample_id = record->sample_count + 1;
    IMSSampleRecord new_sample;
    memset(&new_sample, 0, sizeof(new_sample));
    new_sample.feature = *feature;
    new_sample.sample_id = sample_id;
    new_sample.valid = true;

    record->latest_sample = new_sample;
    IMS_DB_UpdateBestSamples(record, &new_sample);
    record->sample_count++;
    IMS_DB_UpdateQualityStatsOnAdd(record, feature);

    if (!IMS_DB_UpdateTemplate(index)) {
        record->sample_count--;
        memset(&record->latest_sample, 0, sizeof(record->latest_sample));
        return false;
    }

    Serial.printf("[DB] Add sample index=%d samples=%u q=%.2f w=%.2f avgQ=%.2f note=%s usable=%d templateW=%.2f best1=%.2f best2=%.2f\n",
                  index,
                  record->sample_count,
                  IMS_DB_Clamp01(feature->quality_score),
                  weight,
                  record->avg_quality_score,
                  IMS_DB_SampleNoteText(record->latest_sample_note),
                  feature->usable_for_identification ? 1 : 0,
                  record->template_weight_sum,
                  IMS_DB_SampleQuality(&record->best_sample_1),
                  IMS_DB_SampleQuality(&record->best_sample_2));

    return true;
}

bool IMS_DB_UpdateTemplate(int index) {
    // 当前模板更新策略：
    // - 只有 1 个样本时，模板就是第一个样本；
    // - 多样本时，IMS_DB_AccumulateTemplate() 已经做过增量平均，这里只检查有效性。
    if (!IMS_DB_EnsureStorage()) return false;
    if (index < 0 || index >= IMS_DB_MAX_SUBSTANCES) return false;

    IMSSubstanceCompactRecord *record = &g_substance_db[index];
    if (!record->valid || record->sample_count == 0) return false;

    if (record->sample_count == 1) {
        if (!record->best_sample_1.valid || !record->best_sample_1.feature.valid) return false;
        record->template_feature = record->best_sample_1.feature;
        record->template_feature.valid = true;
        return true;
    }

    return record->template_feature.valid;
}

bool IMS_DB_DeleteSubstance(int index) {
    // 删除只清空运行时槽位。UI 成功删除后会调用 IMS_DB_SaveToFile() 持久化这个变化。
    if (!IMS_DB_EnsureStorage()) return false;
    if (index < 0 || index >= IMS_DB_MAX_SUBSTANCES) return false;
    if (!g_substance_db[index].valid) return false;

    memset(&g_substance_db[index], 0, sizeof(g_substance_db[index]));
    return true;
}

bool IMS_DB_SaveToFile() {
    // 安全保存流程：
    // 1. 构造完整文件镜像；
    // 2. 写入 tmp；
    // 3. 读回 tmp 并校验；
    // 4. 将旧 main 备份为 bak；
    // 5. 将 tmp 改名为 main。
    // 任一步失败都只返回 false 和打印日志，不重启、不阻塞系统。
    Serial.println("[DB] Save begin");

    if (!IMS_DB_EnsureStorage()) {
        Serial.println("[DB] Save failed: storage alloc");
        return false;
    }

    if (!IMS_DB_EnsureMounted()) {
        Serial.println("[DB] Save failed: SPIFFS not mounted");
        return false;
    }

    memset(g_file_image, 0, sizeof(*g_file_image));
    g_file_image->magic = IMS_DB_FILE_MAGIC;
    g_file_image->version = IMS_DB_FILE_VERSION;
    g_file_image->reserved = 0;
    g_file_image->payload_size = sizeof(g_file_image->substances);
    memcpy(g_file_image->substances, g_substance_db, sizeof(IMSSubstanceCompactRecord) * IMS_DB_MAX_SUBSTANCES);
    g_file_image->crc32 = IMS_DB_FileCRC(g_file_image);

    if (SPIFFS.exists(IMS_DB_TMP_PATH)) {
        SPIFFS.remove(IMS_DB_TMP_PATH);
    }

    File file = SPIFFS.open(IMS_DB_TMP_PATH, FILE_WRITE);
    if (!file) {
        Serial.println("[DB] Save failed: open tmp");
        return false;
    }

    size_t bytes = file.write((const uint8_t *)g_file_image, sizeof(*g_file_image));
    file.flush();
    file.close();

    if (bytes != sizeof(*g_file_image)) {
        Serial.println("[DB] Save failed: short write");
        SPIFFS.remove(IMS_DB_TMP_PATH);
        return false;
    }

    Serial.printf("[DB] Write tmp ok, bytes=%u\n", (unsigned)bytes);

    if (!IMS_DB_VerifyFileOnly(IMS_DB_TMP_PATH)) {
        Serial.println("[DB] Save failed: tmp verify");
        SPIFFS.remove(IMS_DB_TMP_PATH);
        return false;
    }

    Serial.println("[DB] Verify tmp ok");

    if (SPIFFS.exists(IMS_DB_BAK_PATH) && !SPIFFS.remove(IMS_DB_BAK_PATH)) {
        Serial.println("[DB] Save failed: remove old backup");
        SPIFFS.remove(IMS_DB_TMP_PATH);
        return false;
    }

    if (SPIFFS.exists(IMS_DB_MAIN_PATH)) {
        if (!SPIFFS.rename(IMS_DB_MAIN_PATH, IMS_DB_BAK_PATH)) {
            Serial.println("[DB] Save failed: backup old DB");
            SPIFFS.remove(IMS_DB_TMP_PATH);
            return false;
        }
        Serial.println("[DB] Backup old DB ok");
    }

    if (!SPIFFS.rename(IMS_DB_TMP_PATH, IMS_DB_MAIN_PATH)) {
        Serial.println("[DB] Save failed: rename");
        return false;
    }

    Serial.println("[DB] Rename tmp to main ok");
    Serial.printf("[DB] Save ok, substances=%d\n", IMS_DB_CountValidRecords());
    return true;
}

bool IMS_DB_LoadFromFile() {
    // 加载流程先尝试主文件，主文件损坏或不存在时再尝试备份文件。
    if (!IMS_DB_EnsureMounted()) {
        return false;
    }

    if (IMS_DB_LoadOneFile(IMS_DB_MAIN_PATH)) {
        return true;
    }

    Serial.println("[DB] Main DB invalid, trying backup");

    if (IMS_DB_LoadOneFile(IMS_DB_BAK_PATH)) {
        return true;
    }

    Serial.println("[DB] Backup DB invalid, start empty");
    return false;
}

bool IMS_DB_ClearFile() {
    // 清空 RAM 数据库，并删除 main/tmp/bak 文件。
    // 这用于调试或未来的“清空物质库”功能。
    IMS_DB_Clear();

    if (!IMS_DB_EnsureMounted()) {
        return false;
    }

    bool ok = true;
    if (SPIFFS.exists(IMS_DB_TMP_PATH)) ok = SPIFFS.remove(IMS_DB_TMP_PATH) && ok;
    if (SPIFFS.exists(IMS_DB_MAIN_PATH)) ok = SPIFFS.remove(IMS_DB_MAIN_PATH) && ok;
    if (SPIFFS.exists(IMS_DB_BAK_PATH)) ok = SPIFFS.remove(IMS_DB_BAK_PATH) && ok;

    if (!ok) {
        Serial.println("[DB] Clear file failed");
    }

    return ok;
}

bool IMS_DB_SaveToNVS() {
    // 兼容旧调用名：新版数据库实际保存到 SPIFFS，不再把大结构体 blob 放进 NVS。
    return IMS_DB_SaveToFile();
}

bool IMS_DB_LoadFromNVS() {
    return IMS_DB_LoadFromFile();
}

bool IMS_DB_ClearNVS() {
    return IMS_DB_ClearFile();
}
