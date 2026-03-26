#pragma once
#include <Arduino.h>

// 用函数指针做回调：简单、稳定、占用小
typedef void (*ims_remote_cb_t)();

class IMS_RemoteWeb {
public:
  bool beginAP(const char* ssid, const char* pass);
  void onStart(ims_remote_cb_t cb) { cb_start_ = cb; }
  void onPause(ims_remote_cb_t cb) { cb_pause_ = cb; }
  void onSave(ims_remote_cb_t cb)  { cb_save_  = cb; }

  // ===== WebSocket 推送接口（给网页端同步状态与谱图）=====
  // 只推状态（扫描状态 + 峰值）
  void pushStatus(float peakTimeMs, float peakAmp, bool scanning);

  // 推送谱图 + 状态（y 为 int16 数组，n=点数，建议 200 点）
  void pushWaveform(const int16_t* y, int n,
                    float peakTimeMs, float peakAmp,
                    bool scanning);

private:
  ims_remote_cb_t cb_start_ = nullptr;
  ims_remote_cb_t cb_pause_ = nullptr;
  ims_remote_cb_t cb_save_  = nullptr;

  void setupRoutes_();
};
