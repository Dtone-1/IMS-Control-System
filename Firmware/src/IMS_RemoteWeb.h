#pragma once

#include <Arduino.h>

typedef void (*ims_remote_cb_t)();

class IMS_RemoteWeb {
public:
  bool beginAP(const char* ssid, const char* pass);

  void onStart(ims_remote_cb_t cb) { cb_start_ = cb; }
  void onPause(ims_remote_cb_t cb) { cb_pause_ = cb; }
  void onSave(ims_remote_cb_t cb)  { cb_save_  = cb; }

  void pushStatus(float peakTimeMs, float peakAmp, bool scanning);
  size_t clientCount() const;

  void pushWaveform(const float* yV, int n,
                    float peakTimeMs, float peakAmp,
                    bool scanning,
                    const char* idName = nullptr,
                    int idStatus = 0,
                    float featureQuality = 0.0f,
                    uint32_t frameId = 0,
                    float xRangeMs = 24.0f,
                    float fwhmMs = 0.0f,
                    float mainPeakAreaVMs = 0.0f,
                    float totalFrameResponseVMs = 0.0f,
                    bool hasSecondPeak = false,
                    float mainSecondHeightRatio = 0.0f,
                    float mainSecondTimeDiffMs = 0.0f);

private:
  ims_remote_cb_t cb_start_ = nullptr;
  ims_remote_cb_t cb_pause_ = nullptr;
  ims_remote_cb_t cb_save_  = nullptr;

  void setupRoutes_();
};
