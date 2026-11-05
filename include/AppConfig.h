// SPDX-License-Identifier: MIT
// Application wide compile time configuration (task layout, rates, limits).
#pragma once

#include <cstdint>

namespace appcfg {

// ---- FreeRTOS task layout ---------------------------------------------------
// Core 1: time critical measurement + UI. Core 0: WiFi stack, network, system.
constexpr int kImuTaskCore = 1;
constexpr int kImuTaskPrio = 20;
constexpr uint32_t kImuTaskStack = 4096;

constexpr int kFusionTaskCore = 1;
constexpr int kFusionTaskPrio = 19;
constexpr uint32_t kFusionTaskStack = 8192;

constexpr int kUiTaskCore = 1;
constexpr int kUiTaskPrio = 5;
constexpr uint32_t kUiTaskStack = 12288;

constexpr int kNetTaskCore = 0;
constexpr int kNetTaskPrio = 3;
constexpr uint32_t kNetTaskStack = 8192;

constexpr int kSystemTaskCore = 0;
constexpr int kSystemTaskPrio = 2;
constexpr uint32_t kSystemTaskStack = 6144;

constexpr int kLogTaskCore = 0;
constexpr int kLogTaskPrio = 1;
constexpr uint32_t kLogTaskStack = 3072;

// ---- Measurement --------------------------------------------------------------
// QMI8658 ODR code 5 = "250 Hz" in accel-only mode; in 6-DOF mode the output rate
// is derived from the gyro clock (nominal 224.2 Hz, datasheet). The real rate is
// measured at runtime (TimingEstimator) and shown in the diagnostics.
constexpr float kImuNominalRateHz = 224.2f;
constexpr uint32_t kImuPollIntervalMs = 1;     // poll period while waiting for data
constexpr uint32_t kImuIdleAfterSampleMs = 3;  // sleep after a sample (period 4.46 ms)
constexpr uint32_t kImuDataTimeoutMs = 100;    // no data -> sensor fault
constexpr uint32_t kImuQueueLength = 32;

// ---- UI -----------------------------------------------------------------------
constexpr uint32_t kUiTargetFps = 40;          // while something moves / is touched
constexpr uint32_t kUiIdleFps = 15;            // nothing changes (still, hold, menus idle)
constexpr uint8_t kDimBrightnessPct = 10;      // backlight level when dimmed
// power saving while the display is off
constexpr uint32_t kCpuMhzIdle = 80;
constexpr uint32_t kCpuMhzEconomy = 160;
constexpr uint32_t kCpuMhzMax = 240;
constexpr uint8_t kImuOdrCodeNormal = 5;       // 224.2 Hz (6DOF)
constexpr uint8_t kImuOdrCodeLowPower = 8;     // 28.0 Hz (6DOF)
constexpr float kImuRateNormalHz = 224.2f;
constexpr float kImuRateLowPowerHz = 28.025f;
constexpr uint32_t kLvglBufferLines = 40;      // two DMA buffers of 240 x 40 px
constexpr uint32_t kLongPressMs = 800;
constexpr uint32_t kDoubleTapMs = 350;

// ---- Storage ------------------------------------------------------------------
constexpr uint32_t kSettingsSaveDelayMs = 4000; // debounce flash writes

// ---- Network ------------------------------------------------------------------
constexpr uint16_t kHttpPort = 80;
constexpr uint16_t kWebSocketPort = 81;
constexpr uint8_t kMaxWsClients = 3;
constexpr uint32_t kWifiReconnectMs = 10000;
constexpr uint32_t kClientFallbackToApMs = 30000;  // client connect fails -> AP

}  // namespace appcfg
