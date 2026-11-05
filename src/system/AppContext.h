// SPDX-License-Identifier: MIT
// All long lived components of the firmware. Exactly one instance exists
// (created in main.cpp); modules receive a reference instead of using globals.
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "../hardware/BatteryManager.h"
#include "../hardware/BootButton.h"
#include "../hardware/DisplayDriver.h"
#include "../hardware/I2CBus.h"
#include "../hardware/IMUDriver.h"
#include "../hardware/TouchDriver.h"
#include "../measurement/MeasurementService.h"
#include "../storage/CalibrationStorage.h"
#include "../storage/ProfileManager.h"
#include "../storage/SettingsManager.h"
#include "AppTypes.h"
#include "SharedState.h"

struct PowerStatus {
    float batteryV = 0;
    int batteryPercent = -1;     ///< estimate, -1 unknown
    sda::BatteryLevel level = sda::BatteryLevel::Unknown;
    sda::ChargeState charge = sda::ChargeState::Unknown;
    float trendMvPerMin = 0;
    bool usbHost = false;
    bool flashWritesAllowed = true;
    uint32_t idleMs = 0;
    float cpuLoad[2] = {-1, -1};
    uint32_t freeHeap = 0;
    uint32_t minFreeHeap = 0;
    uint32_t freePsram = 0;
    uint32_t uptimeS = 0;
};

struct NetStatus {
    sda::WifiMode mode = sda::WifiMode::Off;
    bool connected = false;      ///< client: associated + IP; AP: running
    bool apFallback = false;     ///< client failed -> AP started
    char ip[16] = "";
    char ssid[33] = "";
    int8_t rssi = 0;
    uint8_t wsClients = 0;
    uint32_t httpRequests = 0;
    uint32_t authFailures = 0;
    bool otaInProgress = false;
    uint8_t otaProgress = 0;
};

struct AppContext {
    AppContext() : imu(bus), touch(bus) {}

    I2CBus bus;
    QMI8658Driver imu;
    TouchDriver touch;
    DisplayDriver display;
    BatteryManager battery;  // only used by the system task (and once in setup)
    BootButton button;

    SettingsManager settings;
    ProfileManager profiles;
    CalibrationStorage calStore;
    MeasurementService meas;

    SharedState<UiStatus> uiStatus;
    SharedState<PowerStatus> power;
    SharedState<NetStatus> net;
    QueueHandle_t uiQueue = nullptr;   ///< UiCommand
    BootReport boot;
    volatile bool networkActive = false;
    volatile bool startupGyroPending = false;  ///< automatic gyro calibration after boot
    volatile bool chargeMode = false;          ///< "switched off" but showing the charge screen

    bool postUi(const UiCommand& c) const {
        return uiQueue && xQueueSend(uiQueue, &c, pdMS_TO_TICKS(5)) == pdTRUE;
    }
    /// Pipeline configuration derived from the current settings.
    sda::PipelineConfig pipelineConfig(const sda::AppSettings& s) const;
};
