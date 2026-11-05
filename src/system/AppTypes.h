// SPDX-License-Identifier: MIT
// Small POD types exchanged between UI, network and system tasks.
#pragma once

#include <cstdint>

#include "Settings.h"
#include "DisplayLogic.h"

/// Published by the UI task (read by the web interface).
struct UiStatus {
    sda::AppMode mode = sda::AppMode::Drill;
    bool hold = false;
    bool touchLocked = false;
    bool displayOn = true;
    float targetAngleDeg = 90.0f;
    float toleranceDeg = 0.5f;
    sda::TargetState targetState = sda::TargetState::Invalid;
    float devXDeg = 0, devYDeg = 0, devTotalDeg = 0;
    float valueDeg = 0;         ///< main value of the current mode (e.g. inclination)
    float scaleDeg = 10;
    uint8_t profile = 0;
    float fps = 0;
    bool lateralFree = false;
    bool lateralFromGyro = false;
    uint8_t savedCount = 0;     ///< values stored with "save" while holding
    float lastSavedDeg = 0;
};

/// Commands posted to the UI task (thread safe via FreeRTOS queue).
struct UiCommand {
    enum class Type : uint8_t {
        SetMode, ToggleHold, Zero, ReloadProfile, ReloadSettings, Wake, DisplayDim, DisplayOff,
        ButtonShort, ButtonDouble, PowerHold, PowerHoldCancel, ShowMessage, BatteryWarning, SetTarget,
        PrepareSleep, ChargeChanged, ChargeScreen,
    } type;
    int32_t i = 0;
    float f = 0;
    char text[48] = "";
};

/// Results of the start-up self test.
struct BootReport {
    bool i2c = false;
    bool imu = false;
    bool touch = false;
    bool display = false;
    bool lvgl = false;
    bool settingsLoaded = false;
    bool calibrationLoaded = false;
    bool fs = false;
    bool otaPendingVerify = false;
    bool chargeBoot = false;   ///< woke from "off" because a charger was detected
    uint8_t imuAddress = 0;
    uint8_t touchChip = 0;
    int resetReason = 0;
};
