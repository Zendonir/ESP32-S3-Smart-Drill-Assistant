// SPDX-License-Identifier: MIT
// Persistent configuration data model with validation. Serialization (JSON in
// NVS / LittleFS) lives in src/storage; this header is hardware independent so
// that validation can be unit tested.
//
// Separation (requirement 4.5 / 11):
//   * AppSettings      - user preferences and system configuration
//   * SensorCalibration (SensorCalibration.h) - base calibration of the sensor
//   * Profile          - tool specific mounting + measurement defaults
#pragma once

#include <cstddef>
#include <cstdint>

#include "CoordinateTransforms.h"
#include "DisplayLogic.h"
#include "SensorFusion.h"

namespace sda {

constexpr uint16_t kSettingsVersion = 1;
constexpr uint16_t kProfilesVersion = 1;
constexpr uint16_t kCalibrationVersion = 1;
constexpr int kMaxProfiles = 8;
constexpr int kProfileNameLen = 20;

enum class AppMode : uint8_t { Drill = 0, Level, Angle, Zero, Reference, Precision, Count };
const char* appModeName(AppMode m);
const char* appModeShort(AppMode m);

enum class ButtonAction : uint8_t { Hold = 0, Zero, NextMode, TouchLock, Count };
const char* buttonActionName(ButtonAction a);

enum class WifiMode : uint8_t { Off = 0, AccessPoint = 1, Client = 2 };
const char* wifiModeName(WifiMode m);

enum class StartupGyroCal : uint8_t { Never = 0, IfStill = 1, Always = 2 };

enum class ZeroKind : uint8_t { Tilt = 0, Full3D = 1 };

struct MeasurementSettings {
    uint8_t filterStrength = 2;   ///< 0 (fast) .. 4 (very smooth)
    FusionType fusion = FusionType::Mahony;
    float precisionWindowSec = 3.0f;
    float holdTimeSec = 0.4f;
    int8_t hemisphere = 0;        ///< -1 down, +1 up, 0 auto
    ZeroKind zeroKind = ZeroKind::Tilt;
    bool reference3D = false;     ///< reference surface mode: use gyro heading
    float customAngleDeg = 22.5f; ///< user defined quick select angle
    float tempCoeffAssumedDpsPerK = 0.02f;  ///< ASSUMED gyro bias tempco for uncertainty model
    bool onlineBiasTracking = true;
    StartupGyroCal startupGyroCal = StartupGyroCal::IfStill;
    AppMode lastMode = AppMode::Drill;
};

struct DisplaySettings {
    uint8_t brightness = 80;     ///< 5..100 %
    uint16_t dimAfterSec = 90;   ///< 0 = never
    uint16_t offAfterSec = 600;  ///< 0 = never
    uint8_t rotation = 0;        ///< 0..3 (x 90 deg)
    uint8_t theme = 0;           ///< 0 dark, 1 high contrast
    uint8_t colorMode = 0;       ///< 0 standard, 1 colour-blind friendly (blue/orange)
    uint8_t largeFont = 0;       ///< 0 normal, 1 large numbers
    uint8_t decimals = 1;        ///< 1 or 2 decimal places (2 only meaningful in precision)
    bool touchLockAuto = false;  ///< lock touch while vibration is detected
    bool showHistory = true;
    uint8_t historyWindowSec = 15; ///< 5, 15, 30
};

struct SystemSettings {
    WifiMode wifiMode = WifiMode::Off;
    char ssid[33] = "";
    char password[65] = "";
    char apPassword[17] = "";     ///< generated on first boot if empty
    char adminPassword[17] = "";  ///< generated on first boot if empty (web API auth)
    char hostname[25] = "smartdrill";
    bool mdns = true;
    uint16_t sleepAfterMin = 30;  ///< deep sleep after inactivity, 0 = never
    ButtonAction bootButton = ButtonAction::Hold;
    AxisMap axisMap;              ///< sensor -> device axis mapping
    uint8_t logLevel = 2;         ///< 0 error .. 3 debug
    float batteryDividerFactor = 3.0f;
    float lowBatteryV = 3.50f;
    float criticalBatteryV = 3.35f;
    uint8_t activeProfile = 0;
    uint8_t liveRateHz = 10;      ///< websocket live data rate
};

struct AppSettings {
    uint16_t version = kSettingsVersion;
    MeasurementSettings measurement;
    DisplaySettings display;
    SystemSettings system;
};

struct Profile {
    bool used = false;
    char name[kProfileNameLen + 1] = "";
    MountModel mount;
    float targetAngleDeg = 90.0f;   ///< inclination target (0 horizontal .. 90 vertical)
    float toleranceDeg = 0.5f;
    ScaleMode scale = ScaleMode::Auto;
    bool invertMarker = false;      ///< false: marker = axis position; true: bubble style
    bool showNumbers = true;
    uint32_t mountCalTime = 0;      ///< seconds since boot counter epoch (informative)
};

struct ProfileSet {
    uint16_t version = kProfilesVersion;
    Profile profiles[kMaxProfiles];
};

/// Restores defaults (5 standard profiles from requirement 4.5).
void defaultSettings(AppSettings& s);
void defaultProfiles(ProfileSet& p);

/// Validates and repairs. Returns the number of corrected fields (0 = was valid).
int validateSettings(AppSettings& s);
int validateProfile(Profile& p);
int validateProfiles(ProfileSet& p, uint8_t& activeProfile);

/// Quick select angles (requirement 5.3): 0, 15, 30, 45, 60, 90, custom.
constexpr int kQuickAngleCount = 7;
float quickAngle(int idx, const MeasurementSettings& m);

/// Mapping of the filter strength (0..4) to filter parameters.
struct FilterParams {
    float displayMinCutoffHz;
    float displayBeta;
    float fusionMinTauSec;   ///< limits the accel correction gain at rest
};
FilterParams filterParams(uint8_t strength);

/// Safe bounded string copy (always terminates).
void copyString(char* dst, size_t dstSize, const char* src);

}  // namespace sda
