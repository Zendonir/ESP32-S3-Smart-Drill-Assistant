// SPDX-License-Identifier: MIT
// Smart Drill Assistant - firmware entry point.
//
// Start-up sequence (requirement 6.2): logo, hardware initialisation with
// self test (I2C, QMI8658, touch, display), load calibration + settings + last
// profile, optional start-up gyro calibration, then the main view. All long
// running work happens in dedicated FreeRTOS tasks:
//
//   core 1: imu (20), fusion (19), ui (5)
//   core 0: WiFi stack, network (3), system (2), log (1)
#include <Arduino.h>
#include <esp_system.h>
#include <lvgl.h>

#include "AppConfig.h"
#include "BoardConfig.h"
#include "Version.h"
#include "network/NetworkManager.h"
#include "system/AppContext.h"
#include "system/Log.h"
#include "system/OtaGuard.h"
#include "system/SystemTask.h"
#include "ui/UIManager.h"

static const char* TAG = "main";

namespace {
// The single application object (all components, no scattered globals).
AppContext g_app;
UIManager g_ui;
NetworkManager g_net(g_app);
}  // namespace

void setup() {
    AppContext& app = g_app;
    sda_log::begin(sda_log::Level::Info);
    sda_log::startTask(appcfg::kLogTaskCore, appcfg::kLogTaskPrio, appcfg::kLogTaskStack);
    app.boot.resetReason = static_cast<int>(esp_reset_reason());
    LOGI(TAG, "%s %s (%s) on %s, reset reason %d", SDA_FW_NAME, SDA_FW_VERSION_STR, SDA_FW_BUILD_DATE, SDA_HW_NAME,
         app.boot.resetReason);
    app.boot.otaPendingVerify = ota_guard::pendingVerify();

    app.uiQueue = xQueueCreate(16, sizeof(UiCommand));

    // settings first: display rotation/brightness, axis mapping, log level
    app.settings.begin();
    sda::AppSettings s = app.settings.get();
    app.boot.settingsLoaded = app.settings.loadedFromFlash();
    sda_log::setLevel(static_cast<sda_log::Level>(s.system.logLevel));

    // display + LVGL (boot screen)
    app.display.setBrightness(s.display.brightness);
    app.boot.display = app.display.begin(s.display.rotation);
    app.boot.i2c = app.bus.begin(board::kI2cPort, board::kI2cSda, board::kI2cScl, board::kI2cFreqHz);
    app.boot.touch = app.boot.i2c && app.touch.begin();
    app.boot.lvgl = app.boot.display && app.display.initLvgl(&app.touch);
    if (app.boot.lvgl) g_ui.bootBegin(app);
    g_ui.bootStep("Display", app.boot.display);
    g_ui.bootStep(app.boot.touch ? "Touch CST816S" : "Touch fehlt", app.boot.touch);

    // IMU
    app.boot.imu = app.boot.i2c && app.imu.begin();
    app.boot.imuAddress = app.imu.address();
    g_ui.bootStep(app.boot.imu ? "Sensor QMI8658" : "QMI8658 FEHLER", app.boot.imu);

    // storage: profiles (LittleFS) and calibration (NVS)
    uint8_t active = s.system.activeProfile;
    app.profiles.begin(active);
    if (active != s.system.activeProfile) app.settings.update([active](sda::AppSettings& x) { x.system.activeProfile = active; });
    app.boot.fs = app.profiles.fsOk();
    sda::SensorCalibration cal;
    app.boot.calibrationLoaded = app.calStore.load(cal);
    g_ui.bootStep(app.boot.calibrationLoaded ? "Kalibrierung" : "Unkalibriert", app.boot.calibrationLoaded);
    const sda::Profile prof = app.profiles.get(active);
    g_ui.bootStep(prof.name, true);

    // battery before the measurement tasks (first reading for the warning logic)
    app.battery.begin(s.system.batteryDividerFactor, s.system.lowBatteryV, s.system.criticalBatteryV);

    // measurement tasks
    s = app.settings.get();
    app.meas.begin(&app.imu, &app.bus, app.pipelineConfig(s), cal, prof.mount);
    // automatic start-up gyro calibration: runs only while the device is still;
    // the result is used in RAM (not written to flash -> no wear on every boot)
    // (stored to flash only if no gyro calibration exists yet, see UIManager).
    if (app.boot.imu && (s.measurement.startupGyroCal != sda::StartupGyroCal::Never || !cal.hasGyro())) {
        app.meas.startGyroCalibration();
        app.startupGyroPending = true;
    }

    system_task::start(app);
    g_net.start();
    g_ui.start();  // from here on LVGL belongs to the UI task
    LOGI(TAG, "start-up complete, free heap %u", static_cast<unsigned>(esp_get_free_heap_size()));
}

void loop() {
    // All work is done in tasks; the Arduino loop task is not needed.
    vTaskDelete(nullptr);
}
