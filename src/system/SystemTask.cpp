// SPDX-License-Identifier: MIT
#include "SystemTask.h"

#include <Arduino.h>
#include <HWCDC.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_heap_caps.h>
#include <esp_sleep.h>

#include "AppConfig.h"
#include "AppContext.h"
#include "BoardConfig.h"
#include "CpuMonitor.h"
#include "LoadEvents.h"
#include "Log.h"
#include "OtaGuard.h"
#include "WriteGuard.h"

static const char* TAG = "sys";

sda::PipelineConfig AppContext::pipelineConfig(const sda::AppSettings& s) const {
    sda::PipelineConfig pc;
    pc.axisMap = s.system.axisMap;
    pc.accelRangeG = imu.info().accelRangeG > 0 ? imu.info().accelRangeG : 8.0f;
    pc.gyroRangeDps = imu.info().gyroRangeDps > 0 ? imu.info().gyroRangeDps : 512.0f;
    pc.nominalRateHz = imu.info().nominalRateHz > 0 ? imu.info().nominalRateHz : appcfg::kImuNominalRateHz;
    pc.fusion = s.measurement.fusion;
    pc.fusionMinTauSec = sda::filterParams(s.measurement.filterStrength).fusionMinTauSec;
    pc.tempCoeffAssumedDpsPerK = s.measurement.tempCoeffAssumedDpsPerK;
    pc.onlineBiasTracking = s.measurement.onlineBiasTracking;
    return pc;
}

namespace system_task {
namespace {
constexpr uint64_t kChargeCheckIntervalUs = 30ULL * 1000000ULL;
// survives deep sleep
RTC_DATA_ATTR bool g_rtcChargeWake = false;
RTC_DATA_ATTR float g_rtcDivider = 3.0f;
RTC_DATA_ATTR float g_rtcLastV = 0.0f;
RTC_DATA_ATTR uint32_t g_rtcChecks = 0;

struct State {
    AppContext* app = nullptr;
    uint32_t loadEvents = 0;
    uint32_t unplugSleepAtMs = 0;
    uint32_t lastActivityMs = 0;
    uint32_t criticalSinceMs = 0;
    bool dimmed = false;
    bool off = false;
    uint8_t lastBatteryWarn = 0;
};
State g;

void handleButton(AppContext& app, uint32_t now) {
    using E = BootButton::Event;
    static uint32_t lastProgressMs = 0;
    const E ev = app.button.poll(now);
    UiCommand c{};
    // power-off countdown: progress updates while the button is held
    if (app.button.powerOffProgress(now) > 0.0f && now - lastProgressMs >= 100) {
        lastProgressMs = now;
        c.type = UiCommand::Type::PowerHold;
        c.f = app.button.powerOffProgress(now);
        app.postUi(c);
    }
    if (ev == E::None) return;
    g.lastActivityMs = now;
    switch (ev) {
        case E::Short: c.type = UiCommand::Type::ButtonShort; break;
        case E::Double: c.type = UiCommand::Type::ButtonDouble; break;
        case E::HoldStart: c.type = UiCommand::Type::PowerHold; c.f = 0.0f; break;
        case E::HoldCancel: c.type = UiCommand::Type::PowerHoldCancel; break;
        case E::PowerOff:
            LOGI(TAG, "BOOT held -> power off");
            requestPowerOff(app);
            return;
        default: return;
    }
    app.postUi(c);
}

void handleDisplayPower(AppContext& app, const sda::AppSettings& s, uint32_t now) {
    // activity: touch interrupts, motion of the device, button (above)
    if (app.touch.consumeActivity()) g.lastActivityMs = now;
    sda::OrientationSnapshot snap;
    app.meas.snapshot(snap);
    if (snap.valid && (snap.gyroRateDps > 3.0f || snap.vibration != sda::VibrationLevel::None))
        g.lastActivityMs = now;
    const uint32_t idle = now - g.lastActivityMs;

    const uint32_t dimMs = uint32_t(s.display.dimAfterSec) * 1000u;
    const uint32_t offMs = uint32_t(s.display.offAfterSec) * 1000u;
    UiCommand c{};
    if (offMs && idle >= offMs) {
        if (!g.off) {
            c.type = UiCommand::Type::DisplayOff;
            app.postUi(c);
            g.off = true;
        }
    } else if (dimMs && idle >= dimMs) {
        if (!g.dimmed || g.off) {
            c.type = UiCommand::Type::DisplayDim;
            app.postUi(c);
            g.dimmed = true;
            g.off = false;
        }
    } else if (g.dimmed || g.off) {
        c.type = UiCommand::Type::Wake;
        app.postUi(c);
        g.dimmed = g.off = false;
    }

    // automatic deep sleep after long inactivity (not while network clients may need us)
    const uint32_t sleepMs = uint32_t(s.system.sleepAfterMin) * 60000u;
    if (sleepMs && idle >= sleepMs && !app.networkActive) {
        LOGI(TAG, "inactive for %u min -> power off", s.system.sleepAfterMin);
        requestPowerOff(app);
        g.lastActivityMs = now;
    }
}

void handleBattery(AppContext& app, const sda::AppSettings& s, uint32_t now, PowerStatus& ps) {
    app.battery.setDivider(s.system.batteryDividerFactor);
    app.battery.setUsbHost(HWCDC::isConnected());
    const uint32_t le = load_events::value();
    if (le != g.loadEvents) {
        g.loadEvents = le;
        app.battery.model().notifyLoadChange();
    }
    app.battery.update(0.5f);
    sda::BatteryModel& m = app.battery.model();
    // charger plugged / unplugged -> charge screen on / off
    if (m.takePlugEvent()) {
        LOGI(TAG, "charger connected (voltage step)");
        g.lastActivityMs = now;
        if (s.system.chargeScreen >= 1) {
            UiCommand cs{};
            cs.type = UiCommand::Type::ChargeScreen;
            cs.i = 1;
            app.postUi(cs);
        }
    }
    if (m.takeUnplugEvent()) {
        LOGI(TAG, "charger disconnected");
        g.lastActivityMs = now;
        UiCommand cs{};
        cs.type = UiCommand::Type::ChargeScreen;
        cs.i = 0;
        app.postUi(cs);
        // switched "off" while charging: go back to sleep after the message
        if (app.chargeMode) g.unplugSleepAtMs = now + 3000;
    }
    if (g.unplugSleepAtMs && now >= g.unplugSleepAtMs) {
        g.unplugSleepAtMs = 0;
        if (app.chargeMode && !m.chargerConnected()) enterDeepSleep(app);
    }
    ps.batteryV = m.voltage();
    ps.batteryPercent = m.percentEstimate();
    ps.level = m.level();
    const sda::ChargeState oldCharge = ps.charge;
    ps.charge = m.chargeState();
    ps.trendMvPerMin = m.trendMvPerMin();
    ps.usbHost = m.usbHost();
    if (ps.charge != oldCharge && (ps.charge == sda::ChargeState::Charging || ps.charge == sda::ChargeState::Full ||
                                   oldCharge == sda::ChargeState::Charging)) {
        UiCommand cc{};
        cc.type = UiCommand::Type::ChargeChanged;
        cc.i = static_cast<int32_t>(ps.charge);
        app.postUi(cc);
    }
    const bool critical = m.level() == sda::BatteryLevel::Critical;
    const bool charging = m.chargeState() == sda::ChargeState::Charging;
    write_guard::set(!critical);
    ps.flashWritesAllowed = !critical;
    const uint8_t warn = critical ? 2 : (m.level() == sda::BatteryLevel::Low ? 1 : 0);
    if (warn != g.lastBatteryWarn) {
        g.lastBatteryWarn = warn;
        if (warn) {
            UiCommand c{};
            c.type = UiCommand::Type::BatteryWarning;
            c.i = warn;
            app.postUi(c);
            LOGW(TAG, "battery %s: %.2f V", warn == 2 ? "CRITICAL" : "low", ps.batteryV);
        }
    }
    // shut down after 30 s of critical voltage to protect the cell and the flash
    if (critical && !charging) {  // never shut down while a charger is connected
        if (!g.criticalSinceMs) g.criticalSinceMs = now;
        if (now - g.criticalSinceMs > 30000) {
            LOGE(TAG, "battery critical for 30 s -> deep sleep");
            enterDeepSleep(app);
        }
    } else {
        g.criticalSinceMs = 0;
    }
}

void taskMain(void*) {
    AppContext& app = *g.app;
    g.lastActivityMs = millis();
    uint32_t lastSlow = 0, lastSec = 0;
    PowerStatus ps;
    for (;;) {
        const uint32_t now = millis();
        handleButton(app, now);
        if (now - lastSlow >= 500) {
            lastSlow = now;
            const sda::AppSettings s = app.settings.get();
            handleBattery(app, s, now, ps);
            handleDisplayPower(app, s, now);
            app.settings.tick(now);
            app.profiles.tick(now);
            ps.idleMs = now - g.lastActivityMs;
            ps.freeHeap = esp_get_free_heap_size();
            ps.minFreeHeap = esp_get_minimum_free_heap_size();
            ps.freePsram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            ps.uptimeS = now / 1000;
            app.power.write(ps);
        }
        if (now - lastSec >= 1000) {
            lastSec = now;
            cpu_monitor::update();
            ps.cpuLoad[0] = cpu_monitor::loadPercent(0);
            ps.cpuLoad[1] = cpu_monitor::loadPercent(1);
            ota_guard::tick(now, app.boot.imu && app.boot.display);
            if (sda_log::level() != static_cast<sda_log::Level>(app.settings.get().system.logLevel))
                sda_log::setLevel(static_cast<sda_log::Level>(app.settings.get().system.logLevel));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
}  // namespace

void start(AppContext& app) {
    g.app = &app;
    app.button.begin();
    cpu_monitor::begin();
    xTaskCreatePinnedToCore(taskMain, "system", appcfg::kSystemTaskStack, nullptr, appcfg::kSystemTaskPrio, nullptr,
                            appcfg::kSystemTaskCore);
}

void requestPowerOff(AppContext& app) {
    const sda::AppSettings s = app.settings.get();
    if (s.system.chargeScreen >= 1 && app.battery.model().chargerConnected()) {
        // like a phone: "off" while charging shows the charge screen instead
        LOGI(TAG, "power off while charging -> charge screen");
        app.chargeMode = true;
        UiCommand c{};
        c.type = UiCommand::Type::ChargeScreen;
        c.i = 2;
        app.postUi(c);
        return;
    }
    enterDeepSleep(app);
}

void enterDeepSleep(AppContext& app) {
    // remember the sleep settings for the wake-up check (RTC memory)
    const sda::AppSettings st = app.settings.get();
    setChargeWakeConfig(st.system.chargeScreen >= 2, st.system.batteryDividerFactor,
                        app.battery.model().voltage());
    // 1. persist pending changes (unless the battery is critical)
    app.settings.tick(millis(), true);
    app.profiles.tick(millis(), true);
    // 2. let the UI show the "off" screen and switch the display off (it owns the SPI bus)
    UiCommand c{};
    c.type = UiCommand::Type::PrepareSleep;
    app.postUi(c);
    vTaskDelay(pdMS_TO_TICKS(1200));
    // 3. sensors to low power
    app.meas.suspend();
    app.touch.sleep();
    LOGI(TAG, "entering deep sleep (wake: BOOT button)");
    vTaskDelay(pdMS_TO_TICKS(50));
    Serial.flush();
    sleepNow();
}

void releaseSleepHolds() {
    // undo the pin holds of a previous deep sleep (backlight would stay off otherwise)
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(static_cast<gpio_num_t>(board::kLcdBacklight));
}

void sleepNow() {
    // the button must be released first: ext0 wakes on LOW level and would
    // otherwise restart the device immediately
    pinMode(board::kBootButton, INPUT_PULLUP);
    const uint32_t t0 = millis();
    while (digitalRead(board::kBootButton) == LOW && millis() - t0 < 15000) delay(10);
    delay(50);
    // keep the backlight driver switched off during sleep (pin would float)
    pinMode(board::kLcdBacklight, OUTPUT);
    digitalWrite(board::kLcdBacklight, LOW);
    gpio_hold_en(static_cast<gpio_num_t>(board::kLcdBacklight));
    gpio_deep_sleep_hold_en();
    // wake-up source: BOOT button (GPIO0, RTC capable, active low)
    rtc_gpio_pullup_en(static_cast<gpio_num_t>(board::kBootButton));
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(board::kBootButton), 0);
    // periodic short wake-ups to detect a connected charger (setting)
    if (g_rtcChargeWake) esp_sleep_enable_timer_wakeup(kChargeCheckIntervalUs);
    esp_deep_sleep_start();
}

void setChargeWakeConfig(bool enabled, float dividerFactor, float voltageNow) {
    g_rtcChargeWake = enabled;
    g_rtcDivider = dividerFactor;
    g_rtcLastV = voltageNow;
    g_rtcChecks = 0;
}

bool chargeWakeCheck() {
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER || !g_rtcChargeWake) return false;
    const float v = BatteryManager::measureOnce(g_rtcDivider);
    ++g_rtcChecks;
    // The first checks after switching off only follow the voltage recovery of the
    // cell (it rises for a while after the load is removed). Later a jump of >= 60 mV
    // between two checks, or a voltage above the cell maximum, means: charger connected.
    const bool charger = (g_rtcChecks > 2 && v - g_rtcLastV >= 0.060f) || v > 4.30f;
    g_rtcLastV = v;
    return charger;
}

}  // namespace system_task
