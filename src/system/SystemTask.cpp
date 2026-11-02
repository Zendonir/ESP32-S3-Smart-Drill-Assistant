// SPDX-License-Identifier: MIT
#include "SystemTask.h"

#include <Arduino.h>
#include <driver/rtc_io.h>
#include <esp_heap_caps.h>
#include <esp_sleep.h>

#include "AppConfig.h"
#include "AppContext.h"
#include "BoardConfig.h"
#include "CpuMonitor.h"
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

struct State {
    AppContext* app = nullptr;
    uint32_t lastActivityMs = 0;
    uint32_t criticalSinceMs = 0;
    bool dimmed = false;
    bool off = false;
    uint8_t lastBatteryWarn = 0;
};
State g;

void handleButton(AppContext& app, uint32_t now) {
    const BootButton::Event ev = app.button.poll(now);
    if (ev == BootButton::Event::None) return;
    g.lastActivityMs = now;
    UiCommand c{};
    c.type = ev == BootButton::Event::Short ? UiCommand::Type::ButtonShort : UiCommand::Type::ButtonLong;
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
        LOGI(TAG, "inactive for %u min -> deep sleep", s.system.sleepAfterMin);
        enterDeepSleep(app);
    }
}

void handleBattery(AppContext& app, const sda::AppSettings& s, uint32_t now, PowerStatus& ps) {
    app.battery.setDivider(s.system.batteryDividerFactor);
    app.battery.update(0.5f);
    const sda::BatteryModel& m = app.battery.model();
    ps.batteryV = m.voltage();
    ps.batteryPercent = m.percentEstimate();
    ps.level = m.level();
    const bool critical = m.level() == sda::BatteryLevel::Critical;
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
    if (critical) {
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

void enterDeepSleep(AppContext& app) {
    // 1. persist pending changes (unless the battery is critical)
    app.settings.tick(millis(), true);
    app.profiles.tick(millis(), true);
    // 2. let the UI switch the display off (it owns the SPI bus)
    UiCommand c{};
    c.type = UiCommand::Type::PrepareSleep;
    app.postUi(c);
    vTaskDelay(pdMS_TO_TICKS(300));
    // 3. sensors to low power
    app.meas.suspend();
    app.touch.sleep();
    // 4. wake-up source: BOOT button (GPIO0, RTC capable, active low)
    rtc_gpio_pullup_en(static_cast<gpio_num_t>(board::kBootButton));
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(board::kBootButton), 0);
    LOGI(TAG, "entering deep sleep (wake: BOOT button)");
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.flush();
    esp_deep_sleep_start();
}

}  // namespace system_task
