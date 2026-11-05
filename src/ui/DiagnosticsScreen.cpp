// SPDX-License-Identifier: MIT
#include "DiagnosticsScreen.h"

#include <Arduino.h>
#include <esp_chip_info.h>

#include <cstdio>

#include "../system/OtaGuard.h"
#include "Theme.h"
#include "Version.h"

using namespace sda;

void DiagnosticsScreen::open(AppContext& app, std::function<void()> onBack, float* uiFps) {
    app_ = &app;
    onBack_ = std::move(onBack);
    fps_ = uiFps;
    lv_obj_t* old = scr_;
    scr_ = theme::screen();
    lv_obj_add_event_cb(scr_, gesture, LV_EVENT_GESTURE, this);
    lv_obj_t* t = theme::label(scr_, theme::medium(), theme::pal().accent, "Diagnose");
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 14);
    lv_obj_t* cont = lv_obj_create(scr_);
    lv_obj_remove_style_all(cont);
    lv_obj_set_size(cont, 186, 180);
    lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 42);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_flag(cont, LV_OBJ_FLAG_GESTURE_BUBBLE);
    text_ = theme::label(cont, theme::small(), theme::pal().fg, "");
    lv_obj_set_width(text_, 186);
    lv_label_set_long_mode(text_, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_pad_bottom(text_, 60, 0);
    lv_obj_t* hint = theme::label(scr_, theme::small(), theme::pal().dim, LV_SYMBOL_LEFT " wischen = zurück");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);
    update();
    lv_scr_load(scr_);
    if (old) lv_obj_del_async(old);
}

void DiagnosticsScreen::close() {
    if (!scr_) return;
    lv_obj_t* s = scr_;
    scr_ = nullptr;
    text_ = nullptr;
    lv_obj_del_async(s);
}

void DiagnosticsScreen::gesture(lv_event_t* e) {
    auto* self = static_cast<DiagnosticsScreen*>(lv_event_get_user_data(e));
    if (lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT && self->onBack_) {
        auto fn = self->onBack_;
        fn();
    }
}

void DiagnosticsScreen::update() {
    if (!scr_ || !text_) return;
    OrientationSnapshot s;
    app_->meas.snapshot(s);
    PowerStatus p;
    app_->power.read(p);
    NetStatus n;
    app_->net.read(n);
    const MeasurementService::Stats ms = app_->meas.stats();
    const I2CBus::Stats bs = app_->bus.stats();
    char buf[1800];
    int w = 0;
    auto add = [&](const char* fmt, auto... args) {
        if (w < int(sizeof(buf)) - 1) w += snprintf(buf + w, sizeof(buf) - size_t(w), fmt, args...);
    };
    add("FW %s (%s)\n", SDA_FW_VERSION_STR, ota_guard::runningPartition());
    add("%s\n", SDA_HW_NAME);
    add("Laufzeit %lu s\n", static_cast<unsigned long>(p.uptimeS));
    add("RAM frei %lu kB (min %lu)\n", static_cast<unsigned long>(p.freeHeap / 1024),
        static_cast<unsigned long>(p.minFreeHeap / 1024));
    add("PSRAM frei %lu kB\n", static_cast<unsigned long>(p.freePsram / 1024));
    add("CPU ca. %.0f%% / %.0f%% (Schätzung)\n", double(p.cpuLoad[0]), double(p.cpuLoad[1]));
    add("Fusion-Last %.1f%%  max %lu us\n", double(ms.fusionLoadPct), static_cast<unsigned long>(ms.maxProcessUs));
    add("UI %.0f FPS\n", fps_ ? double(*fps_) : 0.0);
    add("-- Sensor --\n");
    add("QMI8658 %s @0x%02X rev %02X\n", app_->imu.isHealthy() ? "ok" : "FEHLER", app_->imu.address(),
        app_->imu.revision());
    add("Rate %.1f Hz (nom. %.1f)\n", double(s.timing.estimatedRateHz), double(s.timing.nominalRateHz));
    add("Zähler %s\n", s.timing.counterMode ? "Sensor" : "Host (Fallback)");
    add("Samples %lu  verloren %lu\n", static_cast<unsigned long>(ms.samples),
        static_cast<unsigned long>(s.timing.lostSamples));
    add("max. Intervall %.1f ms\n", double(s.timing.maxIntervalMs));
    add("Queue-Überlauf %lu\n", static_cast<unsigned long>(ms.queueOverflows));
    add("I2C Fehler %lu  Timeouts %lu\n", static_cast<unsigned long>(bs.errors), static_cast<unsigned long>(bs.timeouts));
    add("I2C Recovery %lu\n", static_cast<unsigned long>(bs.recoveries));
    add("Clipping %lu  unplaus. %lu\n", static_cast<unsigned long>(s.clipCount),
        static_cast<unsigned long>(s.implausibleCount));
    add("Temp %.1f °C\n", double(s.temperatureC));
    add("a raw %.3f %.3f %.3f g\n", double(s.rawAccelG.x), double(s.rawAccelG.y), double(s.rawAccelG.z));
    add("g raw %.2f %.2f %.2f °/s\n", double(s.rawGyroDps.x), double(s.rawGyroDps.y), double(s.rawGyroDps.z));
    add("Online-Bias %.3f %.3f %.3f\n", double(s.onlineBiasDps.x), double(s.onlineBiasDps.y),
        double(s.onlineBiasDps.z));
    add("Bias-Unsich. %.3f °/s\n", double(s.biasSigmaDps));
    add("Gyro-Kal %s  TK %s\n", s.hasGyroCal ? "ja" : "NEIN", s.gyroTemp.compensated ? "aktiv" : "inaktiv");
    add("Accel-Kal %s  TK %s\n", s.hasAccelCal ? "ja" : "NEIN", s.accelTemp.compensated ? "aktiv" : "inaktiv");
    add("-- Fusion --\n");
    add("Filter %s  k %.4f\n", fusionTypeName(s.fusion), double(s.fusionGain));
    add("Accel-Vertrauen %.2f\n", double(s.accelTrust));
    add("Neig.-Unsich. ±%.3f° (Schätz.)\n", double(s.tiltSigmaDeg));
    add("Richt.-Unsich. ±%.2f°\n", double(s.headingSigmaDeg));
    add("Bewegung %s  %.1f s  Block 0x%X\n", motionStateName(s.motion), double(s.stillSec), s.stillBlockers);
    add("Werkzeug: %s (nur IMU)\n", drillActivityName(s.activity));
    add("Vibration %s %.0f mg\n", vibrationLevelName(s.vibration), double(s.vibrationRmsMg));
    add("Qualität %s\nFlags 0x%05lX\n", qualityStateName(s.quality.state), static_cast<unsigned long>(s.quality.flags));
    add("-- System --\n");
    add("Akku %.2f V (ca. %d%%)\n", double(p.batteryV), p.batteryPercent);
    add("ADC %.0f mV x %.2f\n", double(app_->battery.lastRawMilliVolts()), double(app_->settings.get().system.batteryDividerFactor));
    add("Touch 0x%02X fw %02X Fehler %lu\n", app_->touch.chipId(), app_->touch.firmware(),
        static_cast<unsigned long>(app_->touch.errors()));
    add("Touch x %d y %d %s\n", app_->display.touchX(), app_->display.touchY(),
        app_->display.touchPressed() ? "gedrückt" : "");
    add("WLAN %s %s\n", wifiModeName(n.mode), n.connected ? n.ip : "-");
    add("Flash-Schreiben %s\n", p.flashWritesAllowed ? "erlaubt" : "GESPERRT");
    add("Settings-Writes %lu\n", static_cast<unsigned long>(app_->settings.writeCount()));
    add("Stack frei imu %lu fus %lu\n", static_cast<unsigned long>(ms.imuStackFree),
        static_cast<unsigned long>(ms.fusionStackFree));
    lv_label_set_text(text_, buf);
}
