// SPDX-License-Identifier: MIT
#include "UIManager.h"

#include <Arduino.h>

#include <cstdio>
#include <cstring>

#include "../system/Log.h"
#include "../system/SystemTask.h"
#include "AppConfig.h"
#include "Theme.h"
#include "Version.h"

using namespace sda;

static const char* TAG = "ui";

namespace {
constexpr uint32_t kGestureTapBlockMs = 350;
uint32_t g_lastGestureMs = 0;
uint32_t g_pressStartMs = 0;
bool g_unlockFired = false;

const float kTolerances[] = {0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f};
constexpr int kTolCount = sizeof(kTolerances) / sizeof(kTolerances[0]);
}  // namespace

// ===========================================================================
// boot
// ===========================================================================
void UIManager::bootBegin(AppContext& app) {
    app_ = &app;
    settings_ = app.settings.get();
    theme::apply(settings_.display.theme, settings_.display.colorMode);
    const theme::Palette& p = theme::pal();
    bootScr_ = theme::screen();
    lv_obj_t* t = theme::label(bootScr_, theme::large(), p.accent, "SMART DRILL");
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_t* v = theme::label(bootScr_, theme::small(), p.dim, "Assistant v" SDA_FW_VERSION_STR);
    lv_obj_align(v, LV_ALIGN_TOP_MID, 0, 70);
    bootList_ = theme::label(bootScr_, theme::small(), p.fg, "");
    lv_obj_set_width(bootList_, 160);
    lv_obj_align(bootList_, LV_ALIGN_TOP_MID, 0, 96);
    lv_scr_load(bootScr_);
    lv_timer_handler();
}

void UIManager::bootStep(const char* text, bool ok) {
    if (!bootList_) return;
    const char* old = lv_label_get_text(bootList_);
    char buf[400];
    snprintf(buf, sizeof(buf), "%s%s %s\n", old ? old : "", ok ? LV_SYMBOL_OK : LV_SYMBOL_WARNING, text);
    lv_label_set_text(bootList_, buf);
    lv_timer_handler();
}

void UIManager::start() {
    settings_ = app_->settings.get();
    main_.create(*app_);
    lv_obj_add_event_cb(main_.obj(), mainEvent, LV_EVENT_ALL, this);
    applySettings(true);
    applyProfile();
    setMode(settings_.measurement.lastMode, false);
    lv_scr_load(main_.obj());
    if (bootScr_) {
        lv_obj_del(bootScr_);
        bootScr_ = nullptr;
        bootList_ = nullptr;
    }
    if (app_->boot.chargeBoot) {
        // woke up for the charge screen: no start-up hints
    } else if (!app_->boot.imu) main_.showToast("Sensor QMI8658 nicht gefunden!", 6000);
    else if (!app_->boot.calibrationLoaded)
        main_.showToast("Bitte kalibrieren:\nMenü > Kalibrierung", 4000);

    if (app_->boot.imu && settings_.measurement.startupGyroCal == StartupGyroCal::Always) {
        // mandatory start-up calibration with guidance on screen
        view_ = View::Calibration;
        app_->startupGyroPending = false;
        cal_.open(*app_, CalibrationScreen::Kind::Gyro, [this] { backToMain(); });
    }
    xTaskCreatePinnedToCore(taskEntry, "ui", appcfg::kUiTaskStack, this, appcfg::kUiTaskPrio, nullptr,
                            appcfg::kUiTaskCore);
}

// ===========================================================================
// task
// ===========================================================================
void UIManager::taskEntry(void* arg) { static_cast<UIManager*>(arg)->task(); }

void UIManager::task() {
    const TickType_t period = pdMS_TO_TICKS(1000 / appcfg::kUiTargetFps);
    TickType_t wake = xTaskGetTickCount();
    uint32_t lastMs = millis();
    uint32_t lastSlow = 0;
    uint32_t frames = 0, fpsStart = millis();
    for (;;) {
        UiCommand c;
        while (xQueueReceive(app_->uiQueue, &c, 0) == pdTRUE) handleCommand(c);
        if (app_->settings.version() != settingsVersion_) applySettings(false);
        if (app_->profiles.version() != profilesVersion_) applyProfile();

        const uint32_t now = millis();
        if (chargeOverlay_ && now - chargeLastUpdate_ >= 1000) {
            chargeLastUpdate_ = now;
            updateChargeScreen();
        }
        if (chargeHideAtMs_ && now >= chargeHideAtMs_) {
            chargeHideAtMs_ = 0;
            closeChargeScreen(false);
        }
        if (sleepAtMs_ && now >= sleepAtMs_) {
            sleepAtMs_ = 0;
            app_->display.sleep();
            displayOff_ = true;
        }
        const float dt = clampv(static_cast<float>(now - lastMs) * 0.001f, 0.001f, 0.2f);
        lastMs = now;
        app_->meas.snapshot(snap_);
        checkStartupGyro();
        PowerStatus ps;
        app_->power.read(ps);

        if (!displayOff_) {
            switch (view_) {
                case View::Main: {
                    main_.update(snap_, ps, dt);
                    UiStatus st = main_.status();
                    st.touchLocked = touchLocked_;
                    st.displayOn = !displayOff_;
                    st.fps = fps_;
                    st.savedCount = savedCount_;
                    st.lastSavedDeg = savedCount_ ? savedValues_[(savedHead_ + 9) % 10] : 0.0f;
                    app_->uiStatus.write(st);
                    break;
                }
                case View::Calibration: cal_.update(snap_); break;
                case View::Menu:
                    if (now - lastSlow > 500) menu_.refresh();
                    break;
                case View::Diagnostics:
                    if (now - lastSlow > 500) diag_.update();
                    break;
            }
            if (now - lastSlow > 500) lastSlow = now;
        }
        lv_timer_handler();
        ++frames;
        if (now - fpsStart >= 1000) {
            fps_ = static_cast<float>(frames) * 1000.0f / static_cast<float>(now - fpsStart);
            frames = 0;
            fpsStart = now;
        }
        // while the display is off a slow loop keeps touch handling alive
        if (displayOff_) vTaskDelay(pdMS_TO_TICKS(100)), wake = xTaskGetTickCount();
        else vTaskDelayUntil(&wake, period);
    }
}

void UIManager::handleCommand(const UiCommand& c) {
    using T = UiCommand::Type;
    switch (c.type) {
        case T::SetMode: setMode(static_cast<AppMode>(c.i)); break;
        case T::ToggleHold: toggleHold(); break;
        case T::Zero: main_.primaryAction(); break;
        case T::ReloadProfile: applyProfile(); break;
        case T::ReloadSettings: applySettings(false); break;
        case T::Wake: wakeDisplay(); break;
        case T::DisplayDim:
            if (!displayOff_) {
                app_->display.setBrightness(settings_.display.brightness > 20 ? 20 : settings_.display.brightness);
                dimmed_ = true;
            }
            break;
        case T::DisplayOff:
            app_->display.sleep();
            displayOff_ = true;
            break;
        case T::ButtonShort:
            if (displayOff_ || dimmed_) {
                wakeDisplay();
                break;
            }
            if (chargeOverlay_) {  // BOOT on the charge screen: switch on / back to measuring
                closeChargeScreen(true);
                break;
            }
            buttonAction(settings_.system.bootButton);
            break;
        case T::ButtonDouble:
            wakeDisplay();
            setTouchLock(!touchLocked_);
            break;
        case T::PowerHold:
            wakeDisplay();
            showPowerHold(c.f);
            break;
        case T::PowerHoldCancel: hidePowerOverlay(); break;
        case T::ChargeChanged:
            if (!chargeOverlay_) {
                wakeDisplay();
                showMessage(batteryText(true).c_str(), 2500);
            }
            break;
        case T::ChargeScreen:
            if (c.i == 0) {
                // charger removed: short message, then close (charge mode -> system sleeps)
                if (chargeOverlay_) {
                    wakeDisplay();
                    lv_label_set_text(chargeState_, "Ladegerät getrennt");
                    chargeHideAtMs_ = millis() + 2500;
                }
            } else {
                wakeDisplay();
                showChargeScreen(c.i == 2);
            }
            break;
        case T::ShowMessage: showMessage(c.text); break;
        case T::BatteryWarning:
            wakeDisplay();
            showMessage(c.i >= 2 ? "Akku kritisch!\nSpeichern gesperrt" : "Akku schwach", 3000);
            break;
        case T::SetTarget:
            app_->profiles.update(settings_.system.activeProfile, [&](Profile& p) { p.targetAngleDeg = c.f; });
            break;
        case T::PrepareSleep:
            // "off" screen with the charge level, then display off
            wakeDisplay();
            showGoodbye();
            sleepAtMs_ = millis() + 900;
            break;
    }
}

void UIManager::checkStartupGyro() {
    if (!app_->startupGyroPending || snap_.cal.resultSeq == calResultSeq_) return;
    calResultSeq_ = snap_.cal.resultSeq;
    if (snap_.cal.procedure != CalProcedure::None) return;  // still running
    app_->startupGyroPending = false;
    GyroCalPoint r;
    bool ok = false;
    if (!app_->meas.lastGyroResult(r, ok) || !ok) {
        LOGI(TAG, "start-up gyro calibration skipped (device moved) - stored offsets used");
        return;
    }
    LOGI(TAG, "start-up gyro calibration: bias %.3f %.3f %.3f dps", double(r.biasDps.x), double(r.biasDps.y),
         double(r.biasDps.z));
    // first calibration ever: persist it (requirement 4.1). Later start-up results
    // only live in RAM to avoid a flash write on every boot.
    sda::SensorCalibration stored;
    if (!app_->calStore.load(stored) || !stored.hasGyro()) {
        SensorCalibration c;
        if (app_->meas.currentCalibration(c)) app_->calStore.save(c);
    }
}

std::string UIManager::batteryText(bool longForm) {
    PowerStatus p;
    app_->power.read(p);
    char b[64];
    if (p.charge == ChargeState::NoBattery || p.batteryPercent < 0) {
        snprintf(b, sizeof(b), "Kein Akku erkannt%s", p.usbHost ? " (USB)" : "");
    } else if (p.charge == ChargeState::Charging) {
        snprintf(b, sizeof(b), "%s Akku ca. %d %%%s", LV_SYMBOL_CHARGE, p.batteryPercent, longForm ? "\nlädt" : "");
    } else if (p.charge == ChargeState::Full) {
        snprintf(b, sizeof(b), "%s Akku voll (USB)", LV_SYMBOL_BATTERY_FULL);
    } else {
        snprintf(b, sizeof(b), "Akku ca. %d %%%s", p.batteryPercent, longForm ? "\nAkkubetrieb" : "");
    }
    return b;
}

// ---------------------------------------------------------------------------
// Charge screen: shown automatically when a charger is connected. Large battery
// symbol with fill level, estimated charge and state. Tap / BOOT closes it (and
// switches the device on when it was "off").
// ---------------------------------------------------------------------------
void UIManager::showChargeScreen(bool fromOff) {
    hidePowerOverlay();
    if (!chargeOverlay_) {
        const theme::Palette& p = theme::pal();
        chargeOverlay_ = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(chargeOverlay_);
        lv_obj_set_size(chargeOverlay_, 240, 240);
        lv_obj_set_style_bg_color(chargeOverlay_, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(chargeOverlay_, LV_OPA_COVER, 0);
        lv_obj_clear_flag(chargeOverlay_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(chargeOverlay_, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(chargeOverlay_, [](lv_event_t* e) {
            static_cast<UIManager*>(lv_event_get_user_data(e))->closeChargeScreen(true);
        }, LV_EVENT_CLICKED, this);
        // battery body
        lv_obj_t* body = lv_obj_create(chargeOverlay_);
        lv_obj_remove_style_all(body);
        lv_obj_set_size(body, 128, 62);
        lv_obj_align(body, LV_ALIGN_CENTER, -5, -30);
        lv_obj_set_style_border_width(body, 4, 0);
        lv_obj_set_style_border_color(body, p.fg, 0);
        lv_obj_set_style_radius(body, 10, 0);
        lv_obj_set_style_pad_all(body, 6, 0);
        lv_obj_clear_flag(body, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t* nub = lv_obj_create(chargeOverlay_);
        lv_obj_remove_style_all(nub);
        lv_obj_set_size(nub, 8, 26);
        lv_obj_align_to(nub, body, LV_ALIGN_OUT_RIGHT_MID, 0, 0);
        lv_obj_set_style_bg_color(nub, p.fg, 0);
        lv_obj_set_style_bg_opa(nub, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(nub, 3, 0);
        lv_obj_clear_flag(nub, LV_OBJ_FLAG_CLICKABLE);
        chargeFill_ = lv_obj_create(body);
        lv_obj_remove_style_all(chargeFill_);
        lv_obj_set_height(chargeFill_, lv_pct(100));
        lv_obj_set_style_radius(chargeFill_, 5, 0);
        lv_obj_set_style_bg_opa(chargeFill_, LV_OPA_COVER, 0);
        lv_obj_clear_flag(chargeFill_, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t* bolt = theme::label(body, theme::large(), p.fg, LV_SYMBOL_CHARGE);
        lv_obj_center(bolt);
        chargePct_ = theme::label(chargeOverlay_, theme::huge(), p.fg, "--");
        lv_obj_align(chargePct_, LV_ALIGN_CENTER, 0, 34);
        chargeState_ = theme::label(chargeOverlay_, theme::medium(), p.dim, "");
        lv_obj_align(chargeState_, LV_ALIGN_CENTER, 0, 70);
        chargeHint_ = theme::label(chargeOverlay_, theme::small(), p.dim, "");
        lv_obj_align(chargeHint_, LV_ALIGN_TOP_MID, 0, 24);
    }
    lv_label_set_text(chargeHint_, fromOff ? "BOOT/Tippen = einschalten" : "Tippen = Messung");
    chargeHideAtMs_ = 0;
    chargeLastUpdate_ = 0;
    updateChargeScreen();
}

void UIManager::updateChargeScreen() {
    if (!chargeOverlay_) return;
    PowerStatus p;
    app_->power.read(p);
    const theme::Palette& pal = theme::pal();
    const int pct = p.batteryPercent < 0 ? 0 : p.batteryPercent;
    const bool full = p.charge == ChargeState::Full || pct >= 100;
    lv_obj_set_width(chargeFill_, lv_pct(pct < 4 ? 4 : pct));
    lv_obj_set_style_bg_color(chargeFill_, full ? pal.ok : (pct < 15 ? pal.far : pal.ok), 0);
    if (p.batteryPercent < 0) lv_label_set_text(chargePct_, "--");
    else lv_label_set_text_fmt(chargePct_, "%d%%", p.batteryPercent);
    if (!chargeHideAtMs_) {
        if (full) lv_label_set_text(chargeState_, "voll");
        else if (p.charge == ChargeState::Charging) lv_label_set_text_fmt(chargeState_, "lädt · %.2f V", double(p.batteryV));
        else lv_label_set_text_fmt(chargeState_, "%.2f V", double(p.batteryV));
    }
}

void UIManager::closeChargeScreen(bool leaveChargeMode) {
    if (chargeOverlay_) lv_obj_del_async(chargeOverlay_);
    chargeOverlay_ = chargeFill_ = chargePct_ = chargeState_ = chargeHint_ = nullptr;
    chargeHideAtMs_ = 0;
    if (leaveChargeMode && app_->chargeMode) {
        app_->chargeMode = false;  // user switched the device on
        if (view_ == View::Main) main_.showToast("Eingeschaltet", 1000);
    }
}

void UIManager::showPowerHold(float progress) {
    if (!powerOverlay_) {
        const theme::Palette& p = theme::pal();
        powerOverlay_ = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(powerOverlay_);
        lv_obj_set_size(powerOverlay_, 240, 240);
        lv_obj_set_style_bg_color(powerOverlay_, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(powerOverlay_, LV_OPA_90, 0);
        lv_obj_clear_flag(powerOverlay_, LV_OBJ_FLAG_SCROLLABLE);
        powerArc_ = lv_arc_create(powerOverlay_);
        lv_obj_set_size(powerArc_, 220, 220);
        lv_obj_center(powerArc_);
        lv_arc_set_rotation(powerArc_, 270);
        lv_arc_set_bg_angles(powerArc_, 0, 360);
        lv_arc_set_range(powerArc_, 0, 100);
        lv_obj_remove_style(powerArc_, nullptr, LV_PART_KNOB);
        lv_obj_clear_flag(powerArc_, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_arc_width(powerArc_, 8, LV_PART_MAIN);
        lv_obj_set_style_arc_width(powerArc_, 8, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(powerArc_, p.grid, LV_PART_MAIN);
        lv_obj_set_style_arc_color(powerArc_, p.far, LV_PART_INDICATOR);
        lv_obj_t* sym = theme::label(powerOverlay_, theme::large(), p.fg, LV_SYMBOL_POWER);
        lv_obj_align(sym, LV_ALIGN_CENTER, 0, -36);
        lv_obj_t* t = theme::label(powerOverlay_, theme::medium(), p.fg, "Ausschalten ...");
        lv_obj_align(t, LV_ALIGN_CENTER, 0, 0);
        lv_obj_t* h = theme::label(powerOverlay_, theme::small(), p.dim, "Loslassen = abbrechen");
        lv_obj_align(h, LV_ALIGN_CENTER, 0, 28);
        lv_obj_t* bat = theme::label(powerOverlay_, theme::small(), p.dim, batteryText(false).c_str());
        lv_obj_align(bat, LV_ALIGN_CENTER, 0, 52);
    }
    lv_arc_set_value(powerArc_, static_cast<int16_t>(progress * 100));
}

void UIManager::hidePowerOverlay() {
    if (powerOverlay_) lv_obj_del(powerOverlay_);
    powerOverlay_ = nullptr;
    powerArc_ = nullptr;
}

void UIManager::showGoodbye() {
    hidePowerOverlay();
    showPowerHold(1.0f);  // reuse the overlay layout, then replace the texts
    lv_obj_clean(powerOverlay_);
    const theme::Palette& p = theme::pal();
    lv_obj_t* sym = theme::label(powerOverlay_, theme::large(), p.dim, LV_SYMBOL_POWER);
    lv_obj_align(sym, LV_ALIGN_CENTER, 0, -50);
    lv_obj_t* t = theme::label(powerOverlay_, theme::large(), p.fg, "Aus");
    lv_obj_align(t, LV_ALIGN_CENTER, 0, -14);
    lv_obj_t* bat = theme::label(powerOverlay_, theme::medium(), p.fg, batteryText(false).c_str());
    lv_obj_align(bat, LV_ALIGN_CENTER, 0, 20);
    lv_obj_t* h = theme::label(powerOverlay_, theme::small(), p.dim,
                               settings_.system.powerOnHold ? "Ein: BOOT 1 s halten" : "Ein: BOOT drücken");
    lv_obj_align(h, LV_ALIGN_CENTER, 0, 50);
    lv_obj_t* u = theme::label(powerOverlay_, theme::small(), p.dim, "USB/Flashen nur eingeschaltet");
    lv_obj_align(u, LV_ALIGN_CENTER, 0, 70);
    powerArc_ = nullptr;
}

void UIManager::wakeDisplay() {
    if (displayOff_ && !sleepAtMs_) {
        // display was off: show the charge level briefly after waking up
        if (view_ == View::Main) main_.showToast(batteryText(true).c_str(), 1500);
    }
    if (displayOff_) {
        app_->display.wake();
        displayOff_ = false;
    }
    if (dimmed_) dimmed_ = false;
    app_->display.setBrightness(settings_.display.brightness);
}

void UIManager::showMessage(const char* text, uint32_t ms) {
    if (view_ == View::Main) main_.showToast(text, ms);
    LOGI(TAG, "message: %s", text);
}

void UIManager::applySettings(bool initial) {
    settingsVersion_ = app_->settings.version();
    const AppSettings old = settings_;
    settings_ = app_->settings.get();
    if (!initial && (old.display.theme != settings_.display.theme || old.display.colorMode != settings_.display.colorMode)) {
        // colours are baked into the objects: rebuild the main screen
        theme::apply(settings_.display.theme, settings_.display.colorMode);
        lv_obj_t* oldScr = main_.obj();
        const AppMode m = main_.mode();
        // rebuild in place (a temporary MainScreen would not fit on the UI task stack)
        main_.create(*app_);
        lv_obj_add_event_cb(main_.obj(), mainEvent, LV_EVENT_ALL, this);
        main_.setSettings(settings_);
        main_.setProfile(app_->profiles.get(settings_.system.activeProfile), settings_.system.activeProfile);
        main_.setMode(m);
        quick_ = picker_ = nullptr;
        if (view_ == View::Main) lv_scr_load(main_.obj());
        if (oldScr) lv_obj_del_async(oldScr);
    }
    if (initial || old.display.brightness != settings_.display.brightness) {
        if (!dimmed_ && !displayOff_) app_->display.setBrightness(settings_.display.brightness);
    }
    if (initial || old.display.rotation != settings_.display.rotation) app_->display.setRotation(settings_.display.rotation);
    if (!initial) {
        if (old.measurement.fusion != settings_.measurement.fusion) app_->meas.setFusion(settings_.measurement.fusion);
        if (old.measurement.filterStrength != settings_.measurement.filterStrength)
            app_->meas.setFilterTau(filterParams(settings_.measurement.filterStrength).fusionMinTauSec);
        bool cfgChange = std::memcmp(&old.system.axisMap, &settings_.system.axisMap, sizeof(AxisMap)) != 0 ||
                         old.measurement.onlineBiasTracking != settings_.measurement.onlineBiasTracking ||
                         old.measurement.tempCoeffAssumedDpsPerK != settings_.measurement.tempCoeffAssumedDpsPerK;
        const bool axisChange = std::memcmp(&old.system.axisMap, &settings_.system.axisMap, sizeof(AxisMap)) != 0;
        if (axisChange) {
            // keep existing calibration valid: express it in the corrected device frame
            SensorCalibration c;
            if (app_->meas.currentCalibration(c) && (c.hasGyro() || c.hasAccel())) {
                c.remap(axisRemapRotation(old.system.axisMap, settings_.system.axisMap));
                app_->meas.setCalibration(c);
                app_->calStore.save(c);
            }
            app_->meas.clearReference();  // zero points refer to the old frame
            LOGI(TAG, "axis mapping changed, calibration remapped");
        }
        if (cfgChange) app_->meas.setConfig(app_->pipelineConfig(settings_));
        if (old.display.rotation != settings_.display.rotation && settings_.system.axisMapVerified)
            showMessage("Anzeige gedreht:\nKalibrierung > Achsen\nerkennen erneut ausführen", 4000);
        if (old.system.activeProfile != settings_.system.activeProfile) applyProfile();
        if (old.measurement.lastMode != settings_.measurement.lastMode && main_.mode() != settings_.measurement.lastMode)
            setMode(settings_.measurement.lastMode, false);
    }
    main_.setSettings(settings_);
}

void UIManager::applyProfile() {
    profilesVersion_ = app_->profiles.version();
    const uint8_t idx = settings_.system.activeProfile;
    const Profile p = app_->profiles.get(idx);
    main_.setProfile(p, idx);
    app_->meas.setMount(p.mount);
}

void UIManager::setMode(AppMode m, bool persist) {
    if (m >= AppMode::Count) m = AppMode::Drill;
    if (hold_) toggleHold();
    main_.setMode(m);
    if (persist) app_->settings.update([m](AppSettings& s) { s.measurement.lastMode = m; });
}

void UIManager::nextMode(int dir) {
    const int n = static_cast<int>(AppMode::Count);
    const int m = (static_cast<int>(main_.mode()) + dir + n) % n;
    setMode(static_cast<AppMode>(m));
    char b[40];
    snprintf(b, sizeof(b), "%s", appModeName(static_cast<AppMode>(m)));
    main_.showToast(b, 700);
}

void UIManager::toggleHold() {
    hold_ = !hold_;
    main_.setHold(hold_);
}

void UIManager::setTouchLock(bool on) {
    touchLocked_ = on;
    main_.setTouchLocked(on);
    if (view_ == View::Main) main_.showToast(on ? "Touchsperre ein\n(2,5 s halten = aus)" : "Touchsperre aus", 1200);
}

void UIManager::buttonAction(ButtonAction a) {
    switch (a) {
        case ButtonAction::Hold: toggleHold(); break;
        case ButtonAction::Zero: main_.primaryAction(); break;
        case ButtonAction::NextMode: nextMode(1); break;
        case ButtonAction::TouchLock: setTouchLock(!touchLocked_); break;
        default: break;
    }
}

// ===========================================================================
// main screen touch handling
// ===========================================================================
void UIManager::mainEvent(lv_event_t* e) {
    auto* self = static_cast<UIManager*>(lv_event_get_user_data(e));
    const lv_event_code_t code = lv_event_get_code(e);
    if (self->quick_ || self->picker_) return;  // overlays handle their own input
    const bool vibrating = self->snap_.vibration != VibrationLevel::None;
    const bool autoLock = vibrating && self->settings_.display.touchLockAuto;
    switch (code) {
        case LV_EVENT_PRESSED:
            g_pressStartMs = millis();
            g_unlockFired = false;
            break;
        case LV_EVENT_LONG_PRESSED_REPEAT:
            if (self->touchLocked_ && !g_unlockFired && millis() - g_pressStartMs > 2500) {
                g_unlockFired = true;
                self->setTouchLock(false);
            }
            break;
        case LV_EVENT_LONG_PRESSED:
            if (self->touchLocked_ || autoLock) {
                if (self->touchLocked_) self->main_.showToast("Gesperrt: weiter halten", 900);
                break;
            }
            self->onLongPress();
            break;
        case LV_EVENT_SHORT_CLICKED:
            if (self->touchLocked_ || autoLock) break;
            if (millis() - g_lastGestureMs < kGestureTapBlockMs) break;  // tail of a swipe
            self->onTap();
            break;
        case LV_EVENT_GESTURE: {
            g_lastGestureMs = millis();
            // swipes during drilling vibration are most likely accidental
            if (self->touchLocked_ || vibrating) break;
            self->onGesture(lv_indev_get_gesture_dir(lv_indev_get_act()));
            break;
        }
        default: break;
    }
}

void UIManager::tapTimerCb(lv_timer_t* t) {
    auto* self = static_cast<UIManager*>(t->user_data);
    self->tapTimer_ = nullptr;
    self->lastTapMs_ = 0;
    self->openQuickMenu();
}

void UIManager::onTap() {
    const uint32_t now = millis();
    if (tapTimer_ && now - lastTapMs_ < appcfg::kDoubleTapMs) {
        lv_timer_del(tapTimer_);
        tapTimer_ = nullptr;
        lastTapMs_ = 0;
        onDoubleTap();
        return;
    }
    lastTapMs_ = now;
    if (tapTimer_) lv_timer_del(tapTimer_);
    tapTimer_ = lv_timer_create(tapTimerCb, appcfg::kDoubleTapMs, this);
    lv_timer_set_repeat_count(tapTimer_, 1);
}

void UIManager::onDoubleTap() { toggleHold(); }

void UIManager::onLongPress() {
    if (hold_) {
        // while holding: long press stores the frozen value
        savedValues_[savedHead_] = main_.currentValue();
        savedHead_ = (savedHead_ + 1) % 10;
        if (savedCount_ < 10) ++savedCount_;
        char b[48];
        snprintf(b, sizeof(b), "Gespeichert: %.2f°", double(main_.currentValue()));
        main_.showToast(b);
        LOGI(TAG, "saved value %.3f", double(main_.currentValue()));
        return;
    }
    main_.primaryAction();
}

void UIManager::onGesture(lv_dir_t dir) {
    if (dir == LV_DIR_LEFT) nextMode(1);
    else if (dir == LV_DIR_RIGHT) nextMode(-1);
}

// ===========================================================================
// quick menu
// ===========================================================================
void UIManager::openQuickMenu() {
    if (quick_) return;
    const theme::Palette& p = theme::pal();
    quick_ = lv_obj_create(main_.obj());
    lv_obj_remove_style_all(quick_);
    lv_obj_set_size(quick_, 240, 240);
    lv_obj_set_style_bg_color(quick_, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(quick_, LV_OPA_90, 0);
    lv_obj_clear_flag(quick_, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(quick_, LV_OBJ_FLAG_CLICKABLE);
    // tap on the dimmed background closes the quick menu
    lv_obj_add_event_cb(quick_, [](lv_event_t* e) {
        auto* self = static_cast<UIManager*>(lv_event_get_user_data(e));
        if (lv_event_get_target(e) == self->quick_) self->closeQuickMenu();
    }, LV_EVENT_CLICKED, this);
    struct Item {
        const char* sym;
        const char* txt;
    };
    const Item items[9] = {
        {LV_SYMBOL_GPS, "Soll"},      {LV_SYMBOL_LOOP, "Bezug"},   {LV_SYMBOL_SHUFFLE, "Tol."}, {LV_SYMBOL_IMAGE, "Skala"},
        {hold_ ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE, hold_ ? "Live" : "Hold"},
        {hold_ ? LV_SYMBOL_SAVE : LV_SYMBOL_REFRESH, hold_ ? "Speich." : "Null"},
        {LV_SYMBOL_EYE_CLOSE, "Sperre"}, {LV_SYMBOL_LIST, "Profil"}, {LV_SYMBOL_SETTINGS, "Menü"},
    };
    for (int i = 0; i < 9; ++i) {
        const float a = deg2rad(90.0f - 40.0f * i);
        lv_obj_t* b = lv_btn_create(quick_);
        lv_obj_set_size(b, 50, 50);
        lv_obj_set_style_radius(b, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(b, lv_color_hex(0x22272E), 0);
        lv_obj_set_style_bg_color(b, p.accent, LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(b, 0, 0);
        lv_obj_set_style_pad_all(b, 0, 0);
        lv_obj_align(b, LV_ALIGN_CENTER, static_cast<lv_coord_t>(84 * cosf(a)), static_cast<lv_coord_t>(-84 * sinf(a)));
        lv_obj_set_user_data(b, reinterpret_cast<void*>(intptr_t(i)));
        lv_obj_add_event_cb(b, quickEvent, LV_EVENT_CLICKED, this);
        lv_obj_t* s = theme::label(b, theme::medium(), p.fg, items[i].sym);
        lv_obj_align(s, LV_ALIGN_CENTER, 0, -8);
        lv_obj_t* l = theme::label(b, theme::small(), p.dim, items[i].txt);
        lv_obj_align(l, LV_ALIGN_CENTER, 0, 13);
    }
    lv_obj_t* close = lv_btn_create(quick_);
    lv_obj_set_size(close, 58, 58);
    lv_obj_center(close);
    lv_obj_set_style_radius(close, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(close, lv_color_hex(0x30363F), 0);
    lv_obj_set_style_shadow_width(close, 0, 0);
    lv_obj_set_user_data(close, reinterpret_cast<void*>(intptr_t(-1)));
    lv_obj_add_event_cb(close, quickEvent, LV_EVENT_CLICKED, this);
    lv_obj_t* cl = theme::label(close, theme::large(), p.fg, LV_SYMBOL_CLOSE);
    lv_obj_center(cl);
}

void UIManager::closeQuickMenu() {
    if (quick_) lv_obj_del_async(quick_);
    quick_ = nullptr;
}

void UIManager::quickEvent(lv_event_t* e) {
    auto* self = static_cast<UIManager*>(lv_event_get_user_data(e));
    const intptr_t id = reinterpret_cast<intptr_t>(lv_obj_get_user_data(lv_event_get_current_target(e)));
    self->closeQuickMenu();
    switch (id) {
        case 0: self->openPicker(1); break;
        case 1: self->openPicker(5); break;
        case 2: self->openPicker(2); break;
        case 3: self->openPicker(3); break;
        case 4: self->toggleHold(); break;
        case 5: self->onLongPress(); break;
        case 6: self->setTouchLock(true); break;
        case 7: self->openPicker(4); break;
        case 8: self->openMenuRoot(); break;
        default: break;
    }
}

void UIManager::openPicker(int kind) {
    closePicker();
    pickerKind_ = kind;
    const theme::Palette& p = theme::pal();
    const Profile prof = app_->profiles.get(settings_.system.activeProfile);
    std::string opts;
    int sel = 0;
    const char* title = "";
    switch (kind) {
        case 1: {
            title = "Sollwinkel";
            char b[32];
            for (int i = 0; i < kQuickAngleCount; ++i) {
                const float a = quickAngle(i, settings_.measurement);
                snprintf(b, sizeof(b), i == kQuickAngleCount - 1 ? "Eigen %.1f°" : "%.0f°", double(a));
                if (i) opts += "\n";
                opts += b;
                if (fabsf(a - prof.targetAngleDeg) < 0.01f) sel = i;
            }
            break;
        }
        case 2: {
            title = "Toleranz";
            char b[16];
            for (int i = 0; i < kTolCount; ++i) {
                snprintf(b, sizeof(b), "±%.2g°", double(kTolerances[i]));
                if (i) opts += "\n";
                opts += b;
                if (fabsf(kTolerances[i] - prof.toleranceDeg) < 0.001f) sel = i;
            }
            break;
        }
        case 3:
            title = "Skala";
            opts = "Grob ±20°\nNormal ±10°\nFein ±5°\nPräzision ±2°\nAutomatisch";
            sel = static_cast<int>(prof.scale);
            break;
        case 5:
            title = "Winkelbezug";
            for (int i = 0; i < static_cast<int>(AngleReference::Count); ++i) {
                if (i) opts += "\n";
                opts += angleReferenceName(static_cast<AngleReference>(i));
            }
            sel = static_cast<int>(prof.angleRef);
            break;
        case 4: {
            title = "Profil";
            profileNames_.clear();
            const ProfileSet ps = app_->profiles.all();
            int n = 0;
            for (int i = 0; i < kMaxProfiles; ++i) {
                if (!ps.profiles[i].used) continue;
                if (n) opts += "\n";
                opts += ps.profiles[i].name;
                profileNames_.push_back(std::to_string(i));
                if (i == settings_.system.activeProfile) sel = n;
                ++n;
            }
            break;
        }
    }
    picker_ = lv_obj_create(main_.obj());
    lv_obj_remove_style_all(picker_);
    lv_obj_set_size(picker_, 240, 240);
    lv_obj_set_style_bg_color(picker_, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(picker_, LV_OPA_COVER, 0);
    lv_obj_add_flag(picker_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(picker_, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* t = theme::label(picker_, theme::small(), p.accent, title);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 22);
    pickerRoller_ = lv_roller_create(picker_);
    lv_roller_set_options(pickerRoller_, opts.c_str(), LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(pickerRoller_, 3);
    lv_obj_set_width(pickerRoller_, 184);
    // short entries (angles, tolerances) large, texts in the medium font so that
    // they fit the round panel
    lv_obj_set_style_text_font(pickerRoller_, (kind == 1 || kind == 2) ? theme::large() : theme::medium(), 0);
    lv_obj_set_style_bg_color(pickerRoller_, lv_color_hex(0x101214), 0);
    lv_obj_set_style_text_color(pickerRoller_, p.fg, 0);
    lv_obj_set_style_bg_color(pickerRoller_, p.accent, LV_PART_SELECTED);
    lv_obj_set_style_text_color(pickerRoller_, lv_color_hex(0x000000), LV_PART_SELECTED);
    lv_obj_align(pickerRoller_, LV_ALIGN_CENTER, 0, -14);
    lv_obj_set_style_text_align(pickerRoller_, LV_TEXT_ALIGN_CENTER, 0);
    lv_roller_set_selected(pickerRoller_, static_cast<uint16_t>(sel), LV_ANIM_OFF);
    auto mkBtn = [&](const char* sym, int x, bool ok) {
        lv_obj_t* b = lv_btn_create(picker_);
        lv_obj_set_size(b, 64, 44);
        lv_obj_align(b, LV_ALIGN_CENTER, x, 76);
        lv_obj_set_style_radius(b, 22, 0);
        lv_obj_set_style_bg_color(b, lv_color_hex(0x2A3038), 0);
        lv_obj_set_style_shadow_width(b, 0, 0);
        lv_obj_set_user_data(b, reinterpret_cast<void*>(intptr_t(ok ? 1 : 0)));
        lv_obj_add_event_cb(b, pickerOk, LV_EVENT_CLICKED, this);
        lv_obj_t* l = theme::label(b, theme::large(), p.fg, sym);
        lv_obj_center(l);
    };
    mkBtn(LV_SYMBOL_CLOSE, -40, false);
    mkBtn(LV_SYMBOL_OK, 40, true);
}

void UIManager::closePicker() {
    if (picker_) lv_obj_del_async(picker_);
    picker_ = nullptr;
    pickerRoller_ = nullptr;
}

void UIManager::pickerOk(lv_event_t* e) {
    auto* self = static_cast<UIManager*>(lv_event_get_user_data(e));
    const bool ok = reinterpret_cast<intptr_t>(lv_obj_get_user_data(lv_event_get_current_target(e))) == 1;
    const int sel = self->pickerRoller_ ? lv_roller_get_selected(self->pickerRoller_) : 0;
    const int kind = self->pickerKind_;
    self->closePicker();
    if (!ok) return;
    const uint8_t active = self->settings_.system.activeProfile;
    switch (kind) {
        case 1: {
            const float a = quickAngle(sel, self->settings_.measurement);
            self->app_->profiles.update(active, [a](Profile& p) { p.targetAngleDeg = a; });
            break;
        }
        case 2:
            if (sel >= 0 && sel < kTolCount)
                self->app_->profiles.update(active, [sel](Profile& p) { p.toleranceDeg = kTolerances[sel]; });
            break;
        case 3:
            self->app_->profiles.update(active, [sel](Profile& p) { p.scale = static_cast<ScaleMode>(sel); });
            break;
        case 5:
            self->app_->profiles.update(active, [sel](Profile& p) { p.angleRef = static_cast<AngleReference>(sel); });
            if (angleReferenceNeedsZero(static_cast<AngleReference>(sel)))
                self->main_.showToast("Bezugsachse ausrichten\nund lange drücken", 2500);
            break;
        case 4:
            if (sel >= 0 && sel < static_cast<int>(self->profileNames_.size())) {
                const int idx = std::stoi(self->profileNames_[sel]);
                self->app_->settings.update([idx](AppSettings& s) { s.system.activeProfile = static_cast<uint8_t>(idx); });
            }
            break;
    }
}

// ===========================================================================
// navigation
// ===========================================================================
void UIManager::backToMain() {
    view_ = View::Main;
    lv_scr_load(main_.obj());
    menu_.close();
    cal_.close();
    diag_.close();
}

void UIManager::openCalibration(CalibrationScreen::Kind k) {
    view_ = View::Calibration;
    cal_.open(*app_, k, [this] {
        view_ = View::Menu;
        menuCalibration();
        cal_.close();
    });
    menu_.close();
}

void UIManager::openDiagnostics() {
    view_ = View::Diagnostics;
    diag_.open(*app_, [this] {
        view_ = View::Menu;
        menuSystem();
        diag_.close();
    }, &fps_);
    menu_.close();
}
