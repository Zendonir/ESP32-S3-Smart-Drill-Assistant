// SPDX-License-Identifier: MIT
#include "CalibrationScreen.h"

#include <Arduino.h>

#include <cstdio>
#include <cstring>

#include "../system/Log.h"
#include "Theme.h"

using namespace sda;

static const char* TAG = "calui";

namespace {
const char* poseInstruction(int p) {
    switch (p) {
        case 0: return "Rechte Kante nach OBEN\n(auf linke Kante stellen)";
        case 1: return "Linke Kante nach OBEN\n(auf rechte Kante stellen)";
        case 2: return "Obere Kante nach OBEN\n(aufrecht hinstellen)";
        case 3: return "Obere Kante nach UNTEN\n(Kopfstand)";
        case 4: return "Display nach OBEN\n(flach hinlegen)";
        case 5: return "Display nach UNTEN\n(flach umdrehen)";
    }
    return "";
}
void hide(lv_obj_t* o, bool h) {
    if (!o) return;
    if (h) lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
}
}  // namespace

lv_obj_t* CalibrationScreen::button(const char* text, int x, int y, int w, lv_event_cb_t cb) {
    lv_obj_t* b = lv_btn_create(scr_);
    lv_obj_set_size(b, w, 40);
    lv_obj_align(b, LV_ALIGN_CENTER, x, y);
    lv_obj_set_style_radius(b, 20, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x2A3038), 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, this);
    lv_obj_t* l = theme::label(b, theme::small(), theme::pal().fg, text);
    lv_obj_center(l);
    return b;
}

void CalibrationScreen::setButtons(const char* left, const char* mid, const char* right) {
    auto set = [](lv_obj_t* b, const char* t) {
        if (!t) {
            hide(b, true);
            return;
        }
        hide(b, false);
        lv_label_set_text(lv_obj_get_child(b, 0), t);
    };
    set(btnLeft_, left);
    set(btnMid_, mid);
    set(btnRight_, right);
    // a single button sits in the middle (the round panel clips the corners)
    const int count = (left ? 1 : 0) + (mid ? 1 : 0) + (right ? 1 : 0);
    lv_obj_align(btnLeft_, LV_ALIGN_CENTER, count == 1 ? 0 : -52, 72);
    lv_obj_align(btnRight_, LV_ALIGN_CENTER, count == 1 ? 0 : 52, 72);
}

void CalibrationScreen::open(AppContext& app, Kind kind, std::function<void()> onDone) {
    app_ = &app;
    kind_ = kind;
    onDone_ = std::move(onDone);
    phase_ = Phase::Running;
    solving_ = false;
    lastMountPositions_ = 0;
    const theme::Palette& p = theme::pal();
    lv_obj_t* old = scr_;
    scr_ = theme::screen();

    arc_ = lv_arc_create(scr_);
    lv_obj_set_size(arc_, 226, 226);
    lv_obj_center(arc_);
    lv_arc_set_rotation(arc_, 270);
    lv_arc_set_bg_angles(arc_, 0, 360);
    lv_arc_set_range(arc_, 0, 1000);
    lv_arc_set_value(arc_, 0);
    lv_obj_remove_style(arc_, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(arc_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(arc_, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc_, 6, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc_, p.grid, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc_, p.accent, LV_PART_INDICATOR);

    title_ = theme::label(scr_, theme::medium(), p.accent, "");
    lv_obj_align(title_, LV_ALIGN_TOP_MID, 0, 24);
    text_ = theme::label(scr_, theme::medium(), p.fg, "");
    lv_obj_set_width(text_, 180);
    lv_label_set_long_mode(text_, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(text_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(text_, LV_ALIGN_CENTER, 0, -28);
    detail_ = theme::label(scr_, theme::small(), p.dim, "");
    lv_obj_set_width(detail_, 180);
    lv_label_set_long_mode(detail_, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(detail_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(detail_, LV_ALIGN_CENTER, 0, 22);

    btnLeft_ = button("", -52, 72, 84, [](lv_event_t* e) { static_cast<CalibrationScreen*>(lv_event_get_user_data(e))->onLeft(); });
    btnRight_ = button("", 52, 72, 84, [](lv_event_t* e) { static_cast<CalibrationScreen*>(lv_event_get_user_data(e))->onRight(); });
    btnMid_ = button("", 0, 72, 120, [](lv_event_t* e) { static_cast<CalibrationScreen*>(lv_event_get_user_data(e))->onMid(); });

    for (int i = 0; i < 6; ++i) {
        poseDots_[i] = theme::label(scr_, theme::small(), p.dim, accelPoseName(AccelPose(i)));
        const float a = deg2rad(-150.0f + 60.0f * i);  // across the upper area
        (void)a;
        lv_obj_align(poseDots_[i], LV_ALIGN_TOP_MID, -75 + 30 * i, 50);
        hide(poseDots_[i], kind != Kind::Accel && kind != Kind::AccelExtended);
    }

    haveBefore_ = app.meas.currentCalibration(before_);
    OrientationSnapshot s;
    app.meas.snapshot(s);
    lastResultSeq_ = s.cal.resultSeq;

    switch (kind) {
        case Kind::Gyro:
            lv_label_set_text(title_, "Gyroskop");
            setButtons(nullptr, "Abbrechen", nullptr);
            app.meas.startGyroCalibration();
            break;
        case Kind::Accel:
        case Kind::AccelExtended:
            lv_label_set_text(title_, kind == Kind::Accel ? "6-Lagen-Kalibr." : "6 Lagen erweitert");
            app.meas.startAccelCalibration(kind == Kind::AccelExtended);
            lastPoseMask_ = 0;
            askConfirm("6 Lagen", "Gerät in eine der 6 Lagen\nlegen, dann Start", "Start");
            break;
        case Kind::MountRoll:
            lv_label_set_text(title_, "Montage (V-Nut)");
            setButtons("Abbr.", nullptr, "Erfassen");
            app.meas.startMountCalibration(CalProcedure::MountRoll);
            break;
        case Kind::MountVertical:
            lv_label_set_text(title_, "Montage (Ständer)");
            setButtons("Abbr.", nullptr, "Erfassen");
            app.meas.startMountCalibration(CalProcedure::MountVertical);
            break;
        case Kind::AxisDetect:
            lv_label_set_text(title_, "Achsen erkennen");
            setButtons("Abbr.", nullptr, nullptr);
            axisStep_ = 0;
            axisFrames_ = 0;
            axisSum_ = Vec3();
            axisOld_ = app.settings.get().system.axisMap;
            askConfirm("Schritt 1/2", "Display nach OBEN\nflach hinlegen,\ndann Start", "Start");
            break;
    }
    lv_scr_load(scr_);
    if (old) lv_obj_del_async(old);
}

void CalibrationScreen::close() {
    if (!scr_) return;
    lv_obj_t* s = scr_;
    scr_ = nullptr;
    lv_obj_del_async(s);
}

void CalibrationScreen::finish(bool saved) {
    if (!saved && haveBefore_ && (kind_ == Kind::Gyro)) app_->meas.setCalibration(before_);  // discard new point
    app_->meas.cancelCalibration();
    auto fn = onDone_;
    if (fn) fn();
}

void CalibrationScreen::update(const OrientationSnapshot& s) {
    if (!scr_) return;
    switch (kind_) {
        case Kind::Gyro: updateGyro(s); break;
        case Kind::Accel:
        case Kind::AccelExtended: updateAccel(s); break;
        case Kind::MountRoll:
        case Kind::MountVertical: updateMount(s); break;
        case Kind::AxisDetect: updateAxis(s); break;
    }
}

// Two still positions are averaged in SENSOR coordinates (the current mapping is
// undone), the mapping is then derived by detectAxisMap().
void CalibrationScreen::askConfirm(const char* title, const char* nextStep, const char* button) {
    awaitConfirm_ = true;
    lv_arc_set_value(arc_, 0);
    lv_label_set_text(text_, title);
    lv_label_set_text(detail_, nextStep);
    setButtons("Abbr.", nullptr, button);
}

void CalibrationScreen::updateAxis(const OrientationSnapshot& s) {
    if (phase_ != Phase::Running || awaitConfirm_) return;
    constexpr int kFrames = 40;  // ~1 s at the UI rate
    lv_label_set_text(text_, axisStep_ == 0 ? "Display nach OBEN\nflach hinlegen"
                                            : "Aufrecht hinstellen,\ndieser Text lesbar");
    if (!s.valid || s.motion != MotionState::Still) {
        axisFrames_ = 0;
        axisSum_ = Vec3();
        lv_arc_set_value(arc_, 0);
        // tell the user what prevents the still-stand detection
        const char* why = !s.valid                 ? "keine Sensordaten"
                          : (s.stillBlockers & 1)  ? "Drehung erkannt"
                          : (s.stillBlockers & 2)  ? "Erschütterung erkannt"
                          : (s.stillBlockers & 4)  ? "Beschleunigung erkannt"
                          : (s.stillBlockers & 8)  ? "langsame Lageänderung"
                                                   : "warte auf Ruhe";
        lv_label_set_text_fmt(detail_, "Schritt %d/2 - %s", axisStep_ + 1, why);
        return;
    }
    lv_label_set_text_fmt(detail_, "Schritt %d/2 - messe ...", axisStep_ + 1);
    // device frame (current mapping) -> sensor frame
    axisSum_ += axisOld_.matrix().transposed() * s.rawAccelG;
    ++axisFrames_;
    lv_arc_set_value(arc_, static_cast<int16_t>(1000 * axisFrames_ / kFrames));
    if (axisFrames_ < kFrames) return;
    const Vec3 mean = axisSum_ / float(axisFrames_);
    axisFrames_ = 0;
    axisSum_ = Vec3();
    if (axisStep_ == 0) {
        axisFlatS_ = mean;
        axisStep_ = 1;
        askConfirm(LV_SYMBOL_OK " Schritt 1 erledigt", "Als Nächstes: aufrecht\nhinstellen, Text lesbar,\ndann Weiter", "Weiter");
        return;
    }
    // step 2 must be a different position (device turned upright, ~90 deg)
    if (rad2deg(angleBetween(mean, axisFlatS_)) < 45.0f) {
        lv_arc_set_value(arc_, 0);
        lv_label_set_text(detail_, "Gerät liegt noch flach -\nbitte aufrecht hinstellen");
        return;
    }
    axisResult_ = detectAxisMap(axisFlatS_, mean);
    if (!axisResult_.ok) {
        phase_ = Phase::Failed;
        lv_label_set_text(text_, "Nicht erkannt");
        lv_label_set_text_fmt(detail_, "%s.\nLagen genauer einnehmen.", axisResult_.error);
        setButtons(nullptr, "Zurück", nullptr);
        return;
    }
    phase_ = Phase::Result;
    char nb[8], ob[8];
    axisResult_.map.format(nb);
    axisOld_.format(ob);
    const bool same = std::memcmp(axisResult_.map.code, axisOld_.code, 3) == 0;
    lv_label_set_text_fmt(text_, "Zuordnung\n%s", nb);
    lv_label_set_text_fmt(detail_, "%s (bisher %s)\nLagefehler %.1f°", same ? "unverändert" : "geändert", ob,
                          double(axisResult_.orthogonalityErrDeg));
    setButtons("Verwerfen", nullptr, "Speichern");
}

void CalibrationScreen::updateGyro(const OrientationSnapshot& s) {
    if (phase_ != Phase::Running) return;
    lv_arc_set_value(arc_, static_cast<int16_t>(s.cal.progress * 1000));
    lv_label_set_text(text_, "Gerät ruhig\nhinlegen");
    lv_label_set_text(detail_, s.cal.motionRestart ? "Bewegung erkannt - Neustart" : "Nicht berühren");
    if (s.cal.resultSeq == lastResultSeq_) return;
    lastResultSeq_ = s.cal.resultSeq;
    GyroCalPoint r;
    bool ok = false;
    app_->meas.lastGyroResult(r, ok);
    if (ok) {
        phase_ = Phase::Result;
        lv_arc_set_value(arc_, 1000);
        lv_label_set_text(text_, "Fertig");
        lv_label_set_text_fmt(detail_, "Offset %.2f %.2f %.2f °/s\nRauschen %.3f °/s  %.1f °C", double(r.biasDps.x),
                              double(r.biasDps.y), double(r.biasDps.z), double(r.noiseDps.norm()), double(r.tempC));
        setButtons("Verwerfen", nullptr, "Speichern");
    } else {
        phase_ = Phase::Failed;
        lv_label_set_text(text_, "Fehlgeschlagen");
        lv_label_set_text(detail_, s.cal.failure == 1 ? "Zu viel Bewegung" : s.cal.failure == 2 ? "Messrauschen zu hoch"
                                                                                                : "Werte unplausibel");
        setButtons(nullptr, "Zurück", nullptr);
    }
}

void CalibrationScreen::updateAccel(const OrientationSnapshot& s) {
    const theme::Palette& p = theme::pal();
    for (int i = 0; i < 6; ++i) {
        const bool acc = s.cal.acceptedPoseMask & (1u << i);
        const bool act = s.cal.activePose == i;
        lv_obj_set_style_text_color(poseDots_[i], acc ? p.ok : (act ? p.near : p.dim), 0);
    }
    if (phase_ != Phase::Running) return;
    const uint8_t all = 0x3F;
    // a pose was just accepted: confirm before the next one is measured
    if (s.cal.acceptedPoseMask != lastPoseMask_) {
        int done = -1;
        for (int i = 0; i < 6; ++i)
            if ((s.cal.acceptedPoseMask & (1u << i)) && !(lastPoseMask_ & (1u << i))) done = i;
        lastPoseMask_ = s.cal.acceptedPoseMask;
        if (s.cal.acceptedPoseMask != all && done >= 0) {
            int count = 0;
            for (int i = 0; i < 6; ++i) count += (s.cal.acceptedPoseMask >> i) & 1;
            static char title[40];
            snprintf(title, sizeof(title), LV_SYMBOL_OK " Lage %s erledigt (%d/6)", accelPoseName(AccelPose(done)), count);
            askConfirm(title, "Gerät in die nächste Lage\nbringen, dann Weiter", "Weiter");
            return;
        }
    }
    if (awaitConfirm_) return;
    if (s.cal.acceptedPoseMask == all && !solving_) {
        solving_ = true;
        if (app_->meas.accelSolve(accelResult_)) {
            phase_ = Phase::Result;
            lv_label_set_text(text_, "Fertig");
            lv_label_set_text_fmt(detail_, "Restfehler %.1f mg\nSkala %.3f %.3f %.3f\nOffset %.0f %.0f %.0f mg",
                                  double(accelResult_.residualMg), double(1.0f / accelResult_.matrix.m[0][0]),
                                  double(1.0f / accelResult_.matrix.m[1][1]), double(1.0f / accelResult_.matrix.m[2][2]),
                                  double(-accelResult_.offset.x * 1000 / accelResult_.matrix.m[0][0]),
                                  double(-accelResult_.offset.y * 1000 / accelResult_.matrix.m[1][1]),
                                  double(-accelResult_.offset.z * 1000 / accelResult_.matrix.m[2][2]));
            setButtons("Verwerfen", nullptr, "Speichern");
        } else {
            phase_ = Phase::Failed;
            lv_label_set_text(text_, "Fehlgeschlagen");
            lv_label_set_text(detail_, "Ergebnis unplausibel.\nLagen genauer ausrichten.");
            setButtons(nullptr, "Zurück", nullptr);
        }
        return;
    }
    // next pending pose
    int next = -1;
    for (int i = 0; i < 6; ++i)
        if (!(s.cal.acceptedPoseMask & (1u << i))) { next = i; break; }
    if (s.cal.activePose >= 0) {
        lv_arc_set_value(arc_, static_cast<int16_t>(s.cal.progress * 1000));
        lv_label_set_text_fmt(text_, "Lage %s\nwird gemessen", accelPoseName(AccelPose(s.cal.activePose)));
        lv_label_set_text(detail_, s.cal.motionRestart ? "Unruhig/falsch - Neustart" : "Ruhig liegen lassen");
        return;
    }
    lv_arc_set_value(arc_, 0);
    // automatic start: detected pose is pending and the device is still
    const int det = s.cal.detectedPose;
    if (det >= 0 && !(s.cal.acceptedPoseMask & (1u << det)) && s.motion == MotionState::Still) {
        app_->meas.accelBeginPose(AccelPose(det));
        return;
    }
    if (next >= 0) lv_label_set_text(text_, poseInstruction(next));
    lv_label_set_text(detail_, det >= 0 && (s.cal.acceptedPoseMask & (1u << det)) ? "Diese Lage ist erledigt"
                                                                                : "Lage beliebig wählbar,\nwird erkannt");
}

void CalibrationScreen::updateMount(const OrientationSnapshot& s) {
    if (phase_ != Phase::Running) return;
    const bool roll = kind_ == Kind::MountRoll;
    if (s.cal.capturing) {
        lv_arc_set_value(arc_, static_cast<int16_t>(s.cal.progress * 1000));
        lv_label_set_text(text_, s.cal.waitingStill ? "Ruhig halten ..." : "Erfasse ...");
        return;
    }
    lv_arc_set_value(arc_, 0);
    if (s.cal.mountPositions != lastMountPositions_ || s.cal.resultSeq != lastResultSeq_) {
        lastResultSeq_ = s.cal.resultSeq;
        lastMountPositions_ = s.cal.mountPositions;
    }
    if (roll) {
        lv_label_set_text_fmt(text_, "Bohrer in V-Nut,\num Achse drehen\nPositionen: %u", s.cal.mountPositions);
        lv_label_set_text(detail_, s.cal.failure == 1 ? "Zu nah an vorheriger Position"
                                                      : "4-8 Positionen, je ca. 45-90°");
        setButtons("Abbr.", nullptr, s.cal.mountPositions >= 3 ? "Fertig" : "Erfassen");
        // with >= 3 positions the right button computes; middle stays free for more captures
        if (s.cal.mountPositions >= 3) {
            hide(btnMid_, false);
            lv_label_set_text(lv_obj_get_child(btnMid_, 0), "+Pos.");
            lv_obj_align(btnMid_, LV_ALIGN_CENTER, 0, 30);
            lv_obj_set_width(btnMid_, 80);
        }
    } else {
        lv_label_set_text(text_, s.cal.mountPositions ? "Erfasst" : "Bohrachse exakt\nsenkrecht (Ständer)");
        lv_label_set_text(detail_, "Genauigkeit = Genauigkeit\nder mech. Referenz");
        setButtons("Abbr.", nullptr, s.cal.mountPositions ? "Fertig" : "Erfassen");
    }
}

void CalibrationScreen::onLeft() {
    if (phase_ == Phase::Result) {
        finish(false);  // discard
        return;
    }
    finish(false);  // cancel
}

void CalibrationScreen::onMid() {
    if (phase_ == Phase::Running && (kind_ == Kind::MountRoll)) {
        OrientationSnapshot s;
        app_->meas.snapshot(s);
        if (s.cal.mountPositions >= 3) {
            app_->meas.mountCapture();  // additional position
            return;
        }
    }
    finish(false);
}

void CalibrationScreen::onRight() {
    const uint8_t active = app_->settings.get().system.activeProfile;
    if (awaitConfirm_) {
        // user confirmed: the next measurement may start (still-stand required)
        awaitConfirm_ = false;
        axisFrames_ = 0;
        axisSum_ = Vec3();
        setButtons(nullptr, "Abbrechen", nullptr);
        return;
    }
    if (kind_ == Kind::AxisDetect) {
        if (phase_ != Phase::Result) return;
        const AxisMap m = axisResult_.map;
        // UIManager::applySettings remaps the calibration and reconfigures the pipeline
        app_->settings.update([m](AppSettings& st) {
            st.system.axisMap = m;
            st.system.axisMapVerified = true;
        });
        finish(true);
        return;
    }
    if (phase_ == Phase::Running && (kind_ == Kind::MountRoll || kind_ == Kind::MountVertical)) {
        OrientationSnapshot s;
        app_->meas.snapshot(s);
        const bool ready = kind_ == Kind::MountRoll ? s.cal.mountPositions >= 3 : s.cal.mountPositions >= 1;
        if (!ready) {
            app_->meas.mountCapture();
            return;
        }
        const Profile prof = app_->profiles.get(active);
        if (!app_->meas.mountSolve(prof.mount.drillAxisD, true, mountResult_) || !mountResult_.ok) {
            phase_ = Phase::Failed;
            lv_label_set_text(text_, "Fehlgeschlagen");
            static const char* errs[] = {"", "Zu wenige Positionen", "Zu wenig gedreht (<90°)",
                                         "Restfehler zu groß", "Achse ungeeignet"};
            lv_label_set_text(detail_, errs[int(mountResult_.error) % 5]);
            setButtons(nullptr, "Zurück", nullptr);
            return;
        }
        phase_ = Phase::Result;
        lv_label_set_text(text_, "Achse bestimmt");
        lv_label_set_text_fmt(detail_, "(%.3f %.3f %.3f)\nRest %.2f°  Abdeck. %.0f°", double(mountResult_.axisD.x),
                              double(mountResult_.axisD.y), double(mountResult_.axisD.z), double(mountResult_.residualDeg),
                              double(mountResult_.coverageDeg));
        hide(btnMid_, true);
        setButtons("Verwerfen", nullptr, "Speichern");
        return;
    }
    if (phase_ != Phase::Result) return;
    // ---- save ----
    bool ok = false;
    if (kind_ == Kind::Gyro) {
        SensorCalibration c;
        ok = app_->meas.currentCalibration(c) && app_->calStore.save(c);
    } else if (kind_ == Kind::Accel || kind_ == Kind::AccelExtended) {
        SensorCalibration c;
        if (app_->meas.currentCalibration(c)) {
            c.addAccelPoint(accelResult_);
            ok = app_->meas.setCalibration(c) && app_->calStore.save(c);
        }
    } else {
        app_->profiles.update(active, [&](Profile& p) {
            p.mount.drillAxisD = mountResult_.axisD;
            p.mount.method = kind_ == Kind::MountRoll ? MountModel::Method::RollFit : MountModel::Method::VerticalRef;
            p.mount.qualityDeg = mountResult_.residualDeg;
            p.mountCalTime = static_cast<uint32_t>(millis() / 1000);
        });
        app_->meas.setMount(app_->profiles.get(active).mount);
        ok = true;
    }
    LOGI(TAG, "calibration %s", ok ? "saved" : "NOT saved");
    if (!ok) {
        lv_label_set_text(detail_, "Speichern fehlgeschlagen\n(Akku kritisch?)");
        return;
    }
    finish(true);
}
