// SPDX-License-Identifier: MIT
#include "MainScreen.h"

#include <Arduino.h>

#include <cstdio>

#include "Theme.h"

using namespace sda;

namespace {
constexpr int kDot = 18;
constexpr int kR = theme::kTargetRadius;

void hide(lv_obj_t* o, bool h) {
    if (!o) return;
    if (h) lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
}

lv_obj_t* makeLine(lv_obj_t* parent, lv_point_t* pts, uint16_t n, lv_color_t c, int w) {
    lv_obj_t* l = lv_line_create(parent);
    lv_obj_remove_style_all(l);
    lv_obj_set_pos(l, 0, 0);
    lv_obj_set_size(l, 240, 240);
    lv_obj_set_style_line_color(l, c, 0);
    lv_obj_set_style_line_width(l, w, 0);
    lv_obj_set_style_line_rounded(l, true, 0);
    lv_obj_clear_flag(l, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(l, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_line_set_points(l, pts, n);
    return l;
}
}  // namespace

void MainScreen::create(AppContext& app) {
    app_ = &app;
    const theme::Palette& p = theme::pal();
    scr_ = theme::screen();

    // concentric scale rings at 25/50/75 % and the outer scale circle
    for (int i = 0; i < 3; ++i) {
        const int d = kR * 2 * (i + 1) / 4;
        rings_[i] = theme::circle(scr_, d, p.grid, 1, false);
        lv_obj_center(rings_[i]);
    }
    outer_ = theme::circle(scr_, 2 * kR, p.grid, 2, false);
    lv_obj_center(outer_);

    crossHPts_[0] = {static_cast<lv_coord_t>(theme::kCx - kR), theme::kCy};
    crossHPts_[1] = {static_cast<lv_coord_t>(theme::kCx + kR), theme::kCy};
    crossVPts_[0] = {theme::kCx, static_cast<lv_coord_t>(theme::kCy - kR)};
    crossVPts_[1] = {theme::kCx, static_cast<lv_coord_t>(theme::kCy + kR)};
    crossH_ = makeLine(scr_, crossHPts_, 2, p.grid, 1);
    crossV_ = makeLine(scr_, crossVPts_, 2, p.grid, 1);

    tolRing_ = theme::circle(scr_, 20, p.far, 2, false);
    lv_obj_center(tolRing_);

    // level edge line (hidden unless the device stands on an edge)
    edgePts_[0] = {20, 120};
    edgePts_[1] = {220, 120};
    edgeLine_ = makeLine(scr_, edgePts_, 2, p.fg, 3);
    hide(edgeLine_, true);

    // progress arc for the precision mode
    progress_ = lv_arc_create(scr_);
    lv_obj_set_size(progress_, 226, 226);
    lv_obj_center(progress_);
    lv_arc_set_rotation(progress_, 270);
    lv_arc_set_bg_angles(progress_, 0, 360);
    lv_arc_set_range(progress_, 0, 1000);
    lv_arc_set_value(progress_, 0);
    lv_obj_remove_style(progress_, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(progress_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(progress_, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_style_arc_width(progress_, 5, LV_PART_MAIN);
    lv_obj_set_style_arc_width(progress_, 5, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(progress_, p.grid, LV_PART_MAIN);
    lv_obj_set_style_arc_color(progress_, p.accent, LV_PART_INDICATOR);
    hide(progress_, true);

    // history chart (angle mode)
    chart_ = lv_chart_create(scr_);
    lv_obj_set_size(chart_, 150, 40);
    lv_obj_align(chart_, LV_ALIGN_CENTER, 0, 36);
    lv_chart_set_type(chart_, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart_, 60);
    lv_chart_set_div_line_count(chart_, 3, 0);
    lv_obj_set_style_bg_opa(chart_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(chart_, 0, 0);
    lv_obj_set_style_line_color(chart_, p.grid, LV_PART_MAIN);
    lv_obj_set_style_size(chart_, 0, LV_PART_INDICATOR);
    lv_obj_set_style_line_width(chart_, 2, LV_PART_ITEMS);
    lv_obj_clear_flag(chart_, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(chart_, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    series_ = lv_chart_add_series(chart_, p.accent, LV_CHART_AXIS_PRIMARY_Y);
    hide(chart_, true);

    // marker, check mark, edge arrow
    dot_ = theme::circle(scr_, kDot, p.far, 3, true);
    lv_obj_set_pos(dot_, theme::kCx - kDot / 2, theme::kCy - kDot / 2);
    check_ = theme::label(scr_, theme::large(), p.ok, LV_SYMBOL_OK);
    lv_obj_center(check_);
    hide(check_, true);
    arrowPts_[0] = arrowPts_[1] = arrowPts_[2] = {120, 120};
    arrow_ = makeLine(scr_, arrowPts_, 3, p.far, 5);
    hide(arrow_, true);

    holdRing_ = theme::circle(scr_, 236, p.hold, 4, false);
    lv_obj_center(holdRing_);
    hide(holdRing_, true);

    // text
    lblMode_ = theme::label(scr_, theme::small(), p.dim, "");
    lv_obj_align(lblMode_, LV_ALIGN_TOP_MID, 0, 16);
    lblQuality_ = theme::label(scr_, theme::small(), p.dim, "");
    lv_obj_align(lblQuality_, LV_ALIGN_TOP_MID, -28, 36);
    lblBattery_ = theme::label(scr_, theme::small(), p.dim, "");
    lv_obj_align(lblBattery_, LV_ALIGN_TOP_MID, 46, 36);
    lblBig_ = theme::label(scr_, theme::huge(), p.fg, "--");
    lv_obj_align(lblBig_, LV_ALIGN_CENTER, 0, 46);
    lblXY_ = theme::label(scr_, theme::small(), p.fg, "");
    lv_obj_align(lblXY_, LV_ALIGN_CENTER, 0, 76);
    lblInfo_ = theme::label(scr_, theme::small(), p.dim, "");
    lv_obj_align(lblInfo_, LV_ALIGN_CENTER, 0, 94);
    lblHint_ = theme::label(scr_, theme::medium(), p.fg, "");
    lv_obj_set_width(lblHint_, 168);
    lv_label_set_long_mode(lblHint_, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(lblHint_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblHint_, LV_ALIGN_CENTER, 0, -18);
    hide(lblHint_, true);
    lblLock_ = theme::label(scr_, theme::small(), p.warn, "GESPERRT");
    lv_obj_align(lblLock_, LV_ALIGN_TOP_MID, 0, 56);
    hide(lblLock_, true);
    lblToast_ = theme::label(scr_, theme::medium(), p.fg, "");
    lv_obj_set_style_bg_color(lblToast_, lv_color_hex(0x202428), 0);
    lv_obj_set_style_bg_opa(lblToast_, LV_OPA_90, 0);
    lv_obj_set_style_pad_all(lblToast_, 8, 0);
    lv_obj_set_style_radius(lblToast_, 10, 0);
    lv_obj_set_width(lblToast_, 176);
    lv_label_set_long_mode(lblToast_, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(lblToast_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lblToast_);
    hide(lblToast_, true);

    configureFilters();
    tracker_.reset();
    autoScale_.reset();
}

void MainScreen::configureFilters() {
    const FilterParams fp = filterParams(settings_.measurement.filterStrength);
    fx_.configure(fp.displayMinCutoffHz, fp.displayBeta);
    fy_.configure(fp.displayMinCutoffHz, fp.displayBeta);
    fv_.configure(fp.displayMinCutoffHz, fp.displayBeta);
    const float step = settings_.display.decimals >= 2 ? 0.01f : 0.1f;
    nBig_.setStep(step);
    nX_.setStep(step);
    nY_.setStep(step);
}

void MainScreen::setMode(AppMode m) {
    if (m >= AppMode::Count) m = AppMode::Drill;
    mode_ = m;
    tracker_.reset();
    autoScale_.reset();
    fx_.reset();
    fy_.reset();
    fv_.reset();
    history_.clear();
    lastTargetState_ = -1;
    if (m != AppMode::Precision && app_) app_->meas.abortPrecision();
    if (m == AppMode::Angle) angleRefDeg_ = NAN;
}

void MainScreen::setProfile(const Profile& p, uint8_t idx) {
    profile_ = p;
    profileIdx_ = idx;
    TargetConfig tc;
    tc.toleranceDeg = p.toleranceDeg;
    tc.holdSec = settings_.measurement.holdTimeSec;
    tracker_.configure(tc);
}

void MainScreen::setSettings(const AppSettings& s) {
    settings_ = s;
    configureFilters();
    if (lblXY_) lv_obj_set_style_text_font(lblXY_, s.display.largeFont ? theme::medium() : theme::small(), 0);
    TargetConfig tc = tracker_.config();
    tc.holdSec = s.measurement.holdTimeSec;
    tracker_.configure(tc);
}

void MainScreen::setHold(bool on) {
    hold_ = on;
    hide(holdRing_, !on);
    const theme::Palette& p = theme::pal();
    lv_obj_set_style_text_color(lblBig_, on ? p.hold : p.fg, 0);
    lv_obj_set_style_text_color(lblXY_, on ? p.hold : p.fg, 0);
    if (on) {
        lv_obj_set_style_bg_opa(dot_, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(dot_, p.hold, 0);
        lv_label_set_text_fmt(lblMode_, "HOLD  %s", appModeShort(mode_));
        lv_obj_set_style_text_color(lblMode_, p.hold, 0);
    } else {
        lv_obj_set_style_text_color(lblMode_, p.dim, 0);
        lastTargetState_ = -1;  // force marker restyle
    }
}

void MainScreen::setTouchLocked(bool locked) {
    touchLocked_ = locked;
    hide(lblLock_, !locked);
}

void MainScreen::showToast(const char* text, uint32_t ms) {
    lv_label_set_text(lblToast_, text);
    hide(lblToast_, false);
    lv_obj_move_foreground(lblToast_);
    toastUntil_ = millis() + ms;
}

void MainScreen::fmtAngle(char* buf, size_t n, float v, bool sign) const {
    if (!std::isfinite(v)) {
        snprintf(buf, n, "--.-°");
        return;
    }
    const int dec = settings_.display.decimals >= 2 ? 2 : 1;
    if (sign) snprintf(buf, n, "%+.*f°", dec, double(v));
    else snprintf(buf, n, "%.*f°", dec, double(v));
}

float MainScreen::scaleFor(float total, float dt) {
    if (profile_.scale == ScaleMode::Auto) return autoScale_.update(total, dt);
    return scaleRangeDeg(profile_.scale);
}

void MainScreen::setMarkerStyle(TargetState st, bool valid) {
    const theme::Palette& p = theme::pal();
    const lv_color_t c = valid ? theme::targetColor(st) : p.invalid;
    lv_obj_set_style_border_color(dot_, c, 0);
    lv_obj_set_style_bg_color(dot_, c, 0);
    // invalid -> hollow marker (shape cue, not only colour)
    lv_obj_set_style_bg_opa(dot_, valid ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(tolRing_, valid ? c : p.grid, 0);
    const bool locked = valid && st == TargetState::Locked;
    lv_obj_set_style_border_width(tolRing_, locked ? 4 : 2, 0);
    lv_obj_set_style_bg_color(tolRing_, p.ok, 0);
    lv_obj_set_style_bg_opa(tolRing_, locked ? LV_OPA_30 : LV_OPA_TRANSP, 0);
    lv_obj_set_style_line_color(arrow_, c, 0);
    lv_obj_set_style_text_color(lblBig_, hold_ ? p.hold : (valid ? (locked ? p.ok : p.fg) : p.invalid), 0);
    hide(check_, !locked);
}

void MainScreen::layoutTarget(bool visible) {
    for (lv_obj_t* r : rings_) hide(r, !visible);
    hide(outer_, !visible);
    hide(crossH_, !visible);
    hide(crossV_, !visible);
    hide(tolRing_, !visible);
    hide(dot_, !visible);
    if (!visible) {
        hide(check_, true);
        hide(arrow_, true);
    }
}

void MainScreen::updateDeviationView(float devX, float devY, float total, bool valid, float tol, float dt,
                                     bool lateralFree, bool lateralGyro) {
    if (profile_.invertMarker) {  // bubble style: marker shows where to move
        devX = -devX;
        devY = -devY;
    }
    const float x = fx_.update(devX, dt);
    const float y = fy_.update(devY, dt);
    const float t = fv_.update(total, dt);
    const float range = scaleFor(t, dt);
    const TargetState st = tracker_.update(t, valid, dt);
    if (int(st) != lastTargetState_ || tracker_.justLocked()) {
        setMarkerStyle(st, valid);
        lastTargetState_ = int(st);
        if (tracker_.justLocked()) {
            // short confirmation animation: tolerance ring pulse
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, tolRing_);
            lv_anim_set_values(&a, 4, 8);
            lv_anim_set_time(&a, 150);
            lv_anim_set_playback_time(&a, 150);
            lv_anim_set_exec_cb(&a, [](void* o, int32_t v) {
                lv_obj_set_style_border_width(static_cast<lv_obj_t*>(o), v, 0);
            });
            lv_anim_start(&a);
        }
    }

    // tolerance ring radius proportional to tolerance / scale
    int rt = static_cast<int>(kR * tol / range);
    rt = clampv(rt, 6, kR);
    lv_obj_set_size(tolRing_, 2 * rt, 2 * rt);
    lv_obj_center(tolRing_);

    const MarkerPos m = mapDeviation(x, y, range, kR - kDot / 2);
    // "einrasten": inside the locked tolerance the marker snaps to the centre
    const bool snap = valid && st == TargetState::Locked;
    const int px = snap ? 0 : static_cast<int>(lroundf(m.x));
    const int py = snap ? 0 : static_cast<int>(lroundf(m.y));
    lv_obj_set_pos(dot_, theme::kCx + px - kDot / 2, theme::kCy - py - kDot / 2);
    if (m.clipped) {
        const float a = m.directionRad;
        auto pt = [&](float r, float da) {
            return lv_point_t{static_cast<lv_coord_t>(theme::kCx + r * cosf(a + da)),
                              static_cast<lv_coord_t>(theme::kCy - r * sinf(a + da))};
        };
        arrowPts_[0] = pt(104, 0.10f);
        arrowPts_[1] = pt(114, 0.0f);
        arrowPts_[2] = pt(104, -0.10f);
        lv_line_set_points(arrow_, arrowPts_, 3);
        hide(arrow_, false);
    } else {
        hide(arrow_, true);
    }

    char b1[24], b2[24], b3[72];
    hide(lblXY_, !profile_.showNumbers);
    if (valid) {
        fmtAngle(b1, sizeof(b1), nBig_.update(t), false);
        lv_label_set_text(lblBig_, b1);
        fmtAngle(b1, sizeof(b1), nX_.update(x), true);
        fmtAngle(b2, sizeof(b2), nY_.update(y), true);
        if (lateralFree) snprintf(b3, sizeof(b3), "Neig. %s  Seite frei", b2);
        else snprintf(b3, sizeof(b3), "X %s  Y %s%s", b1, b2, lateralGyro ? " G" : "");
        lv_label_set_text(lblXY_, b3);
    } else {
        lv_label_set_text(lblBig_, "--.-°");
        lv_label_set_text(lblXY_, "X --  Y --");
    }
    snprintf(b3, sizeof(b3), "±%.2g°  Skala %.0f°%s", double(tol), double(range),
             profile_.scale == ScaleMode::Auto ? "A" : "");
    lv_label_set_text(lblInfo_, b3);

    status_.devXDeg = x;
    status_.devYDeg = y;
    status_.devTotalDeg = t;
    status_.targetState = st;
    status_.scaleDeg = range;
    status_.toleranceDeg = tol;
    status_.lateralFree = lateralFree;
    status_.lateralFromGyro = lateralGyro;
}

void MainScreen::pushHistory(float x, float y, float total, const OrientationSnapshot& s, float dt) {
    histAccum_ += dt;
    if (histAccum_ < 0.05f) return;  // 20 Hz history
    histAccum_ = 0;
    history_.push(HistPoint{x, y, total, s.tiltSigmaDeg, s.vibrationRmsMg});
}

void MainScreen::refreshChart() {
    const size_t win = static_cast<size_t>(settings_.display.historyWindowSec) * 20;
    const size_t n = history_.size() < win ? history_.size() : win;
    if (n < 2) return;
    float lo = 1e9f, hi = -1e9f;
    for (size_t i = history_.size() - n; i < history_.size(); ++i) {
        lo = fminf(lo, history_.at(i).total);
        hi = fmaxf(hi, history_.at(i).total);
    }
    if (hi - lo < 0.5f) {
        const float mid = 0.5f * (hi + lo);
        lo = mid - 0.25f;
        hi = mid + 0.25f;
    }
    lv_chart_set_range(chart_, LV_CHART_AXIS_PRIMARY_Y, static_cast<lv_coord_t>(lo * 100),
                       static_cast<lv_coord_t>(hi * 100));
    const int points = 60;
    for (int k = 0; k < points; ++k) {
        const size_t idx = history_.size() - n + (n - 1) * size_t(k) / size_t(points - 1);
        series_->y_points[k] = static_cast<lv_coord_t>(history_.at(idx).total * 100);
    }
    lv_chart_refresh(chart_);
}

// ---------------------------------------------------------------------------
void MainScreen::updateHeader(const OrientationSnapshot& s, const PowerStatus& p) {
    char b[48];
    if (!hold_) {
        switch (mode_) {
            case AppMode::Drill:
                snprintf(b, sizeof(b), "BOHREN %.4g° %s", double(profile_.targetAngleDeg),
                         angleReferenceShort(profile_.angleRef));
                break;
            case AppMode::Angle: snprintf(b, sizeof(b), "WINKEL soll %.4g°", double(profile_.targetAngleDeg)); break;
            default: snprintf(b, sizeof(b), "%s", appModeShort(mode_)); break;
        }
        lv_label_set_text(lblMode_, b);
    }
    // quality: hide "calibration required" behind a softer wording when only accel cal is missing
    const QualityState q = s.quality.state;
    const char* qs = qualityStateShort(q);
    if (q == QualityState::CalibrationRequired && s.hasGyroCal) qs = "UNKALIBR.";
    lv_label_set_text(lblQuality_, s.valid ? qs : "START");
    lv_obj_set_style_text_color(lblQuality_, theme::qualityColor(q), 0);
    if (p.charge == ChargeState::Full) {
        snprintf(b, sizeof(b), LV_SYMBOL_USB " " LV_SYMBOL_BATTERY_FULL);
    } else if (p.charge == ChargeState::NoBattery || p.batteryPercent < 0) {
        snprintf(b, sizeof(b), p.usbHost ? LV_SYMBOL_USB : LV_SYMBOL_BATTERY_EMPTY " --");
    } else {
        const char* sym = p.charge == ChargeState::Charging ? LV_SYMBOL_CHARGE
                          : p.batteryPercent > 80           ? LV_SYMBOL_BATTERY_FULL
                          : p.batteryPercent > 55           ? LV_SYMBOL_BATTERY_3
                          : p.batteryPercent > 30           ? LV_SYMBOL_BATTERY_2
                          : p.batteryPercent > 10           ? LV_SYMBOL_BATTERY_1 : LV_SYMBOL_BATTERY_EMPTY;
        snprintf(b, sizeof(b), "%s %d%%", sym, p.batteryPercent);
    }
    lv_label_set_text(lblBattery_, b);
    const theme::Palette& pal = theme::pal();
    lv_obj_set_style_text_color(lblBattery_,
                                p.charge == ChargeState::Charging ? pal.ok
                                : p.level == BatteryLevel::Critical ? pal.far
                                : p.level == BatteryLevel::Low    ? pal.warn : pal.dim, 0);
}

void MainScreen::updateDrill(const OrientationSnapshot& s, float dt) {
    const AngleTargetResult at = makeAngleTarget(profile_.angleRef, profile_.targetAngleDeg,
                                                 settings_.measurement.hemisphere, s.reference, profile_.mount);
    const DrillGeometry g = drillGeometry(s.qWD, profile_.mount);
    status_.valueDeg = g.inclinationDeg;
    if (!at.valid) {
        // zero based angle without (suitable) zero point, or impossible target
        layoutTarget(false);
        hide(lblHint_, false);
        if (at.needsZero)
            lv_label_set_text(lblHint_, angleReferenceUsesGyro(profile_.angleRef)
                                            ? "Bezugsachse ausrichten,\nlange drücken:\n3D-Nullpunkt setzen"
                                            : "Bezugsachse ausrichten,\nlange drücken:\nNullpunkt setzen");
        else
            lv_label_set_text_fmt(lblHint_, "%s\nBezug %.1f°", at.problem, double(at.referenceElevationDeg));
        lv_label_set_text(lblBig_, "");
        lv_label_set_text_fmt(lblXY_, "Achse jetzt %+.1f°", double(g.elevationDeg));
        lv_label_set_text(lblInfo_, angleReferenceName(profile_.angleRef));
        tracker_.update(NAN, false, dt);
        status_.targetState = TargetState::Invalid;
        return;
    }
    hide(lblHint_, true);
    layoutTarget(true);
    const Deviation d = drillDeviation(s.qWD, profile_.mount, at.target, hemi_);
    bool valid = s.valid && s.quality.usableForTarget(profile_.toleranceDeg, s.tiltSigmaDeg);
    const float hSigma = s.reference.info().headingSigmaDeg;
    // gyro based left/right angle: green only while the heading uncertainty fits
    if (at.usesGyro && 2.0f * hSigma > profile_.toleranceDeg) valid = false;
    updateDeviationView(d.screenX, d.screenY, d.totalDeg, valid, profile_.toleranceDeg, dt, d.lateralFree,
                        d.lateralFromGyro);
    if (at.usesGyro)
        lv_label_set_text_fmt(lblInfo_, "±%.2g°  Gyro ±%.1f°  %lus", double(profile_.toleranceDeg), double(hSigma),
                              static_cast<unsigned long>(s.reference.info().ageSec));
    else if (angleReferenceNeedsZero(profile_.angleRef))
        lv_label_set_text_fmt(lblInfo_, "±%.2g°  Ziel %+.1f°", double(profile_.toleranceDeg),
                              double(at.targetElevationDeg));
    pushHistory(d.screenX, d.screenY, d.totalDeg, s, dt);
}

void MainScreen::updateLevel(const OrientationSnapshot& s, float dt) {
    const LevelAngles& l = s.level;
    const bool rel = s.reference.kind() == ReferenceKind::TiltZero;
    const bool valid = s.valid && s.quality.usableForTarget(profile_.toleranceDeg, s.tiltSigmaDeg);
    if (l.flat || rel) {
        hide(edgeLine_, true);
        layoutTarget(true);
        float x = l.tiltXDeg, y = l.tiltYDeg;
        if (rel) relativeTilt(s.reference.tiltZeroUpD(), s.upD, x, y);
        if (s.upD.z < 0 && !rel) { x = -x; }  // face down: mirror so the bubble still rises
        // bubble rises to the high side: right edge up -> bubble right
        const float total = sqrtf(x * x + y * y);
        updateDeviationView(x, y, total, valid, profile_.toleranceDeg, dt, false, false);
        lv_label_set_text(lblMode_, rel ? "WAAGE relativ" : (l.faceCorrected ? "WAAGE · Umschlag" : "WAAGE"));
        status_.valueDeg = total;
        pushHistory(x, y, total, s, dt);
    } else {
        // standing on an edge: tube level, rotation in the vertical plane
        layoutTarget(false);
        hide(edgeLine_, false);
        const float a = deg2rad(fv_.update(l.edgeRollDeg, dt));
        edgePts_[0] = {static_cast<lv_coord_t>(120 - 95 * cosf(a)), static_cast<lv_coord_t>(120 - 95 * sinf(a))};
        edgePts_[1] = {static_cast<lv_coord_t>(120 + 95 * cosf(a)), static_cast<lv_coord_t>(120 + 95 * sinf(a))};
        lv_line_set_points(edgeLine_, edgePts_, 2);
        const TargetState st = tracker_.update(fabsf(l.edgeRollDeg), valid, dt);
        lv_obj_set_style_line_color(edgeLine_, valid ? theme::targetColor(st) : theme::pal().invalid, 0);
        char b[24];
        fmtAngle(b, sizeof(b), valid ? nBig_.update(l.edgeRollDeg) : NAN, true);
        lv_label_set_text(lblBig_, b);
        lv_label_set_text_fmt(lblXY_, "Kante %d  Neigung %+.1f°%s", l.edgeIndex, double(l.edgePitchDeg),
                              l.faceCorrected ? "  U" : "");
        lv_label_set_text_fmt(lblInfo_, "±%.2g°", double(profile_.toleranceDeg));
        lv_label_set_text(lblMode_, "WAAGE Kante");
        status_.targetState = st;
        status_.valueDeg = l.edgeRollDeg;
        status_.devTotalDeg = fabsf(l.edgeRollDeg);
    }
}

void MainScreen::updateAngle(const OrientationSnapshot& s, float dt) {
    layoutTarget(false);
    hide(chart_, !settings_.display.showHistory);
    const DrillGeometry g = drillGeometry(s.qWD, profile_.mount);
    const bool toVertical = profile_.angleRef == AngleReference::Vertical;
    const float absValue = toVertical ? 90.0f - g.inclinationDeg : g.inclinationDeg;
    const float value = fv_.update(std::isfinite(angleRefDeg_) ? g.elevationDeg - angleRefDeg_ : absValue, dt);
    const float diff = std::isfinite(angleRefDeg_) ? value : value - profile_.targetAngleDeg;
    const bool valid = s.valid && s.quality.usableForTarget(profile_.toleranceDeg, s.tiltSigmaDeg);
    const TargetState st = tracker_.update(fabsf(diff), valid, dt);
    char b[32], c[32], line[64];
    fmtAngle(b, sizeof(b), valid ? nBig_.update(value) : NAN, std::isfinite(angleRefDeg_));
    lv_label_set_text(lblBig_, b);
    lv_obj_set_style_text_color(lblBig_, hold_ ? theme::pal().hold : (valid ? theme::targetColor(st) : theme::pal().invalid), 0);
    fmtAngle(c, sizeof(c), valid ? diff : NAN, true);
    if (std::isfinite(angleRefDeg_)) snprintf(line, sizeof(line), "relativ zu %.1f°", double(angleRefDeg_));
    else snprintf(line, sizeof(line), "Soll %.4g°  Diff %s", double(profile_.targetAngleDeg), c);
    lv_label_set_text(lblXY_, line);
    lv_label_set_text_fmt(lblInfo_, "±%.2g°  %s  %s", double(profile_.toleranceDeg),
                          g.elevationDeg >= 0 ? "aufwärts" : "abwärts", toVertical ? "z.Vert." : "z.Horiz.");
    // big value in the centre for this mode
    lv_obj_align(lblBig_, LV_ALIGN_CENTER, 0, -14);
    status_.valueDeg = value;
    status_.devTotalDeg = fabsf(diff);
    status_.targetState = st;
    pushHistory(0, 0, value, s, dt);
    static uint8_t div = 0;
    if (++div >= 5) {  // chart refresh at ~8 Hz is sufficient
        div = 0;
        refreshChart();
    }
}

void MainScreen::updateZero(const OrientationSnapshot& s, float dt) {
    const ReferenceKind k = s.reference.kind();
    const bool have = k == ReferenceKind::TiltZero || k == ReferenceKind::Full3D;
    if (!have) {
        layoutTarget(false);
        hide(lblHint_, false);
        lv_label_set_text(lblHint_, "Lange drücken:\nNullpunkt setzen");
        lv_label_set_text(lblBig_, "");
        lv_label_set_text(lblXY_, settings_.measurement.zeroKind == ZeroKind::Full3D ? "Art: 3D (Gyro, driftet)"
                                                                                   : "Art: Neigung (driftfrei)");
        lv_label_set_text(lblInfo_, "");
        return;
    }
    hide(lblHint_, true);
    layoutTarget(true);
    float x, y;
    relativeTilt(s.reference.tiltZeroUpD(), s.upD, x, y);
    const float tilt = sqrtf(x * x + y * y);
    bool valid = s.valid && s.quality.usableForTarget(profile_.toleranceDeg, s.tiltSigmaDeg);
    float total = tilt;
    float heading = 0, hSigma = 0;
    if (k == ReferenceKind::Full3D) {
        const RelativeRotation r = relativeRotation(s.reference.orientationRef(), s.qWD);
        heading = r.headingDeg;
        hSigma = s.reference.info().headingSigmaDeg;
        total = sqrtf(tilt * tilt + heading * heading);
        // a green indication requires the heading uncertainty to fit the tolerance
        if (2.0f * hSigma > profile_.toleranceDeg) valid = false;
    }
    updateDeviationView(x, y, total, valid, profile_.toleranceDeg, dt, false, k == ReferenceKind::Full3D);
    if (k == ReferenceKind::Full3D)
        lv_label_set_text_fmt(lblInfo_, "Dreh %+.1f° ±%.1f° (Gyro)", double(heading), double(hSigma));
    lv_label_set_text(lblMode_, k == ReferenceKind::Full3D ? "NULL 3D" : "NULL Neigung");
    status_.valueDeg = total;
    pushHistory(x, y, total, s, dt);
}

void MainScreen::updateReference(const OrientationSnapshot& s, float dt) {
    const ReferenceKind k = s.reference.kind();
    const bool have = k == ReferenceKind::SurfaceRestricted || k == ReferenceKind::Surface3D;
    if (!have) {
        layoutTarget(false);
        hide(lblHint_, false);
        lv_label_set_text(lblHint_, "Gerät flach auf die\nFläche legen, dann\nlange drücken");
        lv_label_set_text(lblBig_, "");
        lv_label_set_text(lblXY_, settings_.measurement.reference3D ? "Modus 3D (Gyro, driftet)"
                                                                    : "Modus: nur Neigung");
        lv_label_set_text(lblInfo_, "");
        return;
    }
    hide(lblHint_, true);
    layoutTarget(true);
    const ReferenceInfo& ri = s.reference.info();
    const DrillTarget t = s.reference.surfaceTarget();
    const Deviation d = drillDeviation(s.qWD, profile_.mount, t, hemi_);
    bool valid = s.valid && s.quality.usableForTarget(profile_.toleranceDeg, s.tiltSigmaDeg);
    if (d.lateralFromGyro && 2.0f * ri.headingSigmaDeg > profile_.toleranceDeg) valid = false;
    updateDeviationView(d.screenX, d.screenY, d.totalDeg, valid, profile_.toleranceDeg, dt, d.lateralFree,
                        d.lateralFromGyro);
    if (ri.usesGyro)
        lv_label_set_text_fmt(lblInfo_, "Fl. %.1f°  ±%.1f°  %lus", double(ri.surfaceSlopeDeg),
                              double(ri.headingSigmaDeg), static_cast<unsigned long>(ri.ageSec));
    else
        lv_label_set_text_fmt(lblInfo_, "Fläche %.1f°%s", double(ri.surfaceSlopeDeg),
                              ri.azimuthFree ? "  Azimut frei" : "");
    lv_label_set_text(lblMode_, ri.usesGyro ? "REFERENZ 3D" : "REFERENZ Neigung");
    status_.valueDeg = d.totalDeg;
    pushHistory(d.screenX, d.screenY, d.totalDeg, s, dt);
}

void MainScreen::updatePrecision(const OrientationSnapshot& s, float) {
    layoutTarget(false);
    hide(progress_, !s.precisionRunning);
    if (s.precisionRunning) {
        lv_arc_set_value(progress_, static_cast<int16_t>(s.precisionProgress * 1000));
        hide(lblHint_, false);
        lv_label_set_text(lblHint_, s.precisionWaiting ? "Ruhig halten ..." : "Messung läuft");
        lv_label_set_text(lblBig_, "");
        lv_label_set_text_fmt(lblXY_, "Fenster %.1f s", double(settings_.measurement.precisionWindowSec));
        lv_label_set_text(lblInfo_, "");
        return;
    }
    if (s.precisionSeq != lastPrecisionSeq_ && s.precision.valid) {
        lastPrecisionSeq_ = s.precisionSeq;
        if (s.precision.stable) repeat_.add(s.precision.valueDeg);
    }
    if (!s.precision.valid) {
        hide(lblHint_, false);
        lv_label_set_text(lblHint_, "Lange drücken:\nPräzisionsmessung");
        lv_label_set_text(lblBig_, "");
        lv_label_set_text(lblXY_, "Achsneigung, statisch");
        lv_label_set_text(lblInfo_, "");
        return;
    }
    hide(lblHint_, true);
    const PrecisionResult& r = s.precision;
    lv_obj_align(lblBig_, LV_ALIGN_CENTER, 0, -10);
    lv_label_set_text_fmt(lblBig_, "%.2f°", double(r.valueDeg));
    lv_obj_set_style_text_color(lblBig_, r.stable ? theme::pal().ok : theme::pal().warn, 0);
    lv_label_set_text_fmt(lblXY_, "Rauschen σ %.3f°  n %lu", double(r.stdDeg), static_cast<unsigned long>(r.samples));
    if (repeat_.count() >= 2)
        lv_label_set_text_fmt(lblInfo_, "%s  Wdh. σ %.3f° (%lu)", r.stable ? "stabil" : "instabil",
                              double(repeat_.stdDeg()), static_cast<unsigned long>(repeat_.count()));
    else
        lv_label_set_text_fmt(lblInfo_, "%s  Drift %.3f°/s", r.stable ? "stabil" : "instabil",
                              double(r.slopeDegPerS));
    status_.valueDeg = r.valueDeg;
}

void MainScreen::update(const OrientationSnapshot& s, const PowerStatus& p, float dt) {
    if (toastUntil_ && millis() > toastUntil_) {
        hide(lblToast_, true);
        toastUntil_ = 0;
    }
    updateHeader(s, p);
    status_.mode = mode_;
    status_.hold = hold_;
    status_.touchLocked = touchLocked_;
    status_.targetAngleDeg = profile_.targetAngleDeg;
    status_.toleranceDeg = profile_.toleranceDeg;
    status_.profile = profileIdx_;
    if (hold_) return;  // frozen values stay on screen, clearly marked

    // defaults for every frame, modes override
    hide(lblXY_, false);
    hide(edgeLine_, true);
    hide(chart_, true);
    hide(progress_, mode_ != AppMode::Precision);
    if (mode_ != AppMode::Zero && mode_ != AppMode::Reference && mode_ != AppMode::Precision) hide(lblHint_, true);
    lv_obj_align(lblBig_, LV_ALIGN_CENTER, 0, 46);
    switch (mode_) {
        case AppMode::Drill: layoutTarget(true); updateDrill(s, dt); break;
        case AppMode::Level: updateLevel(s, dt); break;
        case AppMode::Angle: updateAngle(s, dt); break;
        case AppMode::Zero: updateZero(s, dt); break;
        case AppMode::Reference: updateReference(s, dt); break;
        case AppMode::Precision: updatePrecision(s, dt); break;
        default: break;
    }
}

void MainScreen::primaryAction() {
    if (!app_) return;
    switch (mode_) {
        case AppMode::Drill:
            if (angleReferenceNeedsZero(profile_.angleRef)) {
                // angle relative to a zero point: capture the reference axis (3D zero
                // contains the drift-free tilt part as well) and stay in drill mode
                app_->meas.captureFull3D();
                showToast("Bezugsachse gesetzt");
                break;
            }
            // fall through: absolute angle -> classic zero point mode
            [[fallthrough]];
        case AppMode::Zero:
            if (settings_.measurement.zeroKind == ZeroKind::Full3D) app_->meas.captureFull3D();
            else app_->meas.captureTiltZero();
            if (mode_ == AppMode::Drill) {
                UiCommand c{};
                c.type = UiCommand::Type::SetMode;
                c.i = static_cast<int32_t>(AppMode::Zero);
                app_->postUi(c);
            }
            showToast("Nullpunkt gesetzt");
            break;
        case AppMode::Level: {
            OrientationSnapshot s;
            app_->meas.snapshot(s);
            if (s.reference.kind() == ReferenceKind::TiltZero) {
                app_->meas.clearReference();
                showToast("Nullpunkt gelöscht");
            } else {
                app_->meas.captureTiltZero();
                showToast("Relativ-Nullpunkt");
            }
            break;
        }
        case AppMode::Angle:
            if (std::isfinite(angleRefDeg_)) {
                angleRefDeg_ = NAN;
                showToast("Absolute Messung");
            } else {
                OrientationSnapshot s;
                app_->meas.snapshot(s);
                angleRefDeg_ = drillGeometry(s.qWD, profile_.mount).elevationDeg;
                showToast("Referenzwinkel gesetzt");
            }
            fv_.reset();
            break;
        case AppMode::Reference:
            app_->meas.captureSurface(settings_.measurement.reference3D);
            showToast("Fläche erfasst.\nGerät montieren.");
            break;
        case AppMode::Precision:
            app_->meas.startPrecision(settings_.measurement.precisionWindowSec, PrecisionQuantity::AxisInclination);
            break;
        default: break;
    }
}
