// SPDX-License-Identifier: MIT
// Main measurement screen with all operating modes. Graphical layout for the
// round 240x240 panel:
//
//          mode + target            (y ~ 20)
//        quality  |  battery         (y ~ 40)
//     -----  target circle  -----   radius 100 px, crosshair, tolerance ring,
//             moving marker          edge arrow if the deviation exceeds the scale
//          big deviation value       (y ~ 165)
//          X / Y components          (y ~ 197)
//          tolerance | scale         (y ~ 216)
//
// Colour is always accompanied by geometry: locked = filled ring + check mark,
// near = ring outline, far = small ring + edge arrow, invalid = hollow grey marker
// with dashes instead of numbers.
#pragma once

#include <lvgl.h>

#include "../system/AppContext.h"
#include "DisplayLogic.h"
#include "MeasurementPipeline.h"
#include "Orientation.h"
#include "Settings.h"
#include "Statistics.h"

class MainScreen {
public:
    void create(AppContext& app);
    lv_obj_t* obj() const { return scr_; }

    void setMode(sda::AppMode m);
    sda::AppMode mode() const { return mode_; }
    void setProfile(const sda::Profile& p, uint8_t idx);
    void setSettings(const sda::AppSettings& s);

    /// Per frame update with the newest snapshot (ignored while hold is active).
    void update(const sda::OrientationSnapshot& s, const PowerStatus& p, float dt);
    void setHold(bool on);
    bool hold() const { return hold_; }
    /// Long press action of the current mode (zero / capture / start).
    void primaryAction();
    void setTouchLocked(bool locked);
    void showToast(const char* text, uint32_t ms = 1500);

    UiStatus status() const { return status_; }
    /// Value of the current mode for "save value" while holding.
    float currentValue() const { return status_.valueDeg; }

private:
    struct HistPoint {
        float x, y, total, sigma, vib;
    };

    AppContext* app_ = nullptr;
    lv_obj_t* scr_ = nullptr;
    // target graphics
    lv_obj_t* rings_[3] = {};
    lv_obj_t* outer_ = nullptr;
    lv_obj_t* crossH_ = nullptr;
    lv_obj_t* crossV_ = nullptr;
    lv_obj_t* tolRing_ = nullptr;
    lv_obj_t* dot_ = nullptr;
    lv_obj_t* check_ = nullptr;
    lv_obj_t* arrow_ = nullptr;
    lv_obj_t* edgeLine_ = nullptr;
    lv_obj_t* holdRing_ = nullptr;
    lv_obj_t* progress_ = nullptr;
    lv_obj_t* chart_ = nullptr;
    lv_chart_series_t* series_ = nullptr;
    // labels
    lv_obj_t* lblMode_ = nullptr;
    lv_obj_t* lblQuality_ = nullptr;
    lv_obj_t* lblBattery_ = nullptr;
    lv_obj_t* lblBig_ = nullptr;
    lv_obj_t* lblXY_ = nullptr;
    lv_obj_t* lblInfo_ = nullptr;
    lv_obj_t* lblHint_ = nullptr;
    lv_obj_t* lblLock_ = nullptr;
    lv_obj_t* lblToast_ = nullptr;
    lv_point_t crossHPts_[2];
    lv_point_t crossVPts_[2];
    lv_point_t arrowPts_[3];
    lv_point_t edgePts_[2];

    // state
    sda::AppMode mode_ = sda::AppMode::Drill;
    sda::Profile profile_;
    uint8_t profileIdx_ = 0;
    sda::AppSettings settings_;
    bool hold_ = false;
    bool touchLocked_ = false;
    uint32_t toastUntil_ = 0;
    sda::HemisphereState hemi_;
    sda::TargetTracker tracker_;
    sda::AutoScaler autoScale_;
    sda::OneEuroFilter fx_, fy_, fv_;
    sda::StableNumber nBig_{0.1f}, nX_{0.1f}, nY_{0.1f};
    sda::RingBuffer<HistPoint, 600> history_;
    float histAccum_ = 0.0f;
    float angleRefDeg_ = NAN;      ///< angle mode: relative reference (NaN = absolute)
    sda::RepeatabilityTracker repeat_;
    uint32_t lastPrecisionSeq_ = 0;
    int lastTargetState_ = -1;
    UiStatus status_;

    // views
    void layoutTarget(bool visible);
    void updateDeviationView(float devX, float devY, float total, bool valid, float tol, float dt,
                             bool lateralFree, bool lateralGyro);
    void updateDrill(const sda::OrientationSnapshot& s, float dt);
    void updateLevel(const sda::OrientationSnapshot& s, float dt);
    void updateAngle(const sda::OrientationSnapshot& s, float dt);
    void updateZero(const sda::OrientationSnapshot& s, float dt);
    void updateReference(const sda::OrientationSnapshot& s, float dt);
    void updatePrecision(const sda::OrientationSnapshot& s, float dt);
    void updateHeader(const sda::OrientationSnapshot& s, const PowerStatus& p);
    void pushHistory(float x, float y, float total, const sda::OrientationSnapshot& s, float dt);
    void refreshChart();
    void setMarkerStyle(sda::TargetState st, bool valid);
    void configureFilters();
    void fmtAngle(char* buf, size_t n, float v, bool sign) const;
    float scaleFor(float total, float dt);
};
