// SPDX-License-Identifier: MIT
// Guided calibration wizards operated by touch:
//   * gyroscope zero offset (hold still, automatic restart on motion)
//   * accelerometer 6 position calibration (orientation detected automatically,
//     each pose accepted only when still)
//   * mount calibration: roll fit (V-block, >= 3 positions) or vertical reference
#pragma once

#include <lvgl.h>

#include <functional>

#include "../system/AppContext.h"

class CalibrationScreen {
public:
    enum class Kind { Gyro, Accel, AccelExtended, MountRoll, MountVertical, AxisDetect };
    void open(AppContext& app, Kind kind, std::function<void()> onDone);
    void update(const sda::OrientationSnapshot& s);
    bool isOpen() const { return scr_ != nullptr; }
    void close();
    lv_obj_t* obj() const { return scr_; }

private:
    enum class Phase { Running, Result, Failed };
    AppContext* app_ = nullptr;
    Kind kind_ = Kind::Gyro;
    Phase phase_ = Phase::Running;
    std::function<void()> onDone_;
    lv_obj_t* scr_ = nullptr;
    lv_obj_t* arc_ = nullptr;
    lv_obj_t* title_ = nullptr;
    lv_obj_t* text_ = nullptr;
    lv_obj_t* detail_ = nullptr;
    lv_obj_t* btnLeft_ = nullptr;
    lv_obj_t* btnRight_ = nullptr;
    lv_obj_t* btnMid_ = nullptr;
    lv_obj_t* poseDots_[6] = {};
    uint32_t lastResultSeq_ = 0;
    uint8_t lastMountPositions_ = 0;
    bool solving_ = false;
    sda::SensorCalibration before_;
    sda::AccelCalPoint accelResult_;
    sda::MountFitResult mountResult_;
    bool haveBefore_ = false;
    // axis detection
    int axisStep_ = 0;
    int axisFrames_ = 0;
    sda::Vec3 axisSum_;
    sda::Vec3 axisFlatS_;
    sda::AxisMap axisOld_;
    sda::AxisDetectResult axisResult_;
    // step confirmation: measurements only start after "Start"/"Weiter"
    bool awaitConfirm_ = false;
    uint8_t lastPoseMask_ = 0;
    void askConfirm(const char* title, const char* nextStep, const char* button);
    void updateAxis(const sda::OrientationSnapshot& s);

    lv_obj_t* button(const char* text, int x, int y, int w, lv_event_cb_t cb);
    void setButtons(const char* left, const char* mid, const char* right);
    void updateGyro(const sda::OrientationSnapshot& s);
    void updateAccel(const sda::OrientationSnapshot& s);
    void updateMount(const sda::OrientationSnapshot& s);
    void onLeft();
    void onMid();
    void onRight();
    void finish(bool saved);
};
