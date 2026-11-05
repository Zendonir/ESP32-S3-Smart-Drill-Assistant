// SPDX-License-Identifier: MIT
#include "MotionAnalysis.h"

namespace sda {

const char* motionStateName(MotionState s) {
    switch (s) {
        case MotionState::Still: return "Stillstand";
        case MotionState::Slow: return "Langsam";
        case MotionState::Fast: return "Schnell";
    }
    return "?";
}

const char* vibrationLevelName(VibrationLevel v) {
    switch (v) {
        case VibrationLevel::None: return "Keine";
        case VibrationLevel::Light: return "Leicht";
        case VibrationLevel::Strong: return "Stark";
    }
    return "?";
}

namespace {
inline float emaAlpha(float dt, float tau) { return tau > 1e-6f ? clamp01(dt / (tau + dt)) : 1.0f; }
inline float ramp(float v, float start, float zero) {
    if (v <= start) return 1.0f;
    if (v >= zero) return 0.0f;
    return 1.0f - (v - start) / (zero - start);
}
}  // namespace

// ----------------------------------------------------------------------------
void MotionDetector::setNoise(float gyroNoiseDps, float accelNoiseG) {
    if (std::isfinite(gyroNoiseDps) && gyroNoiseDps > 0) gyroNoise_ = gyroNoiseDps;
    if (std::isfinite(accelNoiseG) && accelNoiseG > 0) accelNoise_ = accelNoiseG;
}

float MotionDetector::gyroThresholdDps() const { return std::fmax(cfg_.stillGyroDps, 5.0f * gyroNoise_); }
float MotionDetector::accelStdThresholdG() const {
    return std::fmax(cfg_.stillAccelStdG, 4.0f * accelNoise_);
}

void MotionDetector::reset() {
    init_ = false;
    var_ = 0;
    candTime_ = 0;
    stillTime_ = 0;
    state_ = MotionState::Fast;
}

void MotionDetector::update(const Vec3& gyroDps, const Vec3& accelG, float dt) {
    if (!(dt > 0.0f)) return;
    if (!init_) {
        mean_ = accelG;
        holdRef_ = accelG.normalized();
        var_ = 0.0f;
        gyroRef_ = gyroCalibrated_ ? Vec3() : gyroDps;
        normRef_ = accelG.norm();
        init_ = true;
    }
    const float a = emaAlpha(dt, cfg_.windowSec);
    const Vec3 d = accelG - mean_;
    mean_ += d * a;
    // exponentially weighted variance of the vector (sum over axes)
    var_ = (1.0f - a) * (var_ + a * d.norm2());
    const float an = accelG.norm();
    absNormDev_ = std::fabs(an - 1.0f);

    // Rotation rate relative to a slowly tracked offset. Without a gyro
    // calibration the zero-rate offset (QMI8658 typ. a few deg/s) would otherwise
    // look like a permanent rotation and still-stand could never be detected -
    // which the calibrations themselves need. Slow rotations that hide in the
    // reference are caught by the gravity-direction criterion below.
    const Vec3 rel = gyroDps - gyroRef_;
    // calibrated: the absolute rate is meaningful, the residual reference only
    // absorbs small leftover offsets (+/-0.5 deg/s)
    rate_ = rel.norm();
    const float r = emaAlpha(dt, cfg_.referenceTauSec);
    if (rel.norm() < 5.0f) {
        gyroRef_ += rel * r;
        const float lim = gyroCalibrated_ ? 0.5f : 25.0f;
        gyroRef_ = Vec3(clampv(gyroRef_.x, -lim, lim), clampv(gyroRef_.y, -lim, lim), clampv(gyroRef_.z, -lim, lim));
    }
    // |a| relative to its slow reference: independent of accel offset/scale errors
    normDev_ = std::fabs(an - normRef_);
    if (normDev_ < 0.1f) normRef_ += (an - normRef_) * r;

    blockers_ = 0;
    if (rate_ >= gyroThresholdDps()) blockers_ |= 1;
    if (accelStdG() >= accelStdThresholdG()) blockers_ |= 2;
    if (normDev_ >= cfg_.stillAccelNormDevG || absNormDev_ >= cfg_.sanityNormDevG) blockers_ |= 4;
    const bool quiet = blockers_ == 0;
    if (quiet) {
        if (candTime_ == 0.0f) holdRef_ = mean_.normalized();
        candTime_ += dt;
        const float drift = rad2deg(angleBetween(holdRef_, mean_.normalized()));
        if (drift > cfg_.stillDirectionDeg) {
            // slow creeping rotation: restart the hold window
            candTime_ = dt;
            holdRef_ = mean_.normalized();
            blockers_ |= 8;
        }
    } else {
        candTime_ = 0.0f;
    }

    if (candTime_ >= cfg_.stillHoldSec) {
        state_ = MotionState::Still;
        stillTime_ += dt;
    } else {
        stillTime_ = 0.0f;
        // exit from Still is immediate on violation (responsiveness first)
        state_ = rate_ > cfg_.fastGyroDps ? MotionState::Fast : MotionState::Slow;
    }
}

// ----------------------------------------------------------------------------
void VibrationDetector::reset() {
    init_ = false;
    ms_ = gms_ = peak_ = 0.0f;
    lowTime_ = 0.0f;
    level_ = VibrationLevel::None;
}

void VibrationDetector::update(const Vec3& accelG, const Vec3& gyroDps, float dt) {
    if (!(dt > 0.0f)) return;
    if (!init_) {
        lp_ = accelG;
        glp_ = gyroDps;
        init_ = true;
    }
    const float tau = 1.0f / (2.0f * kPi * cfg_.hpCutoffHz);
    const float a = emaAlpha(dt, tau);
    lp_ += (accelG - lp_) * a;
    glp_ += (gyroDps - glp_) * a;
    const Vec3 hp = accelG - lp_;
    const Vec3 ghp = gyroDps - glp_;
    const float b = emaAlpha(dt, cfg_.rmsTimeConstSec);
    ms_ += (hp.norm2() - ms_) * b;
    gms_ += (ghp.norm2() - gms_) * b;
    const float mag = hp.norm();
    peak_ = std::fmax(mag, peak_ * (1.0f - b));

    const float rms = rmsMg();
    VibrationLevel target = VibrationLevel::None;
    if (rms >= cfg_.strongMg) target = VibrationLevel::Strong;
    else if (rms >= cfg_.lightMg) target = VibrationLevel::Light;

    if (target > level_) {
        level_ = target;  // escalate immediately
        lowTime_ = 0.0f;
    } else if (target < level_) {
        lowTime_ += dt;   // de-escalate only after release time
        if (lowTime_ >= cfg_.releaseSec) {
            level_ = target;
            lowTime_ = 0.0f;
        }
    } else {
        lowTime_ = 0.0f;
    }
}

// ----------------------------------------------------------------------------
const char* drillActivityName(DrillActivity a) {
    switch (a) {
        case DrillActivity::Holding: return "ruhig gehalten";
        case DrillActivity::Aligning: return "wird ausgerichtet";
        case DrillActivity::Vibrating: return "vibriert stark";
        case DrillActivity::Settling: return "steht wieder";
    }
    return "?";
}

DrillActivity DrillActivityTracker::update(MotionState m, VibrationLevel v, float dt) {
    if (v == VibrationLevel::Strong) {
        state_ = DrillActivity::Vibrating;
        sinceVibration_ = 0.0f;
        return state_;
    }
    sinceVibration_ += dt;
    if (m == MotionState::Still) {
        // "settled" for 3 s after a vibration phase, then plain holding
        state_ = sinceVibration_ < 3.0f ? DrillActivity::Settling : DrillActivity::Holding;
    } else {
        state_ = DrillActivity::Aligning;
    }
    return state_;
}

// ----------------------------------------------------------------------------
float accelTrust(const AccelTrustConfig& c, float normDevG, float vibRmsMg, float rateDps,
                 bool clipped) {
    if (clipped) return 0.0f;
    return ramp(normDevG, c.normDevStartG, c.normDevZeroG) * ramp(vibRmsMg, c.vibStartMg, c.vibZeroMg) *
           ramp(rateDps, c.rateStartDps, c.rateZeroDps);
}

}  // namespace sda
