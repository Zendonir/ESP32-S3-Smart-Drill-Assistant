// SPDX-License-Identifier: MIT
// Motion state (still / slow / fast), vibration analysis and accelerometer trust.
//
// Still-stand is derived from several independent quantities and must hold for a
// minimum time (hysteresis on entry and exit):
//   * calibrated gyro rate magnitude
//   * short-time standard deviation of the accelerometer vector
//   * deviation of |a| from 1 g
//   * change of the low-pass gravity direction over the hold window
// Vibration is the RMS of the high-pass filtered accelerometer magnitude.
//
// NOTE: the QMI8658 is configured with an internal low-pass (~13 % of ODR, i.e.
// ~30 Hz at 224 Hz). The vibration measure is therefore band limited; drill
// vibrations at hundreds of Hz are attenuated and partly aliased. The measure is a
// relative indicator of disturbance of the gravity measurement, not a calibrated
// vibration spectrum.
#pragma once

#include <cstdint>

#include "MathTypes.h"

namespace sda {

enum class MotionState : uint8_t { Still = 0, Slow = 1, Fast = 2 };
enum class VibrationLevel : uint8_t { None = 0, Light = 1, Strong = 2 };
const char* motionStateName(MotionState s);
const char* vibrationLevelName(VibrationLevel v);

struct MotionConfig {
    float stillGyroDps = 0.8f;       ///< floor; actual = max(floor, 5 * gyro noise)
    float stillAccelStdG = 0.004f;   ///< floor; actual = max(floor, 4 * accel noise)
    float stillAccelNormDevG = 0.03f;
    float stillDirectionDeg = 0.25f; ///< max drift of gravity direction during hold
    float stillHoldSec = 0.6f;       ///< time all criteria must hold
    float fastGyroDps = 25.0f;       ///< above -> Fast
    float windowSec = 0.25f;         ///< EMA time constant for short-time statistics
};

struct VibrationConfig {
    float hpCutoffHz = 2.0f;        ///< high pass separating orientation changes from vibration
    float rmsTimeConstSec = 0.2f;
    float lightMg = 20.0f;
    float strongMg = 80.0f;
    float releaseSec = 0.5f;        ///< level must stay low this long before decreasing
};

class MotionDetector {
public:
    void configure(const MotionConfig& c) { cfg_ = c; }
    const MotionConfig& config() const { return cfg_; }
    /// Noise levels from the gyro calibration (used to adapt thresholds).
    void setNoise(float gyroNoiseDps, float accelNoiseG);
    void reset();
    void update(const Vec3& gyroDps, const Vec3& accelG, float dt);

    MotionState state() const { return state_; }
    bool isStill() const { return state_ == MotionState::Still; }
    float stillSeconds() const { return stillTime_; }
    float gyroRateDps() const { return rate_; }
    float accelStdG() const { return std::sqrt(var_ > 0 ? var_ : 0); }
    float accelNormDevG() const { return normDev_; }
    float gyroThresholdDps() const;
    float accelStdThresholdG() const;

private:
    MotionConfig cfg_;
    float gyroNoise_ = 0.05f;
    float accelNoise_ = 0.001f;
    bool init_ = false;
    Vec3 mean_;
    float var_ = 0.0f;
    float rate_ = 0.0f;
    float normDev_ = 0.0f;
    Vec3 holdRef_;
    float candTime_ = 0.0f;
    float stillTime_ = 0.0f;
    MotionState state_ = MotionState::Fast;
};

class VibrationDetector {
public:
    void configure(const VibrationConfig& c) { cfg_ = c; }
    const VibrationConfig& config() const { return cfg_; }
    void reset();
    void update(const Vec3& accelG, const Vec3& gyroDps, float dt);
    float rmsMg() const { return std::sqrt(ms_) * 1000.0f; }
    float peakMg() const { return peak_ * 1000.0f; }
    float gyroRmsDps() const { return std::sqrt(gms_); }
    VibrationLevel level() const { return level_; }

private:
    VibrationConfig cfg_;
    bool init_ = false;
    Vec3 lp_;
    Vec3 glp_;
    float ms_ = 0.0f;
    float gms_ = 0.0f;
    float peak_ = 0.0f;
    float lowTime_ = 0.0f;
    VibrationLevel level_ = VibrationLevel::None;
};

/// Activity of the tool derived ONLY from IMU data (requirement 7.4). This is an
/// interpretation of motion/vibration - it does not detect the motor state.
enum class DrillActivity : uint8_t {
    Holding = 0,     ///< still, no vibration
    Aligning = 1,    ///< moving without vibration
    Vibrating = 2,   ///< strong vibration (e.g. drilling)
    Settling = 3,    ///< vibration ended recently, back to still
};
const char* drillActivityName(DrillActivity a);

class DrillActivityTracker {
public:
    DrillActivity update(MotionState m, VibrationLevel v, float dt);
    DrillActivity state() const { return state_; }

private:
    DrillActivity state_ = DrillActivity::Aligning;
    float sinceVibration_ = 1e6f;
};

/// Trust in the accelerometer as gravity reference (0..1). Combines |a|-deviation,
/// vibration RMS, rotation rate (centripetal/tangential accelerations) and clipping.
struct AccelTrustConfig {
    float normDevStartG = 0.015f;
    float normDevZeroG = 0.12f;
    float vibStartMg = 15.0f;
    float vibZeroMg = 150.0f;
    float rateStartDps = 20.0f;
    float rateZeroDps = 250.0f;
};
float accelTrust(const AccelTrustConfig& c, float normDevG, float vibRmsMg, float rateDps,
                 bool clipped);

}  // namespace sda
