// SPDX-License-Identifier: MIT
// Sensor base calibration (independent of the tool/mount):
//   * gyroscope zero-rate offset (bias) + noise, optionally at several temperatures
//   * accelerometer offset + scale (6 position), optional full 3x3 matrix
//   * temperature compensation by piecewise linear interpolation between points
//
// The calibration operates on DEVICE frame data (after the fixed axis mapping).
// Model:   gyro_cal  = gyro_raw - bias(T)
//          accel_cal = M(T) * accel_raw + c(T)
#pragma once

#include <cstdint>

#include "MathTypes.h"

namespace sda {

constexpr int kMaxGyroCalPoints = 6;
constexpr int kMaxAccelCalPoints = 4;

struct GyroCalPoint {
    float tempC = NAN;
    Vec3 biasDps;
    Vec3 noiseDps;      ///< standard deviation per axis during calibration
    uint32_t samples = 0;
};

struct AccelCalPoint {
    float tempC = NAN;
    Mat3 matrix;        ///< correction matrix M
    Vec3 offset;        ///< additive term c (g)
    float residualMg = -1.0f;  ///< RMS of | |a_cal| - 1 g | over the calibration poses
    bool extended = false;     ///< true: full 3x3 (misalignment/cross coupling)
};

/// Result of interpolating a temperature table.
struct TempLookup {
    bool valid = false;        ///< at least one point available
    bool compensated = false;  ///< >=2 points and temperature inside/near the range
    float distanceC = 0.0f;    ///< distance outside the calibrated range (0 = inside)
    int points = 0;
};

class SensorCalibration {
public:
    SensorCalibration() = default;

    // --- gyro -----------------------------------------------------------------
    /// Adds/replaces a gyro calibration point. Points closer than mergeC are replaced.
    void addGyroPoint(const GyroCalPoint& p, float mergeC = 1.5f);
    int gyroPointCount() const { return gyroCount_; }
    const GyroCalPoint& gyroPoint(int i) const { return gyro_[i]; }
    Vec3 gyroBias(float tempC, TempLookup* info = nullptr) const;
    Vec3 gyroNoise() const;  ///< noise of the most recent / nearest point

    // --- accel ----------------------------------------------------------------
    void addAccelPoint(const AccelCalPoint& p, float mergeC = 3.0f);
    int accelPointCount() const { return accelCount_; }
    const AccelCalPoint& accelPoint(int i) const { return accel_[i]; }
    /// Returns interpolated correction (identity if no calibration).
    void accelCorrection(float tempC, Mat3& m, Vec3& c, TempLookup* info = nullptr) const;

    // --- apply ----------------------------------------------------------------
    Vec3 applyGyro(const Vec3& rawDps, float tempC) const;
    Vec3 applyAccel(const Vec3& rawG, float tempC) const;

    bool hasGyro() const { return gyroCount_ > 0; }
    bool hasAccel() const { return accelCount_ > 0; }
    /// Temperature compensation only counts as active with >= 2 points spanning >= 3 K.
    bool gyroTempCompActive() const;
    bool accelTempCompActive() const;

    void clearGyro() { gyroCount_ = 0; }
    void clearAccel() { accelCount_ = 0; }
    void clearAll() { clearGyro(); clearAccel(); }

    /// Plausibility check (used after loading from flash).
    bool validate() const;

    // raw access for (de)serialization
    void setGyroPoints(const GyroCalPoint* p, int n);
    void setAccelPoints(const AccelCalPoint* p, int n);

private:
    GyroCalPoint gyro_[kMaxGyroCalPoints];
    int gyroCount_ = 0;
    AccelCalPoint accel_[kMaxAccelCalPoints];
    int accelCount_ = 0;
    void sortGyro();
    void sortAccel();
};

// ============================================================================
// Gyro calibration procedure (still-stand averaging)
// ============================================================================

struct GyroCalConfig {
    uint32_t samples = 600;        ///< samples to average (~2.7 s at 224 Hz)
    float maxMotionDps = 2.0f;     ///< |gyro - running mean| above -> restart
    float maxAccelDevG = 0.02f;    ///< accel change vs. start above -> restart
    float maxNoiseDps = 0.6f;      ///< per-axis std above -> reject
    float maxAbsBiasDps = 20.0f;   ///< implausible bias -> reject
    uint8_t maxRestarts = 20;
};

class GyroCalibrator {
public:
    enum class State : uint8_t { Idle, Collecting, Done, Failed };
    enum class Failure : uint8_t { None, TooMuchMotion, TooNoisy, Implausible, InvalidData };

    void start(const GyroCalConfig& cfg = GyroCalConfig());
    /// Feed raw (device frame, uncalibrated) samples.
    void addSample(const Vec3& rawGyroDps, const Vec3& rawAccelG, float tempC);
    State state() const { return state_; }
    Failure failure() const { return failure_; }
    float progress() const;
    uint8_t restarts() const { return restarts_; }
    bool motionDetected() const { return motionFlag_; }  ///< last sample restarted the run
    const GyroCalPoint& result() const { return result_; }
    void cancel() { state_ = State::Idle; }

private:
    GyroCalConfig cfg_;
    State state_ = State::Idle;
    Failure failure_ = Failure::None;
    uint32_t n_ = 0;
    double sum_[3] = {0, 0, 0};
    double sumSq_[3] = {0, 0, 0};
    double tempSum_ = 0;
    uint32_t tempN_ = 0;
    Vec3 accelRef_;
    uint8_t restarts_ = 0;
    bool motionFlag_ = false;
    GyroCalPoint result_;
    void restart();
};

// ============================================================================
// Accelerometer 6-position calibration
// ============================================================================

enum class AccelPose : uint8_t { PosX = 0, NegX, PosY, NegY, PosZ, NegZ };
constexpr int kAccelPoseCount = 6;
/// Expected measured specific force (device frame) for each pose: the named axis points UP.
Vec3 accelPoseExpected(AccelPose p);
const char* accelPoseName(AccelPose p);

struct AccelCalConfig {
    uint32_t samplesPerPose = 300;
    float maxSampleDevG = 0.015f;   ///< |a - running mean| above -> restart pose
    float maxGyroDps = 3.0f;        ///< rotation above -> restart pose
    float maxPoseTiltDeg = 20.0f;   ///< measured direction vs. expected axis
    bool extended = false;          ///< solve full 3x3 matrix (requires accurate square block)
};

class AccelCalibrator {
public:
    enum class PoseState : uint8_t { Pending, Collecting, Accepted };
    enum class Failure : uint8_t { None, WrongOrientation, Unstable, SolveFailed, Implausible };

    void start(const AccelCalConfig& cfg = AccelCalConfig());
    /// Begin collecting the given pose (user confirmed placement).
    void beginPose(AccelPose p);
    /// Feed raw device frame samples while collecting.
    void addSample(const Vec3& rawAccelG, const Vec3& rawGyroDps, float tempC);
    PoseState poseState(AccelPose p) const { return poses_[int(p)].state; }
    bool collecting() const { return active_ >= 0; }
    AccelPose activePose() const { return AccelPose(active_ < 0 ? 0 : active_); }
    float poseProgress() const;
    bool allPosesAccepted() const;
    Failure lastFailure() const { return failure_; }
    /// Detects which pose the device is currently in (for UI guidance), -1 = none.
    static int detectPose(const Vec3& rawAccelG, float maxTiltDeg = 20.0f);

    /// Solves the calibration; requires all poses. Fills `out` and returns true on success.
    bool solve(AccelCalPoint& out);
    Vec3 poseMean(AccelPose p) const { return poses_[int(p)].mean; }

private:
    struct Pose {
        PoseState state = PoseState::Pending;
        Vec3 mean;
        Vec3 stddev;
        float tempC = NAN;
    };
    AccelCalConfig cfg_;
    Pose poses_[kAccelPoseCount];
    int active_ = -1;
    uint32_t n_ = 0;
    double sum_[3] = {0, 0, 0};
    double sumSq_[3] = {0, 0, 0};
    double tempSum_ = 0;
    uint32_t tempN_ = 0;
    Vec3 runMean_;
    Failure failure_ = Failure::None;
    void resetAccumulators();
};

}  // namespace sda
