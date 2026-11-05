// SPDX-License-Identifier: MIT
// The complete, hardware independent measurement chain:
//
//   raw sample (sensor frame)
//     -> plausibility / clipping / stuck detection
//     -> timing (dt from sample counter, lost samples)
//     -> axis mapping S->D
//     -> base calibration (gyro bias(T) + online bias, accel M(T), c(T))
//     -> motion + vibration analysis, accel trust
//     -> uncertainty model -> adaptive correction gain
//     -> attitude filter (Mahony / complementary / Madgwick)
//     -> reference propagation, quality assessment
//     -> OrientationSnapshot (published to UI / network)
//
// Calibration procedures and precision measurements also run here, so that they
// see every sample at the full rate. On the device this class runs inside the
// fusion task only (single threaded); other tasks interact through commands and
// snapshot copies (see src/measurement/MeasurementService).
#pragma once

#include <cstdint>

#include "CoordinateTransforms.h"
#include "ImuTypes.h"
#include "MeasurementQuality.h"
#include "MotionAnalysis.h"
#include "MountCalibration.h"
#include "Orientation.h"
#include "ReferenceManager.h"
#include "SensorCalibration.h"
#include "SensorFusion.h"
#include "Statistics.h"
#include "TimingEstimator.h"

namespace sda {

struct PipelineConfig {
    AxisMap axisMap;
    float accelRangeG = 8.0f;
    float gyroRangeDps = 512.0f;
    float nominalRateHz = 224.2f;
    FusionType fusion = FusionType::Mahony;
    float fusionMinTauSec = 0.2f;        ///< from filter strength
    MotionConfig motion;
    VibrationConfig vibration;
    AccelTrustConfig trust;
    QualityConfig quality;
    UncertaintyConfig uncertainty;
    float tempCoeffAssumedDpsPerK = 0.02f;  ///< ASSUMPTION (no datasheet value verified)
    float biasFloorDps = 0.01f;
    bool onlineBiasTracking = true;
    float onlineBiasTauSec = 8.0f;
    float onlineBiasMaxDps = 1.5f;
};

enum class CalProcedure : uint8_t { None = 0, Gyro, Accel, MountRoll, MountVertical };

struct CalibrationStatus {
    CalProcedure procedure = CalProcedure::None;
    uint8_t state = 0;        ///< procedure specific (GyroCalibrator::State etc.)
    uint8_t failure = 0;
    float progress = 0.0f;
    int8_t detectedPose = -1; ///< accel: pose detected from current gravity
    int8_t activePose = -1;
    uint8_t acceptedPoseMask = 0;
    uint8_t mountPositions = 0;
    bool capturing = false;   ///< still-averaging in progress
    bool waitingStill = false;
    bool motionRestart = false;
    uint32_t resultSeq = 0;   ///< increments whenever a new result is available
};

enum class PrecisionQuantity : uint8_t { AxisInclination = 0, LevelXY = 1 };

struct OrientationSnapshot {
    uint32_t seq = 0;
    uint64_t timeUs = 0;
    bool valid = false;            ///< at least one fused sample

    Quat qWD;                      ///< fused orientation (device -> world)
    Vec3 upD;                      ///< fused up direction in device frame
    Vec3 accelG;                   ///< calibrated accel (device)
    Vec3 gyroDps;                  ///< calibrated gyro incl. online bias (device)
    Vec3 rawAccelG;                ///< uncalibrated (device frame)
    Vec3 rawGyroDps;
    float temperatureC = NAN;

    LevelAngles level;             ///< fused, gravity referenced
    float tiltSigmaDeg = 0.0f;     ///< ESTIMATED 1-sigma tilt uncertainty (statistical)
    float headingSigmaDeg = 0.0f;  ///< ESTIMATED heading uncertainty since last reset

    MotionState motion = MotionState::Fast;
    float stillSec = 0.0f;
    VibrationLevel vibration = VibrationLevel::None;
    DrillActivity activity = DrillActivity::Aligning;
    float vibrationRmsMg = 0.0f;
    float vibrationPeakMg = 0.0f;
    float accelTrust = 0.0f;
    float fusionGain = 0.0f;
    float accelNoiseG = 0.0f;
    float gyroRateDps = 0.0f;
    uint8_t stillBlockers = 0;     ///< see MotionDetector::stillBlockers()

    QualityAssessment quality;
    TimingStats timing;
    Vec3 onlineBiasDps;
    float biasSigmaDps = 0.0f;
    TempLookup gyroTemp;
    TempLookup accelTemp;
    bool hasGyroCal = false;
    bool hasAccelCal = false;
    uint32_t clipCount = 0;
    uint32_t implausibleCount = 0;
    float secondsSinceReset = 0.0f;
    FusionType fusion = FusionType::Mahony;

    ReferenceManager reference;    ///< copy (small)
    CalibrationStatus cal;
    bool precisionRunning = false;
    bool precisionWaiting = false;
    float precisionProgress = 0.0f;
    PrecisionResult precision;
    uint32_t precisionSeq = 0;
};

class MeasurementPipeline {
public:
    MeasurementPipeline();
    void configure(const PipelineConfig& cfg);
    const PipelineConfig& config() const { return cfg_; }
    void setCalibration(const SensorCalibration& cal);
    const SensorCalibration& calibration() const { return cal_; }
    void setMount(const MountModel& mount) { mount_ = mount; }
    const MountModel& mount() const { return mount_; }
    void setFusion(FusionType t);
    void setFilterTau(float minTauSec) { cfg_.fusionMinTauSec = minTauSec; }
    /// Sensor ODR changed (power saving) - no fusion reset.
    void setNominalRate(float hz) {
        cfg_.nominalRateHz = hz;
        timing_.changeNominalRate(hz);
    }

    /// Re-initializes fusion/motion state (keeps calibration and references).
    void reset();

    /// Processes one raw sample (sensor frame). Returns false if the sample was dropped.
    bool process(const ImuSample& raw);
    /// Reports that no sample arrived for a while / bus errors (from the driver task).
    void setDataTimeout(bool timeout);
    void setBusErrors(bool errors) { busErrors_ = errors; }

    const OrientationSnapshot& snapshot() const { return snap_; }

    // --- calibration procedures -------------------------------------------------
    void startGyroCalibration(const GyroCalConfig& c = GyroCalConfig());
    void startAccelCalibration(const AccelCalConfig& c = AccelCalConfig());
    void accelBeginPose(AccelPose p);
    bool accelSolve(AccelCalPoint& out);
    void startMountCalibration(CalProcedure p);   ///< MountRoll or MountVertical
    void mountCapturePosition();                  ///< still-average one position
    MountFitResult mountSolve(const Vec3& hintAxisD, bool bitDown = true);
    void cancelCalibration();
    /// Results (valid after status.resultSeq changed)
    const GyroCalPoint& gyroResult() const { return gyroResult_; }
    bool gyroResultOk() const { return gyroResultOk_; }

    /// Applies a finished gyro calibration point to the active calibration (RAM).
    void applyGyroPoint(const GyroCalPoint& p);
    void applyAccelPoint(const AccelCalPoint& p);

    // --- references / precision --------------------------------------------------
    void captureTiltZero();
    void captureFull3D();
    void captureSurface(bool use3D);
    void clearReference();
    void startPrecision(const PrecisionConfig& c, PrecisionQuantity q);
    void abortPrecision();

private:
    PipelineConfig cfg_;
    SensorCalibration cal_;
    MountModel mount_;
    TimingEstimator timing_;
    FusionBank fusion_;
    UncertaintyModel unc_;
    MotionDetector motion_;
    VibrationDetector vib_;
    DrillActivityTracker activity_;
    ReferenceManager ref_;
    GyroCalibrator gyroCal_;
    AccelCalibrator accelCal_;
    MountCalibrator mountCal_;
    PrecisionMeasurement precision_;
    PrecisionQuantity precisionQty_ = PrecisionQuantity::AxisInclination;

    OrientationSnapshot snap_;
    bool fusionInit_ = false;
    float sinceReset_ = 0.0f;
    bool dataTimeout_ = false;
    bool busErrors_ = false;
    Vec3 onlineBias_;
    float onlineStillSec_ = 0.0f;
    float biasRefTempC_ = NAN;
    float biasSigmaCal_ = 0.05f;
    float accelNoiseG_ = 0.0f;
    // stuck detection
    Vec3 lastRawA_, lastRawG_;
    uint32_t identicalCount_ = 0;
    uint32_t lossWindow_ = 0;      ///< samples since last loss event
    // calibration bookkeeping
    CalibrationStatus calStatus_;
    GyroCalPoint gyroResult_;
    bool gyroResultOk_ = false;
    // still averager for mount positions
    bool capturing_ = false;
    uint32_t capN_ = 0;
    Vec3 capSum_;
    Vec3 lastMountVector_;

    void updateBiasSigma(float tempC);
    void runCalibration(const Vec3& rawGyroD, const Vec3& rawAccelD, const Vec3& calAccelD,
                        const Vec3& calGyroD, float tempC);
    void runPrecision(const Vec3& calAccelD, float dt);
};

}  // namespace sda
