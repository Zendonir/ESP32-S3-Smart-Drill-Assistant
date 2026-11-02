// SPDX-License-Identifier: MIT
// Exchangeable attitude filters (accelerometer + gyroscope, no magnetometer).
//
// All filters estimate q_WD (device -> world, world Z = up). With 6 axes only the
// tilt (2 DOF) is observable from gravity; the heading (rotation about world Z) is
// pure gyro integration and drifts. Filters must therefore never "correct" the
// heading from accelerometer data - all corrections below act about horizontal axes.
//
// The correction strength is passed in every step as a *fraction* k (0..1) of the
// tilt error to remove in this step. k is computed by the measurement pipeline from
// a scalar tilt-uncertainty model (Kalman-style gain), the accelerometer trust
// weight and the configured filter limits. This keeps the adaptive logic in one
// place and the individual filters small and testable.
#pragma once

#include <cstdint>

#include "MathTypes.h"

namespace sda {

enum class FusionType : uint8_t { Mahony = 0, Complementary = 1, Madgwick = 2 };
const char* fusionTypeName(FusionType t);

struct FusionInput {
    Vec3 gyroRadS;      ///< calibrated angular rate, device frame
    Vec3 accelG;        ///< calibrated specific force, device frame
    float dt = 0.0f;    ///< seconds
    float gain = 0.0f;  ///< fraction of the tilt error to correct in this step (0..1)
};

class IFusionFilter {
public:
    virtual ~IFusionFilter() = default;
    /// Initializes the attitude from a gravity measurement (heading = 0).
    virtual void reset(const Vec3& accelG);
    virtual void update(const FusionInput& in) = 0;
    virtual FusionType type() const = 0;
    const Quat& orientation() const { return q_; }
    /// Overwrites the orientation (e.g. after an external reset).
    void setOrientation(const Quat& q) { q_ = q.normalized(); }
    /// Up direction (world +Z) expressed in device coordinates.
    Vec3 upInDevice() const { return q_.rotateInverse(Vec3::unitZ()); }
    /// Last tilt error between measured and estimated up vector (radians).
    float lastTiltErrorRad() const { return lastErr_; }

protected:
    Quat q_;
    float lastErr_ = 0.0f;
    void integrateGyro(const Vec3& omegaRadS, float dt);
};

/// Mahony style explicit complementary filter on SO(3): the cross product of the
/// measured and estimated gravity direction drives a proportional (and optional
/// integral) rate correction.
class MahonyFilter : public IFusionFilter {
public:
    void update(const FusionInput& in) override;
    FusionType type() const override { return FusionType::Mahony; }
    void setIntegralGain(float ki) { ki_ = ki; }
    void reset(const Vec3& accelG) override;
    Vec3 integralBias() const { return integral_; }

private:
    float ki_ = 0.0f;
    Vec3 integral_;
};

/// Quaternion complementary filter: gyro prediction followed by a fractional
/// rotation (exact, also for large errors) towards the measured gravity direction.
class ComplementaryFilter : public IFusionFilter {
public:
    void update(const FusionInput& in) override;
    FusionType type() const override { return FusionType::Complementary; }
};

/// Madgwick gradient descent filter (IMU variant). `in.gain` is mapped to the
/// beta parameter via beta = gain / dt * 0.5 (comparable correction speed).
class MadgwickFilter : public IFusionFilter {
public:
    void update(const FusionInput& in) override;
    FusionType type() const override { return FusionType::Madgwick; }
};

/// Holds all filter implementations without dynamic allocation; switching the
/// active filter keeps the current orientation.
class FusionBank {
public:
    void select(FusionType t);
    IFusionFilter& active() { return *active_; }
    const IFusionFilter& active() const { return *active_; }
    FusionType type() const { return active_->type(); }

private:
    MahonyFilter mahony_;
    ComplementaryFilter comp_;
    MadgwickFilter madgwick_;
    IFusionFilter* active_ = &mahony_;
};

/// Scalar uncertainty model for the tilt estimate (1-sigma, radians) and for the
/// unobservable heading. It is a deliberately simple, documented model - its output
/// is labelled as an *estimate* everywhere in the UI.
struct UncertaintyConfig {
    float accelNoiseRad = 0.003f;       ///< default per-sample accel tilt noise (overridden by measurement)
    float gyroNoiseDps = 0.05f;         ///< rate noise density proxy (per sample)
    float gyroScaleError = 0.005f;      ///< relative scale/cross-axis error of gyro integration
    float minStepGain = 0.0005f;        ///< lower bound for k when accel is trusted
    float maxStepGain = 0.2f;           ///< upper bound for k
    float initialTiltRad = 0.2f;        ///< uncertainty right after reset
};

class UncertaintyModel {
public:
    void configure(const UncertaintyConfig& c) { cfg_ = c; }
    const UncertaintyConfig& config() const { return cfg_; }
    void reset();
    /// Prediction with gyro integration. `biasSigmaDps` = current bias uncertainty.
    void predict(float dt, float rateRadS, float biasSigmaDps);
    /// Measurement update with the accelerometer. `weight` 0..1 = trust in accel,
    /// `measNoiseRad` = measured accel tilt noise. Returns the step gain k.
    float correct(float weight, float measNoiseRad, float maxGainOverride = -1.0f);
    float tiltSigmaRad() const { return std::sqrt(p_); }
    float headingSigmaRad() const { return yawSigma_; }
    void resetHeading() { yawSigma_ = 0.0f; }

private:
    UncertaintyConfig cfg_;
    float p_ = 0.04f;        ///< tilt variance rad^2
    float yawSigma_ = 0.0f;  ///< heading uncertainty (linear accumulation)
};

}  // namespace sda
