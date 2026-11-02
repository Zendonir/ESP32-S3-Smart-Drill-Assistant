// SPDX-License-Identifier: MIT
#include "SensorFusion.h"

namespace sda {

const char* fusionTypeName(FusionType t) {
    switch (t) {
        case FusionType::Mahony: return "Mahony";
        case FusionType::Complementary: return "Komplementär";
        case FusionType::Madgwick: return "Madgwick";
    }
    return "?";
}

// ----------------------------------------------------------------------------
void IFusionFilter::reset(const Vec3& accelG) {
    // Smallest rotation that maps the measured up direction (device) onto world +Z.
    // q_WD * a_D = +Z  ->  q_WD = fromTwoVectors(a_D, Z). Heading is arbitrary (0).
    const Vec3 a = accelG.normalized();
    q_ = Quat::fromTwoVectors(a, Vec3::unitZ());
    lastErr_ = 0.0f;
}

void IFusionFilter::integrateGyro(const Vec3& omega, float dt) {
    // Exact integration for constant rate during dt: q <- q * exp(omega*dt/2)
    q_ = (q_ * Quat::fromRotationVector(omega * dt)).normalized();
}

// ----------------------------------------------------------------------------
void MahonyFilter::reset(const Vec3& accelG) {
    IFusionFilter::reset(accelG);
    integral_ = Vec3();
}

void MahonyFilter::update(const FusionInput& in) {
    if (!(in.dt > 0.0f)) return;
    Vec3 omega = in.gyroRadS;
    const float an = in.accelG.norm();
    if (an > 0.1f && in.gain > 0.0f) {
        const Vec3 a = in.accelG / an;
        const Vec3 v = upInDevice();
        const Vec3 e = a.cross(v);  // horizontal axis, |e| = sin(error)
        lastErr_ = angleBetween(a, v);
        const float kp = in.gain / in.dt;
        if (ki_ > 0.0f) integral_ += e * (ki_ * in.dt);
        omega += e * kp + integral_;
    } else if (an > 0.1f) {
        lastErr_ = angleBetween(in.accelG / an, upInDevice());
    }
    integrateGyro(omega, in.dt);
}

// ----------------------------------------------------------------------------
void ComplementaryFilter::update(const FusionInput& in) {
    if (!(in.dt > 0.0f)) return;
    integrateGyro(in.gyroRadS, in.dt);
    const float an = in.accelG.norm();
    if (an < 0.1f) return;
    const Vec3 a = in.accelG / an;
    const Vec3 v = upInDevice();
    lastErr_ = angleBetween(a, v);
    if (in.gain <= 0.0f) return;
    // body-frame rotation Delta with Delta^T v = a, applied fractionally.
    const Quat delta = Quat::fromTwoVectors(a, v);
    const Vec3 rv = delta.toRotationVector() * clamp01(in.gain);
    q_ = (q_ * Quat::fromRotationVector(rv)).normalized();
}

// ----------------------------------------------------------------------------
void MadgwickFilter::update(const FusionInput& in) {
    if (!(in.dt > 0.0f)) return;
    float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;
    const float gx = in.gyroRadS.x, gy = in.gyroRadS.y, gz = in.gyroRadS.z;
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    const float an = in.accelG.norm();
    if (an > 0.1f) {
        const float ax = in.accelG.x / an, ay = in.accelG.y / an, az = in.accelG.z / an;
        lastErr_ = angleBetween(Vec3(ax, ay, az), upInDevice());
        if (in.gain > 0.0f) {
            const float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
            const float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
            const float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
            const float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
            float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
                       _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
                       _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            const float sn = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (sn > 1e-9f) {
                s0 /= sn; s1 /= sn; s2 /= sn; s3 /= sn;
                // beta chosen so that a fraction `gain` of the current error angle is
                // removed per step (quaternion step ~ half angle).
                const float beta = clamp01(in.gain) * lastErr_ * 0.5f / in.dt;
                qDot1 -= beta * s0;
                qDot2 -= beta * s1;
                qDot3 -= beta * s2;
                qDot4 -= beta * s3;
            }
        }
    }
    q0 += qDot1 * in.dt;
    q1 += qDot2 * in.dt;
    q2 += qDot3 * in.dt;
    q3 += qDot4 * in.dt;
    q_ = Quat(q0, q1, q2, q3).normalized();
}

// ----------------------------------------------------------------------------
void FusionBank::select(FusionType t) {
    const Quat q = active_->orientation();
    switch (t) {
        case FusionType::Mahony: active_ = &mahony_; break;
        case FusionType::Complementary: active_ = &comp_; break;
        case FusionType::Madgwick: active_ = &madgwick_; break;
    }
    active_->setOrientation(q);
}

// ----------------------------------------------------------------------------
void UncertaintyModel::reset() {
    p_ = cfg_.initialTiltRad * cfg_.initialTiltRad;
    yawSigma_ = 0.0f;
}

void UncertaintyModel::predict(float dt, float rateRadS, float biasSigmaDps) {
    if (!(dt > 0.0f)) return;
    const float noise = deg2rad(cfg_.gyroNoiseDps) * dt;              // random part
    const float systematic = (deg2rad(biasSigmaDps) + cfg_.gyroScaleError * std::fabs(rateRadS)) * dt;
    float s = std::sqrt(p_ + noise * noise) + systematic;
    if (s > 1.0f) s = 1.0f;  // ~57 deg: no information left
    p_ = s * s;
    yawSigma_ += systematic;
    if (yawSigma_ > kPi) yawSigma_ = kPi;
}

float UncertaintyModel::correct(float weight, float measNoiseRad, float maxGainOverride) {
    if (weight <= 1e-3f) return 0.0f;
    const float sigma = measNoiseRad > 1e-5f ? measNoiseRad : cfg_.accelNoiseRad;
    const float r = sigma * sigma / weight;
    float k = p_ / (p_ + r);
    const float maxK = maxGainOverride > 0.0f ? maxGainOverride : cfg_.maxStepGain;
    const float minK = cfg_.minStepGain * weight;
    k = clampv(k, minK, maxK);
    // Joseph form keeps the variance consistent when k is clamped.
    p_ = (1.0f - k) * (1.0f - k) * p_ + k * k * r;
    return k;
}

}  // namespace sda
