// SPDX-License-Identifier: MIT
// Deterministic synthetic IMU data generator for host unit tests.
// Produces samples for a given true orientation trajectory with configurable
// noise, bias, vibration and sample loss - no randomness from the OS.
#pragma once

#include <cstdint>

#include "ImuTypes.h"
#include "MathTypes.h"

namespace sda_test {

using namespace sda;

class Rng {
public:
    explicit Rng(uint32_t seed = 12345u) : s_(seed ? seed : 1u) {}
    float uniform() {  // [0,1)
        s_ ^= s_ << 13;
        s_ ^= s_ >> 17;
        s_ ^= s_ << 5;
        return float(s_ & 0xFFFFFF) / float(0x1000000);
    }
    float gauss() {  // Box-Muller
        float u1 = uniform();
        if (u1 < 1e-7f) u1 = 1e-7f;
        const float u2 = uniform();
        return std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * kPi * u2);
    }

private:
    uint32_t s_;
};

struct SynthConfig {
    float rateHz = 224.2f;
    float accelNoiseG = 0.0015f;
    float gyroNoiseDps = 0.04f;
    Vec3 gyroBiasDps{0.3f, -0.2f, 0.15f};
    Vec3 accelOffsetG{0, 0, 0};
    Vec3 accelScale{1, 1, 1};
    float vibrationG = 0.0f;      ///< amplitude of sinusoidal vibration
    float vibrationHz = 23.0f;
    float tempC = 25.0f;
    uint32_t dropEvery = 0;       ///< drop every n-th sample (sample loss)
};

class SyntheticImu {
public:
    explicit SyntheticImu(const SynthConfig& c = SynthConfig(), uint32_t seed = 1) : cfg_(c), rng_(seed) {}

    /// Advances the simulation by one sample period with body rate `rateDps`
    /// (device frame) and returns the sample for the NEW orientation.
    ImuSample step(const Vec3& rateDps, const Vec3& linAccelWorldG = Vec3()) {
        const float dt = 1.0f / cfg_.rateHz;
        q_ = (q_ * Quat::fromRotationVector(rateDps * (kRadPerDeg * dt))).normalized();
        t_ += dt;
        counter_ = (counter_ + 1) & 0xFFFFFF;
        if (cfg_.dropEvery && (++dropCnt_ % cfg_.dropEvery) == 0) {
            counter_ = (counter_ + 1) & 0xFFFFFF;  // one sample lost
            q_ = (q_ * Quat::fromRotationVector(rateDps * (kRadPerDeg * dt))).normalized();
            t_ += dt;
        }
        ImuSample s;
        s.hostTimeUs = uint64_t(t_ * 1e6);
        s.sensorCounter = counter_;
        Vec3 fW = Vec3::unitZ() + linAccelWorldG;
        Vec3 a = q_.rotateInverse(fW);
        if (cfg_.vibrationG > 0) {
            const float v = cfg_.vibrationG * std::sin(2.0f * kPi * cfg_.vibrationHz * t_);
            a += Vec3(v, 0.7f * v, -0.5f * v);
        }
        a = Vec3(a.x * cfg_.accelScale.x, a.y * cfg_.accelScale.y, a.z * cfg_.accelScale.z) + cfg_.accelOffsetG;
        a += Vec3(rng_.gauss(), rng_.gauss(), rng_.gauss()) * cfg_.accelNoiseG;
        s.accelG = a;
        s.gyroDps = rateDps + cfg_.gyroBiasDps +
                    Vec3(rng_.gauss(), rng_.gauss(), rng_.gauss()) * cfg_.gyroNoiseDps;
        s.temperatureC = cfg_.tempC;
        return s;
    }

    void setOrientation(const Quat& q) { q_ = q; }
    const Quat& orientation() const { return q_; }
    SynthConfig& config() { return cfg_; }
    float time() const { return t_; }

private:
    SynthConfig cfg_;
    Rng rng_;
    Quat q_;
    float t_ = 0.0f;
    uint32_t counter_ = 0;
    uint32_t dropCnt_ = 0;
};

/// Orientation whose device axis `axisD` points along world direction with the
/// given elevation (deg) and heading (deg).
inline Quat orientationWithAxis(const Vec3& axisD, float elevationDeg, float headingDeg) {
    const float e = deg2rad(elevationDeg), h = deg2rad(headingDeg);
    const Vec3 target(std::cos(e) * std::cos(h), std::cos(e) * std::sin(h), std::sin(e));
    return Quat::fromTwoVectors(axisD, target);
}

}  // namespace sda_test
