// SPDX-License-Identifier: MIT
// Unit tests: attitude filters and uncertainty model with synthetic data.
#include <unity.h>

#include "../support/SyntheticImu.h"
#include "SensorFusion.h"

using namespace sda;
using namespace sda_test;

void setUp() {}
void tearDown() {}

static float tiltErrorDeg(const IFusionFilter& f, const Quat& truth) {
    return rad2deg(angleBetween(f.upInDevice(), truth.rotateInverse(Vec3::unitZ())));
}

static void runStatic(FusionType type) {
    FusionBank bank;
    bank.select(type);
    SynthConfig c;
    c.gyroBiasDps = Vec3();  // calibrated
    SyntheticImu imu(c, 21);
    const Quat truth = Quat::fromAxisAngle(Vec3(1, 0.5f, 0), deg2rad(35));
    imu.setOrientation(truth);
    // start from a wrong attitude (level) and converge
    bank.active().reset(Vec3::unitZ());
    for (int i = 0; i < 224 * 10; ++i) {
        const ImuSample s = imu.step(Vec3());
        FusionInput in{s.gyroDps * kRadPerDeg, s.accelG, 1.0f / 224.2f, 0.02f};
        bank.active().update(in);
    }
    TEST_ASSERT_TRUE(tiltErrorDeg(bank.active(), truth) < 0.05f);
}

void test_mahony_converges_static() { runStatic(FusionType::Mahony); }
void test_complementary_converges_static() { runStatic(FusionType::Complementary); }
void test_madgwick_converges_static() { runStatic(FusionType::Madgwick); }

void test_gyro_only_tracks_rotation() {
    // with gain 0 the filter is pure integration: 90 deg about X in 1 s
    MahonyFilter f;
    f.reset(Vec3::unitZ());
    SynthConfig c;
    c.gyroBiasDps = Vec3();
    c.gyroNoiseDps = 0.0f;
    c.accelNoiseG = 0.0f;
    SyntheticImu imu(c);
    const int n = 224;
    const float rate = 90.0f * 224.2f / n;
    for (int i = 0; i < n; ++i) {
        const ImuSample s = imu.step(Vec3(rate, 0, 0));
        f.update(FusionInput{s.gyroDps * kRadPerDeg, s.accelG, 1.0f / 224.2f, 0.0f});
    }
    TEST_ASSERT_TRUE(tiltErrorDeg(f, imu.orientation()) < 0.05f);
}

void test_accel_correction_does_not_touch_heading() {
    // a pure heading rotation must survive accel correction (unobservable axis)
    MahonyFilter f;
    f.reset(Vec3::unitZ());
    const Quat yaw = Quat::fromAxisAngle(Vec3::unitZ(), deg2rad(40));
    f.setOrientation(yaw);
    for (int i = 0; i < 2000; ++i) f.update(FusionInput{Vec3(), Vec3::unitZ(), 1.0f / 224.0f, 0.1f});
    const Vec3 fwd = f.orientation().rotate(Vec3::unitX());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, rad2deg(std::atan2(fwd.y, fwd.x)));
}

void test_bias_causes_drift_without_correction() {
    MahonyFilter f;
    f.reset(Vec3::unitZ());
    for (int i = 0; i < 224 * 10; ++i)
        f.update(FusionInput{Vec3(deg2rad(0.5f), 0, 0), Vec3::unitZ(), 1.0f / 224.0f, 0.0f});
    // 0.5 dps for 10 s = 5 deg tilt error without accel correction
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 5.0f, rad2deg(angleBetween(f.upInDevice(), Vec3::unitZ())));
}

void test_uncertainty_grows_and_shrinks() {
    UncertaintyModel m;
    m.reset();
    for (int i = 0; i < 2000; ++i) {
        m.predict(1.0f / 224, 0.0f, 0.05f);
        m.correct(1.0f, 0.002f);
    }
    const float settled = m.tiltSigmaRad();
    TEST_ASSERT_TRUE(rad2deg(settled) < 0.1f);
    // 10 s without accel (vibration): uncertainty must grow, at least by bias * t
    for (int i = 0; i < 2240; ++i) {
        m.predict(1.0f / 224, 0.0f, 0.05f);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, m.correct(0.0f, 0.002f));
    }
    TEST_ASSERT_TRUE(rad2deg(m.tiltSigmaRad()) > 0.45f);
    TEST_ASSERT_TRUE(rad2deg(m.headingSigmaRad()) > 0.45f);
}

void test_bank_switch_keeps_orientation() {
    FusionBank bank;
    bank.active().reset(Vec3(0.3f, 0, 0.95f));
    const Quat before = bank.active().orientation();
    bank.select(FusionType::Madgwick);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, quatDistance(before, bank.active().orientation()));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_mahony_converges_static);
    RUN_TEST(test_complementary_converges_static);
    RUN_TEST(test_madgwick_converges_static);
    RUN_TEST(test_gyro_only_tracks_rotation);
    RUN_TEST(test_accel_correction_does_not_touch_heading);
    RUN_TEST(test_bias_causes_drift_without_correction);
    RUN_TEST(test_uncertainty_grows_and_shrinks);
    RUN_TEST(test_bank_switch_keeps_orientation);
    return UNITY_END();
}
