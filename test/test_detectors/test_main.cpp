// SPDX-License-Identifier: MIT
// Unit tests: still-stand detection, vibration analysis, accel trust, timing.
#include <unity.h>

#include "../support/SyntheticImu.h"
#include "MotionAnalysis.h"
#include "TimingEstimator.h"

using namespace sda;
using namespace sda_test;

void setUp() {}
void tearDown() {}

static constexpr float kDt = 1.0f / 224.2f;

void test_still_detected_after_hold_time() {
    MotionDetector md;
    SynthConfig c;
    c.gyroBiasDps = Vec3();
    SyntheticImu imu(c, 2);
    int firstStill = -1;
    for (int i = 0; i < 448; ++i) {
        const ImuSample s = imu.step(Vec3());
        md.update(s.gyroDps, s.accelG, kDt);
        if (md.isStill() && firstStill < 0) firstStill = i;
    }
    TEST_ASSERT_TRUE(md.isStill());
    // must not be faster than the configured hold time (0.6 s ~ 134 samples)
    TEST_ASSERT_TRUE(firstStill >= 130);
    // a rotation ends still-stand immediately
    const ImuSample m = imu.step(Vec3(5, 0, 0));
    md.update(m.gyroDps, m.accelG, kDt);
    TEST_ASSERT_FALSE(md.isStill());
    TEST_ASSERT_EQUAL(int(MotionState::Slow), int(md.state()));
    const ImuSample f = imu.step(Vec3(60, 0, 0));
    md.update(f.gyroDps, f.accelG, kDt);
    TEST_ASSERT_EQUAL(int(MotionState::Fast), int(md.state()));
}

void test_still_detected_without_calibration() {
    // uncalibrated sensor: 2.5 deg/s gyro offset, |a| = 1.05 g (scale/offset error)
    MotionDetector md;
    SynthConfig c;
    c.gyroBiasDps = Vec3(1.5f, -2.0f, 0.4f);
    c.accelScale = Vec3(1.05f, 1.05f, 1.05f);
    SyntheticImu imu(c, 9);
    for (int i = 0; i < 224 * 3; ++i) {
        const ImuSample s = imu.step(Vec3());
        md.update(s.gyroDps, s.accelG, kDt);
    }
    TEST_ASSERT_TRUE(md.isStill());
    TEST_ASSERT_EQUAL(0, md.stillBlockers());
    // a real rotation is still detected immediately
    const ImuSample m = imu.step(Vec3(10, 0, 0));
    md.update(m.gyroDps, m.accelG, kDt);
    TEST_ASSERT_FALSE(md.isStill());
    TEST_ASSERT_TRUE(md.stillBlockers() & 1);
}

void test_slow_creep_is_not_still() {
    MotionDetector md;
    SynthConfig c;
    c.gyroBiasDps = Vec3();
    c.gyroNoiseDps = 0.01f;
    SyntheticImu imu(c, 3);
    // 0.5 dps creep is below the gyro threshold, but the direction criterion catches it
    int still = 0;
    for (int i = 0; i < 224 * 4; ++i) {
        const ImuSample s = imu.step(Vec3(0.5f, 0, 0));
        md.update(s.gyroDps, s.accelG, kDt);
        if (md.isStill()) ++still;
    }
    TEST_ASSERT_TRUE(still < 224 * 4 / 2);
}

void test_vibration_levels_and_release() {
    VibrationDetector vd;
    SynthConfig c;
    c.vibrationG = 0.2f;  // strong
    SyntheticImu imu(c, 4);
    for (int i = 0; i < 224; ++i) {
        const ImuSample s = imu.step(Vec3());
        vd.update(s.accelG, s.gyroDps, kDt);
    }
    TEST_ASSERT_EQUAL(int(VibrationLevel::Strong), int(vd.level()));
    imu.config().vibrationG = 0.0f;
    int samplesUntilNone = 0;
    while (vd.level() != VibrationLevel::None && samplesUntilNone < 2000) {
        const ImuSample s = imu.step(Vec3());
        vd.update(s.accelG, s.gyroDps, kDt);
        ++samplesUntilNone;
    }
    TEST_ASSERT_EQUAL(int(VibrationLevel::None), int(vd.level()));
    TEST_ASSERT_TRUE(samplesUntilNone > 100);  // release time applied
}

void test_accel_trust() {
    AccelTrustConfig c;
    TEST_ASSERT_EQUAL_FLOAT(1.0f, accelTrust(c, 0.0f, 5.0f, 0.0f, false));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, accelTrust(c, 0.0f, 5.0f, 0.0f, true));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, accelTrust(c, 0.2f, 5.0f, 0.0f, false));
    const float mid = accelTrust(c, 0.0f, 80.0f, 0.0f, false);
    TEST_ASSERT_TRUE(mid > 0.2f && mid < 0.8f);
}

void test_timing_counter_mode_and_loss() {
    TimingEstimator te;
    te.configure(224.2f);
    // true sensor rate deviates by +3 % from nominal
    const float trueRate = 224.2f * 1.03f;
    uint64_t t = 0;
    uint32_t cnt = 0x00FFFFF0;  // close to wrap-around
    TimingStep st;
    for (int i = 0; i < 224 * 6; ++i) {
        cnt = (cnt + 1) & 0xFFFFFF;
        t += uint64_t(1e6 / trueRate);
        st = te.update(cnt, t + (i % 3) * 300);  // host jitter
    }
    TEST_ASSERT_TRUE(te.stats().counterMode);
    TEST_ASSERT_FLOAT_WITHIN(trueRate * 0.005f, trueRate, te.stats().estimatedRateHz);
    TEST_ASSERT_EQUAL_UINT32(0, te.stats().lostSamples);
    // lose 2 samples
    cnt = (cnt + 3) & 0xFFFFFF;
    t += uint64_t(3e6 / trueRate);
    st = te.update(cnt, t);
    TEST_ASSERT_EQUAL_UINT32(2, st.lostSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f / trueRate, st.dtSec);
    // duplicate
    st = te.update(cnt, t + 1000);
    TEST_ASSERT_TRUE(st.duplicate);
    // long gap -> discontinuity
    cnt = (cnt + 100) & 0xFFFFFF;
    t += 500000;
    st = te.update(cnt, t);
    TEST_ASSERT_TRUE(st.discontinuity);
}

void test_timing_rate_change_without_discontinuity() {
    TimingEstimator te;
    te.configure(224.2f);
    uint64_t t = 0;
    uint32_t cnt = 0;
    for (int i = 0; i < 224 * 3; ++i) te.update(++cnt, t += 4460);
    te.changeNominalRate(28.025f);  // power saving mode
    bool disc = false;
    TimingStep st;
    for (int i = 0; i < 28 * 4; ++i) {
        st = te.update(++cnt, t += 35682);
        disc |= st.discontinuity;
    }
    TEST_ASSERT_FALSE(disc);
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 0.035682f, st.dtSec);
    TEST_ASSERT_EQUAL_UINT32(0, te.stats().lostSamples);
}

void test_timing_fallback_to_host_time() {
    TimingEstimator te;
    te.configure(224.2f);
    uint64_t t = 0;
    uint32_t cnt = 0;
    for (int i = 0; i < 200; ++i) {
        cnt += 37;  // counter is not a sample counter
        t += 4460;
        te.update(cnt, t);
    }
    TEST_ASSERT_FALSE(te.stats().counterMode);
    const TimingStep st = te.update(cnt + 37, t + 4460);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.00446f, st.dtSec);
}

void test_drill_activity() {
    DrillActivityTracker t;
    TEST_ASSERT_EQUAL(int(DrillActivity::Aligning), int(t.update(MotionState::Slow, VibrationLevel::None, 0.1f)));
    TEST_ASSERT_EQUAL(int(DrillActivity::Vibrating), int(t.update(MotionState::Fast, VibrationLevel::Strong, 0.1f)));
    TEST_ASSERT_EQUAL(int(DrillActivity::Settling), int(t.update(MotionState::Still, VibrationLevel::None, 0.1f)));
    for (int i = 0; i < 40; ++i) t.update(MotionState::Still, VibrationLevel::None, 0.1f);
    TEST_ASSERT_EQUAL(int(DrillActivity::Holding), int(t.state()));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_still_detected_after_hold_time);
    RUN_TEST(test_slow_creep_is_not_still);
    RUN_TEST(test_still_detected_without_calibration);
    RUN_TEST(test_vibration_levels_and_release);
    RUN_TEST(test_accel_trust);
    RUN_TEST(test_timing_counter_mode_and_loss);
    RUN_TEST(test_timing_fallback_to_host_time);
    RUN_TEST(test_timing_rate_change_without_discontinuity);
    RUN_TEST(test_drill_activity);
    return UNITY_END();
}
