// SPDX-License-Identifier: MIT
// Unit tests: target hysteresis, hold time, scaling, marker mapping, filters.
#include <unity.h>

#include "DisplayLogic.h"
#include "Statistics.h"

using namespace sda;

void setUp() {}
void tearDown() {}

void test_target_hold_and_hysteresis() {
    TargetTracker tt;
    TargetConfig c;
    c.toleranceDeg = 0.5f;
    c.holdSec = 0.4f;
    c.hysteresis = 0.25f;
    tt.configure(c);
    const float dt = 0.02f;
    TEST_ASSERT_EQUAL(int(TargetState::Far), int(tt.update(5.0f, true, dt)));
    TEST_ASSERT_EQUAL(int(TargetState::Near), int(tt.update(1.0f, true, dt)));
    TEST_ASSERT_EQUAL(int(TargetState::Inside), int(tt.update(0.4f, true, dt)));
    bool locked = false, justLockedSeen = false;
    for (int i = 0; i < 25; ++i) {
        tt.update(0.4f, true, dt);
        if (tt.justLocked()) justLockedSeen = true;
        locked = tt.state() == TargetState::Locked;
    }
    TEST_ASSERT_TRUE(locked);
    TEST_ASSERT_TRUE(justLockedSeen);
    // inside the hysteresis band (0.5 .. 0.625) stays locked
    TEST_ASSERT_EQUAL(int(TargetState::Locked), int(tt.update(0.6f, true, dt)));
    // beyond -> leaves
    TEST_ASSERT_EQUAL(int(TargetState::Near), int(tt.update(0.7f, true, dt)));
    // invalid measurement can never be green
    TEST_ASSERT_EQUAL(int(TargetState::Invalid), int(tt.update(0.0f, false, dt)));
}

void test_auto_scaler() {
    AutoScaler as;
    as.reset();
    TEST_ASSERT_EQUAL_FLOAT(20.0f, as.update(15.0f, 0.02f));
    // shrinks only after sustained small deviation
    TEST_ASSERT_EQUAL_FLOAT(20.0f, as.update(1.0f, 0.02f));
    float r = 0;
    for (int i = 0; i < 300; ++i) r = as.update(0.3f, 0.02f);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, r);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, as.update(3.0f, 0.02f));
}

void test_marker_mapping_clips() {
    MarkerPos p = mapDeviation(1.0f, 0.0f, 2.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 50.0f, p.x);
    TEST_ASSERT_FALSE(p.clipped);
    p = mapDeviation(0.0f, -30.0f, 10.0f, 100.0f);
    TEST_ASSERT_TRUE(p.clipped);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -100.0f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -kPi / 2, p.directionRad);
}

void test_one_euro_smooths_noise_and_follows_steps() {
    OneEuroFilter f;
    f.configure(1.0f, 0.3f);
    // alternating noise around 10 -> output stays close to 10
    float y = 0;
    for (int i = 0; i < 500; ++i) y = f.update(10.0f + ((i & 1) ? 0.1f : -0.1f), 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 10.0f, y);
    // large step: should follow within 0.5 s
    for (int i = 0; i < 50; ++i) y = f.update(20.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, y);
}

void test_stable_number() {
    StableNumber n(0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.2f, n.update(1.21f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.2f, n.update(1.249f));  // inside hysteresis
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.3f, n.update(1.27f));
}

void test_precision_measurement_statistics() {
    PrecisionMeasurement pm;
    PrecisionConfig c;
    c.windowSec = 1.0f;
    pm.configure(c);
    pm.start();
    TEST_ASSERT_TRUE(pm.waitingForStill() || pm.running());
    bool done = false;
    for (int i = 0; i < 300 && !done; ++i) done = pm.add(30.0f + ((i % 2) ? 0.01f : -0.01f), 1.0f, true, 0.005f);
    TEST_ASSERT_TRUE(done);
    const PrecisionResult& r = pm.result();
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_TRUE(r.stable);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 30.0f, r.meanDeg);
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0.01f, r.stdDeg);
    TEST_ASSERT_TRUE(r.semDeg < r.stdDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, r.secondaryDeg);
    // motion restarts the window
    pm.start();
    pm.add(1.0f, 0, true, 0.5f);
    pm.add(1.0f, 0, false, 0.005f);
    TEST_ASSERT_TRUE(pm.waitingForStill());
}

void test_precision_detects_drift() {
    PrecisionMeasurement pm;
    PrecisionConfig c;
    c.windowSec = 2.0f;
    pm.configure(c);
    pm.start();
    bool done = false;
    for (int i = 0; i < 1000 && !done; ++i) done = pm.add(10.0f + 0.1f * i * 0.005f, 0, true, 0.005f);  // 0.1 deg/s
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.1f, pm.result().slopeDegPerS);
    TEST_ASSERT_FALSE(pm.result().stable);
}

void test_ring_buffer() {
    RingBuffer<int, 3> rb;
    for (int i = 1; i <= 5; ++i) rb.push(i);
    TEST_ASSERT_EQUAL(3, (int)rb.size());
    TEST_ASSERT_EQUAL(3, rb.at(0));
    TEST_ASSERT_EQUAL(5, rb.newest());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_target_hold_and_hysteresis);
    RUN_TEST(test_auto_scaler);
    RUN_TEST(test_marker_mapping_clips);
    RUN_TEST(test_one_euro_smooths_noise_and_follows_steps);
    RUN_TEST(test_stable_number);
    RUN_TEST(test_precision_measurement_statistics);
    RUN_TEST(test_precision_detects_drift);
    RUN_TEST(test_ring_buffer);
    return UNITY_END();
}
