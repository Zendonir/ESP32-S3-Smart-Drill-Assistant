// SPDX-License-Identifier: MIT
// Integration tests of the complete measurement pipeline with synthetic datasets.
#include <unity.h>

#include "../support/SyntheticImu.h"
#include "MeasurementPipeline.h"

using namespace sda;
using namespace sda_test;

void setUp() {}
void tearDown() {}

static MeasurementPipeline* makePipeline(const SynthConfig& sc) {
    static MeasurementPipeline p;  // large object: avoid stack
    PipelineConfig pc;
    p.configure(pc);
    SensorCalibration cal;
    GyroCalPoint g;
    g.tempC = sc.tempC;
    g.biasDps = sc.gyroBiasDps;
    g.noiseDps = Vec3(sc.gyroNoiseDps, sc.gyroNoiseDps, sc.gyroNoiseDps);
    g.samples = 600;
    cal.addGyroPoint(g);
    AccelCalPoint a;
    a.tempC = sc.tempC;
    a.residualMg = 1.0f;
    cal.addAccelPoint(a);
    p.setCalibration(cal);
    p.setMount(MountModel{});
    return &p;
}

static void feed(MeasurementPipeline& p, SyntheticImu& imu, int n, const Vec3& rate = Vec3()) {
    for (int i = 0; i < n; ++i) p.process(imu.step(rate));
}

void test_static_tilt_accuracy_and_quality() {
    SynthConfig sc;
    SyntheticImu imu(sc, 31);
    MeasurementPipeline& p = *makePipeline(sc);
    const Quat truth = orientationWithAxis(Vec3(0, 0, -1), -60.0f, 25.0f);
    imu.setOrientation(truth);
    feed(p, imu, 224 * 5);
    const OrientationSnapshot& s = p.snapshot();
    TEST_ASSERT_TRUE(s.valid);
    const float err = rad2deg(angleBetween(s.upD, truth.rotateInverse(Vec3::unitZ())));
    TEST_ASSERT_TRUE(err < 0.05f);
    TEST_ASSERT_EQUAL(int(MotionState::Still), int(s.motion));
    TEST_ASSERT_EQUAL(int(QualityState::Stable), int(s.quality.state));
    TEST_ASSERT_TRUE(s.tiltSigmaDeg < 0.2f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 224.2f, s.timing.estimatedRateHz);
    const DrillGeometry g = drillGeometry(s.qWD, p.mount());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, -60.0f, g.elevationDeg);
}

void test_rotation_then_settle_no_jumps() {
    SynthConfig sc;
    SyntheticImu imu(sc, 32);
    MeasurementPipeline& p = *makePipeline(sc);
    feed(p, imu, 224 * 2);
    // fast rotation (90 deg in 0.5 s) about X
    feed(p, imu, 112, Vec3(180.0f, 0, 0));
    const float errDuring = rad2deg(angleBetween(p.snapshot().upD, imu.orientation().rotateInverse(Vec3::unitZ())));
    TEST_ASSERT_TRUE(errDuring < 1.5f);  // gyro tracks the fast motion
    feed(p, imu, 224 * 3);
    const float errAfter = rad2deg(angleBetween(p.snapshot().upD, imu.orientation().rotateInverse(Vec3::unitZ())));
    TEST_ASSERT_TRUE(errAfter < 0.1f);
}

void test_vibration_lowers_quality_and_blocks_target() {
    SynthConfig sc;
    SyntheticImu imu(sc, 33);
    MeasurementPipeline& p = *makePipeline(sc);
    feed(p, imu, 224 * 3);
    imu.config().vibrationG = 0.4f;
    feed(p, imu, 224 * 2);
    const OrientationSnapshot& s = p.snapshot();
    TEST_ASSERT_EQUAL(int(VibrationLevel::Strong), int(s.vibration));
    TEST_ASSERT_TRUE(s.accelTrust < 0.3f);
    TEST_ASSERT_FALSE(s.quality.usableForTarget(0.5f, s.tiltSigmaDeg));
    // orientation must not be wrecked by the vibration (accel down-weighted)
    const float err = rad2deg(angleBetween(s.upD, imu.orientation().rotateInverse(Vec3::unitZ())));
    TEST_ASSERT_TRUE(err < 0.5f);
}

void test_long_vibration_raises_uncertainty() {
    SynthConfig sc;
    sc.gyroBiasDps = Vec3(0.2f, 0, 0);
    SyntheticImu imu(sc, 34);
    MeasurementPipeline& p = *makePipeline(sc);
    feed(p, imu, 224 * 2);
    const float before = p.snapshot().tiltSigmaDeg;
    imu.config().vibrationG = 0.6f;
    feed(p, imu, 224 * 30);
    TEST_ASSERT_TRUE(p.snapshot().tiltSigmaDeg > before * 3.0f);
}

void test_stuck_data_is_sensor_fault() {
    MeasurementPipeline& p = *makePipeline(SynthConfig());
    ImuSample s;
    s.accelG = Vec3(0, 0, 1);
    s.gyroDps = Vec3(0.1f, 0, 0);
    s.temperatureC = 25;
    for (int i = 0; i < 400; ++i) {
        s.sensorCounter = uint32_t(i + 1);
        s.hostTimeUs = uint64_t((i + 1) * 4460);
        p.process(s);
    }
    TEST_ASSERT_EQUAL(int(QualityState::SensorFault), int(p.snapshot().quality.state));
    p.setDataTimeout(true);
    TEST_ASSERT_TRUE(p.snapshot().quality.flags & kQfNoData);
}

void test_sample_loss_flag() {
    SynthConfig sc;
    sc.dropEvery = 50;
    SyntheticImu imu(sc, 35);
    MeasurementPipeline& p = *makePipeline(sc);
    feed(p, imu, 224 * 2);
    TEST_ASSERT_TRUE(p.snapshot().timing.lostSamples > 5);
    TEST_ASSERT_TRUE(p.snapshot().quality.flags & kQfSampleLoss);
}

void test_gyro_calibration_in_pipeline() {
    SynthConfig sc;
    sc.gyroBiasDps = Vec3(1.5f, -0.7f, 0.4f);
    SyntheticImu imu(sc, 36);
    MeasurementPipeline& p = *makePipeline(SynthConfig());  // calibration with WRONG bias
    feed(p, imu, 50);
    p.startGyroCalibration();
    feed(p, imu, 224 * 4);
    TEST_ASSERT_TRUE(p.gyroResultOk());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.5f, p.gyroResult().biasDps.x);
    TEST_ASSERT_EQUAL(int(CalProcedure::None), int(p.snapshot().cal.procedure));
    // corrected rate near zero
    feed(p, imu, 10);
    TEST_ASSERT_TRUE(p.snapshot().gyroDps.norm() < 0.3f);
}

void test_online_bias_tracking_reduces_residual() {
    SynthConfig sc;
    sc.gyroBiasDps = Vec3(0.3f, 0, 0);
    SyntheticImu imu(sc, 37);
    SynthConfig wrong = sc;
    wrong.gyroBiasDps = Vec3(0.0f, 0, 0);  // calibration misses 0.3 dps
    MeasurementPipeline& p = *makePipeline(wrong);
    feed(p, imu, 224 * 40);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.3f, p.snapshot().onlineBiasDps.x);
}

void test_precision_measurement_in_pipeline() {
    SynthConfig sc;
    SyntheticImu imu(sc, 38);
    MeasurementPipeline& p = *makePipeline(sc);
    imu.setOrientation(orientationWithAxis(Vec3(0, 0, -1), 45.0f, 0.0f));
    feed(p, imu, 224);
    PrecisionConfig pc;
    pc.windowSec = 2.0f;
    p.startPrecision(pc, PrecisionQuantity::AxisInclination);
    feed(p, imu, 224 * 4);
    const PrecisionResult& r = p.snapshot().precision;
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 45.0f, r.valueDeg);
    TEST_ASSERT_TRUE(r.stdDeg < 0.2f);
}

void test_reference_heading_uncertainty_grows() {
    SynthConfig sc;
    SyntheticImu imu(sc, 39);
    MeasurementPipeline& p = *makePipeline(sc);
    feed(p, imu, 224);
    p.captureSurface(true);
    feed(p, imu, 224 * 5, Vec3(0, 0, 30));  // rotate around (remount)
    const ReferenceInfo& ri = p.snapshot().reference.info();
    TEST_ASSERT_TRUE(ri.usesGyro);
    TEST_ASSERT_TRUE(ri.headingSigmaDeg > 0.5f);  // scale error * 150 deg rotation
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 5.0f, ri.ageSec);
}

void test_mount_calibration_in_pipeline() {
    SynthConfig sc;
    SyntheticImu imu(sc, 40);
    MeasurementPipeline& p = *makePipeline(sc);
    const Vec3 axisD = Vec3(0.02f, 0.03f, -1).normalized();
    p.startMountCalibration(CalProcedure::MountRoll);
    for (int k = 0; k < 4; ++k) {
        const Quat q = Quat::fromAxisAngle(Vec3::unitX(), deg2rad(90.0f * k)) * Quat::fromTwoVectors(axisD, Vec3::unitX());
        imu.setOrientation(q);
        feed(p, imu, 224 * 3);  // settle after repositioning -> still
        p.mountCapturePosition();
        feed(p, imu, 224 * 3);  // capture waits for still-stand automatically
    }
    TEST_ASSERT_EQUAL(4, p.snapshot().cal.mountPositions);
    const MountFitResult r = p.mountSolve(Vec3(0, 0, -1));
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(rad2deg(angleBetween(r.axisD, axisD)) < 0.1f);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_static_tilt_accuracy_and_quality);
    RUN_TEST(test_rotation_then_settle_no_jumps);
    RUN_TEST(test_vibration_lowers_quality_and_blocks_target);
    RUN_TEST(test_long_vibration_raises_uncertainty);
    RUN_TEST(test_stuck_data_is_sensor_fault);
    RUN_TEST(test_sample_loss_flag);
    RUN_TEST(test_gyro_calibration_in_pipeline);
    RUN_TEST(test_online_bias_tracking_reduces_residual);
    RUN_TEST(test_precision_measurement_in_pipeline);
    RUN_TEST(test_reference_heading_uncertainty_grows);
    RUN_TEST(test_mount_calibration_in_pipeline);
    return UNITY_END();
}
