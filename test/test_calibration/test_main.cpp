// SPDX-License-Identifier: MIT
// Unit tests: gyro calibration, 6-position accel calibration, temperature tables,
// mount calibration (roll fit / vertical reference).
#include <unity.h>

#include "../support/SyntheticImu.h"
#include "CoordinateTransforms.h"
#include "MountCalibration.h"
#include "SensorCalibration.h"

using namespace sda;
using namespace sda_test;

void setUp() {}
void tearDown() {}

void test_gyro_calibration_estimates_bias_and_noise() {
    SynthConfig c;
    c.gyroBiasDps = Vec3(0.8f, -1.2f, 0.35f);
    c.gyroNoiseDps = 0.05f;
    SyntheticImu imu(c, 7);
    GyroCalibrator cal;
    cal.start();
    for (int i = 0; i < 2000 && cal.state() == GyroCalibrator::State::Collecting; ++i) {
        const ImuSample s = imu.step(Vec3());
        cal.addSample(s.gyroDps, s.accelG, s.temperatureC);
    }
    TEST_ASSERT_EQUAL(int(GyroCalibrator::State::Done), int(cal.state()));
    const GyroCalPoint& r = cal.result();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.8f, r.biasDps.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.2f, r.biasDps.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.35f, r.biasDps.z);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.05f, r.noiseDps.x);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 25.0f, r.tempC);
}

void test_gyro_calibration_restarts_on_motion() {
    SyntheticImu imu(SynthConfig(), 3);
    GyroCalibrator cal;
    cal.start();
    for (int i = 0; i < 300; ++i) {
        const ImuSample s = imu.step(Vec3());
        cal.addSample(s.gyroDps, s.accelG, s.temperatureC);
    }
    const float before = cal.progress();
    TEST_ASSERT_TRUE(before > 0.3f);
    const ImuSample moving = imu.step(Vec3(30, 0, 0));
    cal.addSample(moving.gyroDps, moving.accelG, moving.temperatureC);
    TEST_ASSERT_TRUE(cal.motionDetected());
    TEST_ASSERT_TRUE(cal.progress() < 0.01f);
    TEST_ASSERT_EQUAL(int(GyroCalibrator::State::Collecting), int(cal.state()));
}

void test_gyro_calibration_rejects_noise() {
    SynthConfig c;
    c.gyroNoiseDps = 0.9f;  // far too noisy (but no single sample exceeds motion limit often)
    SyntheticImu imu(c, 11);
    GyroCalConfig gc;
    gc.maxMotionDps = 100.0f;  // disable motion restarts for this test
    GyroCalibrator cal;
    cal.start(gc);
    for (int i = 0; i < 2000 && cal.state() == GyroCalibrator::State::Collecting; ++i) {
        const ImuSample s = imu.step(Vec3());
        cal.addSample(s.gyroDps, s.accelG, s.temperatureC);
    }
    TEST_ASSERT_EQUAL(int(GyroCalibrator::State::Failed), int(cal.state()));
    TEST_ASSERT_EQUAL(int(GyroCalibrator::Failure::TooNoisy), int(cal.failure()));
}

static void runAccelPoses(AccelCalibrator& cal, SyntheticImu& imu, float tiltDeg) {
    for (int p = 0; p < kAccelPoseCount; ++p) {
        // place device with axis p up, with a small placement tilt
        const Vec3 up = accelPoseExpected(AccelPose(p));
        Quat q = Quat::fromTwoVectors(up, Vec3::unitZ());
        q = Quat::fromAxisAngle(Vec3(1, 1, 0), deg2rad(tiltDeg)) * q;
        imu.setOrientation(q);
        cal.beginPose(AccelPose(p));
        for (int i = 0; i < 1000 && cal.collecting(); ++i) {
            const ImuSample s = imu.step(Vec3());
            cal.addSample(s.accelG, s.gyroDps - imu.config().gyroBiasDps, s.temperatureC);
        }
        TEST_ASSERT_EQUAL(int(AccelCalibrator::PoseState::Accepted), int(cal.poseState(AccelPose(p))));
    }
}

void test_accel_six_position_offset_scale() {
    SynthConfig c;
    c.accelOffsetG = Vec3(0.03f, -0.02f, 0.05f);
    c.accelScale = Vec3(1.02f, 0.985f, 1.01f);
    SyntheticImu imu(c, 5);
    AccelCalibrator cal;
    cal.start();
    runAccelPoses(cal, imu, 2.0f);
    AccelCalPoint r;
    TEST_ASSERT_TRUE(cal.solve(r));
    TEST_ASSERT_FALSE(r.extended);
    TEST_ASSERT_TRUE(r.residualMg < 2.0f);
    // apply to arbitrary orientation and verify magnitude/direction
    SensorCalibration sc;
    sc.addAccelPoint(r);
    const Quat q = Quat::fromAxisAngle(Vec3(0.3f, -1, 0.2f), 0.9f);
    const Vec3 trueUp = q.rotateInverse(Vec3::unitZ());
    const Vec3 raw(trueUp.x * c.accelScale.x + c.accelOffsetG.x, trueUp.y * c.accelScale.y + c.accelOffsetG.y,
                   trueUp.z * c.accelScale.z + c.accelOffsetG.z);
    const Vec3 corr = sc.applyAccel(raw, 25.0f);
    TEST_ASSERT_TRUE(rad2deg(angleBetween(corr, trueUp)) < 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.0f, corr.norm());
}

void test_accel_extended_cross_coupling() {
    // misaligned sensor: raw = A * true + o with off-diagonal terms
    SynthConfig c;
    c.accelNoiseG = 0.0005f;
    SyntheticImu imu(c, 9);
    const Mat3 A = Mat3::fromRows(Vec3(1.01f, 0.02f, -0.01f), Vec3(-0.015f, 0.99f, 0.01f), Vec3(0.005f, 0.012f, 1.0f));
    const Vec3 o(0.02f, 0.01f, -0.03f);
    AccelCalibrator cal;
    AccelCalConfig cc;
    cc.extended = true;
    cal.start(cc);
    for (int p = 0; p < kAccelPoseCount; ++p) {
        const Vec3 up = accelPoseExpected(AccelPose(p));
        cal.beginPose(AccelPose(p));
        Rng rng(p + 1);
        for (int i = 0; i < 400 && cal.collecting(); ++i) {
            const Vec3 n(rng.gauss(), rng.gauss(), rng.gauss());
            cal.addSample(A * up + o + n * 0.0005f, Vec3(), 25.0f);
        }
    }
    AccelCalPoint r;
    TEST_ASSERT_TRUE(cal.solve(r));
    TEST_ASSERT_TRUE(r.extended);
    const Vec3 dir = Vec3(0.4f, -0.6f, 0.7f).normalized();
    const Vec3 corrected = r.matrix * (A * dir + o) + r.offset;
    TEST_ASSERT_TRUE(rad2deg(angleBetween(corrected, dir)) < 0.05f);
}

void test_accel_pose_rejects_wrong_orientation() {
    AccelCalibrator cal;
    cal.start();
    cal.beginPose(AccelPose::PosX);
    for (int i = 0; i < 400; ++i) cal.addSample(Vec3(0, 0, 1), Vec3(), 25.0f);  // Z up instead
    TEST_ASSERT_EQUAL(int(AccelCalibrator::Failure::WrongOrientation), int(cal.lastFailure()));
    TEST_ASSERT_TRUE(cal.collecting());
    TEST_ASSERT_EQUAL(4, AccelCalibrator::detectPose(Vec3(0.05f, 0.02f, 0.99f)));
    TEST_ASSERT_EQUAL(-1, AccelCalibrator::detectPose(Vec3(0.7f, 0, 0.7f)));
}

void test_temperature_interpolation() {
    SensorCalibration sc;
    TempLookup info;
    GyroCalPoint p1;
    p1.tempC = 20.0f;
    p1.biasDps = Vec3(1, 0, 0);
    sc.addGyroPoint(p1);
    // a single point: constant, NOT reported as compensated
    sc.gyroBias(30.0f, &info);
    TEST_ASSERT_FALSE(info.compensated);
    TEST_ASSERT_FALSE(sc.gyroTempCompActive());
    GyroCalPoint p2;
    p2.tempC = 40.0f;
    p2.biasDps = Vec3(3, 0, 0);
    sc.addGyroPoint(p2);
    TEST_ASSERT_TRUE(sc.gyroTempCompActive());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, sc.gyroBias(30.0f, &info).x);
    TEST_ASSERT_TRUE(info.compensated);
    // clamped outside the range, no extrapolation
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, sc.gyroBias(50.0f, &info).x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, info.distanceC);
    TEST_ASSERT_FALSE(info.compensated);
    // NaN temperature -> constant first point, not compensated
    sc.gyroBias(NAN, &info);
    TEST_ASSERT_FALSE(info.compensated);
    // merge of a nearby temperature replaces instead of adding
    GyroCalPoint p3;
    p3.tempC = 40.8f;
    p3.biasDps = Vec3(4, 0, 0);
    sc.addGyroPoint(p3);
    TEST_ASSERT_EQUAL(2, sc.gyroPointCount());
}

void test_calibration_validate() {
    SensorCalibration sc;
    TEST_ASSERT_TRUE(sc.validate());
    AccelCalPoint bad;
    bad.matrix = Mat3::diag(2.0f, 1, 1);  // 100 % scale error: implausible
    sc.addAccelPoint(bad);
    TEST_ASSERT_FALSE(sc.validate());
}

void test_mount_roll_fit() {
    // true axis in device frame, slightly misaligned against -Z
    const Vec3 axisD = Vec3(0.03f, -0.02f, -1.0f).normalized();
    MountCalibrator mc;
    Rng rng(4);
    // drill lies horizontally (axis along world X), rotated about its axis in steps
    for (int k = 0; k < 6; ++k) {
        const float roll = deg2rad(60.0f * k);
        const Quat qAlign = Quat::fromTwoVectors(axisD, Vec3::unitX());
        const Quat q = Quat::fromAxisAngle(Vec3::unitX(), roll) * qAlign;
        Vec3 up = q.rotateInverse(Vec3::unitZ());
        up += Vec3(rng.gauss(), rng.gauss(), rng.gauss()) * 0.0005f;
        TEST_ASSERT_TRUE(mc.addPosition(up));
    }
    const MountFitResult r = mc.solveRollFit(Vec3(0, 0, -1));
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(rad2deg(angleBetween(r.axisD, axisD)) < 0.1f);
    TEST_ASSERT_TRUE(r.coverageDeg > 250.0f);
    TEST_ASSERT_TRUE(r.residualDeg < 0.2f);
    // duplicate position is rejected
    TEST_ASSERT_FALSE(mc.addPosition(Quat::fromTwoVectors(axisD, Vec3::unitX()).rotateInverse(Vec3::unitZ())));
}

void test_mount_roll_fit_poor_coverage() {
    const Vec3 axisD(0, 0, -1);
    MountCalibrator mc;
    for (int k = 0; k < 3; ++k) {
        const Quat q = Quat::fromAxisAngle(Vec3::unitX(), deg2rad(20.0f * k)) *
                       Quat::fromTwoVectors(axisD, Vec3::unitX());
        mc.addPosition(q.rotateInverse(Vec3::unitZ()), 10.0f);
    }
    const MountFitResult r = mc.solveRollFit(axisD);
    TEST_ASSERT_FALSE(r.ok);
    TEST_ASSERT_EQUAL(int(MountFitResult::Error::PoorCoverage), int(r.error));
}

void test_mount_vertical() {
    const MountFitResult r = MountCalibrator::solveVertical(Vec3(0.02f, 0.01f, 0.999f), true);
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_TRUE(r.axisD.z < -0.99f);
    TEST_ASSERT_FALSE(MountCalibrator::solveVertical(Vec3(0, 0, 0.3f)).ok);
}

void test_axis_detection_all_24_mappings() {
    const int perms[6][3] = {{1, 2, 3}, {1, 3, 2}, {2, 1, 3}, {2, 3, 1}, {3, 1, 2}, {3, 2, 1}};
    int tested = 0;
    for (const auto& p : perms)
        for (int signs = 0; signs < 8; ++signs) {
            AxisMap truth;
            for (int i = 0; i < 3; ++i) truth.code[i] = int8_t((signs & (1 << i)) ? -p[i] : p[i]);
            if (!truth.isValid()) continue;
            // sensor readings: s = A^T d, with a few degrees of placement error
            const Mat3 At = truth.matrix().transposed();
            const Quat tilt = Quat::fromAxisAngle(Vec3(1, 0.4f, 0.2f), deg2rad(6));
            const Vec3 flatS = At * tilt.rotate(Vec3(0, 0, 1));
            const Vec3 uprightS = At * tilt.rotate(Vec3(0, 1, 0));
            const AxisDetectResult r = detectAxisMap(flatS, uprightS);
            TEST_ASSERT_TRUE(r.ok);
            for (int i = 0; i < 3; ++i) TEST_ASSERT_EQUAL(truth.code[i], r.map.code[i]);
            ++tested;
        }
    TEST_ASSERT_EQUAL(24, tested);
    TEST_ASSERT_FALSE(detectAxisMap(Vec3(0, 0, 1), Vec3(0, 0, 1)).ok);          // same pose twice
    TEST_ASSERT_FALSE(detectAxisMap(Vec3(0.7f, 0, 0.7f), Vec3(0, 1, 0)).ok);    // ambiguous (45 deg)
}

void test_calibration_remap_follows_axis_map() {
    // calibration determined with a wrong mapping must give identical physical
    // results after the mapping is corrected and the calibration remapped
    AxisMap oldMap, newMap;
    TEST_ASSERT_TRUE(newMap.parse("-Y+X+Z"));
    SensorCalibration c;
    GyroCalPoint g;
    g.tempC = 25;
    g.biasDps = Vec3(0.3f, -0.7f, 0.1f);
    g.noiseDps = Vec3(0.01f, 0.02f, 0.03f);
    c.addGyroPoint(g);
    AccelCalPoint a;
    a.tempC = 25;
    a.matrix = Mat3::fromRows(Vec3(1.02f, 0.01f, 0), Vec3(0, 0.98f, 0.02f), Vec3(0.01f, 0, 1.01f));
    a.offset = Vec3(0.02f, -0.03f, 0.01f);
    c.addAccelPoint(a);
    const Vec3 sensorAcc(0.1f, 0.5f, 0.85f), sensorGyro(1.0f, 2.0f, 3.0f);
    const Vec3 oldA = c.applyAccel(oldMap.apply(sensorAcc), 25), oldG = c.applyGyro(oldMap.apply(sensorGyro), 25);
    const Mat3 R = axisRemapRotation(oldMap, newMap);
    SensorCalibration c2 = c;
    c2.remap(R);
    const Vec3 newA = c2.applyAccel(newMap.apply(sensorAcc), 25), newG = c2.applyGyro(newMap.apply(sensorGyro), 25);
    const Vec3 expA = R * oldA, expG = R * oldG;
    for (int i = 0; i < 3; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, expA[i], newA[i]);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, expG[i], newG[i]);
    }
    TEST_ASSERT_TRUE(c2.validate());
}

// housing frame H rests on the table; the device (sensor) frame is misaligned
// against H by a small rotation (the error the reversal measurement finds)
static void flipScenario(SupportFace face, const Quat& misalign, const Vec3& tableTiltAxis, float tableTiltDeg) {
    const Vec3 n = supportFaceNominalUp(face);  // face normal in H (= D nominal)
    // table normal in world, housing placed so that its face normal = table normal
    const Quat tilt = Quat::fromAxisAngle(tableTiltAxis, deg2rad(tableTiltDeg));
    const Vec3 tableN = tilt.rotate(Vec3::unitZ());
    // position 1: q_WH maps n -> tableN (plus some yaw); position 2: rotated 180 deg about tableN
    const Quat q1 = Quat::fromAxisAngle(tableN, 0.7f) * Quat::fromTwoVectors(n, tableN);
    const Quat q2 = Quat::fromAxisAngle(tableN, kPi) * q1;
    auto upD = [&](const Quat& qWH) { return misalign.rotate(qWH.rotateInverse(Vec3::unitZ())); };
    const FlipResult r = solveFlip(face, upD(q1), upD(q2));
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, tableTiltDeg, r.cal.tableTiltDeg);
    // on a LEVEL table the corrected level must read 0 (independent of the table used)
    const Vec3 levelUp = misalign.rotate(Quat::fromTwoVectors(n, Vec3::unitZ()).rotateInverse(Vec3::unitZ()));
    LevelAngles l = levelAngles(levelUp);
    FaceCalibration none;
    if (face == SupportFace::Back) {
        TEST_ASSERT_TRUE(applyFaceCorrections(l, levelUp, r.cal, none));
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, l.tiltXDeg);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, l.tiltYDeg);
    } else {
        TEST_ASSERT_TRUE(applyFaceCorrections(l, levelUp, none, r.cal));
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, l.edgeRollDeg);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, l.edgePitchDeg);
    }
}

void test_flip_calibration_back_and_bottom() {
    const Quat mis = Quat::fromAxisAngle(Vec3(0.3f, -0.8f, 0.2f), deg2rad(1.2f));
    flipScenario(SupportFace::Back, mis, Vec3(1, 0.4f, 0), 1.5f);
    flipScenario(SupportFace::Back, mis, Vec3(-0.2f, 1, 0), 0.0f);
    flipScenario(SupportFace::Bottom, mis, Vec3(1, 0.4f, 0), 2.0f);
    // wrong face / too steep table are rejected
    TEST_ASSERT_FALSE(solveFlip(SupportFace::Back, Vec3(0, 1, 0), Vec3(0, 1, 0)).ok);
    const Vec3 steep = Quat::fromAxisAngle(Vec3::unitX(), deg2rad(8)).rotate(Vec3::unitZ());
    TEST_ASSERT_FALSE(solveFlip(SupportFace::Back, steep, Vec3(-steep.x, -steep.y, steep.z)).ok);
}

void test_flip_combine_and_clear() {
    FaceCalibration p[2];
    p[0].errADeg = 0.10f; p[0].errBDeg = -0.20f; p[0].valid = true;
    p[1].errADeg = 0.14f; p[1].errBDeg = -0.18f; p[1].valid = true;
    const FaceCalibration c = combineFlips(p, 2);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.12f, c.errADeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.04f, c.spreadDeg);
    SensorCalibration sc;
    sc.setFace(SupportFace::Back, c);
    TEST_ASSERT_TRUE(sc.hasFaceCalibration());
    sc.addAccelPoint(AccelCalPoint{});  // new accel calibration invalidates the reversal values
    TEST_ASSERT_FALSE(sc.hasFaceCalibration());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_gyro_calibration_estimates_bias_and_noise);
    RUN_TEST(test_gyro_calibration_restarts_on_motion);
    RUN_TEST(test_gyro_calibration_rejects_noise);
    RUN_TEST(test_accel_six_position_offset_scale);
    RUN_TEST(test_accel_extended_cross_coupling);
    RUN_TEST(test_accel_pose_rejects_wrong_orientation);
    RUN_TEST(test_temperature_interpolation);
    RUN_TEST(test_calibration_validate);
    RUN_TEST(test_mount_roll_fit);
    RUN_TEST(test_mount_roll_fit_poor_coverage);
    RUN_TEST(test_mount_vertical);
    RUN_TEST(test_axis_detection_all_24_mappings);
    RUN_TEST(test_calibration_remap_follows_axis_map);
    RUN_TEST(test_flip_calibration_back_and_bottom);
    RUN_TEST(test_flip_combine_and_clear);
    return UNITY_END();
}
