// SPDX-License-Identifier: MIT
// Unit tests: level angles, drill deviation, references, relative rotation.
#include <unity.h>

#include "../support/SyntheticImu.h"
#include "Orientation.h"
#include "AngleTarget.h"
#include "ReferenceManager.h"

using namespace sda;
using namespace sda_test;

void setUp() {}
void tearDown() {}

void test_level_angles_flat_and_edge() {
    LevelAngles l = levelAngles(Vec3(0, 0, 1));
    TEST_ASSERT_TRUE(l.flat);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, l.tiltXDeg);
    // right edge raised by 3 deg -> up vector gains +x component
    const Quat q = Quat::fromAxisAngle(Vec3::unitY(), deg2rad(-3));  // rotates +X upwards
    l = levelAngles(q.rotateInverse(Vec3::unitZ()));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.0f, l.tiltXDeg);
    // standing on bottom edge, rotated 2 deg in plane
    const Vec3 up(std::sin(deg2rad(2)), std::cos(deg2rad(2)), 0);
    l = levelAngles(up);
    TEST_ASSERT_FALSE(l.flat);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, l.edgeRollDeg);
    TEST_ASSERT_EQUAL(0, l.edgeIndex);
    l = levelAngles(Vec3(1, 0.01f, 0));
    TEST_ASSERT_EQUAL(1, l.edgeIndex);
}

void test_relative_tilt() {
    float x, y;
    relativeTilt(Vec3(0, 0, 1), Vec3(0, 0, 1), x, y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, x);
    const Vec3 ref = Vec3(0.2f, 0.1f, 1).normalized();
    relativeTilt(ref, ref, x, y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, y);
}

void test_drill_vertical_target_full_2d() {
    MountModel mm;  // axis -Z (screen facing user)
    DrillTarget t;
    t.inclinationDeg = 90.0f;
    t.hemisphere = -1;
    HemisphereState hs;
    // drill points straight down: device -Z = world down -> device +Z up (screen up)
    Quat q = Quat::identity();
    Deviation d = drillDeviation(q, mm, t, hs);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, d.totalDeg);
    // tip tilted by 2 deg towards device +X (screen right): rotate about Y
    q = Quat::fromAxisAngle(Vec3::unitY(), deg2rad(2));
    const Vec3 tip = q.rotate(Vec3(0, 0, -1));
    TEST_ASSERT_TRUE(tip.x < 0 || tip.x > 0);
    d = drillDeviation(q, mm, t, hs);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, d.totalDeg);
    TEST_ASSERT_FALSE(d.lateralFree);
    // marker displacement magnitude equals the angle; direction follows the tip on screen
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, std::sqrt(d.screenX * d.screenX + d.screenY * d.screenY));
    // tip leans towards world direction tip.x -> on screen the marker moves to the side
    // where the tip leans (screen right = device +X maps to world via q)
    const Vec3 screenRightW = q.rotate(Vec3::unitX());
    const float sideSign = (tip.x * screenRightW.x + tip.y * screenRightW.y) > 0 ? 1.0f : -1.0f;
    TEST_ASSERT_TRUE(d.screenX * sideSign > 1.9f);
}

void test_drill_horizontal_target_elevation_only() {
    MountModel mm;
    DrillTarget t;
    t.inclinationDeg = 0.0f;  // horizontal (wall)
    HemisphereState hs;
    // axis 3 deg above horizontal, heading arbitrary 70 deg
    const Quat q = orientationWithAxis(Vec3(0, 0, -1), 3.0f, 70.0f);
    const Deviation d = drillDeviation(q, mm, t, hs);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.0f, d.elevationErrDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, d.lateralErrDeg);
    TEST_ASSERT_TRUE(d.lateralFree);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.0f, d.totalDeg);
}

void test_drill_45_target_and_hemisphere() {
    MountModel mm;
    DrillTarget t;
    t.inclinationDeg = 45.0f;
    HemisphereState hs;
    Quat q = orientationWithAxis(Vec3(0, 0, -1), -40.0f, 10.0f);  // bit down 40 deg
    Deviation d = drillDeviation(q, mm, t, hs);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -45.0f, d.targetElevationDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f, d.elevationErrDeg);  // too shallow = tip too high
    // upwards drilling: switches hemisphere
    q = orientationWithAxis(Vec3(0, 0, -1), 50.0f, 10.0f);
    d = drillDeviation(q, mm, t, hs);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 45.0f, d.targetElevationDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f, d.elevationErrDeg);
}

void test_drill_angle_correction() {
    MountModel mm;
    mm.angleCorrectionDeg = 1.5f;
    const Quat q = orientationWithAxis(Vec3(0, 0, -1), 10.0f, 0.0f);
    const DrillGeometry g = drillGeometry(q, mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 11.5f, g.elevationDeg);
}

void test_drill_direction_target_lateral() {
    MountModel mm;
    DrillTarget t;
    t.kind = TargetKind::Direction;
    t.directionW = Vec3(1, 0, 0);  // horizontal towards +X
    HemisphereState hs;
    // axis rotated 2 deg to the left (heading +2)
    const Quat q = orientationWithAxis(Vec3(0, 0, -1), 0.0f, 2.0f);
    const Deviation d = drillDeviation(q, mm, t, hs);
    TEST_ASSERT_TRUE(d.lateralFromGyro);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, d.totalDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -2.0f, d.lateralErrDeg);  // left = negative
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, d.elevationErrDeg);
}

void test_surface_reference_restricted_vs_3d() {
    ReferenceManager rm;
    // device lies on a surface sloped by 20 deg (normal tilted about X)
    const Quat q = Quat::fromAxisAngle(Vec3::unitX(), deg2rad(20));
    rm.captureSurface(q, false, 0.05f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f, rm.info().surfaceSlopeDeg);
    TEST_ASSERT_TRUE(rm.info().azimuthFree);
    DrillTarget t = rm.surfaceTarget();
    TEST_ASSERT_EQUAL(int(TargetKind::Inclination), int(t.kind));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 70.0f, t.inclinationDeg);
    TEST_ASSERT_EQUAL(-1, t.hemisphere);
    rm.captureSurface(q, true, 0.05f);
    t = rm.surfaceTarget();
    TEST_ASSERT_EQUAL(int(TargetKind::Direction), int(t.kind));
    TEST_ASSERT_TRUE(rm.info().usesGyro);
    rm.propagate(10.0f, deg2rad(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, rm.info().headingSigmaDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, rm.info().ageSec);
}

void test_relative_rotation_heading() {
    const Quat ref = Quat::fromAxisAngle(Vec3::unitX(), deg2rad(10));
    const Quat now = Quat::fromAxisAngle(Vec3::unitZ(), deg2rad(5)) * ref;  // 5 deg about vertical
    const RelativeRotation r = relativeRotation(ref, now);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 5.0f, r.headingDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, r.tiltDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 5.0f, r.totalDeg);
}

static float totalFor(const AngleTargetResult& r, const Quat& q, const MountModel& mm) {
    HemisphereState hs;
    return drillDeviation(q, mm, r.target, hs).totalDeg;
}

void test_angle_target_absolute() {
    MountModel mm;
    ReferenceManager rm;
    // 30 deg to the vertical == 60 deg to the horizontal
    AngleTargetResult r = makeAngleTarget(AngleReference::Vertical, 30.0f, 0, rm, mm);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FALSE(r.usesGyro);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 60.0f, r.target.inclinationDeg);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, totalFor(r, orientationWithAxis(Vec3(0, 0, -1), -60.0f, 123.0f), mm));
    r = makeAngleTarget(AngleReference::Horizontal, 45.0f, -1, rm, mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, totalFor(r, orientationWithAxis(Vec3(0, 0, -1), -45.0f, 10.0f), mm));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f, totalFor(r, orientationWithAxis(Vec3(0, 0, -1), -40.0f, 10.0f), mm));
}

void test_angle_target_zero_up_down_is_gravity_based() {
    MountModel mm;
    ReferenceManager rm;
    AngleTargetResult r = makeAngleTarget(AngleReference::ZeroDown, 45.0f, 0, rm, mm);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_TRUE(r.needsZero);
    // zero point: axis horizontal (perpendicular to a wall), heading 0
    const Quat q0 = orientationWithAxis(Vec3(0, 0, -1), 0.0f, 0.0f);
    rm.captureTiltZero(q0.rotateInverse(Vec3::unitZ()), 0.05f);  // tilt zero is enough
    r = makeAngleTarget(AngleReference::ZeroDown, 45.0f, 0, rm, mm);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FALSE(r.usesGyro);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -45.0f, r.targetElevationDeg);
    // any heading is accepted (azimuth free), 45 deg down is on target
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, totalFor(r, orientationWithAxis(Vec3(0, 0, -1), -45.0f, 77.0f), mm));
    // too far: beyond the vertical
    r = makeAngleTarget(AngleReference::ZeroUp, 45.0f, 0, rm, mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 45.0f, r.targetElevationDeg);
    rm.captureTiltZero(orientationWithAxis(Vec3(0, 0, -1), 60.0f, 0.0f).rotateInverse(Vec3::unitZ()), 0.05f);
    r = makeAngleTarget(AngleReference::ZeroUp, 45.0f, 0, rm, mm);
    TEST_ASSERT_FALSE(r.valid);
}

void test_angle_target_zero_left_right_uses_gyro() {
    MountModel mm;
    ReferenceManager rm;
    const Quat q0 = orientationWithAxis(Vec3(0, 0, -1), 0.0f, 0.0f);
    rm.captureTiltZero(q0.rotateInverse(Vec3::unitZ()), 0.05f);
    // tilt zero is NOT sufficient for left/right
    AngleTargetResult r = makeAngleTarget(AngleReference::ZeroRight, 45.0f, 0, rm, mm);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_TRUE(r.needsZero);
    rm.captureFull3D(q0, 0.05f);
    r = makeAngleTarget(AngleReference::ZeroRight, 45.0f, 0, rm, mm);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_TRUE(r.usesGyro);
    // right = negative heading
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, totalFor(r, orientationWithAxis(Vec3(0, 0, -1), 0.0f, -45.0f), mm));
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 90.0f, totalFor(r, orientationWithAxis(Vec3(0, 0, -1), 0.0f, 45.0f), mm));
    r = makeAngleTarget(AngleReference::ZeroLeft, 30.0f, 0, rm, mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, totalFor(r, orientationWithAxis(Vec3(0, 0, -1), 0.0f, 30.0f), mm));
    // vertical reference axis: left/right undefined
    rm.captureFull3D(Quat::identity(), 0.05f);
    r = makeAngleTarget(AngleReference::ZeroLeft, 30.0f, 0, rm, mm);
    TEST_ASSERT_FALSE(r.valid);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_level_angles_flat_and_edge);
    RUN_TEST(test_relative_tilt);
    RUN_TEST(test_drill_vertical_target_full_2d);
    RUN_TEST(test_drill_horizontal_target_elevation_only);
    RUN_TEST(test_drill_45_target_and_hemisphere);
    RUN_TEST(test_drill_angle_correction);
    RUN_TEST(test_drill_direction_target_lateral);
    RUN_TEST(test_surface_reference_restricted_vs_3d);
    RUN_TEST(test_relative_rotation_heading);
    RUN_TEST(test_angle_target_absolute);
    RUN_TEST(test_angle_target_zero_up_down_is_gravity_based);
    RUN_TEST(test_angle_target_zero_left_right_uses_gyro);
    return UNITY_END();
}
