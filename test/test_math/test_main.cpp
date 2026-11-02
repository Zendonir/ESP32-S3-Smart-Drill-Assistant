// SPDX-License-Identifier: MIT
// Unit tests: vectors, quaternions, rotation matrices, axis mapping, linear algebra.
#include <unity.h>

#include "CoordinateTransforms.h"
#include "LinearAlgebra.h"
#include "MathTypes.h"

using namespace sda;

void setUp() {}
void tearDown() {}

static void assertVecNear(const Vec3& e, const Vec3& a, float tol) {
    TEST_ASSERT_FLOAT_WITHIN(tol, e.x, a.x);
    TEST_ASSERT_FLOAT_WITHIN(tol, e.y, a.y);
    TEST_ASSERT_FLOAT_WITHIN(tol, e.z, a.z);
}

void test_quat_axis_angle_rotation() {
    const Quat q = Quat::fromAxisAngle(Vec3::unitZ(), deg2rad(90));
    assertVecNear(Vec3(0, 1, 0), q.rotate(Vec3::unitX()), 1e-6f);
    assertVecNear(Vec3(1, 0, 0), q.rotateInverse(Vec3(0, 1, 0)), 1e-6f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, deg2rad(90), q.angle());
}

void test_quat_matrix_roundtrip() {
    const Quat q = (Quat::fromAxisAngle(Vec3(1, 2, 3), 1.1f) * Quat::fromAxisAngle(Vec3(-1, 0, 2), -0.4f)).normalized();
    const Mat3 m = q.toMatrix();
    TEST_ASSERT_TRUE(m.isRotation(1e-5f));
    const Vec3 v(0.3f, -1.2f, 2.0f);
    assertVecNear(q.rotate(v), m * v, 1e-5f);
    const Quat q2 = Quat::fromMatrix(m);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, quatDistance(q, q2));
}

void test_quat_composition_order() {
    // q_AC = q_AB * q_BC : first B<-C, then A<-B
    const Quat qAB = Quat::fromAxisAngle(Vec3::unitZ(), deg2rad(90));
    const Quat qBC = Quat::fromAxisAngle(Vec3::unitX(), deg2rad(90));
    const Vec3 vC = Vec3::unitY();
    assertVecNear(qAB.rotate(qBC.rotate(vC)), (qAB * qBC).rotate(vC), 1e-6f);
}

void test_rotation_vector_roundtrip() {
    const Vec3 rv(0.2f, -0.5f, 1.3f);
    const Quat q = Quat::fromRotationVector(rv);
    assertVecNear(rv, q.toRotationVector(), 1e-5f);
    const Vec3 tiny(1e-7f, -2e-7f, 3e-7f);
    assertVecNear(tiny, Quat::fromRotationVector(tiny).toRotationVector(), 1e-9f);
}

void test_from_two_vectors() {
    const Vec3 a = Vec3(1, 2, -0.5f).normalized();
    const Vec3 b = Vec3(-0.3f, 0.1f, 1).normalized();
    assertVecNear(b, Quat::fromTwoVectors(a, b).rotate(a), 1e-5f);
    // antiparallel
    assertVecNear(-a, Quat::fromTwoVectors(a, -a).rotate(a), 1e-5f);
    // identical
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, Quat::fromTwoVectors(a, a).angle());
}

void test_mat3_inverse() {
    const Mat3 m = Mat3::fromRows(Vec3(2, 0.1f, 0), Vec3(0.2f, 1.5f, -0.3f), Vec3(0, 0.4f, 0.9f));
    Mat3 inv;
    TEST_ASSERT_TRUE(m.inverse(inv));
    const Mat3 id = m * inv;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) TEST_ASSERT_FLOAT_WITHIN(1e-5f, i == j ? 1.0f : 0.0f, id.m[i][j]);
    TEST_ASSERT_FALSE(Mat3::zero().inverse(inv));
}

void test_wrap_and_angles() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -170.0f, wrap180(190.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 180.0f, wrap180(-180.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kPi / 2, angleBetween(Vec3::unitX(), Vec3::unitY()));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kPi / 2, safeAsin(1.0000001f));
}

void test_axis_map() {
    AxisMap m;
    TEST_ASSERT_TRUE(m.parse("-Y+X+Z"));
    assertVecNear(Vec3(-2, 1, 3), m.apply(Vec3(1, 2, 3)), 1e-6f);
    assertVecNear(m.apply(Vec3(1, 2, 3)), m.matrix() * Vec3(1, 2, 3), 1e-6f);
    char buf[8];
    m.format(buf);
    TEST_ASSERT_EQUAL_STRING("-Y+X+Z", buf);
    TEST_ASSERT_FALSE(m.parse("+X+X+Z"));   // duplicate axis
    TEST_ASSERT_FALSE(m.parse("-X+Y+Z"));   // mirror (det -1)
    TEST_ASSERT_FALSE(m.parse("+X+Y"));
    TEST_ASSERT_EQUAL_STRING("-Y+X+Z", (m.format(buf), buf));  // unchanged after failures
}

void test_mount_display_basis() {
    MountModel mm;
    mm.drillAxisD = Vec3(0, 0, -1);
    Vec3 r, u;
    mm.displayBasis(r, u);
    assertVecNear(Vec3(1, 0, 0), r, 1e-6f);
    assertVecNear(Vec3(0, 1, 0), u, 1e-6f);
    mm.drillAxisD = Vec3(0, 1, 0);  // top edge forward -> fallback to screen normal
    mm.displayBasis(r, u);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, u.dot(mm.drillAxisD));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, r.dot(mm.drillAxisD));
    TEST_ASSERT_TRUE(mm.toolToDevice().isRotation(1e-4f));
}

void test_sym_eigen() {
    // build A = R diag(1,4,9) R^T
    const Mat3 R = Quat::fromAxisAngle(Vec3(1, 1, 0.3f), 0.7f).toMatrix();
    const Mat3 A = R * Mat3::diag(1, 4, 9) * R.transposed();
    const SymEigen3Result e = symEigen3(A);
    TEST_ASSERT_TRUE(e.converged);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, e.eigenvalues[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.0f, e.eigenvalues[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 9.0f, e.eigenvalues[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, std::fabs(e.eigenvectors[0].dot(R.col(0))));
}

void test_least_squares_line() {
    LeastSquares ls(2);
    for (int i = 0; i < 10; ++i) {
        const float h[2] = {float(i), 1.0f};
        ls.add(h, 3.0f * i - 2.0f);
    }
    float x[2];
    TEST_ASSERT_TRUE(ls.solve(x));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, x[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.0f, x[1]);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_quat_axis_angle_rotation);
    RUN_TEST(test_quat_matrix_roundtrip);
    RUN_TEST(test_quat_composition_order);
    RUN_TEST(test_rotation_vector_roundtrip);
    RUN_TEST(test_from_two_vectors);
    RUN_TEST(test_mat3_inverse);
    RUN_TEST(test_wrap_and_angles);
    RUN_TEST(test_axis_map);
    RUN_TEST(test_mount_display_basis);
    RUN_TEST(test_sym_eigen);
    RUN_TEST(test_least_squares_line);
    return UNITY_END();
}
