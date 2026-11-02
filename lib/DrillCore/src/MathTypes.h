// SPDX-License-Identifier: MIT
// Smart Drill Assistant - hardware independent math primitives.
//
// All measurement mathematics lives in lib/DrillCore and has no dependency on
// Arduino, ESP-IDF or FreeRTOS so that it can be unit tested on the host
// (PlatformIO "native" environment).
//
// Conventions (see docs/Mathematics.md):
//  * Right handed coordinate systems.
//  * Quaternion q = (w, x, y, z), Hamilton product, unit length.
//  * A quaternion q_AB rotates vectors from frame B into frame A:
//        v_A = q_AB * v_B * conj(q_AB)
//  * Angles inside the library are radians unless the name ends in "Deg".
#pragma once

#include <cmath>
#include <cstdint>

namespace sda {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegPerRad = 180.0f / kPi;
constexpr float kRadPerDeg = kPi / 180.0f;
constexpr float kStandardGravity = 9.80665f;  // m/s^2, only used for documentation/units

inline float deg2rad(float d) { return d * kRadPerDeg; }
inline float rad2deg(float r) { return r * kDegPerRad; }

template <typename T>
inline T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline float clamp01(float v) { return clampv(v, 0.0f, 1.0f); }

/// Wraps an angle to (-pi, pi].
inline float wrapPi(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a <= -kPi) a += 2.0f * kPi;
    return a;
}

/// Wraps an angle in degrees to (-180, 180].
inline float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

/// asin with argument clamping (protects against |x| marginally > 1 from rounding).
inline float safeAsin(float x) { return std::asin(clampv(x, -1.0f, 1.0f)); }
inline float safeAcos(float x) { return std::acos(clampv(x, -1.0f, 1.0f)); }

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float norm2() const { return dot(*this); }
    float norm() const { return std::sqrt(norm2()); }

    /// Returns the normalized vector; returns `fallback` if the norm is (nearly) zero.
    Vec3 normalized(const Vec3& fallback = Vec3(0, 0, 1)) const {
        const float n = norm();
        if (!(n > 1e-9f)) return fallback;
        return *this / n;
    }

    bool isFinite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }

    static constexpr Vec3 unitX() { return {1, 0, 0}; }
    static constexpr Vec3 unitY() { return {0, 1, 0}; }
    static constexpr Vec3 unitZ() { return {0, 0, 1}; }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

/// Angle between two vectors in radians (robust atan2 formulation).
inline float angleBetween(const Vec3& a, const Vec3& b) {
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

/// Component of v perpendicular to the unit vector n.
inline Vec3 rejectFrom(const Vec3& v, const Vec3& nUnit) { return v - nUnit * v.dot(nUnit); }

struct Mat3 {
    // Row-major: m[row][col]
    float m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    static Mat3 identity() { return Mat3(); }
    static Mat3 zero() {
        Mat3 r;
        for (auto& row : r.m)
            for (float& v : row) v = 0.0f;
        return r;
    }
    static Mat3 fromRows(const Vec3& r0, const Vec3& r1, const Vec3& r2) {
        Mat3 r;
        r.m[0][0] = r0.x; r.m[0][1] = r0.y; r.m[0][2] = r0.z;
        r.m[1][0] = r1.x; r.m[1][1] = r1.y; r.m[1][2] = r1.z;
        r.m[2][0] = r2.x; r.m[2][1] = r2.y; r.m[2][2] = r2.z;
        return r;
    }
    static Mat3 fromCols(const Vec3& c0, const Vec3& c1, const Vec3& c2) {
        return fromRows(c0, c1, c2).transposed();
    }
    static Mat3 diag(float a, float b, float c) {
        Mat3 r = zero();
        r.m[0][0] = a; r.m[1][1] = b; r.m[2][2] = c;
        return r;
    }

    Vec3 row(int i) const { return {m[i][0], m[i][1], m[i][2]}; }
    Vec3 col(int j) const { return {m[0][j], m[1][j], m[2][j]}; }

    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r = zero();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k) r.m[i][j] += m[i][k] * o.m[k][j];
        return r;
    }
    Mat3 operator*(float s) const {
        Mat3 r = *this;
        for (auto& row : r.m)
            for (float& v : row) v *= s;
        return r;
    }
    Mat3 operator+(const Mat3& o) const {
        Mat3 r = *this;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] += o.m[i][j];
        return r;
    }
    Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
        return r;
    }
    float determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }
    /// Inverse; returns false if the matrix is singular (|det| < eps).
    bool inverse(Mat3& out, float eps = 1e-9f) const {
        const float det = determinant();
        if (std::fabs(det) < eps) return false;
        const float inv = 1.0f / det;
        out.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv;
        out.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv;
        out.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv;
        out.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv;
        out.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
        out.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
        out.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv;
        out.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
        out.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
        return true;
    }
    /// True if the matrix is a proper rotation within the given tolerance.
    bool isRotation(float tol = 1e-4f) const {
        const Mat3 p = (*this) * transposed();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (std::fabs(p.m[i][j] - (i == j ? 1.0f : 0.0f)) > tol) return false;
        return std::fabs(determinant() - 1.0f) < tol * 10.0f;
    }
};

struct Quat {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Quat() = default;
    constexpr Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat identity() { return Quat(); }

    /// Rotation of `angle` radians around the (not necessarily unit) axis.
    static Quat fromAxisAngle(const Vec3& axis, float angle) {
        const Vec3 a = axis.normalized(Vec3(1, 0, 0));
        const float h = 0.5f * angle;
        const float s = std::sin(h);
        return {std::cos(h), a.x * s, a.y * s, a.z * s};
    }

    /// Exponential map of a rotation vector (axis * angle, radians). Exact for any size,
    /// uses a series expansion for tiny angles to stay accurate.
    static Quat fromRotationVector(const Vec3& rv) {
        const float theta2 = rv.norm2();
        if (theta2 < 1e-12f) {
            return Quat(1.0f, 0.5f * rv.x, 0.5f * rv.y, 0.5f * rv.z).normalized();
        }
        const float theta = std::sqrt(theta2);
        const float h = 0.5f * theta;
        const float s = std::sin(h) / theta;
        return {std::cos(h), rv.x * s, rv.y * s, rv.z * s};
    }

    /// Shortest rotation that maps unit vector `from` onto unit vector `to`.
    static Quat fromTwoVectors(const Vec3& from, const Vec3& to) {
        const Vec3 f = from.normalized();
        const Vec3 t = to.normalized();
        const float d = f.dot(t);
        if (d < -0.999999f) {
            // 180 deg: pick any axis perpendicular to f
            Vec3 axis = Vec3::unitX().cross(f);
            if (axis.norm2() < 1e-6f) axis = Vec3::unitY().cross(f);
            return fromAxisAngle(axis, kPi);
        }
        const Vec3 c = f.cross(t);
        return Quat(1.0f + d, c.x, c.y, c.z).normalized();
    }

    /// Rotation matrix -> quaternion (Shepperd's method).
    static Quat fromMatrix(const Mat3& r) {
        const float tr = r.m[0][0] + r.m[1][1] + r.m[2][2];
        Quat q;
        if (tr > 0.0f) {
            const float s = std::sqrt(tr + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (r.m[2][1] - r.m[1][2]) / s;
            q.y = (r.m[0][2] - r.m[2][0]) / s;
            q.z = (r.m[1][0] - r.m[0][1]) / s;
        } else if (r.m[0][0] > r.m[1][1] && r.m[0][0] > r.m[2][2]) {
            const float s = std::sqrt(1.0f + r.m[0][0] - r.m[1][1] - r.m[2][2]) * 2.0f;
            q.w = (r.m[2][1] - r.m[1][2]) / s;
            q.x = 0.25f * s;
            q.y = (r.m[0][1] + r.m[1][0]) / s;
            q.z = (r.m[0][2] + r.m[2][0]) / s;
        } else if (r.m[1][1] > r.m[2][2]) {
            const float s = std::sqrt(1.0f + r.m[1][1] - r.m[0][0] - r.m[2][2]) * 2.0f;
            q.w = (r.m[0][2] - r.m[2][0]) / s;
            q.x = (r.m[0][1] + r.m[1][0]) / s;
            q.y = 0.25f * s;
            q.z = (r.m[1][2] + r.m[2][1]) / s;
        } else {
            const float s = std::sqrt(1.0f + r.m[2][2] - r.m[0][0] - r.m[1][1]) * 2.0f;
            q.w = (r.m[1][0] - r.m[0][1]) / s;
            q.x = (r.m[0][2] + r.m[2][0]) / s;
            q.y = (r.m[1][2] + r.m[2][1]) / s;
            q.z = 0.25f * s;
        }
        return q.normalized();
    }

    Quat operator*(const Quat& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }

    Quat conjugate() const { return {w, -x, -y, -z}; }
    float norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }
    Quat normalized() const {
        const float n = norm();
        if (!(n > 1e-12f)) return Quat();
        const float inv = 1.0f / n;
        return {w * inv, x * inv, y * inv, z * inv};
    }
    bool isFinite() const {
        return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    /// Rotates vector v by this (unit) quaternion.
    Vec3 rotate(const Vec3& v) const {
        // v' = v + 2w(u x v) + 2 u x (u x v), u = (x,y,z)
        const Vec3 u(x, y, z);
        const Vec3 t = u.cross(v) * 2.0f;
        return v + t * w + u.cross(t);
    }
    /// Rotates by the inverse rotation.
    Vec3 rotateInverse(const Vec3& v) const { return conjugate().rotate(v); }

    Mat3 toMatrix() const {
        Mat3 r;
        const float xx = x * x, yy = y * y, zz = z * z;
        const float xy = x * y, xz = x * z, yz = y * z;
        const float wx = w * x, wy = w * y, wz = w * z;
        r.m[0][0] = 1 - 2 * (yy + zz); r.m[0][1] = 2 * (xy - wz);     r.m[0][2] = 2 * (xz + wy);
        r.m[1][0] = 2 * (xy + wz);     r.m[1][1] = 1 - 2 * (xx + zz); r.m[1][2] = 2 * (yz - wx);
        r.m[2][0] = 2 * (xz - wy);     r.m[2][1] = 2 * (yz + wx);     r.m[2][2] = 1 - 2 * (xx + yy);
        return r;
    }

    /// Rotation angle in radians [0, pi].
    float angle() const {
        const float vn = std::sqrt(x * x + y * y + z * z);
        return 2.0f * std::atan2(vn, std::fabs(w));
    }

    /// Logarithm map: rotation vector (axis*angle) of the shortest equivalent rotation.
    Vec3 toRotationVector() const {
        Quat q = *this;
        if (q.w < 0) q = Quat(-q.w, -q.x, -q.y, -q.z);
        const float vn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
        if (vn < 1e-9f) return Vec3(2.0f * q.x, 2.0f * q.y, 2.0f * q.z);
        const float ang = 2.0f * std::atan2(vn, q.w);
        return Vec3(q.x, q.y, q.z) * (ang / vn);
    }
};

/// Angular distance between two unit quaternions (radians).
inline float quatDistance(const Quat& a, const Quat& b) { return (a.conjugate() * b).angle(); }

}  // namespace sda
