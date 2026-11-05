// SPDX-License-Identifier: MIT
#include "CoordinateTransforms.h"

#include <cstdlib>

namespace sda {

Mat3 AxisMap::matrix() const {
    Mat3 r = Mat3::zero();
    for (int i = 0; i < 3; ++i) {
        const int c = code[i];
        const int axis = std::abs(c) - 1;
        if (axis < 0 || axis > 2) return Mat3::identity();
        r.m[i][axis] = c > 0 ? 1.0f : -1.0f;
    }
    return r;
}

Vec3 AxisMap::apply(const Vec3& s) const {
    Vec3 out;
    for (int i = 0; i < 3; ++i) {
        const int c = code[i];
        const int axis = std::abs(c) - 1;
        if (axis < 0 || axis > 2) return s;
        out[i] = c > 0 ? s[axis] : -s[axis];
    }
    return out;
}

bool AxisMap::isValid() const {
    bool used[3] = {false, false, false};
    for (int i = 0; i < 3; ++i) {
        const int axis = std::abs(code[i]) - 1;
        if (axis < 0 || axis > 2 || used[axis]) return false;
        used[axis] = true;
    }
    return matrix().determinant() > 0.5f;  // proper rotation only (no mirroring)
}

bool AxisMap::parse(const char* str) {
    if (!str) return false;
    AxisMap tmp;
    int idx = 0;
    const char* p = str;
    while (*p && idx < 3) {
        int sign = 1;
        if (*p == '+') { sign = 1; ++p; }
        else if (*p == '-') { sign = -1; ++p; }
        else return false;
        int axis;
        switch (*p) {
            case 'X': case 'x': axis = 1; break;
            case 'Y': case 'y': axis = 2; break;
            case 'Z': case 'z': axis = 3; break;
            default: return false;
        }
        ++p;
        tmp.code[idx++] = static_cast<int8_t>(sign * axis);
    }
    if (idx != 3 || *p != '\0' || !tmp.isValid()) return false;
    *this = tmp;
    return true;
}

void AxisMap::format(char* out) const {
    for (int i = 0; i < 3; ++i) {
        out[2 * i] = code[i] > 0 ? '+' : '-';
        const int axis = std::abs(code[i]);
        out[2 * i + 1] = axis == 1 ? 'X' : (axis == 2 ? 'Y' : 'Z');
    }
    out[6] = '\0';
}

void MountModel::displayBasis(Vec3& rightD, Vec3& upD) const {
    const Vec3 d = drillAxisD.normalized(Vec3(0, 0, -1));
    Vec3 up = rejectFrom(Vec3::unitY(), d);
    if (up.norm() < 0.35f) {
        // Device Y nearly parallel to the drill axis -> use the screen normal (+Z)
        // as "up" on the display plane. This happens for the TopEdgeForward preset.
        up = rejectFrom(Vec3::unitZ(), d);
    }
    upD = up.normalized();
    rightD = upD.cross(-d).normalized();
}

Mat3 MountModel::toolToDevice() const {
    Vec3 r, u;
    displayBasis(r, u);
    const Vec3 x = drillAxisD.normalized(Vec3(0, 0, -1));
    const Vec3 z = u;
    const Vec3 y = z.cross(x);
    return Mat3::fromCols(x, y, z);
}

bool MountModel::isValid() const {
    if (!drillAxisD.isFinite()) return false;
    const float n = drillAxisD.norm();
    return n > 0.99f && n < 1.01f && std::isfinite(angleCorrectionDeg) &&
           std::fabs(angleCorrectionDeg) <= 10.0f;
}

namespace {
/// Signed dominant axis of v (code 1..3, negative = inverted), 0 if not unique.
int dominantAxis(const Vec3& v, float cosLimit) {
    const float n = v.norm();
    if (!(n > 0.5f)) return 0;
    int idx = 0;
    for (int i = 1; i < 3; ++i)
        if (std::fabs(v[i]) > std::fabs(v[idx])) idx = i;
    if (std::fabs(v[idx]) / n < cosLimit) return 0;
    return v[idx] > 0 ? idx + 1 : -(idx + 1);
}

Vec3 axisVector(int code) {
    Vec3 v;
    v[std::abs(code) - 1] = code > 0 ? 1.0f : -1.0f;
    return v;
}
}  // namespace

AxisDetectResult detectAxisMap(const Vec3& upFlatS, const Vec3& upUprightS) {
    AxisDetectResult r;
    const float cosLim = std::cos(25.0f * kPi / 180.0f);
    const int z = dominantAxis(upFlatS, cosLim);
    const int y = dominantAxis(upUprightS, cosLim);
    if (z == 0) {
        r.error = "Flachlage nicht eindeutig";
        return r;
    }
    if (y == 0) {
        r.error = "Aufrecht-Lage nicht eindeutig";
        return r;
    }
    if (std::abs(z) == std::abs(y)) {
        r.error = "Beide Lagen gleich";
        return r;
    }
    const Vec3 ez = axisVector(z), ey = axisVector(y);
    const int x = dominantAxis(ey.cross(ez), 0.9f);
    AxisMap m;
    m.code[0] = static_cast<int8_t>(x);
    m.code[1] = static_cast<int8_t>(y);
    m.code[2] = static_cast<int8_t>(z);
    if (!m.isValid()) {
        r.error = "Ungültige Zuordnung";
        return r;
    }
    r.map = m;
    r.orthogonalityErrDeg = std::fabs(angleBetween(upFlatS, upUprightS) * 180.0f / kPi - 90.0f);
    r.ok = true;
    return r;
}

Mat3 axisRemapRotation(const AxisMap& oldMap, const AxisMap& newMap) {
    return newMap.matrix() * oldMap.matrix().transposed();
}

Vec3 mountPresetAxis(MountPreset p) {
    switch (p) {
        case MountPreset::ScreenFacingUser: return {0, 0, -1};
        case MountPreset::TopEdgeForward: return {0, 1, 0};
        case MountPreset::RightEdgeForward: return {1, 0, 0};
        case MountPreset::ScreenFacingBit: return {0, 0, 1};
    }
    return {0, 0, -1};
}

}  // namespace sda
