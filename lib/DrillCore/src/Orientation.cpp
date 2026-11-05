// SPDX-License-Identifier: MIT
#include "Orientation.h"

namespace sda {

LevelAngles levelAngles(const Vec3& upDIn) {
    LevelAngles r;
    const Vec3 u = upDIn.normalized();
    r.tiltXDeg = rad2deg(safeAsin(u.x));
    r.tiltYDeg = rad2deg(safeAsin(u.y));
    r.screenTiltDeg = rad2deg(safeAcos(u.z));
    r.flat = std::fabs(u.z) >= 0.70710678f;
    // In-plane rotation (only meaningful when the screen is near vertical):
    // 0 when device +Y points up.
    const float roll = rad2deg(std::atan2(u.x, u.y));
    float rr = roll;
    int idx = 0;
    while (rr > 45.0f) { rr -= 90.0f; idx = (idx + 1) & 3; }
    while (rr <= -45.0f) { rr += 90.0f; idx = (idx + 3) & 3; }
    r.edgeRollDeg = rr;
    r.edgePitchDeg = rad2deg(safeAsin(u.z));
    r.edgeIndex = idx;
    return r;
}

void relativeTilt(const Vec3& upRefD, const Vec3& upD, float& relXDeg, float& relYDeg) {
    // Minimal rotation that maps the reference up vector onto device +Z keeps the
    // device X/Y directions as close as possible to the screen axes.
    const Quat r0 = Quat::fromTwoVectors(upRefD.normalized(), Vec3::unitZ());
    const Vec3 v = r0.rotate(upD.normalized());
    relXDeg = rad2deg(safeAsin(v.x));
    relYDeg = rad2deg(safeAsin(v.y));
}

float headingOf(const Vec3& v) {
    if (v.x * v.x + v.y * v.y < 1e-10f) return 0.0f;
    return rad2deg(std::atan2(v.y, v.x));
}

namespace {

/// Local tangent basis at the axis d (world frame): h = "up" direction perpendicular
/// to d, s = d x h ("right" when looking along d with h up).
void tangentBasis(const Quat& qWD, const MountModel& mount, const Vec3& d, Vec3& h, Vec3& s,
                  bool& nearVertical) {
    Vec3 hh = rejectFrom(Vec3::unitZ(), d);
    nearVertical = hh.norm() < 0.02f;  // within ~1.1 deg of vertical
    if (hh.norm() < 1e-4f) {
        // exactly vertical: use the display up direction as reference
        Vec3 rightD, upD;
        mount.displayBasis(rightD, upD);
        hh = rejectFrom(qWD.rotate(upD), d);
    }
    h = hh.normalized();
    s = d.cross(h).normalized();
}

}  // namespace

DrillGeometry drillGeometry(const Quat& qWD, const MountModel& mount) {
    DrillGeometry g;
    Vec3 d = qWD.rotate(mount.drillAxisD.normalized(Vec3(0, 0, -1)));
    Vec3 h, s;
    bool nv;
    tangentBasis(qWD, mount, d, h, s, nv);
    if (mount.angleCorrectionDeg != 0.0f) {
        // correction tilts the axis in its vertical plane (+ = upwards)
        const float c = deg2rad(mount.angleCorrectionDeg);
        d = (d * std::cos(c) + h * std::sin(c)).normalized();
        tangentBasis(qWD, mount, d, h, s, nv);
    }
    g.axisW = d;
    g.elevationDeg = rad2deg(safeAsin(d.z));
    g.inclinationDeg = std::fabs(g.elevationDeg);
    g.nearVertical = nv;
    // roll: angle from h to the display-up direction about d
    Vec3 rightD, upD;
    mount.displayBasis(rightD, upD);
    const Vec3 upW = qWD.rotate(upD);
    g.rollDeg = rad2deg(std::atan2(h.cross(upW).dot(d), h.dot(upW)));
    g.headingDeg = headingOf(d);
    return g;
}

Deviation drillDeviation(const Quat& qWD, const MountModel& mount, const DrillTarget& target,
                         HemisphereState& hemi) {
    Deviation r;
    const DrillGeometry g = drillGeometry(qWD, mount);
    const Vec3 d = g.axisW;
    Vec3 h, s;
    bool nv;
    tangentBasis(qWD, mount, d, h, s, nv);

    Vec3 t;
    if (target.kind == TargetKind::Inclination) {
        const float beta = clampv(target.inclinationDeg, 0.0f, 90.0f);
        int8_t sign = target.hemisphere;
        if (sign == 0) {
            // automatic: follow the current hemisphere with hysteresis around horizontal
            const float hyst = 0.5f * beta + 2.0f;
            if (g.elevationDeg > hyst) hemi.current = 1;
            else if (g.elevationDeg < -hyst) hemi.current = -1;
            sign = hemi.current == 0 ? -1 : hemi.current;
        }
        const float et = deg2rad(sign * beta);
        // same azimuth as the current axis -> lateral error is zero by construction
        Vec3 az(d.x, d.y, 0.0f);
        if (az.norm() < 1e-4f) az = Vec3(-h.x, -h.y, 0.0f);  // axis vertical: any azimuth
        if (az.norm() < 1e-6f) az = Vec3(1, 0, 0);
        az = az.normalized();
        t = (az * std::cos(et) + Vec3::unitZ() * std::sin(et)).normalized();
        r.targetElevationDeg = sign * beta;
        r.lateralFree = beta < 89.99f;
        r.lateralFromGyro = false;
    } else {
        t = target.directionW.normalized(Vec3(0, 0, -1));
        r.targetElevationDeg = rad2deg(safeAsin(t.z));
        r.lateralFree = false;
        // lateral information is gravity-observable only for vertical targets
        r.lateralFromGyro = std::fabs(t.z) < 0.9998f;
    }

    // express the target in the local basis around d
    const float tx = t.dot(s), ty = t.dot(h), tz = t.dot(d);
    const float tn = std::sqrt(tx * tx + ty * ty);
    const float theta = std::atan2(tn, tz);
    r.totalDeg = rad2deg(theta);
    float ex = 0.0f, ey = 0.0f;  // where the axis is relative to the target (s/h comps)
    if (tn > 1e-9f) {
        ex = -theta * tx / tn;
        ey = -theta * ty / tn;
    }
    r.elevationErrDeg = rad2deg(ey);
    r.lateralErrDeg = rad2deg(ex);

    // map (s, h) to screen coordinates via the display basis
    Vec3 rightD, upD;
    mount.displayBasis(rightD, upD);
    const Vec3 hD = qWD.rotateInverse(h);
    const Vec3 sD = qWD.rotateInverse(s);
    r.screenX = rad2deg(ex * sD.dot(rightD) + ey * hD.dot(rightD));
    r.screenY = rad2deg(ex * sD.dot(upD) + ey * hD.dot(upD));
    return r;
}

RelativeRotation relativeRotation(const Quat& qRef, const Quat& qNow) {
    RelativeRotation r;
    // rotation from reference to now, expressed in the reference device frame
    const Quat qRel = qRef.conjugate() * qNow;
    const Vec3 rv = qRel.toRotationVector();
    r.rotationDeg = rv * kDegPerRad;
    r.totalDeg = rad2deg(qRel.angle());
    // tilt change (gravity observable): angle between the up vectors
    const Vec3 upRef = qRef.rotateInverse(Vec3::unitZ());
    const Vec3 upNow = qNow.rotateInverse(Vec3::unitZ());
    r.tiltDeg = rad2deg(angleBetween(upRef, upNow));
    // heading change: rotation of a horizontal device-fixed direction about world Z
    Vec3 fwd = qRef.rotate(Vec3::unitY());
    if (Vec3(fwd.x, fwd.y, 0).norm() < 0.3f) fwd = qRef.rotate(Vec3::unitX());
    const Vec3 dirRef = qRef.conjugate().rotate(fwd);  // device frame
    const Vec3 nowW = qNow.rotate(dirRef);
    r.headingDeg = wrap180(headingOf(nowW) - headingOf(fwd));
    return r;
}

}  // namespace sda
