// SPDX-License-Identifier: MIT
#include "AngleTarget.h"

namespace sda {

const char* angleReferenceName(AngleReference r) {
    switch (r) {
        case AngleReference::Horizontal: return "zur Horizontalen";
        case AngleReference::Vertical: return "zur Vertikalen";
        case AngleReference::ZeroUp: return "Nullpunkt + oben";
        case AngleReference::ZeroDown: return "Nullpunkt + unten";
        case AngleReference::ZeroLeft: return "Nullpunkt + links (Gyro)";
        case AngleReference::ZeroRight: return "Nullpunkt + rechts (Gyro)";
        default: return "?";
    }
}

const char* angleReferenceShort(AngleReference r) {
    switch (r) {
        case AngleReference::Horizontal: return "z.Horiz.";
        case AngleReference::Vertical: return "z.Vert.";
        case AngleReference::ZeroUp: return "N+oben";
        case AngleReference::ZeroDown: return "N+unten";
        case AngleReference::ZeroLeft: return "N+links";
        case AngleReference::ZeroRight: return "N+rechts";
        default: return "?";
    }
}

AngleTargetResult makeAngleTarget(AngleReference ref, float angleDeg, int8_t hemi, const ReferenceManager& rm,
                                  const MountModel& mount) {
    AngleTargetResult r;
    const float a = clampv(angleDeg, 0.0f, 90.0f);
    switch (ref) {
        case AngleReference::Horizontal:
        case AngleReference::Vertical: {
            r.target.kind = TargetKind::Inclination;
            r.target.inclinationDeg = ref == AngleReference::Horizontal ? a : 90.0f - a;
            r.target.hemisphere = hemi;
            r.targetElevationDeg = (hemi > 0 ? 1.0f : -1.0f) * r.target.inclinationDeg;
            r.valid = true;
            return r;
        }
        default: break;
    }

    r.needsZero = true;
    r.usesGyro = angleReferenceUsesGyro(ref);
    const ReferenceKind k = rm.kind();
    const bool haveTilt = k == ReferenceKind::TiltZero || k == ReferenceKind::Full3D;
    const bool have3D = k == ReferenceKind::Full3D;
    if (r.usesGyro ? !have3D : !haveTilt) {
        r.problem = "Nullpunkt fehlt";
        return r;
    }
    r.needsZero = false;
    const Vec3 d = mount.drillAxisD.normalized(Vec3(0, 0, -1));

    if (ref == AngleReference::ZeroUp || ref == AngleReference::ZeroDown) {
        // elevation of the drill axis at the zero point from the stored gravity
        // direction (device frame) -> drift free, independent of the heading
        const float eRef = rad2deg(safeAsin(d.dot(rm.tiltZeroUpD()))) + mount.angleCorrectionDeg;
        float eT = eRef + (ref == AngleReference::ZeroUp ? a : -a);
        r.referenceElevationDeg = eRef;
        if (eT > 90.0f || eT < -90.0f) {
            eT = clampv(eT, -90.0f, 90.0f);
            r.problem = "Winkel über die Senkrechte";
            r.targetElevationDeg = eT;
            return r;
        }
        r.target.kind = TargetKind::Inclination;
        r.target.inclinationDeg = std::fabs(eT);
        r.target.hemisphere = eT >= 0.0f ? 1 : -1;
        r.targetElevationDeg = eT;
        r.valid = true;
        return r;
    }

    // left / right: rotate the captured axis about the world vertical
    const Vec3 dRef = drillGeometry(rm.orientationRef(), mount).axisW;
    r.referenceElevationDeg = rad2deg(safeAsin(dRef.z));
    if (std::fabs(dRef.z) > 0.985f) {  // within ~10 deg of vertical: left/right undefined
        r.problem = "Bezugsachse fast senkrecht";
        r.targetElevationDeg = r.referenceElevationDeg;
        return r;
    }
    // positive rotation about +Z turns the axis to the LEFT (seen from above)
    const float th = deg2rad(ref == AngleReference::ZeroLeft ? a : -a);
    r.target.kind = TargetKind::Direction;
    r.target.directionW = Quat::fromAxisAngle(Vec3::unitZ(), th).rotate(dRef).normalized();
    r.targetElevationDeg = r.referenceElevationDeg;
    r.valid = true;
    return r;
}

}  // namespace sda
