// SPDX-License-Identifier: MIT
#include "ReferenceManager.h"

namespace sda {

const char* referenceKindName(ReferenceKind k) {
    switch (k) {
        case ReferenceKind::None: return "keine";
        case ReferenceKind::TiltZero: return "Neigungs-Nullpunkt";
        case ReferenceKind::Full3D: return "3D-Nullpunkt (Gyro)";
        case ReferenceKind::SurfaceRestricted: return "Fläche (nur Neigung)";
        case ReferenceKind::Surface3D: return "Fläche 3D (Gyro)";
    }
    return "?";
}

void ReferenceManager::clear() {
    info_ = ReferenceInfo{};
    hasQ_ = false;
}

void ReferenceManager::captureTiltZero(const Vec3& upD, float tiltSigmaDeg, const Quat* qWD) {
    info_ = ReferenceInfo{};
    info_.kind = ReferenceKind::TiltZero;
    info_.tiltSigmaAtCaptureDeg = tiltSigmaDeg;
    upRefD_ = upD.normalized();
    hasQ_ = qWD != nullptr;
    if (hasQ_) qRef_ = *qWD;
}

void ReferenceManager::captureFull3D(const Quat& qWD, float tiltSigmaDeg) {
    info_ = ReferenceInfo{};
    info_.kind = ReferenceKind::Full3D;
    info_.usesGyro = true;
    hasQ_ = true;
    info_.tiltSigmaAtCaptureDeg = tiltSigmaDeg;
    qRef_ = qWD;
    upRefD_ = qWD.rotateInverse(Vec3::unitZ());
}

void ReferenceManager::captureSurface(const Quat& qWD, bool use3D, float tiltSigmaDeg) {
    info_ = ReferenceInfo{};
    info_.kind = use3D ? ReferenceKind::Surface3D : ReferenceKind::SurfaceRestricted;
    info_.usesGyro = use3D;
    info_.tiltSigmaAtCaptureDeg = tiltSigmaDeg;
    qRef_ = qWD;
    hasQ_ = true;
    normalW_ = qWD.rotate(Vec3::unitZ()).normalized();
    info_.surfaceSlopeDeg = rad2deg(angleBetween(normalW_, Vec3::unitZ()));
    if (info_.surfaceSlopeDeg > 90.0f) info_.surfaceSlopeDeg = 180.0f - info_.surfaceSlopeDeg;
    info_.azimuthFree = !use3D && info_.surfaceSlopeDeg > 0.05f;
}

void ReferenceManager::propagate(float dt, float inc) {
    if (info_.kind == ReferenceKind::None) return;
    info_.ageSec += dt;
    // also for a tilt zero with orientation (rotation compensation uses the heading)
    if (info_.usesGyro || hasQ_) info_.headingSigmaDeg += rad2deg(inc);
}

DrillTarget ReferenceManager::surfaceTarget() const {
    DrillTarget t;
    const Vec3 into = -normalW_;  // drill into the surface
    if (info_.kind == ReferenceKind::Surface3D) {
        t.kind = TargetKind::Direction;
        t.directionW = into;
    } else {
        t.kind = TargetKind::Inclination;
        const float elev = rad2deg(safeAsin(into.z));
        t.inclinationDeg = std::fabs(elev);
        t.hemisphere = elev < 0 ? -1 : 1;
    }
    return t;
}

DrillTarget ReferenceManager::axisTarget(const MountModel& mount) const {
    DrillTarget t;
    t.kind = TargetKind::Direction;
    t.directionW = qRef_.rotate(mount.drillAxisD.normalized(Vec3(0, 0, -1)));
    return t;
}

}  // namespace sda
