// SPDX-License-Identifier: MIT
// Zero points and reference surfaces.
//
// Two fundamentally different kinds of reference exist and are kept apart:
//
//  * Gravity referenced (tilt only): stores a gravity direction. Drift free and
//    valid indefinitely, but blind to rotation about the vertical.
//  * Gyro referenced (full 3D relative): stores a complete orientation. The part
//    about the vertical is held only by gyro integration and DRIFTS. Its estimated
//    heading uncertainty and its age are tracked and reported.
//
// The reference surface mode captures the surface normal while the device lies
// flat on the surface (device back on the surface, i.e. outward normal = +Z_D).
// Drilling perpendicular into the surface means the drill axis must point along
// -n. In "restricted" mode only the inclination of -n is used (gravity only); the
// azimuth then stays free and must be aligned by the user (e.g. along the fall
// line). In "3D" mode the full direction is used and the lateral component relies
// on the gyro-held heading.
#pragma once

#include <cstdint>

#include "Orientation.h"

namespace sda {

enum class ReferenceKind : uint8_t { None = 0, TiltZero = 1, Full3D = 2, SurfaceRestricted = 3, Surface3D = 4 };
const char* referenceKindName(ReferenceKind k);

struct ReferenceInfo {
    ReferenceKind kind = ReferenceKind::None;
    float ageSec = 0.0f;
    float headingSigmaDeg = 0.0f;   ///< only meaningful for gyro based kinds
    float tiltSigmaAtCaptureDeg = 0.0f;
    bool usesGyro = false;
    float surfaceSlopeDeg = 0.0f;   ///< inclination of the captured surface (0 = horizontal)
    bool azimuthFree = false;       ///< restricted mode with a sloped surface
};

class ReferenceManager {
public:
    void clear();
    /// Captures the current tilt as zero (gravity referenced).
    void captureTiltZero(const Vec3& upD, float tiltSigmaDeg);
    /// Captures the full orientation (gyro referenced).
    void captureFull3D(const Quat& qWD, float tiltSigmaDeg);
    /// Captures a reference surface; device lies with its back on the surface.
    void captureSurface(const Quat& qWD, bool use3D, float tiltSigmaDeg);

    /// Must be called every fusion step with the heading uncertainty increment.
    void propagate(float dt, float headingSigmaIncrementRad);

    ReferenceKind kind() const { return info_.kind; }
    const ReferenceInfo& info() const { return info_; }
    bool active() const { return info_.kind != ReferenceKind::None; }

    const Vec3& tiltZeroUpD() const { return upRefD_; }
    const Quat& orientationRef() const { return qRef_; }
    /// Drill target derived from a surface reference.
    DrillTarget surfaceTarget() const;
    /// Drill target "keep the current axis direction" from a full 3D zero.
    DrillTarget axisTarget(const MountModel& mount) const;

private:
    ReferenceInfo info_;
    Vec3 upRefD_{0, 0, 1};
    Quat qRef_;
    Vec3 normalW_{0, 0, 1};
};

}  // namespace sda
