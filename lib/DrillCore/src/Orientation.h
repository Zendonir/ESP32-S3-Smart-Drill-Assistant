// SPDX-License-Identifier: MIT
// Derivation of displayable angles from the fused orientation.
//
// Observable from gravity (drift free): inclination of any device-fixed axis and
// the roll about it (2 DOF). NOT observable: rotation about the vertical (heading).
// Every function here states which kind of quantity it returns.
#pragma once

#include <cstdint>

#include "CoordinateTransforms.h"
#include "MathTypes.h"

namespace sda {

// ---------------------------------------------------------------------------
// Spirit level (Wasserwaage)
// ---------------------------------------------------------------------------
struct LevelAngles {
    float tiltXDeg = 0;     ///< elevation of the device X axis (right edge up = +)  [gravity]
    float tiltYDeg = 0;     ///< elevation of the device Y axis (top edge up = +)    [gravity]
    float screenTiltDeg = 0;///< angle between screen normal and vertical (0 = flat, face up)
    bool flat = true;       ///< screen closer to horizontal than to vertical -> 2D bubble
    float edgeRollDeg = 0;  ///< standing on an edge: rotation in the vertical plane, (-45,45]
    int edgeIndex = 0;      ///< which edge is down (0..3, multiples of 90 deg)
};
LevelAngles levelAngles(const Vec3& upD);

/// Gravity referenced relative tilt: angles of `upD` in a frame where `upRefD` is "up".
/// Drift free, but blind to rotations about the vertical.
void relativeTilt(const Vec3& upRefD, const Vec3& upD, float& relXDeg, float& relYDeg);

// ---------------------------------------------------------------------------
// Drill axis geometry
// ---------------------------------------------------------------------------
struct DrillGeometry {
    Vec3 axisW;             ///< drill axis in world frame (incl. angle correction)
    float elevationDeg = 0; ///< signed elevation of the axis (+ = bit points upwards) [gravity]
    float inclinationDeg = 0; ///< |elevation|, 0 = horizontal, 90 = vertical       [gravity]
    float rollDeg = 0;      ///< rotation of the tool about its axis (0 = display up = world up) [gravity]
    bool nearVertical = false;
    float headingDeg = 0;   ///< azimuth of the axis in the gyro world frame          [DRIFTS]
};
DrillGeometry drillGeometry(const Quat& qWD, const MountModel& mount);

enum class TargetKind : uint8_t {
    Inclination = 0,  ///< only the inclination is prescribed (azimuth free) - gravity only
    Direction = 1,    ///< full direction in the world frame (lateral part from gyro, drifts)
};

struct DrillTarget {
    TargetKind kind = TargetKind::Inclination;
    float inclinationDeg = 90.0f;  ///< 0 = horizontal, 90 = vertical (Inclination kind)
    int8_t hemisphere = 0;         ///< -1 bit down, +1 bit up, 0 = automatic
    Vec3 directionW{0, 0, -1};     ///< Direction kind
};

/// Hysteresis state for the automatic hemisphere selection.
struct HemisphereState {
    int8_t current = -1;
};

struct Deviation {
    float screenX = 0;        ///< deviation on screen (deg), + = axis points to the right of target
    float screenY = 0;        ///< + = axis points above target (as seen on screen)
    float totalDeg = 0;       ///< angle between axis and target                   (exact)
    float elevationErrDeg = 0;///< component along the vertical plane (+ = too high) [gravity]
    float lateralErrDeg = 0;  ///< component perpendicular to it                    [gyro or 0]
    bool lateralFromGyro = false;  ///< lateral component relies on drifting heading
    bool lateralFree = false;      ///< lateral component is unconstrained (azimuth free)
    float targetElevationDeg = 0;
};

/// Computes the deviation of the drill axis from the target.
Deviation drillDeviation(const Quat& qWD, const MountModel& mount, const DrillTarget& target,
                         HemisphereState& hemi);

// ---------------------------------------------------------------------------
// 3D relative rotation (zero point "3D"): gyro referenced, drifts about vertical
// ---------------------------------------------------------------------------
struct RelativeRotation {
    Vec3 rotationDeg;     ///< rotation vector components (device axes at reference), deg
    float totalDeg = 0;
    float headingDeg = 0; ///< rotation about world vertical since reference   [DRIFTS]
    float tiltDeg = 0;    ///< change of tilt since reference                   [gravity]
};
RelativeRotation relativeRotation(const Quat& qRef, const Quat& qNow);

/// Heading of a world-frame vector (azimuth in the horizontal plane, deg).
float headingOf(const Vec3& vW);

}  // namespace sda
