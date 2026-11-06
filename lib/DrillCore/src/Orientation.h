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
    float edgePitchDeg = 0; ///< standing on an edge: lean of the screen (+ = leaning back)
    bool faceCorrected = false; ///< reversal (Umschlag) correction applied
};
LevelAngles levelAngles(const Vec3& upD);

/// Gravity referenced relative tilt: angles of `upD` in a frame where `upRefD` is "up".
/// Drift free, but blind to rotations about the vertical.
void relativeTilt(const Vec3& upRefD, const Vec3& upD, float& relXDeg, float& relYDeg);

/// Angle between two surfaces measured one after the other with the same device
/// face (digital angle gauge): angle between the two gravity directions in the
/// device frame, 0..180 deg. Gravity based and drift free; like every gravity
/// gauge it measures the full angle only if the common edge of the surfaces is
/// horizontal (a rotation about the vertical is not observable).
float surfaceAngleDeg(const Vec3& upRefD, const Vec3& upD);

/// Change of the surface the device rests on (its back, normal = device +Z)
/// between the reference orientation and now, from the fused 3D orientation.
/// Unlike relativeTilt() this is independent of a rotation of the device on the
/// surface (about its own Z axis): turning the device on the same sloped surface
/// keeps the value at 0. X/Y are expressed in the current screen axes with the
/// same sign convention as relativeTilt() (identical for a pure tilt).
/// The part of the change about the vertical relies on the gyro-held heading;
/// a heading error psi causes up to psi * sin(refSlope) error.
struct NormalDeviation {
    float xDeg = 0.0f;
    float yDeg = 0.0f;
    float totalDeg = 0.0f;    ///< angle between reference normal and current normal
    float refSlopeDeg = 0.0f; ///< slope of the reference surface against the horizontal
};
NormalDeviation surfaceNormalDeviation(const Quat& qRef, const Quat& qNow);

/// Angle of the face the device rests on against the horizontal (0..90 deg):
/// the device axis closest to the vertical defines the support face.
float supportFaceTiltDeg(const Vec3& upD);

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
