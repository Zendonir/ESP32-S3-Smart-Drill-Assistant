// SPDX-License-Identifier: MIT
// Angle drilling: builds the drill target for "drill at X degrees" with a
// selectable reference.
//
//   Horizontal  X deg against the horizontal plane           absolute, gravity, drift free
//   Vertical    X deg against the plumb line                 absolute, gravity, drift free
//   ZeroUp/Down X deg up/down from a captured axis direction relative, gravity, drift free
//               (tilt in the vertical plane; azimuth stays free)
//   ZeroLeft/Right X deg left/right from the captured axis   relative, GYRO, drifts
//               (rotation about the vertical - not observable absolutely)
//
// A left/right angle "absolute" to something is impossible with a 6-axis IMU
// (no heading reference) and therefore only offered relative to a zero point.
#pragma once

#include <cstdint>

#include "ReferenceManager.h"

namespace sda {

enum class AngleReference : uint8_t {
    Horizontal = 0,
    Vertical = 1,
    ZeroUp = 2,
    ZeroDown = 3,
    ZeroLeft = 4,
    ZeroRight = 5,
    Count
};
const char* angleReferenceName(AngleReference r);   ///< long text (menus)
const char* angleReferenceShort(AngleReference r);  ///< short text (header)
inline bool angleReferenceNeedsZero(AngleReference r) { return r >= AngleReference::ZeroUp; }
inline bool angleReferenceUsesGyro(AngleReference r) {
    return r == AngleReference::ZeroLeft || r == AngleReference::ZeroRight;
}

struct AngleTargetResult {
    DrillTarget target;
    bool valid = false;         ///< target could be built
    bool needsZero = false;     ///< a (suitable) zero point is missing
    bool usesGyro = false;      ///< lateral part relies on the drifting heading
    float targetElevationDeg = 0.0f;  ///< resulting elevation of the target axis (+ = up)
    float referenceElevationDeg = 0.0f;
    const char* problem = "";   ///< short German text when !valid
};

/// @param angleDeg          0..90 (Horizontal/Vertical) or 0..90 offset (zero based)
/// @param hemisphereSetting -1/0/+1 (only for the absolute references)
AngleTargetResult makeAngleTarget(AngleReference ref, float angleDeg, int8_t hemisphereSetting,
                                  const ReferenceManager& rm, const MountModel& mount);

}  // namespace sda
