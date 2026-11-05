// SPDX-License-Identifier: MIT
// Reversal ("Umschlag") calibration of the housing support faces.
//
// The 6-position calibration corrects the SENSOR (offset, scale). It cannot see
// how the sensor chip sits relative to the support faces of the housing
// (soldering, PCB, case: typically some 0.1..2 deg). The reversal measurement
// determines exactly this error, independent of the table tilt:
//
//   position 1:            reading = table tilt + device error
//   rotated 180 deg on the
//   same spot (about the
//   face normal):          reading = -table tilt + device error
//   =>  device error = (r1 + r2) / 2,   table tilt = (r1 - r2) / 2
//
// Faces:
//   Back    device lying flat, display up        (nominal up = +Z_D; angles about X/Y)
//   Bottom  standing on the bottom face, display (nominal up = +Y_D; angles X and Z)
//           facing the user
// The corrections are applied to the spirit level angles and the precision mode.
// They depend on the sensor calibration and are cleared when it changes.
#pragma once

#include <cstdint>

#include "MathTypes.h"
#include "Orientation.h"

namespace sda {

enum class SupportFace : uint8_t { Back = 0, Bottom = 1 };
constexpr int kSupportFaceCount = 2;
const char* supportFaceName(SupportFace f);
Vec3 supportFaceNominalUp(SupportFace f);

struct FaceCalibration {
    bool valid = false;
    float errADeg = 0.0f;       ///< Back: tilt X (right edge up = +); Bottom: tilt X
    float errBDeg = 0.0f;       ///< Back: tilt Y (top edge up = +);   Bottom: tilt Z (screen leans back = +)
    float tableTiltDeg = 0.0f;  ///< tilt of the table during the measurement (informative)
    float spreadDeg = -1.0f;    ///< difference between repeated pairs (<0 = single pair)
    uint8_t pairs = 0;
};

struct FlipResult {
    bool ok = false;
    FaceCalibration cal;
    const char* error = "";
};

/// Gravity "up" directions (calibrated, device frame) of the two positions.
FlipResult solveFlip(SupportFace face, const Vec3& upPos1D, const Vec3& upPos2D);
/// Combines several flip pairs (mean, spread = max - min of the components).
FaceCalibration combineFlips(const FaceCalibration* pairs, int n);

/// Angles of `upD` in the two horizontal axes of the face (deg).
void faceAngles(SupportFace face, const Vec3& upD, float& aDeg, float& bDeg);

/// Applies the face corrections to the level angles (flat -> back face,
/// standing on the bottom edge -> bottom face). Returns true if corrected.
bool applyFaceCorrections(LevelAngles& l, const Vec3& upD, const FaceCalibration& back,
                          const FaceCalibration& bottom);

}  // namespace sda
