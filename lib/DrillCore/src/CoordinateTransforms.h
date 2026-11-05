// SPDX-License-Identifier: MIT
// Coordinate systems (see docs/Mathematics.md, section 1):
//
//   S  sensor frame  - axes of the QMI8658 die as soldered on the PCB
//   D  device frame  - X = screen right, Y = screen up, Z = out of the screen
//   T  tool frame    - X_T = drill axis (towards the bit), Z_T = "top" of the drill
//                      as seen on screen (display up projected), Y_T = Z_T x X_T
//   W  world frame   - Z_W = up (opposite to gravity); X_W/Y_W horizontal with an
//                      arbitrary, gyro-defined heading (NOT observable absolutely)
//   R  reference     - a captured world orientation (zero point / reference surface)
//
// S->D is a fixed signed axis permutation (board property, configurable).
// D->T is the mounting transformation determined by the mount calibration.
// D->W is estimated by the sensor fusion (quaternion q_WD).
#pragma once

#include <cstdint>

#include "MathTypes.h"

namespace sda {

/// Signed axis permutation mapping sensor axes onto device axes.
/// code[i] describes device axis i: value 1..3 = sensor X..Z, negative = inverted.
struct AxisMap {
    int8_t code[3] = {1, 2, 3};

    Mat3 matrix() const;              ///< R_DS (device <- sensor)
    Vec3 apply(const Vec3& s) const;  ///< v_D = R_DS * v_S
    bool isValid() const;             ///< proper rotation (det = +1, permutation)
    /// Parses strings like "+X+Y+Z", "-Y+X+Z". Returns false on invalid input.
    bool parse(const char* str);
    /// Writes "+X+Y+Z" style representation (buffer >= 7 chars).
    void format(char* out) const;
};

/// Automatic determination of the sensor->device axis mapping from two still
/// gravity measurements in SENSOR coordinates:
///   upFlatS     - device lying flat, display facing up      (device +Z = up)
///   upUprightS  - device upright, displayed text readable   (device +Y = up)
/// Each measurement must be within 25 deg of a sensor axis; the two must use
/// different axes. Device X follows from Y x Z (always a proper rotation).
struct AxisDetectResult {
    bool ok = false;
    AxisMap map;
    float orthogonalityErrDeg = 0.0f;  ///< |angle(upFlat, upUpright) - 90 deg|
    const char* error = "";
};
AxisDetectResult detectAxisMap(const Vec3& upFlatS, const Vec3& upUprightS);

/// Rotation that converts device-frame vectors of the old mapping into the new one:
/// v_new = R * v_old with R = A_new * A_old^T.
Mat3 axisRemapRotation(const AxisMap& oldMap, const AxisMap& newMap);

/// Mounting model of the device on a tool. Stores the drill axis in device
/// coordinates and derives an orthonormal display basis perpendicular to it.
struct MountModel {
    enum class Method : uint8_t {
        Default = 0,      ///< no calibration: nominal axis selected by the user
        AxisPreset = 1,   ///< user selected a device axis (coarse, no measurement)
        RollFit = 2,      ///< multi-position rotation about the drill axis (V-block)
        VerticalRef = 3,  ///< drill axis held vertical against a mechanical reference
    };

    Vec3 drillAxisD{0, 0, -1};  ///< unit vector, drill axis in device frame (towards bit)
    Method method = Method::Default;
    float qualityDeg = -1.0f;   ///< RMS residual of the fit in degrees, <0 = unknown
    float angleCorrectionDeg = 0.0f;  ///< additional elevation correction (profile)

    /// Display basis perpendicular to the drill axis (device frame). `upD` is the
    /// device Y axis projected onto the plane perpendicular to the drill axis,
    /// `rightD` = upD x (-drillAxis). If the screen is mounted (almost) parallel to the
    /// drill axis with Y along the axis, device Z is used as fallback.
    void displayBasis(Vec3& rightD, Vec3& upD) const;

    /// Rotation tool <- device. Columns of R_DT are the tool axes in device coordinates.
    Mat3 toolToDevice() const;
    bool isValid() const;
};

/// Presets for the coarse mounting selection.
enum class MountPreset : uint8_t {
    ScreenFacingUser = 0,  ///< display faces the user, drill axis = -Z_D
    TopEdgeForward = 1,    ///< display on the side, device +Y points to the bit
    RightEdgeForward = 2,  ///< device +X points to the bit
    ScreenFacingBit = 3,   ///< display faces the bit (unusual), axis = +Z_D
};
Vec3 mountPresetAxis(MountPreset p);

}  // namespace sda
