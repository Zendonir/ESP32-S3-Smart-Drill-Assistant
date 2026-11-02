// SPDX-License-Identifier: MIT
// Mount calibration: determines the drill axis in device coordinates.
//
// Two measurement based procedures are provided. Neither derives the axis from a
// single arbitrary pose:
//
//  1. RollFit ("V-Nut-Verfahren"): the drill (with the device mounted) is rotated
//     about its own axis, e.g. lying with the chuck/bit in a V-block, in >= 3
//     (recommended 4-8) clearly different roll positions. In each still position
//     the gravity direction is recorded in device coordinates. Because the drill
//     axis is fixed in space while rolling, all gravity vectors lie on a cone around
//     the axis: their tips lie in a plane whose normal IS the drill axis. The normal
//     is the eigenvector of the smallest eigenvalue of the scatter matrix.
//     Requirements: roll coverage >= 90 deg, residual small.
//
//  2. VerticalRef ("Bohrstaender"): the drill axis is held exactly vertical against a
//     mechanical reference (drill stand, precision square). One still measurement then
//     gives the axis directly (2 DOF are determined; the roll about the axis is not
//     needed for drilling). The accuracy equals the accuracy of the mechanical reference.
#pragma once

#include <cstdint>

#include "CoordinateTransforms.h"

namespace sda {

constexpr int kMaxMountPositions = 12;

struct MountFitResult {
    bool ok = false;
    Vec3 axisD;
    float residualDeg = 0.0f;   ///< RMS out-of-plane residual (angular)
    float coverageDeg = 0.0f;   ///< angular spread of positions around the axis
    float axisTiltDeg = 0.0f;   ///< inclination of the axis during the procedure
    int positions = 0;
    enum class Error : uint8_t { None, TooFewPositions, PoorCoverage, PoorFit, Degenerate } error =
        Error::None;
};

class MountCalibrator {
public:
    void reset();
    /// Adds the mean (calibrated, device frame) accel vector of one still position.
    /// Positions closer than minSeparationDeg to an existing one are rejected.
    bool addPosition(const Vec3& gravityUpD, float minSeparationDeg = 15.0f);
    int count() const { return n_; }
    /// RollFit: requires >= 3 positions. `hintAxisD` resolves the sign (towards the bit).
    MountFitResult solveRollFit(const Vec3& hintAxisD, float minCoverageDeg = 90.0f,
                                float maxResidualDeg = 1.0f) const;
    /// VerticalRef: drill axis vertical pointing DOWN (bit down) -> axis = -up.
    static MountFitResult solveVertical(const Vec3& gravityUpD, bool bitPointsDown = true);

private:
    Vec3 pts_[kMaxMountPositions];
    int n_ = 0;
};

}  // namespace sda
