// SPDX-License-Identifier: MIT
#include "MountCalibration.h"

#include "LinearAlgebra.h"

namespace sda {

void MountCalibrator::reset() { n_ = 0; }

bool MountCalibrator::addPosition(const Vec3& g, float minSeparationDeg) {
    if (n_ >= kMaxMountPositions || !g.isFinite()) return false;
    const Vec3 u = g.normalized();
    for (int i = 0; i < n_; ++i)
        if (rad2deg(angleBetween(pts_[i], u)) < minSeparationDeg) return false;
    pts_[n_++] = u;
    return true;
}

MountFitResult MountCalibrator::solveRollFit(const Vec3& hint, float minCoverageDeg,
                                             float maxResidualDeg) const {
    MountFitResult r;
    r.positions = n_;
    if (n_ < 3) {
        r.error = MountFitResult::Error::TooFewPositions;
        return r;
    }
    // centroid
    Vec3 c;
    for (int i = 0; i < n_; ++i) c += pts_[i];
    c = c / float(n_);
    // scatter matrix
    Mat3 s = Mat3::zero();
    for (int i = 0; i < n_; ++i) {
        const Vec3 d = pts_[i] - c;
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) s.m[a][b] += d[a] * d[b];
    }
    const SymEigen3Result e = symEigen3(s);
    // The two largest eigenvalues must be clearly non-zero: points span a plane.
    if (e.eigenvalues[1] < 1e-4f) {
        r.error = MountFitResult::Error::Degenerate;
        return r;
    }
    Vec3 axis = e.eigenvectors[0].normalized();
    if (axis.dot(hint) < 0) axis = -axis;

    // residual: angular distance of each point from the fitted cone
    // (cone half angle = angle between axis and centroid direction)
    float coneAngle = 0;
    for (int i = 0; i < n_; ++i) coneAngle += angleBetween(axis, pts_[i]);
    coneAngle /= float(n_);
    double sq = 0;
    for (int i = 0; i < n_; ++i) {
        const double d = angleBetween(axis, pts_[i]) - coneAngle;
        sq += d * d;
    }
    r.residualDeg = rad2deg(float(std::sqrt(sq / n_)));
    r.axisTiltDeg = rad2deg(coneAngle) - 90.0f;  // 0 = axis horizontal during procedure

    // coverage: angular span of the projections around the axis
    Vec3 ref = rejectFrom(pts_[0], axis).normalized();
    const Vec3 ref2 = axis.cross(ref);
    float angles[kMaxMountPositions];
    for (int i = 0; i < n_; ++i) {
        const Vec3 p = rejectFrom(pts_[i], axis);
        angles[i] = std::atan2(p.dot(ref2), p.dot(ref));
    }
    // sort angles and find the largest gap; coverage = 360 - largest gap
    for (int i = 1; i < n_; ++i) {
        const float k = angles[i];
        int j = i - 1;
        while (j >= 0 && angles[j] > k) { angles[j + 1] = angles[j]; --j; }
        angles[j + 1] = k;
    }
    float maxGap = angles[0] + 2.0f * kPi - angles[n_ - 1];
    for (int i = 1; i < n_; ++i) maxGap = std::fmax(maxGap, angles[i] - angles[i - 1]);
    r.coverageDeg = 360.0f - rad2deg(maxGap);

    // Projections must have a meaningful radius (axis must not be near vertical while
    // rolling, otherwise the plane is ill-conditioned).
    if (std::fabs(r.axisTiltDeg) > 60.0f) {
        r.error = MountFitResult::Error::Degenerate;
        return r;
    }
    r.axisD = axis;
    if (r.coverageDeg < minCoverageDeg) {
        r.error = MountFitResult::Error::PoorCoverage;
        return r;
    }
    if (r.residualDeg > maxResidualDeg) {
        r.error = MountFitResult::Error::PoorFit;
        return r;
    }
    r.ok = true;
    return r;
}

MountFitResult MountCalibrator::solveVertical(const Vec3& gravityUpD, bool bitPointsDown) {
    MountFitResult r;
    r.positions = 1;
    const float n = gravityUpD.norm();
    if (!gravityUpD.isFinite() || n < 0.8f || n > 1.2f) {
        r.error = MountFitResult::Error::Degenerate;
        return r;
    }
    const Vec3 up = gravityUpD / n;
    r.axisD = bitPointsDown ? -up : up;
    r.ok = true;
    r.residualDeg = 0.0f;
    r.coverageDeg = 0.0f;
    return r;
}

}  // namespace sda
