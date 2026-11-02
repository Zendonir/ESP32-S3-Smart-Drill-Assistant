// SPDX-License-Identifier: MIT
// Small dense linear algebra helpers used by calibration routines.
#pragma once

#include "MathTypes.h"

namespace sda {

/// Eigen decomposition of a symmetric 3x3 matrix using cyclic Jacobi rotations.
/// eigenvalues are sorted ascending; eigenvectors[i] belongs to eigenvalues[i].
struct SymEigen3Result {
    float eigenvalues[3];
    Vec3 eigenvectors[3];
    bool converged;
};
SymEigen3Result symEigen3(const Mat3& a, int maxSweeps = 50);

/// Solves the n x n linear system A x = b (n <= 8) by Gaussian elimination with
/// partial pivoting. A is row-major, size n*n. Returns false if singular.
bool solveLinear(float* a, float* b, float* x, int n);

/// Accumulates normal equations for a linear least-squares problem with up to
/// 8 unknowns and an arbitrary number of observations: minimize sum (h^T x - y)^2.
class LeastSquares {
public:
    explicit LeastSquares(int unknowns);
    void reset();
    void add(const float* h, float y, float weight = 1.0f);
    bool solve(float* x) const;
    int observations() const { return count_; }
    int unknowns() const { return n_; }

private:
    static constexpr int kMax = 8;
    int n_;
    int count_ = 0;
    double ata_[kMax][kMax];
    double atb_[kMax];
};

}  // namespace sda
