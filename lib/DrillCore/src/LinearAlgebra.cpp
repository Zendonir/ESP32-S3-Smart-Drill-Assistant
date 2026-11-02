// SPDX-License-Identifier: MIT
#include "LinearAlgebra.h"

#include <cstring>

namespace sda {

SymEigen3Result symEigen3(const Mat3& input, int maxSweeps) {
    double a[3][3];
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) a[i][j] = 0.5 * (input.m[i][j] + input.m[j][i]);

    bool converged = false;
    for (int sweep = 0; sweep < maxSweeps; ++sweep) {
        const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        const double scale = a[0][0] * a[0][0] + a[1][1] * a[1][1] + a[2][2] * a[2][2] + 1e-300;
        if (off <= 1e-24 * scale) {
            converged = true;
            break;
        }
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::fabs(a[p][q]) < 1e-300) continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) /
                                 (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;
                for (int k = 0; k < 3; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }

    SymEigen3Result r{};
    int idx[3] = {0, 1, 2};
    // sort ascending by eigenvalue
    for (int i = 0; i < 2; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (a[idx[j]][idx[j]] < a[idx[i]][idx[i]]) {
                const int t = idx[i];
                idx[i] = idx[j];
                idx[j] = t;
            }
    for (int i = 0; i < 3; ++i) {
        const int k = idx[i];
        r.eigenvalues[i] = static_cast<float>(a[k][k]);
        r.eigenvectors[i] = Vec3(static_cast<float>(v[0][k]), static_cast<float>(v[1][k]),
                                 static_cast<float>(v[2][k]))
                                .normalized();
    }
    r.converged = converged;
    return r;
}

namespace {
// Gauss-Jordan elimination with partial pivoting on an augmented n x (n+1) matrix.
bool eliminate(double m[8][9], int n, float* x) {
    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int r = col + 1; r < n; ++r)
            if (std::fabs(m[r][col]) > std::fabs(m[piv][col])) piv = r;
        if (std::fabs(m[piv][col]) < 1e-15) return false;
        if (piv != col)
            for (int k = 0; k <= n; ++k) {
                const double t = m[col][k];
                m[col][k] = m[piv][k];
                m[piv][k] = t;
            }
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            const double f = m[r][col] / m[col][col];
            if (f == 0.0) continue;
            for (int k = col; k <= n; ++k) m[r][k] -= f * m[col][k];
        }
    }
    for (int i = 0; i < n; ++i) x[i] = static_cast<float>(m[i][n] / m[i][i]);
    return true;
}
}  // namespace

bool solveLinear(float* a, float* b, float* x, int n) {
    if (n <= 0 || n > 8) return false;
    double m[8][9];
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) m[i][j] = a[i * n + j];
        m[i][n] = b[i];
    }
    return eliminate(m, n, x);
}

LeastSquares::LeastSquares(int unknowns) : n_(unknowns > kMax ? kMax : unknowns) { reset(); }

void LeastSquares::reset() {
    count_ = 0;
    std::memset(ata_, 0, sizeof(ata_));
    std::memset(atb_, 0, sizeof(atb_));
}

void LeastSquares::add(const float* h, float y, float weight) {
    for (int i = 0; i < n_; ++i) {
        for (int j = 0; j < n_; ++j) ata_[i][j] += weight * double(h[i]) * double(h[j]);
        atb_[i] += weight * double(h[i]) * double(y);
    }
    ++count_;
}

bool LeastSquares::solve(float* x) const {
    if (count_ < n_) return false;
    double m[8][9];
    for (int i = 0; i < n_; ++i) {
        for (int j = 0; j < n_; ++j) m[i][j] = ata_[i][j];
        m[i][n_] = atb_[i];
    }
    return eliminate(m, n_, x);
}

}  // namespace sda
