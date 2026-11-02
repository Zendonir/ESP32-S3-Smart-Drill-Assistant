// SPDX-License-Identifier: MIT
#include "Statistics.h"

#include <algorithm>

namespace sda {

float medianInPlace(float* v, size_t n) {
    if (n == 0) return NAN;
    const size_t mid = n / 2;
    std::nth_element(v, v + mid, v + n);
    const float m = v[mid];
    if (n % 2 == 1) return m;
    const float lowerMax = *std::max_element(v, v + mid);
    return 0.5f * (m + lowerMax);
}

float madInPlace(float* v, size_t n, float median) {
    if (n == 0) return NAN;
    for (size_t i = 0; i < n; ++i) v[i] = std::fabs(v[i] - median);
    return 1.4826f * medianInPlace(v, n);
}

void PrecisionMeasurement::start() {
    running_ = true;
    collecting_ = false;
    elapsed_ = 0;
    n_ = 0;
    sumSecondary_ = 0;
    result_ = PrecisionResult{};
}

float PrecisionMeasurement::progress() const {
    if (!running_) return result_.valid ? 1.0f : 0.0f;
    if (!collecting_) return 0.0f;
    return clamp01(elapsed_ / cfg_.windowSec);
}

bool PrecisionMeasurement::add(float primary, float secondary, bool still, float dt) {
    if (!running_) return false;
    if (!still || !std::isfinite(primary)) {
        // movement during the window: restart and wait for still-stand again
        collecting_ = false;
        elapsed_ = 0;
        n_ = 0;
        sumSecondary_ = 0;
        return false;
    }
    collecting_ = true;
    if (n_ < kMaxSamples) values_[n_++] = primary;
    sumSecondary_ += secondary;
    elapsed_ += dt;
    if (elapsed_ >= cfg_.windowSec) {
        finish();
        return true;
    }
    return false;
}

void PrecisionMeasurement::finish() {
    running_ = false;
    collecting_ = false;
    PrecisionResult r;
    r.samples = static_cast<uint32_t>(n_);
    if (n_ < 10) {
        result_ = r;
        return;
    }
    // mean, std and linear trend (least squares slope vs. sample index)
    RunningStats st;
    double sx = 0, sxx = 0, sxy = 0, sy = 0;
    for (size_t i = 0; i < n_; ++i) {
        st.add(values_[i]);
        const double x = double(i);
        sx += x;
        sxx += x * x;
        sxy += x * values_[i];
        sy += values_[i];
    }
    const double n = double(n_);
    const double denom = n * sxx - sx * sx;
    const double slopePerSample = denom > 0 ? (n * sxy - sx * sy) / denom : 0.0;
    const double samplePeriod = elapsed_ / n;
    r.slopeDegPerS = float(slopePerSample / (samplePeriod > 0 ? samplePeriod : 1.0));
    r.meanDeg = float(st.mean());
    r.stdDeg = float(st.stddev());
    r.semDeg = float(st.stddev() / std::sqrt(n));
    r.valueDeg = medianInPlace(values_, n_);
    r.madDeg = madInPlace(values_, n_, r.valueDeg);
    r.secondaryDeg = float(sumSecondary_ / n);
    r.valid = true;
    r.stable = r.stdDeg <= cfg_.maxStdDeg && std::fabs(r.slopeDegPerS) <= cfg_.maxSlopeDegPerS;
    result_ = r;
}

}  // namespace sda
