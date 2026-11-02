// SPDX-License-Identifier: MIT
// Statistics helpers: fixed-capacity ring buffer, running statistics, robust
// estimators and the precision (static averaging) measurement.
#pragma once

#include <cstddef>
#include <cstdint>

#include "MathTypes.h"

namespace sda {

/// Fixed capacity ring buffer without heap allocation.
template <typename T, size_t N>
class RingBuffer {
public:
    void clear() { head_ = 0; size_ = 0; }
    void push(const T& v) {
        buf_[head_] = v;
        head_ = (head_ + 1) % N;
        if (size_ < N) ++size_;
    }
    size_t size() const { return size_; }
    static constexpr size_t capacity() { return N; }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == N; }
    /// i = 0 is the oldest element
    const T& at(size_t i) const { return buf_[(head_ + N - size_ + i) % N]; }
    const T& newest() const { return buf_[(head_ + N - 1) % N]; }

private:
    T buf_[N];
    size_t head_ = 0;
    size_t size_ = 0;
};

/// Welford running mean / variance.
class RunningStats {
public:
    void reset() { n_ = 0; mean_ = 0; m2_ = 0; min_ = 1e30; max_ = -1e30; }
    void add(double x) {
        ++n_;
        const double d = x - mean_;
        mean_ += d / double(n_);
        m2_ += d * (x - mean_);
        if (x < min_) min_ = x;
        if (x > max_) max_ = x;
    }
    uint32_t count() const { return n_; }
    double mean() const { return mean_; }
    double variance() const { return n_ > 1 ? m2_ / double(n_ - 1) : 0.0; }
    double stddev() const { return std::sqrt(variance()); }
    double min() const { return min_; }
    double max() const { return max_; }

private:
    uint32_t n_ = 0;
    double mean_ = 0, m2_ = 0, min_ = 1e30, max_ = -1e30;
};

/// Median and median absolute deviation of `n` values (array is reordered).
float medianInPlace(float* v, size_t n);
float madInPlace(float* v, size_t n, float median);  ///< scaled MAD (x1.4826 ~ sigma)

/// Static precision measurement over a time window. Collects per-sample angle
/// values while the device is still and evaluates mean, median (robust estimate),
/// noise, standard error and linear trend. The samples come from the low-noise
/// accelerometer-only path (see MeasurementPipeline) - no gyro integration.
struct PrecisionConfig {
    float windowSec = 3.0f;      ///< 0.5 .. 10 s
    float maxStdDeg = 0.05f;     ///< sample std threshold for "stabil"
    float maxSlopeDegPerS = 0.02f;
};

struct PrecisionResult {
    bool valid = false;
    bool stable = false;
    uint32_t samples = 0;
    float valueDeg = 0;      ///< robust estimate (median of per-sample values)
    float meanDeg = 0;       ///< mean from vector averaging
    float stdDeg = 0;        ///< per-sample standard deviation (noise)
    float madDeg = 0;        ///< robust spread
    float semDeg = 0;        ///< standard error of the mean (statistical only)
    float slopeDegPerS = 0;  ///< linear trend during the window (drift / creep)
    float secondaryDeg = 0;  ///< second axis (e.g. tilt Y) mean
};

class PrecisionMeasurement {
public:
    static constexpr size_t kMaxSamples = 2400;  ///< 10 s at 224 Hz
    void configure(const PrecisionConfig& c) { cfg_ = c; }
    const PrecisionConfig& config() const { return cfg_; }
    void start();
    void abort() { running_ = false; }
    /// Feed per-sample primary/secondary angle. `still` = motion detector state.
    /// Returns true when the window completed in this call.
    bool add(float primaryDeg, float secondaryDeg, bool still, float dt);
    bool running() const { return running_; }
    float progress() const;
    bool waitingForStill() const { return running_ && !collecting_; }
    const PrecisionResult& result() const { return result_; }

private:
    PrecisionConfig cfg_;
    bool running_ = false;
    bool collecting_ = false;
    float elapsed_ = 0;
    size_t n_ = 0;
    float values_[kMaxSamples];
    double sumSecondary_ = 0;
    PrecisionResult result_;
    void finish();
};

/// Repeatability across several precision results (std of the results).
class RepeatabilityTracker {
public:
    void reset() { stats_.reset(); }
    void add(float v) { stats_.add(v); }
    uint32_t count() const { return stats_.count(); }
    float stdDeg() const { return float(stats_.stddev()); }
    float meanDeg() const { return float(stats_.mean()); }
    float spanDeg() const { return float(stats_.max() - stats_.min()); }

private:
    RunningStats stats_;
};

}  // namespace sda
