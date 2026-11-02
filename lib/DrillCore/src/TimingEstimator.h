// SPDX-License-Identifier: MIT
// Derives integration time steps, real output data rate and lost samples from
// the sensor's sample counter and the host clock.
//
// The QMI8658 provides a 24 bit sample counter that increments once per output
// sample. Its internal oscillator has a tolerance of a few percent, so the
// nominal ODR must not be used directly for gyro integration. The estimator
// measures the actual sample period against the host's crystal clock over a
// sliding window and uses  dt = (counter delta) * (estimated period).
//
// If the counter does not behave like a sample counter (e.g. a different sensor
// or chip revision), the estimator automatically falls back to host time stamps.
#pragma once

#include <cstdint>

namespace sda {

struct TimingStep {
    float dtSec = 0.0f;         ///< integration step for this sample
    uint32_t lostSamples = 0;   ///< samples skipped since the previous one
    bool duplicate = false;     ///< no new data (same counter value)
    bool discontinuity = false; ///< gap too long, filters should re-initialize
};

struct TimingStats {
    float estimatedRateHz = 0.0f;
    float nominalRateHz = 0.0f;
    uint32_t totalSamples = 0;
    uint32_t lostSamples = 0;
    uint32_t duplicates = 0;
    uint32_t discontinuities = 0;
    float maxIntervalMs = 0.0f;  ///< longest host interval between consecutive samples
    bool counterMode = true;     ///< true: sample counter used, false: host time fallback
};

class TimingEstimator {
public:
    /// @param nominalRateHz   datasheet ODR
    /// @param counterBits     width of the sensor counter (24 for QMI8658)
    void configure(float nominalRateHz, uint8_t counterBits = 24, float maxGapSec = 0.1f);
    void reset();
    TimingStep update(uint32_t counter, uint64_t hostTimeUs);
    const TimingStats& stats() const { return stats_; }
    float periodSec() const { return period_; }
    void clearStats();

private:
    float nominal_ = 224.0f;
    float period_ = 1.0f / 224.0f;
    uint32_t mask_ = 0xFFFFFF;
    float maxGap_ = 0.1f;
    bool havePrev_ = false;
    uint32_t prevCounter_ = 0;
    uint64_t prevHostUs_ = 0;
    // sliding window anchor for the period estimate
    uint64_t anchorHostUs_ = 0;
    uint64_t anchorCount_ = 0;
    uint64_t accumulatedCount_ = 0;
    // counter-mode self check
    uint32_t checkEvents_ = 0;
    uint64_t checkCountSum_ = 0;
    TimingStats stats_;
};

}  // namespace sda
