// SPDX-License-Identifier: MIT
#include "TimingEstimator.h"

namespace sda {

namespace {
float clampPeriod(float measured, float expected) {
    if (measured < 0.5f * expected) return 0.5f * expected;
    if (measured > 1.5f * expected) return expected;  // gaps must not bias the estimate
    return measured;
}
}  // namespace

void TimingEstimator::configure(float nominalRateHz, uint8_t counterBits, float maxGapSec) {
    nominal_ = nominalRateHz > 1.0f ? nominalRateHz : 100.0f;
    mask_ = counterBits >= 32 ? 0xFFFFFFFFu : ((1u << counterBits) - 1u);
    maxGap_ = maxGapSec;
    stats_ = TimingStats{};
    stats_.nominalRateHz = nominal_;
    reset();
}

void TimingEstimator::reset() {
    period_ = 1.0f / nominal_;
    havePrev_ = false;
    anchorHostUs_ = 0;
    anchorCount_ = 0;
    accumulatedCount_ = 0;
    checkEvents_ = 0;
    checkCountSum_ = 0;
    stats_.counterMode = true;
    stats_.estimatedRateHz = nominal_;
}

void TimingEstimator::changeNominalRate(float r) {
    if (!(r > 1.0f)) return;
    nominal_ = r;
    period_ = 1.0f / r;
    stats_.nominalRateHz = r;
    stats_.estimatedRateHz = r;
    // restart the period estimate from here (old samples had the old period)
    anchorHostUs_ = prevHostUs_;
    anchorCount_ = accumulatedCount_;
}

void TimingEstimator::clearStats() {
    const bool cm = stats_.counterMode;
    const float est = stats_.estimatedRateHz;
    stats_ = TimingStats{};
    stats_.nominalRateHz = nominal_;
    stats_.counterMode = cm;
    stats_.estimatedRateHz = est;
}

TimingStep TimingEstimator::update(uint32_t counter, uint64_t hostTimeUs) {
    TimingStep step;
    counter &= mask_;
    if (!havePrev_) {
        havePrev_ = true;
        prevCounter_ = counter;
        prevHostUs_ = hostTimeUs;
        anchorHostUs_ = hostTimeUs;
        anchorCount_ = 0;
        accumulatedCount_ = 0;
        step.dtSec = period_;
        step.discontinuity = true;
        stats_.totalSamples++;
        return step;
    }

    const uint32_t dCount = (counter - prevCounter_) & mask_;
    const float dHost = static_cast<float>(hostTimeUs - prevHostUs_) * 1e-6f;

    if (stats_.counterMode && dCount == 0) {
        step.duplicate = true;
        stats_.duplicates++;
        return step;  // do not advance prev host time: sample not new
    }

    const float intervalMs = dHost * 1000.0f;
    if (intervalMs > stats_.maxIntervalMs) stats_.maxIntervalMs = intervalMs;

    // Self check during the first 64 events: in counter mode a new sample normally
    // advances the counter by exactly one. If the average advance is far from that
    // the counter is not a sample counter -> fall back to host time.
    if (checkEvents_ < 64) {
        checkEvents_++;
        checkCountSum_ += dCount;
        if (checkEvents_ == 64) {
            const float avg = static_cast<float>(checkCountSum_) / 64.0f;
            if (avg > 3.0f || avg < 0.5f) {
                stats_.counterMode = false;
            }
        }
    }

    if (stats_.counterMode) {
        accumulatedCount_ += dCount;
        // Period estimate from the cumulative baseline since the window anchor
        // (>= 2 s for low noise). The anchor is moved every 30 s so that slow
        // oscillator drift (temperature) is followed.
        const uint64_t spanUs = hostTimeUs - anchorHostUs_;
        const uint64_t spanCount = accumulatedCount_ - anchorCount_;
        if (spanUs > 2000000ULL && spanCount > 0) {
            const float meas = static_cast<float>(spanUs) * 1e-6f / static_cast<float>(spanCount);
            const float nominalPeriod = 1.0f / nominal_;
            // accept only plausible values (+/-25 % of nominal) - otherwise keep old estimate
            if (meas > 0.75f * nominalPeriod && meas < 1.25f * nominalPeriod) period_ = meas;
            if (spanUs > 30000000ULL) {
                anchorHostUs_ = hostTimeUs;
                anchorCount_ = accumulatedCount_;
            }
        }
        if (dCount > 1) {
            step.lostSamples = dCount - 1;
            stats_.lostSamples += dCount - 1;
        }
        step.dtSec = static_cast<float>(dCount) * period_;
    } else {
        step.dtSec = dHost;
        const float expected = 1.0f / nominal_;
        if (dHost > 1.8f * expected) {
            const uint32_t lost = static_cast<uint32_t>(dHost / expected + 0.5f) - 1u;
            step.lostSamples = lost;
            stats_.lostSamples += lost;
        }
        period_ = 0.995f * period_ + 0.005f * clampPeriod(dHost, expected);
    }

    if (step.dtSec > maxGap_ || dHost > maxGap_) {
        step.discontinuity = true;
        stats_.discontinuities++;
        step.dtSec = 1.0f / nominal_;  // do not integrate across the gap
    }
    stats_.estimatedRateHz = 1.0f / period_;
    stats_.totalSamples++;
    prevCounter_ = counter;
    prevHostUs_ = hostTimeUs;
    return step;
}

}  // namespace sda
