// SPDX-License-Identifier: MIT
// Display related logic that is independent of the graphics library:
//  * One-Euro filter for jitter-free yet responsive numbers/markers
//  * target state machine with tolerance hysteresis and minimum hold time
//  * scale selection (fixed ranges and automatic scaling with hysteresis)
//  * mapping of an angular deviation to a marker position inside the circle
#pragma once

#include <cstdint>

#include "MathTypes.h"

namespace sda {

/// One-Euro filter (Casiez et al., CHI 2012): adaptive low pass whose cutoff rises
/// with the signal speed -> strong smoothing at rest, low lag during motion.
class OneEuroFilter {
public:
    void configure(float minCutoffHz, float beta, float dCutoffHz = 1.0f);
    void reset() { init_ = false; }
    float update(float x, float dt);
    float value() const { return x_; }

private:
    float minCutoff_ = 1.0f, beta_ = 0.05f, dCutoff_ = 1.0f;
    bool init_ = false;
    float x_ = 0, dx_ = 0;
};

enum class TargetState : uint8_t {
    Invalid = 0,  ///< measurement not usable -> grey, never green
    Far = 1,      ///< clear deviation -> red/orange
    Near = 2,     ///< close to the target -> yellow
    Inside = 3,   ///< inside tolerance, hold time running (not yet confirmed)
    Locked = 4,   ///< inside tolerance for the hold time -> green
};
const char* targetStateName(TargetState s);

struct TargetConfig {
    float toleranceDeg = 0.5f;
    float nearFactor = 3.0f;      ///< near zone = nearFactor * tolerance (>= tolerance + 0.5 deg)
    float hysteresis = 0.25f;     ///< leave tolerance only beyond (1 + h) * tolerance
    float holdSec = 0.4f;         ///< time inside before "Locked"
};

class TargetTracker {
public:
    void configure(const TargetConfig& c) { cfg_ = c; }
    const TargetConfig& config() const { return cfg_; }
    void reset();
    /// @param deviationDeg  angular deviation magnitude
    /// @param valid         false -> Invalid (e.g. sensor fault, vibration, high uncertainty)
    TargetState update(float deviationDeg, bool valid, float dt);
    TargetState state() const { return state_; }
    /// true exactly once when entering Locked (for a confirmation animation)
    bool justLocked() const { return justLocked_; }
    float insideSeconds() const { return inside_; }

private:
    TargetConfig cfg_;
    TargetState state_ = TargetState::Invalid;
    float inside_ = 0.0f;
    bool latchedInside_ = false;
    bool justLocked_ = false;
};

enum class ScaleMode : uint8_t { Coarse20 = 0, Normal10 = 1, Fine5 = 2, Precision2 = 3, Auto = 4 };
float scaleRangeDeg(ScaleMode m);
const char* scaleModeName(ScaleMode m);

class AutoScaler {
public:
    void reset() { current_ = 10.0f; lowTime_ = 0; }
    /// Returns the full-scale range (deg) for the current deviation.
    float update(float deviationDeg, float dt);
    float range() const { return current_; }

private:
    float current_ = 10.0f;
    float lowTime_ = 0.0f;
};

/// Marker position for a 2D deviation in a circle of radius `radiusPx`.
struct MarkerPos {
    float x = 0, y = 0;      ///< pixels relative to centre, +y = up
    bool clipped = false;     ///< deviation beyond scale -> show edge arrow
    float directionRad = 0;   ///< direction of the deviation (for the edge arrow)
};
MarkerPos mapDeviation(float devXDeg, float devYDeg, float rangeDeg, float radiusPx);

/// Quantizes a displayed value with hysteresis to avoid flickering last digits.
class StableNumber {
public:
    explicit StableNumber(float step = 0.1f) : step_(step) {}
    void setStep(float s) { step_ = s; }
    float update(float v);
    float value() const { return shown_; }

private:
    float step_;
    float shown_ = 0.0f;
    bool init_ = false;
};

}  // namespace sda
