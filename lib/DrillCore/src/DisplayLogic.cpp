// SPDX-License-Identifier: MIT
#include "DisplayLogic.h"

namespace sda {

namespace {
inline float smoothingAlpha(float cutoffHz, float dt) {
    const float tau = 1.0f / (2.0f * kPi * cutoffHz);
    return 1.0f / (1.0f + tau / dt);
}
}  // namespace

void OneEuroFilter::configure(float minCutoffHz, float beta, float dCutoffHz) {
    minCutoff_ = minCutoffHz > 0.001f ? minCutoffHz : 0.001f;
    beta_ = beta >= 0 ? beta : 0;
    dCutoff_ = dCutoffHz > 0.001f ? dCutoffHz : 1.0f;
}

float OneEuroFilter::update(float x, float dt) {
    if (!std::isfinite(x)) return x_;
    if (!init_ || !(dt > 0.0f)) {
        x_ = x;
        dx_ = 0.0f;
        init_ = true;
        return x_;
    }
    const float rawDx = (x - x_) / dt;
    dx_ += smoothingAlpha(dCutoff_, dt) * (rawDx - dx_);
    const float cutoff = minCutoff_ + beta_ * std::fabs(dx_);
    x_ += smoothingAlpha(cutoff, dt) * (x - x_);
    return x_;
}

// ----------------------------------------------------------------------------
const char* targetStateName(TargetState s) {
    switch (s) {
        case TargetState::Invalid: return "ungültig";
        case TargetState::Far: return "weit";
        case TargetState::Near: return "nah";
        case TargetState::Inside: return "innen";
        case TargetState::Locked: return "ok";
    }
    return "?";
}

void TargetTracker::reset() {
    state_ = TargetState::Invalid;
    inside_ = 0.0f;
    latchedInside_ = false;
    justLocked_ = false;
}

TargetState TargetTracker::update(float dev, bool valid, float dt) {
    justLocked_ = false;
    if (!valid || !std::isfinite(dev)) {
        state_ = TargetState::Invalid;
        inside_ = 0.0f;
        latchedInside_ = false;
        return state_;
    }
    const float tol = cfg_.toleranceDeg > 0.01f ? cfg_.toleranceDeg : 0.01f;
    const float exitTol = tol * (1.0f + cfg_.hysteresis);
    if (latchedInside_) {
        if (dev > exitTol) latchedInside_ = false;
    } else if (dev <= tol) {
        latchedInside_ = true;
    }
    if (latchedInside_) {
        inside_ += dt;
        if (inside_ >= cfg_.holdSec) {
            if (state_ != TargetState::Locked) justLocked_ = true;
            state_ = TargetState::Locked;
        } else {
            state_ = TargetState::Inside;
        }
        return state_;
    }
    inside_ = 0.0f;
    const float nearTol = std::fmax(tol * cfg_.nearFactor, tol + 0.5f);
    // small hysteresis between Near and Far as well
    if (state_ == TargetState::Near || state_ == TargetState::Inside || state_ == TargetState::Locked)
        state_ = dev > nearTol * 1.1f ? TargetState::Far : TargetState::Near;
    else
        state_ = dev <= nearTol ? TargetState::Near : TargetState::Far;
    return state_;
}

// ----------------------------------------------------------------------------
float scaleRangeDeg(ScaleMode m) {
    switch (m) {
        case ScaleMode::Coarse20: return 20.0f;
        case ScaleMode::Normal10: return 10.0f;
        case ScaleMode::Fine5: return 5.0f;
        case ScaleMode::Precision2: return 2.0f;
        case ScaleMode::Auto: return 10.0f;
    }
    return 10.0f;
}

const char* scaleModeName(ScaleMode m) {
    switch (m) {
        case ScaleMode::Coarse20: return "Grob 20";
        case ScaleMode::Normal10: return "Normal 10";
        case ScaleMode::Fine5: return "Fein 5";
        case ScaleMode::Precision2: return "Präz. 2";
        case ScaleMode::Auto: return "Auto";
    }
    return "?";
}

float AutoScaler::update(float dev, float dt) {
    static const float ranges[] = {2.0f, 5.0f, 10.0f, 20.0f};
    if (!std::isfinite(dev)) return current_;
    // grow immediately when the marker would leave 90 % of the circle
    if (dev > 0.9f * current_) {
        for (float r : ranges)
            if (dev <= 0.9f * r || r == 20.0f) {
                if (r > current_) current_ = r;
                break;
            }
        lowTime_ = 0.0f;
        return current_;
    }
    // shrink only after the deviation stayed below 35 % of the smaller range for 0.8 s
    float smaller = current_;
    for (int i = 3; i >= 0; --i)
        if (ranges[i] < current_) { smaller = ranges[i]; break; }
    if (smaller < current_ && dev < 0.35f * smaller) {
        lowTime_ += dt;
        if (lowTime_ >= 0.8f) {
            current_ = smaller;
            lowTime_ = 0.0f;
        }
    } else {
        lowTime_ = 0.0f;
    }
    return current_;
}

// ----------------------------------------------------------------------------
MarkerPos mapDeviation(float dx, float dy, float range, float radius) {
    MarkerPos p;
    const float mag = std::sqrt(dx * dx + dy * dy);
    p.directionRad = std::atan2(dy, dx);
    if (!(range > 0.0f) || !std::isfinite(mag)) return p;
    float r = mag / range * radius;  // linear (proportional) mapping
    if (r > radius) {
        r = radius;
        p.clipped = true;
    }
    if (mag > 1e-9f) {
        p.x = dx / mag * r;
        p.y = dy / mag * r;
    }
    return p;
}

float StableNumber::update(float v) {
    if (!std::isfinite(v)) return shown_;
    if (!init_) {
        shown_ = std::round(v / step_) * step_;
        init_ = true;
        return shown_;
    }
    // change the shown value only if v left the current bin by 60 % of a step
    if (std::fabs(v - shown_) > 0.6f * step_) shown_ = std::round(v / step_) * step_;
    return shown_;
}

}  // namespace sda
