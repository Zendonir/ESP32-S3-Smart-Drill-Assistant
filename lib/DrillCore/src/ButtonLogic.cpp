// SPDX-License-Identifier: MIT
#include "ButtonLogic.h"

namespace sda {

void ButtonLogic::begin(bool pressedNow, uint32_t nowMs) {
    raw_ = stable_ = pressedNow;
    changeMs_ = downMs_ = nowMs;
    ignoreUntilRelease_ = pressedNow;
    holdStarted_ = powerOffFired_ = pendingShort_ = secondPress_ = false;
}

ButtonLogic::Event ButtonLogic::update(bool pressedRaw, uint32_t now) {
    if (pressedRaw != raw_) {
        raw_ = pressedRaw;
        changeMs_ = now;
    }
    Event ev = Event::None;
    if (now - changeMs_ >= cfg_.debounceMs && stable_ != raw_) {
        stable_ = raw_;
        if (stable_) {  // pressed
            downMs_ = now;
            holdStarted_ = powerOffFired_ = false;
            secondPress_ = pendingShort_ && (now - pendingSinceMs_) < cfg_.doubleGapMs;
            pendingShort_ = false;
        } else {        // released
            if (ignoreUntilRelease_) {
                ignoreUntilRelease_ = false;
            } else if (powerOffFired_) {
                // nothing: device is shutting down
            } else if (holdStarted_) {
                ev = Event::HoldCancel;
            } else if (secondPress_) {
                ev = Event::Double;
            } else {
                pendingShort_ = true;
                pendingSinceMs_ = now;
            }
            secondPress_ = false;
            holdStarted_ = false;
        }
        return ev;
    }
    if (stable_ && !ignoreUntilRelease_) {
        const uint32_t held = now - downMs_;
        if (!holdStarted_ && held >= cfg_.holdStartMs) {
            holdStarted_ = true;
            secondPress_ = false;
            return Event::HoldStart;
        }
        if (holdStarted_ && !powerOffFired_ && held >= cfg_.powerOffMs) {
            powerOffFired_ = true;
            return Event::PowerOff;
        }
    }
    if (!stable_ && pendingShort_ && (now - pendingSinceMs_) >= cfg_.doubleGapMs) {
        pendingShort_ = false;
        return Event::Short;
    }
    return Event::None;
}

float ButtonLogic::powerOffProgress(uint32_t now) const {
    if (!stable_ || !holdStarted_ || ignoreUntilRelease_) return 0.0f;
    const float held = static_cast<float>(now - downMs_);
    const float p = (held - cfg_.holdStartMs) / static_cast<float>(cfg_.powerOffMs - cfg_.holdStartMs);
    return p < 0 ? 0.0f : (p > 1 ? 1.0f : p);
}

}  // namespace sda
