// SPDX-License-Identifier: MIT
// Press classification for the single hardware button (BOOT, GPIO0) which also
// serves as on/off button:
//   short press          -> Short  (configurable action, emitted after the double-press window)
//   two presses < 400 ms -> Double (touch lock)
//   hold >= 0.6 s        -> HoldStart (UI shows the power-off countdown)
//   hold >= 3 s          -> PowerOff
//   release in between   -> HoldCancel (no other action)
// Hardware independent (fed with the debounced-or-raw level and a millisecond clock).
#pragma once

#include <cstdint>

namespace sda {

class ButtonLogic {
public:
    enum class Event : uint8_t { None, Short, Double, HoldStart, HoldCancel, PowerOff };

    struct Config {
        uint32_t debounceMs = 25;
        uint32_t doubleGapMs = 400;
        uint32_t holdStartMs = 600;
        uint32_t powerOffMs = 3000;
    };

    void configure(const Config& c) { cfg_ = c; }
    /// If the button is already pressed at start (e.g. held for power-on), that
    /// press is ignored until it is released.
    void begin(bool pressedNow, uint32_t nowMs);
    Event update(bool pressedRaw, uint32_t nowMs);
    bool pressed() const { return stable_; }
    /// 0..1 progress of the power-off hold (0 when not holding).
    float powerOffProgress(uint32_t nowMs) const;

private:
    Config cfg_;
    bool raw_ = false;
    bool stable_ = false;
    uint32_t changeMs_ = 0;
    uint32_t downMs_ = 0;
    bool ignoreUntilRelease_ = false;
    bool holdStarted_ = false;
    bool powerOffFired_ = false;
    bool pendingShort_ = false;
    uint32_t pendingSinceMs_ = 0;
    bool secondPress_ = false;
};

}  // namespace sda
