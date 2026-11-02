// SPDX-License-Identifier: MIT
// BOOT button (GPIO0, active low) with debouncing and short/long press detection.
#pragma once

#include <cstdint>

class BootButton {
public:
    enum class Event : uint8_t { None, Short, Long };
    void begin();
    /// Call every ~10 ms.
    Event poll(uint32_t nowMs);
    bool pressed() const { return stable_; }

private:
    bool stable_ = false;
    bool raw_ = false;
    uint32_t changeMs_ = 0;
    uint32_t downMs_ = 0;
    bool longFired_ = false;
};
