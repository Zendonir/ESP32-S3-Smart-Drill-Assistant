// SPDX-License-Identifier: MIT
// BOOT button (GPIO0, active low) = operating and on/off button.
// Press classification: sda::ButtonLogic (unit tested).
#pragma once

#include <cstdint>

#include "ButtonLogic.h"

class BootButton {
public:
    using Event = sda::ButtonLogic::Event;
    void begin();
    /// Call every ~10 ms.
    Event poll(uint32_t nowMs);
    bool pressed() const { return logic_.pressed(); }
    float powerOffProgress(uint32_t nowMs) const { return logic_.powerOffProgress(nowMs); }
    static bool rawPressed();

private:
    sda::ButtonLogic logic_;
};
