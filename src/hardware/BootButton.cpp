// SPDX-License-Identifier: MIT
#include "BootButton.h"

#include <Arduino.h>

#include "BoardConfig.h"

namespace {
constexpr uint32_t kDebounceMs = 25;
constexpr uint32_t kLongMs = 1500;
}  // namespace

void BootButton::begin() { pinMode(board::kBootButton, INPUT_PULLUP); }

BootButton::Event BootButton::poll(uint32_t now) {
    const bool r = digitalRead(board::kBootButton) == LOW;
    if (r != raw_) {
        raw_ = r;
        changeMs_ = now;
    }
    Event ev = Event::None;
    if ((now - changeMs_) >= kDebounceMs && stable_ != raw_) {
        stable_ = raw_;
        if (stable_) {
            downMs_ = now;
            longFired_ = false;
        } else if (!longFired_) {
            ev = Event::Short;
        }
    }
    if (stable_ && !longFired_ && (now - downMs_) >= kLongMs) {
        longFired_ = true;
        ev = Event::Long;
    }
    return ev;
}
