// SPDX-License-Identifier: MIT
#include "BootButton.h"

#include <Arduino.h>

#include "BoardConfig.h"

bool BootButton::rawPressed() { return digitalRead(board::kBootButton) == LOW; }

void BootButton::begin() {
    pinMode(board::kBootButton, INPUT_PULLUP);
    // a press that is still held from switching the device on is ignored
    logic_.begin(rawPressed(), millis());
}

BootButton::Event BootButton::poll(uint32_t now) { return logic_.update(rawPressed(), now); }
