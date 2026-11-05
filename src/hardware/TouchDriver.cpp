// SPDX-License-Identifier: MIT
#include "TouchDriver.h"

#include <Arduino.h>

#include "../system/Log.h"
#include "BoardConfig.h"

static const char* TAG = "touch";

namespace {
volatile bool g_touchIrq = false;     // set from ISR
volatile bool g_touchActivity = false;
void IRAM_ATTR touchIsr() {
    g_touchIrq = true;
    g_touchActivity = true;
}
}  // namespace

void TouchDriver::reset() {
    pinMode(board::kTouchRst, OUTPUT);
    digitalWrite(board::kTouchRst, LOW);
    delay(10);
    digitalWrite(board::kTouchRst, HIGH);
    delay(60);
}

bool TouchDriver::begin() {
    reset();
    uint8_t id = 0;
    for (int i = 0; i < 3 && !present_; ++i) {
        present_ = bus_.readRegs(board::kTouchAddr, 0xA7, &id, 1);
        if (!present_) delay(20);
    }
    if (!present_) {
        LOGE(TAG, "CST816S not responding at 0x%02X", board::kTouchAddr);
        return false;
    }
    chipId_ = id;
    bus_.readRegs(board::kTouchAddr, 0xA9, &fw_, 1);
    // keep the controller awake so that I2C reads never NACK during operation
    bus_.writeReg(board::kTouchAddr, 0xFE, 0x01);
    pinMode(board::kTouchInt, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(board::kTouchInt), touchIsr, FALLING);
    LOGI(TAG, "CST816S chip 0x%02X fw 0x%02X", chipId_, fw_);
    return true;
}

bool TouchDriver::read(int16_t& x, int16_t& y, bool& pressed) {
    if (!present_) {
        pressed = false;
        return false;
    }
    const uint32_t now = millis();
    // Read only after an interrupt, while a finger is down, or as a slow safety poll.
    if (!g_touchIrq && !lastPressed_ && (lowPower_ || (now - lastReadMs_) < 200)) {
        x = lastX_;
        y = lastY_;
        pressed = false;
        return true;
    }
    g_touchIrq = false;
    lastReadMs_ = now;
    uint8_t b[6];
    if (!bus_.readRegs(board::kTouchAddr, 0x01, b, 6, 3)) {
        errors_++;
        pressed = false;
        lastPressed_ = false;
        return false;
    }
    const uint8_t fingers = b[1];
    pressed = fingers > 0 && fingers < 3;
    if (pressed) {
        lastX_ = static_cast<int16_t>(((b[2] & 0x0F) << 8) | b[3]);
        lastY_ = static_cast<int16_t>(((b[4] & 0x0F) << 8) | b[5]);
    }
    x = lastX_;
    y = lastY_;
    lastPressed_ = pressed;
    return true;
}

bool TouchDriver::consumeActivity() {
    const bool a = g_touchActivity;
    g_touchActivity = false;
    return a;
}

void TouchDriver::setLowPower(bool lp) {
    if (!present_ || lp == lowPower_) return;
    lowPower_ = lp;
    // 0xFE: 0 = auto sleep allowed, 1 = always awake (fast, reliable I2C)
    if (lp) bus_.writeReg(board::kTouchAddr, 0xFE, 0x00);
    else if (!bus_.writeReg(board::kTouchAddr, 0xFE, 0x01)) wake();  // asleep: reset wakes it
}

void TouchDriver::sleep() {
    if (present_) bus_.writeReg(board::kTouchAddr, 0xE5, 0x03);  // deep sleep (wake via reset)
}

void TouchDriver::wake() {
    if (!present_) return;
    reset();
    bus_.writeReg(board::kTouchAddr, 0xFE, 0x01);
}
