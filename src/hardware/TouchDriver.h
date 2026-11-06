// SPDX-License-Identifier: MIT
// CST816S capacitive touch controller (single touch, I2C 0x15).
// Registers: 0x01 gesture, 0x02 finger count, 0x03..0x06 X/Y (12 bit),
// 0xA7 chip id, 0xA9 firmware version, 0xFE disable auto sleep, 0xE5 sleep.
// The INT line (GPIO5) pulses on touch activity; it is used to avoid needless
// I2C reads when nobody touches the screen.
#pragma once

#include <cstdint>

#include "I2CBus.h"

class TouchDriver {
public:
    explicit TouchDriver(I2CBus& bus) : bus_(bus) {}
    bool begin();
    /// Reads the current touch state (cheap if no interrupt occurred and not pressed).
    bool read(int16_t& x, int16_t& y, bool& pressed);
    bool present() const { return present_; }
    uint8_t chipId() const { return chipId_; }
    uint8_t firmware() const { return fw_; }
    uint32_t errors() const { return errors_; }
    /// True if a touch interrupt occurred since the last call (used for wake-up).
    bool consumeActivity();
    /// A touch interrupt arrived that was not read yet (finger down / moved).
    bool irqPending() const;
    void sleep();
    void wake();
    /// Display off: controller may enter its own auto sleep (still wakes on touch
    /// and raises INT); no periodic polling meanwhile.
    void setLowPower(bool lp);

private:
    I2CBus& bus_;
    bool present_ = false;
    bool lastPressed_ = false;
    int16_t lastX_ = 0, lastY_ = 0;
    uint8_t chipId_ = 0, fw_ = 0;
    uint32_t errors_ = 0;
    uint32_t lastReadMs_ = 0;
    uint32_t failSinceMs_ = 0;   ///< first failed read of the current error streak (0 = none)
    bool lowPower_ = false;
    void reset();
};
