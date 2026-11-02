// SPDX-License-Identifier: MIT
// GC9A01A round display (LovyanGFX, SPI + DMA) and LVGL 8.3 integration.
// All LVGL calls are made from the UI task only (see ui/UIManager).
#pragma once

#include <cstdint>

class TouchDriver;

class DisplayDriver {
public:
    bool begin(uint8_t rotation);
    /// Initializes LVGL (buffers, display + input driver). Must be called once
    /// from the task that will run lv_timer_handler().
    bool initLvgl(TouchDriver* touch);
    void setRotation(uint8_t rotation);
    uint8_t rotation() const { return rotation_; }
    /// 0..100 %
    void setBrightness(uint8_t percent);
    uint8_t brightness() const { return brightness_; }
    void sleep();
    void wake();
    bool sleeping() const { return sleeping_; }
    /// Touch statistics for diagnostics
    bool touchPressed() const;
    int16_t touchX() const;
    int16_t touchY() const;
    uint32_t flushCount() const;
    bool ok() const { return ok_; }

private:
    uint8_t rotation_ = 0;
    uint8_t brightness_ = 80;
    bool sleeping_ = false;
    bool ok_ = false;
};
