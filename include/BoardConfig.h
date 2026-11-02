// SPDX-License-Identifier: MIT
// Board definition: Waveshare ESP32-S3-Touch-LCD-1.28 (round 240x240, GC9A01A,
// CST816S touch, QMI8658 IMU).
//
// IMPORTANT: Do NOT confuse with the "ESP32-S3-LCD-1.28" (no touch) which uses a
// different pin assignment. The values below follow the pin table of the project
// specification and the Waveshare demo code for the *Touch* variant. They must be
// verified against the schematic of the board revision at hand (docs/Hardware.md,
// "Verifikation der Pinbelegung"). The firmware probes the I2C devices at startup
// and reports missing parts instead of silently continuing.
#pragma once

#include <cstdint>

namespace board {

// ---- I2C (shared by CST816S touch and QMI8658 IMU) ------------------------
constexpr int kI2cSda = 6;
constexpr int kI2cScl = 7;
constexpr uint32_t kI2cFreqHz = 400000;       // both devices support Fast-mode
constexpr int kI2cPort = 0;                   // I2C_NUM_0

// ---- LCD GC9A01A (SPI) -----------------------------------------------------
constexpr int kLcdDc = 8;
constexpr int kLcdCs = 9;
constexpr int kLcdClk = 10;
constexpr int kLcdMosi = 11;
constexpr int kLcdMiso = 12;
constexpr int kLcdRst = 14;
constexpr int kLcdBacklight = 2;
constexpr int kLcdWidth = 240;
constexpr int kLcdHeight = 240;
constexpr uint32_t kLcdSpiFreqHz = 80000000;  // write clock; reduce to 40 MHz if artefacts appear
constexpr bool kLcdInvertColors = true;       // GC9A01A panels on this board need inversion
constexpr int kBacklightPwmChannel = 7;
constexpr uint32_t kBacklightPwmFreq = 20000;

// ---- Touch CST816S ---------------------------------------------------------
constexpr int kTouchRst = 13;
constexpr int kTouchInt = 5;
constexpr uint8_t kTouchAddr = 0x15;
// Touch coordinate orientation relative to the panel (rotation 0). Adjust if the
// touch point does not match the display (verify with Diagnose -> Touch-Test).
constexpr bool kTouchSwapXY = false;
constexpr bool kTouchInvertX = false;
constexpr bool kTouchInvertY = false;

// ---- IMU QMI8658 -----------------------------------------------------------
constexpr int kImuInt1 = 4;
constexpr int kImuInt2 = 3;
constexpr uint8_t kImuAddrPrimary = 0x6B;     // SA0 high (Waveshare examples)
constexpr uint8_t kImuAddrSecondary = 0x6A;   // SA0 low (probed as fallback)

// ---- Battery / buttons -------------------------------------------------------
constexpr int kBatteryAdc = 1;                // GPIO1 = ADC1_CH0
// Voltage divider factor (V_bat = V_adc * factor). The Waveshare demo uses 3.0
// (2:1 resistor divider 200k/100k). Configurable in the settings because it must
// be checked against the schematic of the actual revision.
constexpr float kBatteryDividerDefault = 3.0f;
constexpr int kBootButton = 0;                // active low, external pull-up + RTC capable

// Default mapping of QMI8658 sensor axes onto the device frame (X right, Y up,
// Z out of the screen). Identity until verified on hardware with the axis check
// in the diagnostics screen (docs/Hardware.md). Configurable in settings.
constexpr const char* kDefaultAxisMap = "+X+Y+Z";

}  // namespace board
