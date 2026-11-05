// SPDX-License-Identifier: MIT
// Battery voltage processing for a single cell Li-Ion/LiPo.
//
// The state of charge is ESTIMATED from the open-circuit-voltage curve of a typical
// LiPo cell. Under load, during charging and with ageing cells the voltage based
// estimate is inaccurate (typ. +/-10..20 %); the UI therefore shows it as "ca." and
// rounded to 5 % steps with hysteresis.
//
// Charging is not signalled by a dedicated status pin to the ESP32 (to be verified
// in the schematic). It is therefore INFERRED from
//   * a USB host connection (USB Serial/JTAG start-of-frame activity) and/or
//   * a rising voltage trend over a 2 minute window (>= 6 mV/min).
// A wall charger without data lines is only detected via the voltage trend.
#pragma once

#include <cstdint>

namespace sda {

enum class BatteryLevel : uint8_t { Unknown = 0, Ok, Low, Critical, External };
enum class ChargeState : uint8_t {
    Unknown = 0,      ///< not enough history yet
    Discharging = 1,
    Charging = 2,     ///< inferred: USB host and/or rising voltage
    Full = 3,         ///< USB connected and voltage at the charge end (~4.15 V+)
    NoBattery = 4,    ///< implausible voltage (no cell connected?)
};
const char* chargeStateName(ChargeState s);

class BatteryModel {
public:
    void configure(float lowV, float criticalV) { lowV_ = lowV; critV_ = criticalV; }
    void reset();
    /// Feed a new (divider corrected) voltage measurement.
    void update(float volts, float dtSec);
    /// USB host present (from the USB Serial/JTAG controller), optional input.
    void setUsbHost(bool connected) { usbHost_ = connected; }
    float voltage() const { return v_; }
    bool valid() const { return init_; }
    /// Estimated state of charge 0..100 in 5 % steps (with hysteresis), -1 if unknown.
    int percentEstimate() const;
    BatteryLevel level() const { return level_; }
    ChargeState chargeState() const { return charge_; }
    float trendMvPerMin() const { return trendMvPerMin_; }
    bool usbHost() const { return usbHost_; }
    static float socFromVoltage(float v);  ///< 0..100, OCV curve

private:
    static constexpr int kHist = 9;           ///< 8 intervals x 15 s = 2 min
    static constexpr float kHistPeriod = 15.0f;
    bool init_ = false;
    float v_ = 0.0f;
    float lowV_ = 3.5f, critV_ = 3.35f;
    BatteryLevel level_ = BatteryLevel::Unknown;
    ChargeState charge_ = ChargeState::Unknown;
    bool usbHost_ = false;
    float hist_[kHist] = {};
    int histN_ = 0;
    float histAccum_ = 0.0f;
    float trendMvPerMin_ = 0.0f;
    int shownPercent_ = -1;
    void updateCharge();
};

}  // namespace sda
