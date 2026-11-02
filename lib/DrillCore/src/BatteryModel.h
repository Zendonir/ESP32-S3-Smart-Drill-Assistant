// SPDX-License-Identifier: MIT
// Battery voltage processing for a single cell Li-Ion/LiPo.
//
// The state of charge is ESTIMATED from the open-circuit-voltage curve of a typical
// LiPo cell. Under load, during charging and with ageing cells the voltage based
// estimate is inaccurate (typ. +/-10..20 %); the UI therefore shows it as "ca." and
// rounded to 5 % steps.
#pragma once

#include <cstdint>

namespace sda {

enum class BatteryLevel : uint8_t { Unknown = 0, Ok, Low, Critical, External };

class BatteryModel {
public:
    void configure(float lowV, float criticalV) { lowV_ = lowV; critV_ = criticalV; }
    void reset() { init_ = false; }
    /// Feed a new (divider corrected) voltage measurement.
    void update(float volts, float dtSec);
    float voltage() const { return v_; }
    bool valid() const { return init_; }
    /// Estimated state of charge 0..100 (rounded to 5 %), -1 if unknown.
    int percentEstimate() const;
    BatteryLevel level() const { return level_; }
    static float socFromVoltage(float v);  ///< 0..100, OCV curve

private:
    bool init_ = false;
    float v_ = 0.0f;
    float lowV_ = 3.5f, critV_ = 3.35f;
    BatteryLevel level_ = BatteryLevel::Unknown;
};

}  // namespace sda
