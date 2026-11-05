// SPDX-License-Identifier: MIT
// Battery voltage measurement via ADC1_CH0 (GPIO1) and the on-board divider.
#pragma once

#include "BatteryModel.h"

class BatteryManager {
public:
    void begin(float dividerFactor, float lowV, float criticalV);
    void setDivider(float f) { factor_ = f; }
    void setUsbHost(bool c) { model_.setUsbHost(c); }
    /// Samples the ADC (16x averaging, eFuse calibrated) and updates the model.
    void update(float dtSec);
    const sda::BatteryModel& model() const { return model_; }
    sda::BatteryModel& model() { return model_; }
    /// One-shot battery voltage measurement without model (used right after a
    /// timer wake-up, before the application starts).
    static float measureOnce(float dividerFactor);
    float lastRawMilliVolts() const { return rawMv_; }

private:
    float factor_ = 3.0f;
    float rawMv_ = 0.0f;
    sda::BatteryModel model_;
};
