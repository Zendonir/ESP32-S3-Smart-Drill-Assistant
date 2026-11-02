// SPDX-License-Identifier: MIT
#include "BatteryModel.h"

#include <cmath>

namespace sda {

float BatteryModel::socFromVoltage(float v) {
    // Typical LiPo open circuit voltage curve (approximate, rest state).
    static const float table[][2] = {
        {3.30f, 0},  {3.50f, 5},  {3.61f, 10}, {3.69f, 20}, {3.73f, 30}, {3.77f, 40},
        {3.81f, 50}, {3.85f, 60}, {3.91f, 70}, {3.98f, 80}, {4.06f, 90}, {4.15f, 100},
    };
    const int n = sizeof(table) / sizeof(table[0]);
    if (!(v > table[0][0])) return 0.0f;
    if (v >= table[n - 1][0]) return 100.0f;
    for (int i = 0; i < n - 1; ++i)
        if (v < table[i + 1][0]) {
            const float t = (v - table[i][0]) / (table[i + 1][0] - table[i][0]);
            return table[i][1] + t * (table[i + 1][1] - table[i][1]);
        }
    return 100.0f;
}

void BatteryModel::update(float volts, float dt) {
    if (!std::isfinite(volts) || volts < 0.0f || volts > 6.0f) return;
    if (!init_) {
        v_ = volts;
        init_ = true;
    } else {
        // 10 s time constant: smooths load steps (backlight, WiFi bursts)
        const float a = dt / (10.0f + dt);
        v_ += (volts - v_) * a;
    }
    // Level with 50 mV hysteresis to avoid toggling warnings
    const float h = 0.05f;
    if (v_ < 2.5f) {
        level_ = BatteryLevel::Unknown;  // no battery connected / divider issue
    } else if (v_ > 4.30f) {
        level_ = BatteryLevel::External;  // above cell maximum: USB supply / charger
    } else if (v_ < critV_ || (level_ == BatteryLevel::Critical && v_ < critV_ + h)) {
        level_ = BatteryLevel::Critical;
    } else if (v_ < lowV_ || (level_ == BatteryLevel::Low && v_ < lowV_ + h)) {
        level_ = BatteryLevel::Low;
    } else {
        level_ = BatteryLevel::Ok;
    }
}

int BatteryModel::percentEstimate() const {
    if (!init_ || level_ == BatteryLevel::Unknown) return -1;
    const float soc = socFromVoltage(v_);
    return static_cast<int>(std::lround(soc / 5.0f) * 5);
}

}  // namespace sda
