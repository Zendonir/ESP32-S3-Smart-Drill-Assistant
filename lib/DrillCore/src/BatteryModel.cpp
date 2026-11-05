// SPDX-License-Identifier: MIT
#include "BatteryModel.h"

#include <cmath>

namespace sda {

const char* chargeStateName(ChargeState s) {
    switch (s) {
        case ChargeState::Unknown: return "unbekannt";
        case ChargeState::Discharging: return "Akkubetrieb";
        case ChargeState::Charging: return "lädt";
        case ChargeState::Full: return "voll (USB)";
        case ChargeState::NoBattery: return "kein Akku?";
    }
    return "?";
}

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

void BatteryModel::reset() {
    init_ = false;
    histN_ = 0;
    histAccum_ = 0;
    trendMvPerMin_ = 0;
    shownPercent_ = -1;
    charge_ = ChargeState::Unknown;
    level_ = BatteryLevel::Unknown;
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

    // voltage history for the trend (one value every 15 s)
    histAccum_ += dt;
    if (histN_ == 0 || histAccum_ >= kHistPeriod) {
        histAccum_ = 0.0f;
        if (histN_ < kHist) {
            hist_[histN_++] = v_;
        } else {
            for (int i = 1; i < kHist; ++i) hist_[i - 1] = hist_[i];
            hist_[kHist - 1] = v_;
        }
        if (histN_ >= 3) {
            const float minutes = (histN_ - 1) * kHistPeriod / 60.0f;
            trendMvPerMin_ = (hist_[histN_ - 1] - hist_[0]) * 1000.0f / minutes;
        }
    }
    updateCharge();

    // displayed percentage: 5 % steps, changes only when the estimate leaves the
    // current step by more than 3 % (no flicker from load changes)
    if (level_ == BatteryLevel::Unknown) {
        shownPercent_ = -1;
    } else {
        const float soc = socFromVoltage(v_);
        const int step = static_cast<int>(std::lround(soc / 5.0f) * 5);
        if (shownPercent_ < 0 || std::fabs(soc - static_cast<float>(shownPercent_)) > 5.5f) shownPercent_ = step;
    }
}

void BatteryModel::updateCharge() {
    if (level_ == BatteryLevel::Unknown) {
        charge_ = ChargeState::NoBattery;
        return;
    }
    if (level_ == BatteryLevel::External) {
        charge_ = ChargeState::Full;  // supply above cell voltage: USB powered
        return;
    }
    const bool history = histN_ >= 5;  // >= 1 minute
    const bool rising = history && trendMvPerMin_ >= (charge_ == ChargeState::Charging ? 2.0f : 6.0f);
    if (usbHost_) {
        charge_ = v_ >= 4.15f && !rising ? ChargeState::Full : ChargeState::Charging;
    } else if (rising) {
        charge_ = ChargeState::Charging;
    } else if (history) {
        charge_ = ChargeState::Discharging;
    } else {
        charge_ = ChargeState::Unknown;
    }
}

int BatteryModel::percentEstimate() const { return init_ ? shownPercent_ : -1; }

}  // namespace sda
