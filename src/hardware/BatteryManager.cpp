// SPDX-License-Identifier: MIT
#include "BatteryManager.h"

#include <Arduino.h>

#include "BoardConfig.h"

void BatteryManager::begin(float dividerFactor, float lowV, float criticalV) {
    factor_ = dividerFactor;
    model_.configure(lowV, criticalV);
    analogReadResolution(12);
    analogSetPinAttenuation(board::kBatteryAdc, ADC_11db);  // up to ~3.1 V at the pin
    model_.notifyLoadChange();  // start-up load changes are not plug events
    update(0.5f);
}

float BatteryManager::measureOnce(float factor) {
    analogReadResolution(12);
    analogSetPinAttenuation(board::kBatteryAdc, ADC_11db);
    uint32_t sum = 0;
    for (int i = 0; i < 32; ++i) sum += analogReadMilliVolts(board::kBatteryAdc);
    return static_cast<float>(sum) / 32.0f * factor / 1000.0f;
}

void BatteryManager::update(float dtSec) {
    uint32_t sum = 0;
    for (int i = 0; i < 16; ++i) sum += analogReadMilliVolts(board::kBatteryAdc);  // eFuse calibrated
    rawMv_ = static_cast<float>(sum) / 16.0f;
    model_.update(rawMv_ * factor_ / 1000.0f, dtSec);
}
