// SPDX-License-Identifier: MIT
// Sensor base calibration in NVS (namespace "sdacal"), separate from settings so
// that a settings reset keeps the calibration and vice versa.
#pragma once

#include "SensorCalibration.h"

class CalibrationStorage {
public:
    bool load(sda::SensorCalibration& out);
    bool save(const sda::SensorCalibration& c);
    bool clear();
    bool lastLoadOk() const { return loadOk_; }

private:
    bool loadOk_ = false;
};
