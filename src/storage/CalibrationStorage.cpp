// SPDX-License-Identifier: MIT
#include "CalibrationStorage.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#include "../system/Log.h"
#include "../system/WriteGuard.h"
#include "ConfigJson.h"

static const char* TAG = "calib";

bool CalibrationStorage::load(sda::SensorCalibration& out) {
    loadOk_ = false;
    Preferences p;
    if (!p.begin("sdacal", true)) return false;
    const size_t len = p.getBytesLength("sensor");
    if (len == 0 || len > 4000) {
        p.end();
        LOGI(TAG, "no stored sensor calibration");
        return false;
    }
    char* buf = static_cast<char*>(malloc(len + 1));
    if (!buf) {
        p.end();
        return false;
    }
    p.getBytes("sensor", buf, len);
    p.end();
    buf[len] = '\0';
    JsonDocument doc;
    const bool parsed = deserializeJson(doc, buf) == DeserializationError::Ok;
    free(buf);
    sda::SensorCalibration c;
    if (!parsed || !cfgjson::calibrationFromJson(doc.as<JsonObjectConst>(), c)) {
        LOGW(TAG, "stored calibration invalid -> ignored (recalibration required)");
        return false;
    }
    out = c;
    loadOk_ = true;
    LOGI(TAG, "calibration loaded: %d gyro, %d accel points", c.gyroPointCount(), c.accelPointCount());
    return true;
}

bool CalibrationStorage::save(const sda::SensorCalibration& c) {
    if (!write_guard::allowed()) {
        LOGW(TAG, "flash write blocked (battery critical)");
        return false;
    }
    JsonDocument doc;
    cfgjson::calibrationToJson(c, doc.to<JsonObject>());
    String s;
    serializeJson(doc, s);
    Preferences p;
    if (!p.begin("sdacal", false)) return false;
    const size_t w = p.putBytes("sensor", s.c_str(), s.length());
    p.end();
    LOGI(TAG, "calibration saved (%u bytes)", static_cast<unsigned>(s.length()));
    return w == s.length();
}

bool CalibrationStorage::clear() {
    Preferences p;
    if (!p.begin("sdacal", false)) return false;
    const bool ok = p.clear();
    p.end();
    return ok;
}
