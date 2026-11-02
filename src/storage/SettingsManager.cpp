// SPDX-License-Identifier: MIT
#include "SettingsManager.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_random.h>

#include "../system/Log.h"
#include "../system/WriteGuard.h"
#include "AppConfig.h"
#include "BoardConfig.h"
#include "ConfigJson.h"

static const char* TAG = "settings";

namespace {
uint32_t fnv1a(const char* s, size_t n) {
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; ++i) {
        h ^= static_cast<uint8_t>(s[i]);
        h *= 16777619u;
    }
    return h;
}

void randomPassword(char* out, size_t len) {
    // unambiguous characters only (no 0/O, 1/l)
    static const char alphabet[] = "abcdefghijkmnpqrstuvwxyzACDEFHJKLMNPQRTUVWXY23456789";
    for (size_t i = 0; i < len; ++i) out[i] = alphabet[esp_random() % (sizeof(alphabet) - 1)];
    out[len] = '\0';
}
}  // namespace

bool SettingsManager::begin() {
    mutex_ = xSemaphoreCreateMutex();
    sda::defaultSettings(s_);
    s_.system.axisMap.parse(board::kDefaultAxisMap);
    s_.system.batteryDividerFactor = board::kBatteryDividerDefault;

    Preferences p;
    if (p.begin("sda", true)) {
        const size_t len = p.getBytesLength("settings");
        if (len > 0 && len < 4096) {
            char* buf = static_cast<char*>(malloc(len + 1));
            if (buf) {
                p.getBytes("settings", buf, len);
                buf[len] = '\0';
                JsonDocument doc;
                if (deserializeJson(doc, buf) == DeserializationError::Ok) {
                    sda::AppSettings tmp = s_;
                    const int ver = doc["version"] | 0;
                    // Migration: version 1 is the first format. Unknown future versions
                    // are still read field by field (unknown keys ignored).
                    if (ver >= 1 && cfgjson::settingsFromJson(doc.as<JsonObjectConst>(), tmp)) {
                        s_ = tmp;
                        loaded_ = true;
                    } else {
                        LOGW(TAG, "stored settings rejected (version %d) -> defaults", ver);
                    }
                } else {
                    LOGW(TAG, "stored settings corrupt -> defaults");
                }
                lastHash_ = fnv1a(buf, len);
                free(buf);
            }
        }
        p.end();
    }
    repaired_ = sda::validateSettings(s_);
    bool needSave = !loaded_ || repaired_ > 0;
    // first boot: generate credentials (shown on the device, System -> WLAN)
    if (s_.system.apPassword[0] == '\0') {
        randomPassword(s_.system.apPassword, 10);
        needSave = true;
    }
    if (s_.system.adminPassword[0] == '\0') {
        randomPassword(s_.system.adminPassword, 10);
        needSave = true;
    }
    LOGI(TAG, "settings %s (%d repaired)", loaded_ ? "loaded" : "defaults", repaired_);
    if (needSave) {
        dirty_ = true;
        dirtySinceMs_ = 0;
    }
    return true;
}

sda::AppSettings SettingsManager::get() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sda::AppSettings c = s_;
    xSemaphoreGive(mutex_);
    return c;
}

void SettingsManager::update(const std::function<void(sda::AppSettings&)>& fn) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    fn(s_);
    sda::validateSettings(s_);
    // validation empties invalid credentials -> never leave them empty (open access)
    if (s_.system.apPassword[0] == '\0') randomPassword(s_.system.apPassword, 10);
    if (s_.system.adminPassword[0] == '\0') randomPassword(s_.system.adminPassword, 10);
    dirty_ = true;
    dirtySinceMs_ = millis();
    version_ = version_ + 1;
    xSemaphoreGive(mutex_);
}

void SettingsManager::replace(const sda::AppSettings& s) {
    update([&](sda::AppSettings& d) { d = s; });
}

bool SettingsManager::save() {
    if (!write_guard::allowed()) return false;
    JsonDocument doc;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    cfgjson::settingsToJson(s_, doc.to<JsonObject>(), true);
    xSemaphoreGive(mutex_);
    String out;
    serializeJson(doc, out);
    const uint32_t h = fnv1a(out.c_str(), out.length());
    if (h == lastHash_) return true;  // unchanged: no flash write
    Preferences p;
    if (!p.begin("sda", false)) return false;
    const size_t w = p.putBytes("settings", out.c_str(), out.length());
    p.end();
    if (w != out.length()) {
        LOGE(TAG, "NVS write failed");
        return false;
    }
    lastHash_ = h;
    writes_++;
    LOGI(TAG, "settings saved (%u bytes)", static_cast<unsigned>(out.length()));
    return true;
}

void SettingsManager::tick(uint32_t nowMs, bool force) {
    if (!dirty_) return;
    if (!force && (nowMs - dirtySinceMs_) < appcfg::kSettingsSaveDelayMs) return;
    if (save()) dirty_ = false;
}

void SettingsManager::factoryReset() {
    Preferences p;
    if (p.begin("sda", false)) {
        p.clear();
        p.end();
    }
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sda::defaultSettings(s_);
    s_.system.axisMap.parse(board::kDefaultAxisMap);
    lastHash_ = 0;
    xSemaphoreGive(mutex_);
}
