// SPDX-License-Identifier: MIT
#include "ProfileManager.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

#include "../system/Log.h"
#include "../system/WriteGuard.h"
#include "AppConfig.h"
#include "ConfigJson.h"

static const char* TAG = "profiles";
static const char* kPath = "/profiles.json";
static const char* kTmpPath = "/profiles.tmp";

bool ProfileManager::begin(uint8_t& active) {
    mutex_ = xSemaphoreCreateMutex();
    sda::defaultProfiles(ps_);
    fsOk_ = LittleFS.begin(true /*format on fail*/, "/littlefs", 5, "spiffs");
    bool loaded = false;
    if (fsOk_ && LittleFS.exists(kPath)) {
        File f = LittleFS.open(kPath, "r");
        if (f) {
            JsonDocument doc;
            if (deserializeJson(doc, f) == DeserializationError::Ok) {
                sda::ProfileSet tmp;
                if (cfgjson::profilesFromJson(doc.as<JsonObjectConst>(), tmp)) {
                    ps_ = tmp;
                    loaded = true;
                }
            }
            f.close();
        }
        if (!loaded) LOGW(TAG, "profiles file invalid -> defaults");
    }
    const int repaired = sda::validateProfiles(ps_, active);
    if (!loaded || repaired) markDirty();
    LOGI(TAG, "profiles %s, FS %s", loaded ? "loaded" : "default", fsOk_ ? "ok" : "unavailable");
    return true;
}

sda::Profile ProfileManager::get(uint8_t idx) const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sda::Profile p = ps_.profiles[idx < sda::kMaxProfiles ? idx : 0];
    xSemaphoreGive(mutex_);
    return p;
}

sda::ProfileSet ProfileManager::all() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sda::ProfileSet c = ps_;
    xSemaphoreGive(mutex_);
    return c;
}

void ProfileManager::markDirty() {
    dirty_ = true;
    dirtySinceMs_ = millis();
    version_ = version_ + 1;
}

void ProfileManager::update(uint8_t idx, const std::function<void(sda::Profile&)>& fn) {
    if (idx >= sda::kMaxProfiles) return;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    fn(ps_.profiles[idx]);
    sda::validateProfile(ps_.profiles[idx]);
    markDirty();
    xSemaphoreGive(mutex_);
}

int ProfileManager::create(const char* name, uint8_t from) {
    int idx = -1;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    for (int i = 0; i < sda::kMaxProfiles; ++i)
        if (!ps_.profiles[i].used) { idx = i; break; }
    if (idx >= 0) {
        ps_.profiles[idx] = ps_.profiles[from < sda::kMaxProfiles ? from : 0];
        ps_.profiles[idx].used = true;
        sda::copyString(ps_.profiles[idx].name, sizeof(ps_.profiles[idx].name), name);
        sda::validateProfile(ps_.profiles[idx]);
        markDirty();
    }
    xSemaphoreGive(mutex_);
    return idx;
}

bool ProfileManager::remove(uint8_t idx) {
    if (idx >= sda::kMaxProfiles) return false;
    bool ok = false;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    int used = 0;
    for (const auto& p : ps_.profiles) used += p.used ? 1 : 0;
    if (used > 1 && ps_.profiles[idx].used) {
        ps_.profiles[idx] = sda::Profile{};
        markDirty();
        ok = true;
    }
    xSemaphoreGive(mutex_);
    return ok;
}

void ProfileManager::replaceAll(const sda::ProfileSet& ps) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ps_ = ps;
    uint8_t a = 0;
    sda::validateProfiles(ps_, a);
    markDirty();
    xSemaphoreGive(mutex_);
}

bool ProfileManager::save() {
    if (!fsOk_ || !write_guard::allowed()) return false;
    JsonDocument doc;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    cfgjson::profilesToJson(ps_, doc.to<JsonObject>());
    xSemaphoreGive(mutex_);
    // write to a temporary file and rename -> an interrupted write never corrupts
    // the existing profile file
    File f = LittleFS.open(kTmpPath, "w");
    if (!f) return false;
    const size_t n = serializeJson(doc, f);
    f.close();
    if (n == 0) return false;
    LittleFS.remove(kPath);
    if (!LittleFS.rename(kTmpPath, kPath)) return false;
    LOGI(TAG, "profiles saved (%u bytes)", static_cast<unsigned>(n));
    return true;
}

void ProfileManager::tick(uint32_t nowMs, bool force) {
    if (!dirty_) return;
    if (!force && (nowMs - dirtySinceMs_) < appcfg::kSettingsSaveDelayMs) return;
    if (save()) dirty_ = false;
}

void ProfileManager::factoryReset() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sda::defaultProfiles(ps_);
    markDirty();
    xSemaphoreGive(mutex_);
}
