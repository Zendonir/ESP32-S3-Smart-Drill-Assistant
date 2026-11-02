// SPDX-License-Identifier: MIT
// Persistent user settings (NVS namespace "sda", key "settings", JSON blob).
// Thread safe; changes are debounced and written by the system task, only if the
// content actually changed and only while flash writes are permitted.
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <functional>

#include "Settings.h"

class SettingsManager {
public:
    bool begin();                         ///< load or create defaults
    sda::AppSettings get() const;         ///< copy
    uint32_t version() const { return version_; }
    /// Applies a modification under the lock, validates and schedules a save.
    void update(const std::function<void(sda::AppSettings&)>& fn);
    void replace(const sda::AppSettings& s);
    /// Called periodically by the system task; writes if dirty and debounce elapsed.
    void tick(uint32_t nowMs, bool force = false);
    void factoryReset();
    bool loadedFromFlash() const { return loaded_; }
    int repairedFields() const { return repaired_; }
    uint32_t writeCount() const { return writes_; }

private:
    mutable SemaphoreHandle_t mutex_ = nullptr;
    sda::AppSettings s_;
    volatile uint32_t version_ = 1;
    bool dirty_ = false;
    uint32_t dirtySinceMs_ = 0;
    uint32_t lastHash_ = 0;
    bool loaded_ = false;
    int repaired_ = 0;
    uint32_t writes_ = 0;
    bool save();
};
