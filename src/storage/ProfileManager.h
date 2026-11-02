// SPDX-License-Identifier: MIT
// Tool profiles (mount calibration + measurement defaults), stored as JSON in
// LittleFS (/profiles.json). Thread safe, debounced writes.
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <functional>

#include "Settings.h"

class ProfileManager {
public:
    bool begin(uint8_t& activeProfile);
    sda::Profile get(uint8_t idx) const;
    sda::ProfileSet all() const;
    uint32_t version() const { return version_; }
    void update(uint8_t idx, const std::function<void(sda::Profile&)>& fn);
    /// Creates a new profile (copy of `from`), returns index or -1 if full.
    int create(const char* name, uint8_t from);
    bool remove(uint8_t idx);  ///< the last used profile cannot be removed
    void replaceAll(const sda::ProfileSet& ps);
    void tick(uint32_t nowMs, bool force = false);
    void factoryReset();
    bool fsOk() const { return fsOk_; }

private:
    mutable SemaphoreHandle_t mutex_ = nullptr;
    sda::ProfileSet ps_;
    volatile uint32_t version_ = 1;
    bool dirty_ = false;
    uint32_t dirtySinceMs_ = 0;
    bool fsOk_ = false;
    bool save();
    void markDirty();
};
