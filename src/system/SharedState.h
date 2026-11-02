// SPDX-License-Identifier: MIT
// Lock protected single-value mailbox for passing POD snapshots between tasks on
// different cores. Copies are short (memcpy of < 2 KB) and happen inside a
// spinlock critical section, so readers always see a consistent object.
#pragma once

#include <freertos/FreeRTOS.h>

#include <cstring>
#include <type_traits>

template <typename T>
class SharedState {
    static_assert(std::is_trivially_copyable<T>::value, "SharedState requires a POD type");

public:
    void write(const T& v) {
        portENTER_CRITICAL(&mux_);
        std::memcpy(&value_, &v, sizeof(T));
        ++version_;
        portEXIT_CRITICAL(&mux_);
    }
    /// Copies the latest value; returns its version (0 = never written).
    uint32_t read(T& out) const {
        portENTER_CRITICAL(&mux_);
        std::memcpy(&out, &value_, sizeof(T));
        const uint32_t v = version_;
        portEXIT_CRITICAL(&mux_);
        return v;
    }
    uint32_t version() const { return version_; }

private:
    T value_{};
    volatile uint32_t version_ = 0;
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
