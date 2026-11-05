// SPDX-License-Identifier: MIT
// Marks changes of the device's own current consumption (display on/off,
// brightness, WiFi on/off). The battery model ignores voltage steps shortly after
// such events, so they are not mistaken for plugging/unplugging a charger.
#pragma once

#include <atomic>
#include <cstdint>

namespace load_events {
inline std::atomic<uint32_t>& counter() {
    static std::atomic<uint32_t> c{0};
    return c;
}
inline void mark() { counter().fetch_add(1); }
inline uint32_t value() { return counter().load(); }
}  // namespace load_events
