// SPDX-License-Identifier: MIT
// Global permission for flash writes. The power manager revokes it when the
// battery is critical (brown-out during an NVS/LittleFS write must be avoided).
#pragma once

#include <atomic>

namespace write_guard {
inline std::atomic<bool>& flag() {
    static std::atomic<bool> allowed{true};
    return allowed;
}
inline bool allowed() { return flag().load(); }
inline void set(bool a) { flag().store(a); }
}  // namespace write_guard
