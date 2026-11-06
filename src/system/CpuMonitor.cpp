// SPDX-License-Identifier: MIT
#include "CpuMonitor.h"

#include <esp_freertos_hooks.h>
#include <esp_timer.h>
#include <hal/cpu_hal.h>

namespace cpu_monitor {
namespace {
struct CoreState {
    volatile int64_t lastHook = 0;
    volatile int64_t idleUs = 0;
    int64_t windowStart = 0;
    float load = -1.0f;
};
CoreState g_core[2];

// The idle task normally executes WAITI (sleep until the next interrupt) after
// the hooks. The hook does that itself and measures how long the core slept:
// that time is idle time. Returning false tells the idle task not to execute
// another WAITI; it calls the hook again right away. Power consumption is the
// same as without the monitor. (The previous version timed the gaps between
// hook calls; with WAITI they are ~1 ms tick periods and the load read ~100 %.)
template <int N>
bool idleHook() {
    CoreState& c = g_core[N];
    const int64_t t0 = esp_timer_get_time();
    cpu_hal_waiti();
    c.idleUs = c.idleUs + (esp_timer_get_time() - t0);
    return false;
}
}  // namespace

void begin() {
    const int64_t now = esp_timer_get_time();
    for (auto& c : g_core) {
        c.windowStart = now;
        c.lastHook = now;
    }
    esp_register_freertos_idle_hook_for_cpu(idleHook<0>, 0);
    esp_register_freertos_idle_hook_for_cpu(idleHook<1>, 1);
}

void update() {
    const int64_t now = esp_timer_get_time();
    for (auto& c : g_core) {
        const int64_t span = now - c.windowStart;
        if (span <= 0) continue;
        const float idle = static_cast<float>(c.idleUs) / static_cast<float>(span);
        float load = 100.0f * (1.0f - idle);
        if (load < 0) load = 0;
        if (load > 100) load = 100;
        c.load = load;
        c.idleUs = 0;
        c.windowStart = now;
    }
}

float loadPercent(int core) { return (core == 0 || core == 1) ? g_core[core].load : -1.0f; }
}  // namespace cpu_monitor
