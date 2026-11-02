// SPDX-License-Identifier: MIT
#include "CpuMonitor.h"

#include <esp_freertos_hooks.h>
#include <esp_timer.h>

namespace cpu_monitor {
namespace {
struct CoreState {
    volatile int64_t lastHook = 0;
    volatile int64_t idleUs = 0;
    int64_t windowStart = 0;
    float load = -1.0f;
};
CoreState g_core[2];

template <int N>
bool idleHook() {
    CoreState& c = g_core[N];
    const int64_t now = esp_timer_get_time();
    const int64_t gap = now - c.lastHook;
    // consecutive idle hook calls within 200 us mean the core was idle in between
    if (gap > 0 && gap < 200) c.idleUs = c.idleUs + gap;
    c.lastHook = now;
    return true;  // keep calling while idle
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
