// SPDX-License-Identifier: MIT
// Estimates the CPU load per core from FreeRTOS idle hook timing. The Arduino
// core is built without FreeRTOS run-time statistics, so this is an estimate
// (accuracy a few percent) and is labelled as such in the diagnostics.
#pragma once

#include <cstdint>

namespace cpu_monitor {
void begin();
/// Returns the load (0..100) of the last completed 1 s window, -1 if unknown.
float loadPercent(int core);
/// Must be called about once per second (system task).
void update();
}  // namespace cpu_monitor
