// SPDX-License-Identifier: MIT
// Low priority housekeeping task (core 0): battery, power states, button,
// debounced flash writes, OTA confirmation, CPU statistics.
#pragma once

struct AppContext;

namespace system_task {
void start(AppContext& app);
/// Enters deep sleep (wake-up: BOOT button). Called from the system task or UI.
[[noreturn]] void enterDeepSleep(AppContext& app);
}  // namespace system_task
