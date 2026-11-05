// SPDX-License-Identifier: MIT
// Low priority housekeeping task (core 0): battery, power states, button,
// debounced flash writes, OTA confirmation, CPU statistics.
#pragma once

struct AppContext;

namespace system_task {
void start(AppContext& app);
/// Enters deep sleep (wake-up: BOOT button). Called from the system task or UI.
[[noreturn]] void enterDeepSleep(AppContext& app);
/// Switch off: deep sleep, or the charge screen if a charger is connected.
void requestPowerOff(AppContext& app);
/// Stores the parameters for the charger check during deep sleep.
void setChargeWakeConfig(bool enabled, float dividerFactor, float voltageNow);
/// Called first in setup(): after a timer wake-up, measures the battery and
/// returns true if a charger was connected (otherwise the caller sleeps again).
bool chargeWakeCheck();
/// Minimal sleep without application context (used for the power-on hold check).
[[noreturn]] void sleepNow();
/// Must be called early in setup(): releases pin holds set before deep sleep.
void releaseSleepHolds();
}  // namespace system_task
