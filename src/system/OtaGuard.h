// SPDX-License-Identifier: MIT
// Safe firmware updates using the ESP-IDF app rollback (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
// is set in the sdkconfig of Arduino-ESP32 2.0.17).
//
// After an OTA update the new image boots in state PENDING_VERIFY. This firmware
// confirms it only after the self test passed and the system ran stable for
// kConfirmAfterMs. A crash/reset before that makes the bootloader roll back to the
// previous image automatically; a failed self test triggers the rollback explicitly.
#pragma once

#include <cstdint>

namespace ota_guard {
/// Checks the state of the running image (call early in setup()).
bool pendingVerify();
/// Call from the system task. `selfTestOk` = IMU + display initialised.
void tick(uint32_t uptimeMs, bool selfTestOk);
bool confirmed();
const char* runningPartition();
}  // namespace ota_guard
