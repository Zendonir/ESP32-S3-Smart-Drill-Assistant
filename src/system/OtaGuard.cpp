// SPDX-License-Identifier: MIT
#include "OtaGuard.h"

#include <esp_ota_ops.h>

#include "Log.h"

static const char* TAG = "ota";

// Tell the Arduino core (esp32-hal-misc.c) that the application verifies the
// update itself instead of auto-confirming it in initArduino().
extern "C" bool verifyRollbackLater() { return true; }

namespace ota_guard {
namespace {
constexpr uint32_t kConfirmAfterMs = 30000;
bool g_checked = false;
bool g_pending = false;
bool g_confirmed = false;
}  // namespace

bool pendingVerify() {
    if (!g_checked) {
        g_checked = true;
        const esp_partition_t* running = esp_ota_get_running_partition();
        esp_ota_img_states_t st;
        if (running && esp_ota_get_state_partition(running, &st) == ESP_OK)
            g_pending = st == ESP_OTA_IMG_PENDING_VERIFY;
        if (g_pending) LOGW(TAG, "new firmware image pending verification");
        else g_confirmed = true;
    }
    return g_pending;
}

void tick(uint32_t uptimeMs, bool selfTestOk) {
    if (!g_checked) pendingVerify();
    if (!g_pending || g_confirmed) return;
    if (!selfTestOk && uptimeMs > 5000) {
        LOGE(TAG, "self test failed on new image -> rollback");
        esp_ota_mark_app_invalid_rollback_and_reboot();
        return;
    }
    if (selfTestOk && uptimeMs >= kConfirmAfterMs) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            g_confirmed = true;
            LOGI(TAG, "firmware image confirmed");
        }
    }
}

bool confirmed() { return g_confirmed; }

const char* runningPartition() {
    const esp_partition_t* p = esp_ota_get_running_partition();
    return p ? p->label : "?";
}
}  // namespace ota_guard
