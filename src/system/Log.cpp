// SPDX-License-Identifier: MIT
#include "Log.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace sda_log {

namespace {
constexpr size_t kRingSize = 8192;
constexpr size_t kRecentSize = 2048;

struct State {
    char ring[kRingSize];
    size_t head = 0;   // write position
    size_t tail = 0;   // read position
    char recent[kRecentSize];
    size_t recentHead = 0;
    bool recentWrapped = false;
    uint32_t dropped = 0;
    Level level = Level::Info;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    TaskHandle_t task = nullptr;
};
State g;  // module state (file local)

size_t used() { return (g.head + kRingSize - g.tail) % kRingSize; }

void pushRecent(const char* s, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        g.recent[g.recentHead] = s[i];
        g.recentHead = (g.recentHead + 1) % kRecentSize;
        if (g.recentHead == 0) g.recentWrapped = true;
    }
}

void drainTask(void*) {
    char chunk[256];
    for (;;) {
        size_t n = 0;
        portENTER_CRITICAL(&g.mux);
        while (g.tail != g.head && n < sizeof(chunk)) {
            chunk[n++] = g.ring[g.tail];
            g.tail = (g.tail + 1) % kRingSize;
        }
        portEXIT_CRITICAL(&g.mux);
        if (n > 0) {
            Serial.write(reinterpret_cast<const uint8_t*>(chunk), n);
        } else {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        }
    }
}
}  // namespace

void begin(Level level) {
    g.level = level;
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // never block when no host is connected (HWCDC)
}

void setLevel(Level level) { g.level = level; }
Level level() { return g.level; }

void write(Level lvl, const char* tag, const char* fmt, ...) {
    if (static_cast<uint8_t>(lvl) > static_cast<uint8_t>(g.level)) return;
    static const char kLetters[] = {'E', 'W', 'I', 'D'};
    char buf[192];
    const unsigned long ms = millis();
    int n = snprintf(buf, sizeof(buf), "%7lu %c [%s] ", ms, kLetters[static_cast<uint8_t>(lvl) & 3], tag);
    if (n < 0) return;
    va_list ap;
    va_start(ap, fmt);
    const int m = vsnprintf(buf + n, sizeof(buf) - size_t(n) - 2, fmt, ap);
    va_end(ap);
    if (m > 0) n += (m < int(sizeof(buf)) - n - 2) ? m : int(sizeof(buf)) - n - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';
    const size_t len = size_t(n);
    portENTER_CRITICAL(&g.mux);
    if (kRingSize - 1 - used() >= len) {
        for (size_t i = 0; i < len; ++i) {
            g.ring[g.head] = buf[i];
            g.head = (g.head + 1) % kRingSize;
        }
    } else {
        g.dropped++;
    }
    pushRecent(buf, len);
    portEXIT_CRITICAL(&g.mux);
    if (g.task) xTaskNotifyGive(g.task);
}

void startTask(int core, int prio, uint32_t stack) {
    if (g.task) return;
    xTaskCreatePinnedToCore(drainTask, "log", stack, nullptr, prio, &g.task, core);
}

uint32_t droppedMessages() { return g.dropped; }

size_t recent(char* out, size_t n) {
    if (!out || n == 0) return 0;
    size_t w = 0;
    portENTER_CRITICAL(&g.mux);
    const size_t total = g.recentWrapped ? kRecentSize : g.recentHead;
    const size_t start = g.recentWrapped ? g.recentHead : 0;
    const size_t skip = total > n - 1 ? total - (n - 1) : 0;
    for (size_t i = skip; i < total; ++i) out[w++] = g.recent[(start + i) % kRecentSize];
    portEXIT_CRITICAL(&g.mux);
    out[w] = '\0';
    return w;
}

}  // namespace sda_log
