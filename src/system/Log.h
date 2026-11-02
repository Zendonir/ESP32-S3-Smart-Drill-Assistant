// SPDX-License-Identifier: MIT
// Non-blocking logger. Formatting happens in the calling task into a small stack
// buffer, the text is copied into a ring buffer and written to USB-Serial by a
// low priority task. Time critical tasks therefore never wait for the USB CDC.
// If the ring buffer is full, messages are dropped and counted.
#pragma once

#include <cstddef>
#include <cstdint>

namespace sda_log {

enum class Level : uint8_t { Error = 0, Warn = 1, Info = 2, Debug = 3 };

void begin(Level level);
void setLevel(Level level);
Level level();
void write(Level lvl, const char* tag, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
/// Starts the drain task (call once after begin()).
void startTask(int core, int prio, uint32_t stack);
uint32_t droppedMessages();
/// Copies the most recent log text (up to n-1 bytes, terminated) for diagnostics.
size_t recent(char* out, size_t n);

}  // namespace sda_log

#define LOGE(tag, ...) ::sda_log::write(::sda_log::Level::Error, tag, __VA_ARGS__)
#define LOGW(tag, ...) ::sda_log::write(::sda_log::Level::Warn, tag, __VA_ARGS__)
#define LOGI(tag, ...) ::sda_log::write(::sda_log::Level::Info, tag, __VA_ARGS__)
#define LOGD(tag, ...) ::sda_log::write(::sda_log::Level::Debug, tag, __VA_ARGS__)
