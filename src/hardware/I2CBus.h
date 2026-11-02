// SPDX-License-Identifier: MIT
// Thread-safe I2C master on top of the ESP-IDF 4.4 legacy driver.
// Touch (CST816S) and IMU (QMI8658) share this bus. Every transaction holds a
// FreeRTOS mutex (priority inheritance), so a touch read in the UI task can never
// interleave with an IMU burst read in the high priority IMU task.
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstddef>
#include <cstdint>

class I2CBus {
public:
    bool begin(int port, int sda, int scl, uint32_t freqHz);
    bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len, uint32_t timeoutMs = 5);
    bool writeReg(uint8_t addr, uint8_t reg, uint8_t value, uint32_t timeoutMs = 5);
    bool probe(uint8_t addr, uint32_t timeoutMs = 5);
    /// Clocks out a stuck slave (9 SCL pulses + STOP) and re-installs the driver.
    bool recover();

    struct Stats {
        uint32_t transactions = 0;
        uint32_t errors = 0;
        uint32_t timeouts = 0;
        uint32_t recoveries = 0;
        uint32_t lockTimeouts = 0;
    };
    Stats stats() const { return stats_; }
    bool ready() const { return ready_; }

private:
    int port_ = 0, sda_ = -1, scl_ = -1;
    uint32_t freq_ = 400000;
    bool ready_ = false;
    SemaphoreHandle_t mutex_ = nullptr;
    StaticSemaphore_t mutexBuf_;
    Stats stats_;
    bool install();
    bool lock(uint32_t timeoutMs);
    void unlock();
};
