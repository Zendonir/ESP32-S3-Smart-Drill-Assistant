// SPDX-License-Identifier: MIT
#include "I2CBus.h"

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_rom_sys.h>

#include "../system/Log.h"

static const char* TAG = "i2c";

bool I2CBus::install() {
    i2c_config_t c = {};
    c.mode = I2C_MODE_MASTER;
    c.sda_io_num = sda_;
    c.scl_io_num = scl_;
    c.sda_pullup_en = GPIO_PULLUP_ENABLE;  // board has external pull-ups; internal as backup
    c.scl_pullup_en = GPIO_PULLUP_ENABLE;
    c.master.clk_speed = freq_;
    c.clk_flags = 0;
    if (i2c_param_config(static_cast<i2c_port_t>(port_), &c) != ESP_OK) return false;
    if (i2c_driver_install(static_cast<i2c_port_t>(port_), I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) return false;
    // Clock stretching timeout: CST816S may stretch while waking up.
    i2c_set_timeout(static_cast<i2c_port_t>(port_), 0x1F);
    return true;
}

bool I2CBus::begin(int port, int sda, int scl, uint32_t freqHz) {
    port_ = port;
    sda_ = sda;
    scl_ = scl;
    freq_ = freqHz;
    if (!mutex_) mutex_ = xSemaphoreCreateMutexStatic(&mutexBuf_);
    ready_ = install();
    if (!ready_) LOGE(TAG, "driver install failed (port %d, SDA %d, SCL %d)", port, sda, scl);
    return ready_;
}

bool I2CBus::lock(uint32_t timeoutMs) {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(timeoutMs + 5)) == pdTRUE) return true;
    stats_.lockTimeouts++;
    return false;
}

void I2CBus::unlock() { xSemaphoreGive(mutex_); }

bool I2CBus::readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len, uint32_t timeoutMs) {
    if (!ready_ || !lock(timeoutMs)) return false;
    const esp_err_t e = i2c_master_write_read_device(static_cast<i2c_port_t>(port_), addr, &reg, 1, buf, len,
                                                     pdMS_TO_TICKS(timeoutMs));
    stats_.transactions++;
    if (e != ESP_OK) {
        stats_.errors++;
        if (e == ESP_ERR_TIMEOUT) stats_.timeouts++;
    }
    unlock();
    return e == ESP_OK;
}

bool I2CBus::writeReg(uint8_t addr, uint8_t reg, uint8_t value, uint32_t timeoutMs) {
    if (!ready_ || !lock(timeoutMs)) return false;
    const uint8_t data[2] = {reg, value};
    const esp_err_t e =
        i2c_master_write_to_device(static_cast<i2c_port_t>(port_), addr, data, 2, pdMS_TO_TICKS(timeoutMs));
    stats_.transactions++;
    if (e != ESP_OK) {
        stats_.errors++;
        if (e == ESP_ERR_TIMEOUT) stats_.timeouts++;
    }
    unlock();
    return e == ESP_OK;
}

bool I2CBus::probe(uint8_t addr, uint32_t timeoutMs) {
    if (!ready_ || !lock(timeoutMs)) return false;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, static_cast<uint8_t>(addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    const esp_err_t e = i2c_master_cmd_begin(static_cast<i2c_port_t>(port_), cmd, pdMS_TO_TICKS(timeoutMs));
    i2c_cmd_link_delete(cmd);
    unlock();
    return e == ESP_OK;
}

bool I2CBus::recover() {
    if (!lock(50)) return false;
    i2c_driver_delete(static_cast<i2c_port_t>(port_));
    // Manual bus clear: 9 clock pulses with SDA released, then a STOP condition.
    gpio_set_direction(static_cast<gpio_num_t>(sda_), GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(static_cast<gpio_num_t>(scl_), GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(static_cast<gpio_num_t>(sda_), 1);
    for (int i = 0; i < 9; ++i) {
        gpio_set_level(static_cast<gpio_num_t>(scl_), 0);
        esp_rom_delay_us(5);
        gpio_set_level(static_cast<gpio_num_t>(scl_), 1);
        esp_rom_delay_us(5);
    }
    gpio_set_level(static_cast<gpio_num_t>(sda_), 0);
    esp_rom_delay_us(5);
    gpio_set_level(static_cast<gpio_num_t>(scl_), 1);
    esp_rom_delay_us(5);
    gpio_set_level(static_cast<gpio_num_t>(sda_), 1);
    esp_rom_delay_us(5);
    ready_ = install();
    stats_.recoveries++;
    unlock();
    LOGW(TAG, "bus recovery %s", ready_ ? "ok" : "FAILED");
    return ready_;
}
