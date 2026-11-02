// SPDX-License-Identifier: MIT
// QMI8658 6-axis IMU driver (accelerometer + gyroscope + temperature + sample counter).
//
// Register map and configuration follow the QMI8658A/C datasheet and the
// official Waveshare driver (github.com/waveshareteam/Waveshare-ESP32-components,
// sensor/qmi8658), which is used as hardware reference for this board:
//   WHO_AM_I 0x00 = 0x05, CTRL1 0x60 (address auto increment), CTRL2/3 = (FS << 4) | ODR,
//   CTRL7 bit0/1 = accel/gyro enable, data block 0x30..0x40 (timestamp, temp, accel, gyro),
//   temperature = raw / 256 degC, little endian samples.
// Soft reset: register 0x60 <- 0xB0 (datasheet), completion flag 0x4D == 0x80.
#pragma once

#include <cstdint>

#include "I2CBus.h"
#include "ImuTypes.h"

class QMI8658Driver : public sda::IOrientationSensor {
public:
    enum class AccelRange : uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };
    enum class GyroRange : uint8_t { Dps16 = 0, Dps32, Dps64, Dps128, Dps256, Dps512, Dps1024, Dps2048 };

    struct Config {
        AccelRange accelRange = AccelRange::G8;  // headroom for drill vibration
        GyroRange gyroRange = GyroRange::Dps512;
        uint8_t odrCode = 5;                     // 6DOF: ~224 Hz (gyro clock based)
        bool lowPass = true;                     // internal LPF, mode 3 (~13 % ODR)
    };

    explicit QMI8658Driver(I2CBus& bus) : bus_(bus) {}
    void setConfig(const Config& c) { cfg_ = c; }

    bool begin() override;
    bool readSample(sda::ImuSample& out) override;
    bool isHealthy() const override { return healthy_ && consecutiveErrors_ < 20; }
    const sda::SensorInfo& info() const override { return info_; }
    bool setPowerDown(bool down) override;

    uint8_t address() const { return addr_; }
    uint8_t revision() const { return revision_; }
    bool byteSwapActive() const { return swap_; }
    uint32_t readErrors() const { return readErrors_; }
    uint32_t consecutiveErrors() const { return consecutiveErrors_; }
    /// Re-runs the configuration after a bus recovery.
    bool reinit();
    /// Register dump for diagnostics (CTRL1..CTRL9)
    bool readControlRegisters(uint8_t out[9]);

private:
    I2CBus& bus_;
    Config cfg_;
    sda::SensorInfo info_;
    uint8_t addr_ = 0;
    uint8_t revision_ = 0;
    bool healthy_ = false;
    bool swap_ = false;
    uint32_t lastCounter_ = 0xFFFFFFFF;
    uint32_t readErrors_ = 0;
    uint32_t consecutiveErrors_ = 0;
    float accelLsbPerG_ = 4096.0f;
    float gyroLsbPerDps_ = 64.0f;

    bool configure();
    bool readBlock(uint8_t* b17);
    void decode(const uint8_t* b, bool swap, sda::Vec3& a, sda::Vec3& g, float& t, int16_t raw[6]) const;
};
