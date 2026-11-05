// SPDX-License-Identifier: MIT
#include "IMUDriver.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "../system/Log.h"
#include "BoardConfig.h"

static const char* TAG = "imu";

namespace {
constexpr uint8_t REG_WHO_AM_I = 0x00;
constexpr uint8_t REG_REVISION = 0x01;
constexpr uint8_t REG_CTRL1 = 0x02;
constexpr uint8_t REG_CTRL2 = 0x03;
constexpr uint8_t REG_CTRL3 = 0x04;
constexpr uint8_t REG_CTRL5 = 0x06;
constexpr uint8_t REG_CTRL7 = 0x08;
constexpr uint8_t REG_TIMESTAMP_L = 0x30;
constexpr uint8_t REG_RESET = 0x60;
constexpr uint8_t REG_RESET_STATUS = 0x4D;
constexpr uint8_t WHO_AM_I_VALUE = 0x05;
constexpr uint8_t CTRL1_ADDR_AI = 0x40;   // register address auto increment
constexpr uint8_t CTRL1_BE = 0x20;        // as in the Waveshare reference (data read LE, verified at runtime)
constexpr uint8_t CTRL1_SENSOR_DISABLE = 0x01;
constexpr uint8_t CTRL7_AEN = 0x01;
constexpr uint8_t CTRL7_GEN = 0x02;
}  // namespace

bool QMI8658Driver::begin() {
    healthy_ = false;
    const uint8_t candidates[2] = {board::kImuAddrPrimary, board::kImuAddrSecondary};
    for (uint8_t a : candidates) {
        uint8_t who = 0;
        if (bus_.readRegs(a, REG_WHO_AM_I, &who, 1) && who == WHO_AM_I_VALUE) {
            addr_ = a;
            break;
        }
    }
    if (addr_ == 0) {
        LOGE(TAG, "QMI8658 not found (WHO_AM_I != 0x05 at 0x6B/0x6A)");
        return false;
    }
    bus_.readRegs(addr_, REG_REVISION, &revision_, 1);
    LOGI(TAG, "QMI8658 at 0x%02X, revision 0x%02X", addr_, revision_);

    // Soft reset and wait for completion (max 50 ms)
    bus_.writeReg(addr_, REG_RESET, 0xB0);
    delay(15);
    for (int i = 0; i < 35; ++i) {
        uint8_t st = 0;
        if (bus_.readRegs(addr_, REG_RESET_STATUS, &st, 1) && st == 0x80) break;
        delay(1);
    }
    return configure();
}

bool QMI8658Driver::reinit() {
    if (addr_ == 0) return begin();
    return configure();
}

bool QMI8658Driver::configure() {
    static const float accelLsb[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    static const float accelRange[] = {2.0f, 4.0f, 8.0f, 16.0f};
    static const float gyroLsb[] = {2048.0f, 1024.0f, 512.0f, 256.0f, 128.0f, 64.0f, 32.0f, 16.0f};
    static const float gyroRange[] = {16, 32, 64, 128, 256, 512, 1024, 2048};
    const uint8_t ar = static_cast<uint8_t>(cfg_.accelRange) & 3;
    const uint8_t gr = static_cast<uint8_t>(cfg_.gyroRange) & 7;
    accelLsbPerG_ = accelLsb[ar];
    gyroLsbPerDps_ = gyroLsb[gr];

    const uint8_t ctrl1 = CTRL1_ADDR_AI | CTRL1_BE;
    const uint8_t ctrl2 = static_cast<uint8_t>((ar << 4) | (cfg_.odrCode & 0x0F));
    const uint8_t ctrl3 = static_cast<uint8_t>((gr << 4) | (cfg_.odrCode & 0x0F));
    // CTRL5: bit6:5 gLPF_MODE, bit4 gLPF_EN, bit2:1 aLPF_MODE, bit0 aLPF_EN
    const uint8_t ctrl5 = cfg_.lowPass ? 0x77 : 0x00;
    const uint8_t ctrl7 = CTRL7_AEN | CTRL7_GEN;

    bool ok = bus_.writeReg(addr_, REG_CTRL7, 0x00);  // disable while configuring
    ok &= bus_.writeReg(addr_, REG_CTRL1, ctrl1);
    ok &= bus_.writeReg(addr_, REG_CTRL2, ctrl2);
    ok &= bus_.writeReg(addr_, REG_CTRL3, ctrl3);
    ok &= bus_.writeReg(addr_, REG_CTRL5, ctrl5);
    ok &= bus_.writeReg(addr_, REG_CTRL7, ctrl7);
    if (!ok) {
        LOGE(TAG, "configuration write failed");
        return false;
    }
    // read back and verify
    uint8_t rb[7] = {0};
    if (!bus_.readRegs(addr_, REG_CTRL1, rb, 7)) return false;
    if (rb[1] != ctrl2 || rb[2] != ctrl3 || rb[6] != ctrl7) {
        LOGE(TAG, "config readback mismatch: CTRL2 %02X/%02X CTRL3 %02X/%02X CTRL7 %02X/%02X", rb[1], ctrl2,
             rb[2], ctrl3, rb[6], ctrl7);
        return false;
    }

    info_.name = "QMI8658";
    info_.accelRangeG = accelRange[ar];
    info_.gyroRangeDps = gyroRange[gr];
    info_.nominalRateHz = 224.2f;  // 6DOF mode, ODR code 5 (datasheet table)
    info_.providesTemperature = true;
    info_.providesSampleCounter = true;

    // Byte order self check: at rest |a| must be ~1 g. Decide between LE (expected)
    // and swapped interpretation using the first valid samples.
    delay(30);
    int votesLe = 0, votesSwap = 0;
    for (int i = 0; i < 10; ++i) {
        uint8_t b[17];
        if (!readBlock(b)) continue;
        sda::Vec3 a1, a2, g;
        float t;
        int16_t raw[6];
        decode(b, false, a1, g, t, raw);
        decode(b, true, a2, g, t, raw);
        if (std::fabs(a1.norm() - 1.0f) < 0.3f) ++votesLe;
        else if (std::fabs(a2.norm() - 1.0f) < 0.3f) ++votesSwap;
        delay(6);
    }
    swap_ = votesSwap > votesLe && votesSwap >= 5;
    if (swap_) LOGW(TAG, "accelerometer data appear big endian -> byte swap enabled");
    if (votesLe == 0 && votesSwap == 0)
        LOGW(TAG, "byte order check inconclusive (device moving or sensor fault)");

    healthy_ = true;
    consecutiveErrors_ = 0;
    lastCounter_ = 0xFFFFFFFF;
    LOGI(TAG, "configured: +/-%.0f g, +/-%.0f dps, ODR code %u, LPF %s", info_.accelRangeG, info_.gyroRangeDps,
         cfg_.odrCode, cfg_.lowPass ? "on" : "off");
    return true;
}

bool QMI8658Driver::readBlock(uint8_t* b) {
    // 0x30..0x40: TIMESTAMP(3) TEMP(2) AX AY AZ GX GY GZ (2 each) = 17 bytes
    return bus_.readRegs(addr_, REG_TIMESTAMP_L, b, 17, 3);
}

void QMI8658Driver::decode(const uint8_t* b, bool swap, sda::Vec3& a, sda::Vec3& g, float& t,
                           int16_t raw[6]) const {
    auto rd = [&](int i) -> int16_t {
        return swap ? static_cast<int16_t>((b[i] << 8) | b[i + 1]) : static_cast<int16_t>((b[i + 1] << 8) | b[i]);
    };
    const int16_t traw = static_cast<int16_t>((b[4] << 8) | b[3]);
    t = static_cast<float>(traw) / 256.0f;
    for (int i = 0; i < 6; ++i) raw[i] = rd(5 + 2 * i);
    a = sda::Vec3(raw[0], raw[1], raw[2]) / accelLsbPerG_;
    g = sda::Vec3(raw[3], raw[4], raw[5]) / gyroLsbPerDps_;
}

bool QMI8658Driver::readSample(sda::ImuSample& out) {
    if (addr_ == 0) return false;
    uint8_t b[17];
    if (!readBlock(b)) {
        readErrors_++;
        consecutiveErrors_++;
        out = sda::ImuSample{};
        out.flags = sda::kSampleBusError;
        out.hostTimeUs = static_cast<uint64_t>(esp_timer_get_time());
        return true;  // report the error to the pipeline
    }
    consecutiveErrors_ = 0;
    const uint32_t counter = static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
                             (static_cast<uint32_t>(b[2]) << 16);
    if (counter == lastCounter_) return false;  // no new sample yet
    lastCounter_ = counter;

    out.hostTimeUs = static_cast<uint64_t>(esp_timer_get_time());
    out.sensorCounter = counter;
    int16_t raw[6];
    decode(b, swap_, out.accelG, out.gyroDps, out.temperatureC, raw);
    out.flags = sda::kSampleOk;
    for (int i = 0; i < 3; ++i)
        if (raw[i] >= 32700 || raw[i] <= -32700) out.flags |= sda::kSampleAccelClipped;
    for (int i = 3; i < 6; ++i)
        if (raw[i] >= 32700 || raw[i] <= -32700) out.flags |= sda::kSampleGyroClipped;
    if (!(out.temperatureC > -40.0f && out.temperatureC < 105.0f)) out.flags |= sda::kSampleTempInvalid;
    out.sourceId = 0;
    return true;
}

bool QMI8658Driver::setOdrCode(uint8_t code) {
    if (addr_ == 0) return false;
    uint8_t c2 = 0, c3 = 0;
    if (!bus_.readRegs(addr_, REG_CTRL2, &c2, 1) || !bus_.readRegs(addr_, REG_CTRL3, &c3, 1)) return false;
    c2 = static_cast<uint8_t>((c2 & 0xF0) | (code & 0x0F));
    c3 = static_cast<uint8_t>((c3 & 0xF0) | (code & 0x0F));
    const bool ok = bus_.writeReg(addr_, REG_CTRL2, c2) && bus_.writeReg(addr_, REG_CTRL3, c3);
    if (ok) {
        cfg_.odrCode = code & 0x0F;
        info_.nominalRateHz = 896.8f / static_cast<float>(1u << ((code & 0x0F) - 3));  // 6DOF table
    }
    return ok;
}

bool QMI8658Driver::setPowerDown(bool down) {
    if (addr_ == 0) return false;
    if (down) {
        bool ok = bus_.writeReg(addr_, REG_CTRL7, 0x00);
        ok &= bus_.writeReg(addr_, REG_CTRL1, CTRL1_ADDR_AI | CTRL1_BE | CTRL1_SENSOR_DISABLE);
        healthy_ = false;
        return ok;
    }
    return configure();
}

bool QMI8658Driver::readControlRegisters(uint8_t out[9]) {
    return addr_ != 0 && bus_.readRegs(addr_, REG_CTRL1, out, 9);
}
