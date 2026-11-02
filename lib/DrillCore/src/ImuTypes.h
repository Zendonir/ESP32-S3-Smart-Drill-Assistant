// SPDX-License-Identifier: MIT
// Data types exchanged between sensor drivers and the measurement pipeline.
#pragma once

#include <cstdint>

#include "MathTypes.h"

namespace sda {

/// Bit flags describing the validity of a single raw sample.
enum SampleFlags : uint16_t {
    kSampleOk = 0,
    kSampleAccelClipped = 1u << 0,   ///< at least one accel axis at/near full scale
    kSampleGyroClipped = 1u << 1,    ///< at least one gyro axis at/near full scale
    kSampleTempInvalid = 1u << 2,    ///< temperature implausible or not provided
    kSampleTimestampGap = 1u << 3,   ///< samples were lost before this one
    kSampleDuplicate = 1u << 4,      ///< same sensor timestamp as previous (should be dropped)
    kSampleBusError = 1u << 5,       ///< read failed (sample content invalid)
    kSampleImplausible = 1u << 6,    ///< NaN / absurd magnitude
};

/// One physical IMU sample in SI-friendly units, *sensor* coordinate frame.
struct ImuSample {
    uint64_t hostTimeUs = 0;    ///< esp_timer time when the read completed
    uint32_t sensorCounter = 0; ///< raw 24 bit sensor timestamp/sample counter
    Vec3 accelG;                ///< specific force in g (at rest: +1 g pointing up)
    Vec3 gyroDps;               ///< angular rate in deg/s
    float temperatureC = NAN;   ///< sensor die temperature, NaN if unavailable
    uint16_t flags = kSampleOk;
    uint8_t sourceId = 0;       ///< which sensor produced the sample (for future multi-sensor)
};

/// Static capabilities of an orientation sensor (used by the pipeline to derive
/// clipping limits and plausibility checks without knowing the concrete chip).
struct SensorInfo {
    const char* name = "unknown";
    float accelRangeG = 0.0f;    ///< full scale (+/-)
    float gyroRangeDps = 0.0f;   ///< full scale (+/-)
    float nominalRateHz = 0.0f;  ///< nominal output data rate
    bool providesTemperature = false;
    bool providesSampleCounter = false;
};

/// Abstract interface for sensors that deliver accelerometer/gyroscope data.
/// Version 1 has exactly one implementation (QMI8658). Future precision
/// inclinometers or external IMUs implement the same interface; the measurement
/// pipeline only depends on this header.
class IOrientationSensor {
public:
    virtual ~IOrientationSensor() = default;
    virtual bool begin() = 0;
    /// Non-blocking poll. Returns true if a *new* sample was produced into `out`.
    virtual bool readSample(ImuSample& out) = 0;
    virtual bool isHealthy() const = 0;
    virtual const SensorInfo& info() const = 0;
    /// Enter/leave a low power state (e.g. before deep sleep).
    virtual bool setPowerDown(bool down) = 0;
};

}  // namespace sda
