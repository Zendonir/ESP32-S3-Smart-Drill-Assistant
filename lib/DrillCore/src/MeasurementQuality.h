// SPDX-License-Identifier: MIT
// Dynamic measurement quality. All states are derived from measured quantities;
// no "precision percentages" are invented.
#pragma once

#include <cstdint>

namespace sda {

enum class QualityState : uint8_t {
    Stable = 0,              ///< still, calibrated, low uncertainty
    Motion = 1,              ///< moving (values valid but dynamic)
    Vibration = 2,           ///< strong vibration, accel down-weighted
    CalibrationRequired = 3, ///< no valid sensor calibration
    SensorFault = 4,         ///< no / invalid data
    OrientationLimited = 5,  ///< tilt uncertainty too high (e.g. long vibration, after reset)
    ReferenceDrift = 6,      ///< active gyro based reference likely drifted
};
const char* qualityStateName(QualityState s);
const char* qualityStateShort(QualityState s);

enum QualityFlags : uint32_t {
    kQfNone = 0,
    kQfNoData = 1u << 0,
    kQfBusErrors = 1u << 1,
    kQfImplausible = 1u << 2,
    kQfSampleLoss = 1u << 3,
    kQfAccelClipping = 1u << 4,
    kQfGyroClipping = 1u << 5,
    kQfNoGyroCal = 1u << 6,
    kQfNoAccelCal = 1u << 7,
    kQfTempOutOfCalRange = 1u << 8,
    kQfVibrationLight = 1u << 9,
    kQfVibrationStrong = 1u << 10,
    kQfMoving = 1u << 11,
    kQfHighTiltUncertainty = 1u << 12,
    kQfConverging = 1u << 13,
    kQfReferenceDrift = 1u << 14,
    kQfReferenceOld = 1u << 15,
    kQfAccelDistrusted = 1u << 16,
    kQfStuckData = 1u << 17,
};

struct QualityInputs {
    bool dataTimeout = false;
    bool busErrors = false;
    bool implausible = false;
    bool stuck = false;
    bool recentSampleLoss = false;
    bool accelClipping = false;
    bool gyroClipping = false;
    bool hasGyroCal = false;
    bool hasAccelCal = false;
    bool tempOutOfRange = false;
    uint8_t vibrationLevel = 0;   ///< 0 none, 1 light, 2 strong
    bool still = false;
    float tiltSigmaDeg = 0.0f;
    float accelTrust = 1.0f;
    float secondsSinceReset = 0.0f;
    bool referenceActive = false;
    bool referenceUsesGyro = false;
    float referenceHeadingSigmaDeg = 0.0f;
    float referenceAgeSec = 0.0f;
};

struct QualityConfig {
    float limitedTiltSigmaDeg = 0.5f;   ///< above -> OrientationLimited
    float convergeSec = 1.0f;
    float referenceDriftSigmaDeg = 1.0f;
    float referenceMaxAgeSec = 600.0f;
};

struct QualityAssessment {
    QualityState state = QualityState::SensorFault;
    uint32_t flags = kQfNoData;
    /// true if the measurement may be used for a green "on target" indication
    /// at the given tolerance (requires 2*sigma <= tolerance).
    bool usableForTarget(float toleranceDeg, float tiltSigmaDeg) const;
};

QualityAssessment assessQuality(const QualityInputs& in, const QualityConfig& cfg);

}  // namespace sda
