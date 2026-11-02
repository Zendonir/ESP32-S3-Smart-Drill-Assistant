// SPDX-License-Identifier: MIT
#include "MeasurementQuality.h"

namespace sda {

const char* qualityStateName(QualityState s) {
    switch (s) {
        case QualityState::Stable: return "Stabil";
        case QualityState::Motion: return "Bewegung";
        case QualityState::Vibration: return "Vibration";
        case QualityState::CalibrationRequired: return "Kalibrierung erforderlich";
        case QualityState::SensorFault: return "Sensor gestört";
        case QualityState::OrientationLimited: return "Orientierung eingeschränkt";
        case QualityState::ReferenceDrift: return "Referenz evtl. gedriftet";
    }
    return "?";
}

const char* qualityStateShort(QualityState s) {
    switch (s) {
        case QualityState::Stable: return "STABIL";
        case QualityState::Motion: return "BEWEGUNG";
        case QualityState::Vibration: return "VIBRATION";
        case QualityState::CalibrationRequired: return "KALIBRIEREN";
        case QualityState::SensorFault: return "SENSORFEHLER";
        case QualityState::OrientationLimited: return "UNSICHER";
        case QualityState::ReferenceDrift: return "REF. DRIFT";
    }
    return "?";
}

bool QualityAssessment::usableForTarget(float toleranceDeg, float tiltSigmaDeg) const {
    if (state == QualityState::SensorFault || state == QualityState::OrientationLimited ||
        state == QualityState::Vibration || state == QualityState::ReferenceDrift)
        return false;
    if (flags & (kQfAccelClipping | kQfConverging)) return false;
    // statistical criterion: 2 sigma must fit into the tolerance band
    return 2.0f * tiltSigmaDeg <= toleranceDeg;
}

QualityAssessment assessQuality(const QualityInputs& in, const QualityConfig& cfg) {
    QualityAssessment a;
    uint32_t f = 0;
    if (in.dataTimeout) f |= kQfNoData;
    if (in.busErrors) f |= kQfBusErrors;
    if (in.implausible) f |= kQfImplausible;
    if (in.stuck) f |= kQfStuckData;
    if (in.recentSampleLoss) f |= kQfSampleLoss;
    if (in.accelClipping) f |= kQfAccelClipping;
    if (in.gyroClipping) f |= kQfGyroClipping;
    if (!in.hasGyroCal) f |= kQfNoGyroCal;
    if (!in.hasAccelCal) f |= kQfNoAccelCal;
    if (in.tempOutOfRange) f |= kQfTempOutOfCalRange;
    if (in.vibrationLevel == 1) f |= kQfVibrationLight;
    if (in.vibrationLevel >= 2) f |= kQfVibrationStrong;
    if (!in.still) f |= kQfMoving;
    if (in.tiltSigmaDeg > cfg.limitedTiltSigmaDeg) f |= kQfHighTiltUncertainty;
    if (in.secondsSinceReset < cfg.convergeSec) f |= kQfConverging;
    if (in.accelTrust < 0.2f) f |= kQfAccelDistrusted;
    if (in.referenceActive && in.referenceUsesGyro) {
        if (in.referenceHeadingSigmaDeg > cfg.referenceDriftSigmaDeg) f |= kQfReferenceDrift;
        if (in.referenceAgeSec > cfg.referenceMaxAgeSec) f |= kQfReferenceOld;
    }
    a.flags = f;

    // precedence: most severe first
    if (f & (kQfNoData | kQfImplausible | kQfStuckData) || (in.busErrors && in.dataTimeout))
        a.state = QualityState::SensorFault;
    else if (f & (kQfHighTiltUncertainty | kQfConverging | kQfGyroClipping))
        a.state = QualityState::OrientationLimited;
    else if (f & kQfVibrationStrong)
        a.state = QualityState::Vibration;
    else if (f & (kQfReferenceDrift | kQfReferenceOld))
        a.state = QualityState::ReferenceDrift;
    else if (f & kQfNoGyroCal)
        a.state = QualityState::CalibrationRequired;
    else if (f & kQfMoving)
        a.state = QualityState::Motion;
    else if (f & kQfNoAccelCal)
        a.state = QualityState::CalibrationRequired;  // still: advise calibration
    else
        a.state = QualityState::Stable;
    return a;
}

}  // namespace sda
