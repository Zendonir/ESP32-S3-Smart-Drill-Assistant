// SPDX-License-Identifier: MIT
#include "MeasurementPipeline.h"

namespace sda {

namespace {
constexpr uint32_t kStuckSamples = 150;     ///< identical raw samples -> data stuck
constexpr uint32_t kMountCaptureSamples = 250;
constexpr float kConvergeBoostGain = 0.05f; ///< fast convergence right after reset
constexpr float kConvergeBoostSec = 1.0f;
}  // namespace

MeasurementPipeline::MeasurementPipeline() { configure(PipelineConfig()); }

void MeasurementPipeline::configure(const PipelineConfig& cfg) {
    cfg_ = cfg;
    timing_.configure(cfg.nominalRateHz, 24, 0.1f);
    motion_.configure(cfg.motion);
    vib_.configure(cfg.vibration);
    unc_.configure(cfg.uncertainty);
    fusion_.select(cfg.fusion);
    reset();
}

void MeasurementPipeline::setCalibration(const SensorCalibration& cal) {
    cal_ = cal;
    onlineBias_ = Vec3();
    onlineStillSec_ = 0;
    if (cal_.hasGyro()) {
        const GyroCalPoint& p = cal_.gyroPoint(cal_.gyroPointCount() - 1);
        const float n = p.samples > 0 ? float(p.samples) : 1.0f;
        biasSigmaCal_ = std::fmax(cfg_.biasFloorDps, p.noiseDps.norm() / std::sqrt(n));
        biasRefTempC_ = p.tempC;
        const Vec3 noise = cal_.gyroNoise();
        motion_.setNoise((noise.x + noise.y + noise.z) / 3.0f, accelNoiseG_ > 0 ? accelNoiseG_ : 0.001f);
    } else {
        biasSigmaCal_ = 0.5f;  // unknown zero-rate offset: conservative
        biasRefTempC_ = NAN;
    }
}

void MeasurementPipeline::setFusion(FusionType t) {
    cfg_.fusion = t;
    fusion_.select(t);
}

void MeasurementPipeline::reset() {
    timing_.reset();
    motion_.reset();
    vib_.reset();
    unc_.reset();
    fusionInit_ = false;
    sinceReset_ = 0.0f;
    identicalCount_ = 0;
    lossWindow_ = 1000000;
    snap_.valid = false;
}

void MeasurementPipeline::setDataTimeout(bool timeout) {
    if (timeout == dataTimeout_) return;
    dataTimeout_ = timeout;
    // publish an updated quality immediately (no sample will trigger it)
    QualityInputs qi;
    qi.dataTimeout = timeout;
    qi.busErrors = busErrors_;
    qi.hasGyroCal = cal_.hasGyro();
    qi.hasAccelCal = cal_.hasAccel();
    qi.tiltSigmaDeg = rad2deg(unc_.tiltSigmaRad());
    qi.secondsSinceReset = sinceReset_;
    snap_.quality = assessQuality(qi, cfg_.quality);
    snap_.seq++;
    if (timeout) {
        // after a data outage the orientation must re-converge
        fusionInit_ = false;
        timing_.reset();
    }
}

void MeasurementPipeline::updateBiasSigma(float tempC) {
    float stat = biasSigmaCal_;
    if (onlineStillSec_ > 2.0f) {
        const Vec3 n = cal_.gyroNoise();
        const float rate = timing_.stats().estimatedRateHz > 1 ? timing_.stats().estimatedRateHz : 200.0f;
        const float online = n.norm() / std::sqrt(onlineStillSec_ * rate) * 3.0f;  // EMA ~ fewer eff. samples
        stat = std::fmin(stat, std::fmax(online, cfg_.biasFloorDps));
    }
    float temp = 0.0f;
    if (std::isfinite(tempC) && std::isfinite(biasRefTempC_))
        temp = cfg_.tempCoeffAssumedDpsPerK * std::fabs(tempC - biasRefTempC_);
    snap_.biasSigmaDps = std::fmax(stat, cfg_.biasFloorDps) + temp;
}

bool MeasurementPipeline::process(const ImuSample& raw) {
    if (raw.flags & kSampleBusError) {
        busErrors_ = true;
        return false;
    }
    if (!raw.accelG.isFinite() || !raw.gyroDps.isFinite() ||
        raw.accelG.norm() > 1.8f * cfg_.accelRangeG || raw.gyroDps.norm() > 1.8f * cfg_.gyroRangeDps) {
        snap_.implausibleCount++;
        return false;
    }
    const TimingStep step = timing_.update(raw.sensorCounter, raw.hostTimeUs);
    if (step.duplicate) return false;
    dataTimeout_ = false;

    // clipping and stuck data
    const float aLim = 0.98f * cfg_.accelRangeG, gLim = 0.98f * cfg_.gyroRangeDps;
    const bool aClip = std::fabs(raw.accelG.x) >= aLim || std::fabs(raw.accelG.y) >= aLim ||
                       std::fabs(raw.accelG.z) >= aLim || (raw.flags & kSampleAccelClipped);
    const bool gClip = std::fabs(raw.gyroDps.x) >= gLim || std::fabs(raw.gyroDps.y) >= gLim ||
                       std::fabs(raw.gyroDps.z) >= gLim || (raw.flags & kSampleGyroClipped);
    if (aClip || gClip) snap_.clipCount++;
    const bool identical = raw.accelG.x == lastRawA_.x && raw.accelG.y == lastRawA_.y &&
                           raw.accelG.z == lastRawA_.z && raw.gyroDps.x == lastRawG_.x &&
                           raw.gyroDps.y == lastRawG_.y && raw.gyroDps.z == lastRawG_.z;
    identicalCount_ = identical ? identicalCount_ + 1 : 0;
    lastRawA_ = raw.accelG;
    lastRawG_ = raw.gyroDps;
    lossWindow_ = step.lostSamples > 0 ? 0 : lossWindow_ + 1;

    const float dt = step.dtSec;
    const float tempC = (raw.flags & kSampleTempInvalid) ? NAN : raw.temperatureC;

    // S -> D
    const Vec3 aD = cfg_.axisMap.apply(raw.accelG);
    const Vec3 gD = cfg_.axisMap.apply(raw.gyroDps);

    // base calibration
    TempLookup gInfo, aInfo;
    const Vec3 gBias = cal_.gyroBias(tempC, &gInfo);
    Mat3 aM;
    Vec3 aC;
    cal_.accelCorrection(tempC, aM, aC, &aInfo);
    const Vec3 gCal = gD - gBias - onlineBias_;
    const Vec3 aCal = aM * aD + aC;

    // motion / vibration
    motion_.update(gCal, aCal, dt);
    vib_.update(aCal, gCal, dt);
    const bool still = motion_.isStill();
    if (still && motion_.stillSeconds() > 0.5f) {
        // accel tilt noise per sample: vector variance spread over ~3 axes
        const float n = motion_.accelStdG() / 1.7320508f;
        accelNoiseG_ = accelNoiseG_ <= 0 ? n : accelNoiseG_ + (n - accelNoiseG_) * 0.01f;
    }

    // online gyro bias tracking during confirmed still-stand
    const bool gyroCalRunning = calStatus_.procedure == CalProcedure::Gyro;
    if (cfg_.onlineBiasTracking && cal_.hasGyro() && still && motion_.stillSeconds() > 1.0f &&
        !gyroCalRunning && vib_.level() == VibrationLevel::None) {
        const float a = dt / (cfg_.onlineBiasTauSec + dt);
        onlineBias_ += gCal * a;
        const float lim = cfg_.onlineBiasMaxDps;
        onlineBias_ = Vec3(clampv(onlineBias_.x, -lim, lim), clampv(onlineBias_.y, -lim, lim),
                           clampv(onlineBias_.z, -lim, lim));
        onlineStillSec_ += dt;
        if (std::isfinite(tempC)) biasRefTempC_ = tempC;
    }
    updateBiasSigma(tempC);

    // fusion
    const bool clipped = aClip;
    if (!fusionInit_ || step.discontinuity) {
        fusion_.active().reset(aCal);
        unc_.reset();
        sinceReset_ = 0.0f;
        fusionInit_ = true;
    }
    const float rateRad = deg2rad(gCal.norm());
    const float trust = accelTrust(cfg_.trust, motion_.accelNormDevG(), vib_.rmsMg(), gCal.norm(), clipped);
    unc_.predict(dt, rateRad, snap_.biasSigmaDps);
    float maxK = cfg_.uncertainty.maxStepGain;
    if (still && cfg_.fusionMinTauSec > 0.0f) maxK = std::fmin(maxK, dt / (cfg_.fusionMinTauSec + dt));
    if (sinceReset_ < kConvergeBoostSec) maxK = std::fmax(maxK, kConvergeBoostGain);
    const float measNoise = accelNoiseG_ > 0 ? std::fmax(accelNoiseG_, 0.0003f) : cfg_.uncertainty.accelNoiseRad;
    float k = unc_.correct(trust, measNoise, maxK);
    if (sinceReset_ < kConvergeBoostSec && trust > 0.2f) k = std::fmax(k, kConvergeBoostGain * trust);
    FusionInput in;
    in.gyroRadS = gCal * kRadPerDeg;
    in.accelG = aCal;
    in.dt = dt;
    in.gain = k;
    fusion_.active().update(in);
    sinceReset_ += dt;

    // references
    const float headingInc =
        (deg2rad(snap_.biasSigmaDps) + cfg_.uncertainty.gyroScaleError * rateRad) * dt;
    ref_.propagate(dt, headingInc);

    // procedures
    runCalibration(gD, aD, aCal, gCal, tempC);
    runPrecision(aCal, dt);

    // quality
    QualityInputs qi;
    qi.dataTimeout = false;
    qi.busErrors = busErrors_;
    qi.implausible = false;
    qi.stuck = identicalCount_ > kStuckSamples;
    qi.recentSampleLoss = lossWindow_ < uint32_t(cfg_.nominalRateHz);
    qi.accelClipping = aClip;
    qi.gyroClipping = gClip;
    qi.hasGyroCal = cal_.hasGyro();
    qi.hasAccelCal = cal_.hasAccel();
    qi.tempOutOfRange = (gInfo.valid && gInfo.distanceC > 10.0f) || (aInfo.valid && aInfo.distanceC > 10.0f);
    qi.vibrationLevel = uint8_t(vib_.level());
    qi.still = still;
    qi.tiltSigmaDeg = rad2deg(unc_.tiltSigmaRad());
    qi.accelTrust = trust;
    qi.secondsSinceReset = sinceReset_;
    qi.referenceActive = ref_.active();
    qi.referenceUsesGyro = ref_.info().usesGyro;
    qi.referenceHeadingSigmaDeg = ref_.info().headingSigmaDeg;
    qi.referenceAgeSec = ref_.info().ageSec;
    busErrors_ = false;  // consumed; the driver reports again if persisting

    // snapshot
    OrientationSnapshot& s = snap_;
    s.seq++;
    s.timeUs = raw.hostTimeUs;
    s.valid = true;
    s.qWD = fusion_.active().orientation();
    s.upD = fusion_.active().upInDevice();
    s.accelG = aCal;
    s.gyroDps = gCal;
    s.rawAccelG = aD;
    s.rawGyroDps = gD;
    s.temperatureC = tempC;
    s.level = levelAngles(s.upD);
    s.tiltSigmaDeg = qi.tiltSigmaDeg;
    s.headingSigmaDeg = rad2deg(unc_.headingSigmaRad());
    s.motion = motion_.state();
    s.stillSec = motion_.stillSeconds();
    s.vibration = vib_.level();
    s.activity = activity_.update(motion_.state(), vib_.level(), dt);
    s.vibrationRmsMg = vib_.rmsMg();
    s.vibrationPeakMg = vib_.peakMg();
    s.accelTrust = trust;
    s.fusionGain = k;
    s.accelNoiseG = accelNoiseG_;
    s.gyroRateDps = gCal.norm();
    s.quality = assessQuality(qi, cfg_.quality);
    s.timing = timing_.stats();
    s.onlineBiasDps = onlineBias_;
    s.gyroTemp = gInfo;
    s.accelTemp = aInfo;
    s.hasGyroCal = cal_.hasGyro();
    s.hasAccelCal = cal_.hasAccel();
    s.secondsSinceReset = sinceReset_;
    s.fusion = fusion_.type();
    s.reference = ref_;
    s.cal = calStatus_;
    s.precisionRunning = precision_.running();
    s.precisionWaiting = precision_.waitingForStill();
    s.precisionProgress = precision_.progress();
    return true;
}

// ---------------------------------------------------------------------------
// calibration procedures
// ---------------------------------------------------------------------------
void MeasurementPipeline::startGyroCalibration(const GyroCalConfig& c) {
    gyroCal_.start(c);
    calStatus_.procedure = CalProcedure::Gyro;
    calStatus_.state = uint8_t(GyroCalibrator::State::Collecting);
    calStatus_.failure = 0;
    calStatus_.progress = 0;
}

void MeasurementPipeline::startAccelCalibration(const AccelCalConfig& c) {
    accelCal_.start(c);
    calStatus_.procedure = CalProcedure::Accel;
    calStatus_.state = 0;
    calStatus_.failure = 0;
    calStatus_.acceptedPoseMask = 0;
    calStatus_.activePose = -1;
}

void MeasurementPipeline::accelBeginPose(AccelPose p) {
    if (calStatus_.procedure != CalProcedure::Accel) return;
    accelCal_.beginPose(p);
    calStatus_.activePose = int8_t(p);
}

bool MeasurementPipeline::accelSolve(AccelCalPoint& out) {
    if (calStatus_.procedure != CalProcedure::Accel) return false;
    const bool ok = accelCal_.solve(out);
    calStatus_.failure = uint8_t(accelCal_.lastFailure());
    calStatus_.state = ok ? 2 : 3;
    calStatus_.resultSeq++;
    return ok;
}

void MeasurementPipeline::startMountCalibration(CalProcedure p) {
    if (p != CalProcedure::MountRoll && p != CalProcedure::MountVertical) return;
    mountCal_.reset();
    calStatus_.procedure = p;
    calStatus_.state = 0;
    calStatus_.failure = 0;
    calStatus_.mountPositions = 0;
    capturing_ = false;
}

void MeasurementPipeline::mountCapturePosition() {
    if (calStatus_.procedure != CalProcedure::MountRoll && calStatus_.procedure != CalProcedure::MountVertical)
        return;
    capturing_ = true;
    capN_ = 0;
    capSum_ = Vec3();
    calStatus_.failure = 0;
}

MountFitResult MeasurementPipeline::mountSolve(const Vec3& hint, bool bitDown) {
    MountFitResult r;
    if (calStatus_.procedure == CalProcedure::MountRoll) r = mountCal_.solveRollFit(hint);
    else if (calStatus_.procedure == CalProcedure::MountVertical)
        r = MountCalibrator::solveVertical(lastMountVector_, bitDown);
    calStatus_.state = r.ok ? 2 : 3;
    calStatus_.failure = uint8_t(r.error);
    calStatus_.resultSeq++;
    return r;
}

void MeasurementPipeline::cancelCalibration() {
    gyroCal_.cancel();
    capturing_ = false;
    calStatus_.procedure = CalProcedure::None;
    calStatus_.capturing = false;
}

void MeasurementPipeline::applyGyroPoint(const GyroCalPoint& p) {
    SensorCalibration c = cal_;
    c.addGyroPoint(p);
    setCalibration(c);
}

void MeasurementPipeline::applyAccelPoint(const AccelCalPoint& p) {
    SensorCalibration c = cal_;
    c.addAccelPoint(p);
    setCalibration(c);
    unc_.reset();  // re-converge on the corrected gravity reference
    sinceReset_ = 0.0f;
}

void MeasurementPipeline::runCalibration(const Vec3& rawGyroD, const Vec3& rawAccelD, const Vec3& calAccelD,
                                         const Vec3& calGyroD, float tempC) {
    calStatus_.motionRestart = false;
    switch (calStatus_.procedure) {
        case CalProcedure::None: break;
        case CalProcedure::Gyro: {
            gyroCal_.addSample(rawGyroD, rawAccelD, tempC);
            calStatus_.progress = gyroCal_.progress();
            calStatus_.motionRestart = gyroCal_.motionDetected();
            calStatus_.state = uint8_t(gyroCal_.state());
            calStatus_.failure = uint8_t(gyroCal_.failure());
            if (gyroCal_.state() == GyroCalibrator::State::Done) {
                gyroResult_ = gyroCal_.result();
                gyroResultOk_ = true;
                applyGyroPoint(gyroResult_);
                calStatus_.resultSeq++;
                calStatus_.procedure = CalProcedure::None;
            } else if (gyroCal_.state() == GyroCalibrator::State::Failed) {
                gyroResultOk_ = false;
                calStatus_.resultSeq++;
                calStatus_.procedure = CalProcedure::None;
            }
            break;
        }
        case CalProcedure::Accel: {
            calStatus_.detectedPose = int8_t(AccelCalibrator::detectPose(rawAccelD));
            if (accelCal_.collecting()) {
                accelCal_.addSample(rawAccelD, calGyroD, tempC);
                calStatus_.progress = accelCal_.poseProgress();
                calStatus_.failure = uint8_t(accelCal_.lastFailure());
                calStatus_.motionRestart = accelCal_.lastFailure() != AccelCalibrator::Failure::None;
                if (!accelCal_.collecting()) {
                    calStatus_.activePose = -1;
                    calStatus_.progress = 1.0f;
                }
            }
            uint8_t mask = 0;
            for (int i = 0; i < kAccelPoseCount; ++i)
                if (accelCal_.poseState(AccelPose(i)) == AccelCalibrator::PoseState::Accepted) mask |= 1u << i;
            calStatus_.acceptedPoseMask = mask;
            break;
        }
        case CalProcedure::MountRoll:
        case CalProcedure::MountVertical: {
            if (!capturing_) break;
            if (!motion_.isStill()) {
                calStatus_.waitingStill = true;
                if (capN_ > 0) calStatus_.motionRestart = true;
                capN_ = 0;
                capSum_ = Vec3();
                break;
            }
            calStatus_.waitingStill = false;
            capSum_ += calAccelD;
            ++capN_;
            calStatus_.progress = float(capN_) / float(kMountCaptureSamples);
            if (capN_ >= kMountCaptureSamples) {
                const Vec3 mean = capSum_ / float(capN_);
                capturing_ = false;
                lastMountVector_ = mean;
                if (calStatus_.procedure == CalProcedure::MountRoll) {
                    calStatus_.failure = mountCal_.addPosition(mean) ? 0 : 1;  // 1 = too close
                    calStatus_.mountPositions = uint8_t(mountCal_.count());
                } else {
                    calStatus_.mountPositions = 1;
                }
                calStatus_.resultSeq++;
            }
            break;
        }
    }
    calStatus_.capturing = capturing_;
}

// ---------------------------------------------------------------------------
// references / precision
// ---------------------------------------------------------------------------
void MeasurementPipeline::captureTiltZero() {
    ref_.captureTiltZero(fusion_.active().upInDevice(), rad2deg(unc_.tiltSigmaRad()));
}
void MeasurementPipeline::captureFull3D() {
    ref_.captureFull3D(fusion_.active().orientation(), rad2deg(unc_.tiltSigmaRad()));
}
void MeasurementPipeline::captureSurface(bool use3D) {
    ref_.captureSurface(fusion_.active().orientation(), use3D, rad2deg(unc_.tiltSigmaRad()));
}
void MeasurementPipeline::clearReference() { ref_.clear(); }

void MeasurementPipeline::startPrecision(const PrecisionConfig& c, PrecisionQuantity q) {
    precision_.configure(c);
    precisionQty_ = q;
    precision_.start();
}
void MeasurementPipeline::abortPrecision() { precision_.abort(); }

void MeasurementPipeline::runPrecision(const Vec3& aCal, float dt) {
    if (!precision_.running()) return;
    const Vec3 u = aCal.normalized();
    float primary, secondary;
    if (precisionQty_ == PrecisionQuantity::AxisInclination) {
        const Vec3 d = mount_.drillAxisD.normalized(Vec3(0, 0, -1));
        primary = rad2deg(safeAsin(d.dot(u))) + mount_.angleCorrectionDeg;  // signed elevation
        secondary = 0.0f;
    } else {
        primary = rad2deg(safeAsin(u.x));
        secondary = rad2deg(safeAsin(u.y));
    }
    // accept only while still and without vibration
    const bool ok = motion_.isStill() && vib_.level() == VibrationLevel::None;
    if (precision_.add(primary, secondary, ok, dt)) {
        snap_.precision = precision_.result();
        snap_.precisionSeq++;
    }
}

}  // namespace sda
