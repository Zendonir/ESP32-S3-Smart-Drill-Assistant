// SPDX-License-Identifier: MIT
#include "MeasurementService.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "../hardware/I2CBus.h"
#include "../hardware/IMUDriver.h"
#include "../system/Log.h"
#include "AppConfig.h"

static const char* TAG = "meas";

bool MeasurementService::begin(QMI8658Driver* imu, I2CBus* bus, const sda::PipelineConfig& cfg,
                               const sda::SensorCalibration& cal, const sda::MountModel& mount) {
    imu_ = imu;
    bus_ = bus;
    imuOk_ = imu_ && imu_->isHealthy();
    pipeline_.configure(cfg);
    pipeline_.setCalibration(cal);
    pipeline_.setMount(mount);

    sampleQueue_ = xQueueCreate(appcfg::kImuQueueLength, sizeof(sda::ImuSample));
    cmdQueue_ = xQueueCreate(16, sizeof(Command));
    done_ = xSemaphoreCreateBinary();
    requestMutex_ = xSemaphoreCreateMutex();
    if (!sampleQueue_ || !cmdQueue_ || !done_ || !requestMutex_) return false;

    xTaskCreatePinnedToCore(fusionTaskEntry, "fusion", appcfg::kFusionTaskStack, this, appcfg::kFusionTaskPrio,
                            &fusionTask_, appcfg::kFusionTaskCore);
    xTaskCreatePinnedToCore(imuTaskEntry, "imu", appcfg::kImuTaskStack, this, appcfg::kImuTaskPrio, &imuTask_,
                            appcfg::kImuTaskCore);
    return imuTask_ && fusionTask_;
}

void MeasurementService::imuTaskEntry(void* arg) { static_cast<MeasurementService*>(arg)->imuTask(); }
void MeasurementService::fusionTaskEntry(void* arg) { static_cast<MeasurementService*>(arg)->fusionTask(); }

// ---------------------------------------------------------------------------
// IMU task: data driven polling. After a new sample the task sleeps most of the
// sample period and then polls every millisecond, so each sample is fetched with
// <= ~1 ms latency while the shared I2C bus stays mostly free for the touch IC.
// ---------------------------------------------------------------------------
void MeasurementService::imuTask() {
    uint32_t lastSampleMs = millis();
    uint32_t consecutiveErr = 0;
    bool timeoutReported = false;
    for (;;) {
        if (suspended_ || !imu_) {
            vTaskDelay(pdMS_TO_TICKS(50));
            lastSampleMs = millis();
            continue;
        }
        // power saving: switch the sensor output rate (done here, in the only task
        // that talks to the IMU), then tell the fusion task about the new rate
        if (lowPowerReq_ != lowPowerActive_) {
            const bool lp = lowPowerReq_;
            if (imu_->setOdrCode(lp ? appcfg::kImuOdrCodeLowPower : appcfg::kImuOdrCodeNormal)) {
                lowPowerActive_ = lp;
                Command c;
                c.type = CmdType::SetRate;
                c.u.f = lp ? appcfg::kImuRateLowPowerHz : appcfg::kImuRateNormalHz;
                post(c);
                LOGI(TAG, "IMU rate %.1f Hz (%s)", double(c.u.f), lp ? "power saving" : "normal");
            }
        }
        sda::ImuSample s;
        bool got = imu_->readSample(s);
        if (got && (s.flags & sda::kSampleBusError)) {
            stats_.busErrors++;
            ++consecutiveErr;
            if (consecutiveErr == 10 || consecutiveErr % 200 == 0) {
                LOGW(TAG, "IMU read errors (%lu consecutive) -> bus recovery", (unsigned long)consecutiveErr);
                if (bus_ && bus_->recover() && imu_->reinit()) stats_.recoveries++;
            }
            if (xQueueSend(sampleQueue_, &s, 0) != pdTRUE) stats_.queueOverflows++;
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (got) {
            consecutiveErr = 0;
            lastSampleMs = millis();
            timeoutReported = false;
            stats_.samples++;
            if (xQueueSend(sampleQueue_, &s, 0) != pdTRUE) stats_.queueOverflows++;
            // sleep most of the sample period, then poll (fewer wake-ups at low rates)
            vTaskDelay(pdMS_TO_TICKS(lowPowerActive_ ? 30 : appcfg::kImuIdleAfterSampleMs));
            continue;
        }
        if (!timeoutReported && millis() - lastSampleMs > appcfg::kImuDataTimeoutMs) {
            timeoutReported = true;
            stats_.timeouts++;
            sda::ImuSample t;
            t.flags = sda::kSampleBusError;  // signals timeout to the fusion task
            t.hostTimeUs = static_cast<uint64_t>(esp_timer_get_time());
            t.sourceId = 0xFF;
            xQueueSend(sampleQueue_, &t, 0);
            LOGW(TAG, "no IMU data for %lu ms", (unsigned long)appcfg::kImuDataTimeoutMs);
            // try to revive a sensor that stopped producing data
            imu_->reinit();
        }
        vTaskDelay(pdMS_TO_TICKS(appcfg::kImuPollIntervalMs));
    }
}

// ---------------------------------------------------------------------------
// Fusion task
// ---------------------------------------------------------------------------
void MeasurementService::fusionTask() {
    sda::ImuSample s;
    uint64_t busyUs = 0;
    uint64_t windowStart = static_cast<uint64_t>(esp_timer_get_time());
    for (;;) {
        Command c;
        while (xQueueReceive(cmdQueue_, &c, 0) == pdTRUE) execute(c);

        if (xQueueReceive(sampleQueue_, &s, pdMS_TO_TICKS(20)) != pdTRUE) continue;
        const uint64_t t0 = static_cast<uint64_t>(esp_timer_get_time());
        bool published = false;
        if (s.sourceId == 0xFF) {
            pipeline_.setDataTimeout(true);
            published = true;
        } else if (s.flags & sda::kSampleBusError) {
            pipeline_.setBusErrors(true);
        } else {
            published = pipeline_.process(s);
        }
        if (published) shared_.write(pipeline_.snapshot());
        const uint64_t t1 = static_cast<uint64_t>(esp_timer_get_time());
        const uint32_t dt = static_cast<uint32_t>(t1 - t0);
        if (dt > stats_.maxProcessUs) stats_.maxProcessUs = dt;
        busyUs += dt;
        if (t1 - windowStart > 1000000ULL) {
            stats_.fusionLoadPct = 100.0f * static_cast<float>(busyUs) / static_cast<float>(t1 - windowStart);
            busyUs = 0;
            windowStart = t1;
            stats_.imuStackFree = imuTask_ ? uxTaskGetStackHighWaterMark(imuTask_) : 0;
            stats_.fusionStackFree = uxTaskGetStackHighWaterMark(nullptr);
        }
    }
}

void MeasurementService::execute(const Command& c) {
    using CT = CmdType;
    bool signal = false;
    switch (c.type) {
        case CT::Reset: pipeline_.reset(); break;
        case CT::SetCalibration: pipeline_.setCalibration(stagedCal_); signal = true; break;
        case CT::SetMount: pipeline_.setMount(stagedMount_); signal = true; break;
        case CT::SetConfig: pipeline_.configure(stagedCfg_); signal = true; break;
        case CT::SetFusion: pipeline_.setFusion(c.u.fusion); break;
        case CT::SetFilterTau: pipeline_.setFilterTau(c.u.f); break;
        case CT::StartGyroCal: pipeline_.startGyroCalibration(); break;
        case CT::StartAccelCal: {
            sda::AccelCalConfig ac;
            ac.extended = c.u.b;
            pipeline_.startAccelCalibration(ac);
            break;
        }
        case CT::AccelBeginPose: pipeline_.accelBeginPose(c.u.pose); break;
        case CT::AccelSolve: requestOk_ = pipeline_.accelSolve(accelResult_); signal = true; break;
        case CT::StartMountCal: pipeline_.startMountCalibration(c.u.proc); break;
        case CT::MountCapture: pipeline_.mountCapturePosition(); break;
        case CT::MountSolve:
            mountResult_ = pipeline_.mountSolve(sda::Vec3(c.u.mount.hint[0], c.u.mount.hint[1], c.u.mount.hint[2]), c.u.mount.bitDown);
            requestOk_ = mountResult_.ok;
            signal = true;
            break;
        case CT::CancelCal: pipeline_.cancelCalibration(); break;
        case CT::CaptureTiltZero: pipeline_.captureTiltZero(); break;
        case CT::CaptureFull3D: pipeline_.captureFull3D(); break;
        case CT::CaptureSurface: pipeline_.captureSurface(c.u.b); break;
        case CT::ClearReference: pipeline_.clearReference(); break;
        case CT::StartPrecision: {
            sda::PrecisionConfig pc;
            pc.windowSec = c.u.precision.window;
            pipeline_.startPrecision(pc, c.u.precision.qty);
            break;
        }
        case CT::AbortPrecision: pipeline_.abortPrecision(); break;
        case CT::GetCalibration: stagedCal_ = pipeline_.calibration(); requestOk_ = true; signal = true; break;
        case CT::GetGyroResult:
            gyroResult_ = pipeline_.gyroResult();
            gyroResultOk_ = pipeline_.gyroResultOk();
            requestOk_ = true;
            signal = true;
            break;
        case CT::Suspend: signal = true; break;
        case CT::SetRate: pipeline_.setNominalRate(c.u.f); break;
    }
    if (signal) xSemaphoreGive(done_);
}

bool MeasurementService::post(const Command& c) { return cmdQueue_ && xQueueSend(cmdQueue_, &c, pdMS_TO_TICKS(10)) == pdTRUE; }

bool MeasurementService::request(const Command& c, uint32_t timeoutMs) {
    // caller holds requestMutex_
    xSemaphoreTake(done_, 0);  // clear stale signal
    if (!post(c)) return false;
    return xSemaphoreTake(done_, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

namespace {
MeasurementService::Command makeCmd(MeasurementService::CmdType t) {
    MeasurementService::Command c;
    c.type = t;
    c.u.f = 0;
    return c;
}
struct RequestLock {
    explicit RequestLock(SemaphoreHandle_t m) : m_(m), ok_(m && xSemaphoreTake(m, pdMS_TO_TICKS(500)) == pdTRUE) {}
    ~RequestLock() { if (ok_) xSemaphoreGive(m_); }
    SemaphoreHandle_t m_;
    bool ok_;
};
}  // namespace

bool MeasurementService::reset() { return post(makeCmd(CmdType::Reset)); }

bool MeasurementService::setCalibration(const sda::SensorCalibration& cal) {
    RequestLock l(requestMutex_);
    if (!l.ok_) return false;
    stagedCal_ = cal;
    return request(makeCmd(CmdType::SetCalibration), 200);
}

bool MeasurementService::setMount(const sda::MountModel& m) {
    RequestLock l(requestMutex_);
    if (!l.ok_) return false;
    stagedMount_ = m;
    return request(makeCmd(CmdType::SetMount), 200);
}

bool MeasurementService::setConfig(const sda::PipelineConfig& cfg) {
    RequestLock l(requestMutex_);
    if (!l.ok_) return false;
    stagedCfg_ = cfg;
    return request(makeCmd(CmdType::SetConfig), 200);
}

bool MeasurementService::setFusion(sda::FusionType t) {
    Command c = makeCmd(CmdType::SetFusion);
    c.u.fusion = t;
    return post(c);
}

bool MeasurementService::setFilterTau(float tau) {
    Command c = makeCmd(CmdType::SetFilterTau);
    c.u.f = tau;
    return post(c);
}

bool MeasurementService::startGyroCalibration() { return post(makeCmd(CmdType::StartGyroCal)); }

bool MeasurementService::startAccelCalibration(bool extended) {
    Command c = makeCmd(CmdType::StartAccelCal);
    c.u.b = extended;
    return post(c);
}

bool MeasurementService::accelBeginPose(sda::AccelPose p) {
    Command c = makeCmd(CmdType::AccelBeginPose);
    c.u.pose = p;
    return post(c);
}

bool MeasurementService::startMountCalibration(sda::CalProcedure p) {
    Command c = makeCmd(CmdType::StartMountCal);
    c.u.proc = p;
    return post(c);
}

bool MeasurementService::mountCapture() { return post(makeCmd(CmdType::MountCapture)); }
bool MeasurementService::cancelCalibration() { return post(makeCmd(CmdType::CancelCal)); }
bool MeasurementService::captureTiltZero() { return post(makeCmd(CmdType::CaptureTiltZero)); }
bool MeasurementService::captureFull3D() { return post(makeCmd(CmdType::CaptureFull3D)); }

bool MeasurementService::captureSurface(bool use3D) {
    Command c = makeCmd(CmdType::CaptureSurface);
    c.u.b = use3D;
    return post(c);
}

bool MeasurementService::clearReference() { return post(makeCmd(CmdType::ClearReference)); }

bool MeasurementService::startPrecision(float windowSec, sda::PrecisionQuantity q) {
    Command c = makeCmd(CmdType::StartPrecision);
    c.u.precision.window = windowSec;
    c.u.precision.qty = q;
    return post(c);
}

bool MeasurementService::abortPrecision() { return post(makeCmd(CmdType::AbortPrecision)); }

bool MeasurementService::accelSolve(sda::AccelCalPoint& out, uint32_t timeoutMs) {
    RequestLock l(requestMutex_);
    if (!l.ok_) return false;
    if (!request(makeCmd(CmdType::AccelSolve), timeoutMs)) return false;
    out = accelResult_;
    return requestOk_;
}

bool MeasurementService::mountSolve(const sda::Vec3& hint, bool bitDown, sda::MountFitResult& out,
                                    uint32_t timeoutMs) {
    RequestLock l(requestMutex_);
    if (!l.ok_) return false;
    Command c = makeCmd(CmdType::MountSolve);
    c.u.mount.hint[0] = hint.x;
    c.u.mount.hint[1] = hint.y;
    c.u.mount.hint[2] = hint.z;
    c.u.mount.bitDown = bitDown;
    if (!request(c, timeoutMs)) return false;
    out = mountResult_;
    return true;
}

bool MeasurementService::currentCalibration(sda::SensorCalibration& out, uint32_t timeoutMs) {
    RequestLock l(requestMutex_);
    if (!l.ok_) return false;
    if (!request(makeCmd(CmdType::GetCalibration), timeoutMs)) return false;
    out = stagedCal_;
    return true;
}

bool MeasurementService::lastGyroResult(sda::GyroCalPoint& out, bool& ok, uint32_t timeoutMs) {
    RequestLock l(requestMutex_);
    if (!l.ok_) return false;
    if (!request(makeCmd(CmdType::GetGyroResult), timeoutMs)) return false;
    out = gyroResult_;
    ok = gyroResultOk_;
    return true;
}

void MeasurementService::suspend() {
    suspended_ = true;
    vTaskDelay(pdMS_TO_TICKS(20));
    if (imu_) imu_->setPowerDown(true);
}
