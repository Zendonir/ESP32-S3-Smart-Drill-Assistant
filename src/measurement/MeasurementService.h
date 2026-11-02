// SPDX-License-Identifier: MIT
// Device side of the measurement system: owns the IMU task and the fusion task.
//
//   IMU task (core 1, prio 20)    polls the QMI8658 (data driven, ~224 Hz), stamps
//                                 samples, detects bus errors/timeouts, queues samples.
//   Fusion task (core 1, prio 19) runs the MeasurementPipeline for every sample,
//                                 executes commands and publishes snapshots.
//
// Other tasks (UI, network) never touch the pipeline directly: they post commands
// (lock-free FreeRTOS queue) and read snapshot copies (SharedState).
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "MeasurementPipeline.h"
#include "../system/SharedState.h"

class QMI8658Driver;
class I2CBus;

class MeasurementService {
public:
    enum class CmdType : uint8_t {
        Reset, SetCalibration, SetMount, SetFusion, SetFilterTau, SetConfig,
        StartGyroCal, StartAccelCal, AccelBeginPose, AccelSolve, StartMountCal, MountCapture, MountSolve,
        CancelCal, CaptureTiltZero, CaptureFull3D, CaptureSurface, ClearReference, StartPrecision, AbortPrecision,
        GetCalibration, GetGyroResult, Suspend,
    };

    struct Stats {
        uint32_t samples = 0;
        uint32_t queueOverflows = 0;
        uint32_t busErrors = 0;
        uint32_t recoveries = 0;
        uint32_t timeouts = 0;
        float fusionLoadPct = 0.0f;   ///< share of time spent in pipeline (core 1)
        uint32_t maxProcessUs = 0;
        uint32_t imuStackFree = 0;
        uint32_t fusionStackFree = 0;
    };

    bool begin(QMI8658Driver* imu, I2CBus* bus, const sda::PipelineConfig& cfg, const sda::SensorCalibration& cal,
               const sda::MountModel& mount);
    bool imuOk() const { return imuOk_; }

    /// Latest snapshot (copy). Returns snapshot version.
    uint32_t snapshot(sda::OrientationSnapshot& out) const { return shared_.read(out); }
    Stats stats() const { return stats_; }

    // ---- asynchronous commands ---------------------------------------------
    bool reset();
    bool setCalibration(const sda::SensorCalibration& cal);
    bool setMount(const sda::MountModel& m);
    bool setFusion(sda::FusionType t);
    bool setFilterTau(float tau);
    bool setConfig(const sda::PipelineConfig& cfg);
    bool startGyroCalibration();
    bool startAccelCalibration(bool extended);
    bool accelBeginPose(sda::AccelPose p);
    bool startMountCalibration(sda::CalProcedure p);
    bool mountCapture();
    bool cancelCalibration();
    bool captureTiltZero();
    bool captureFull3D();
    bool captureSurface(bool use3D);
    bool clearReference();
    bool startPrecision(float windowSec, sda::PrecisionQuantity q);
    bool abortPrecision();

    // ---- synchronous requests (wait for the fusion task, max timeoutMs) -------
    bool accelSolve(sda::AccelCalPoint& out, uint32_t timeoutMs = 200);
    bool mountSolve(const sda::Vec3& hint, bool bitDown, sda::MountFitResult& out, uint32_t timeoutMs = 200);
    /// Copies the current calibration (after gyro calibration it contains the new point)
    bool currentCalibration(sda::SensorCalibration& out, uint32_t timeoutMs = 200);
    bool lastGyroResult(sda::GyroCalPoint& out, bool& ok, uint32_t timeoutMs = 200);

    /// Power down / up the IMU (deep sleep preparation)
    void suspend();

    struct Command {
        CmdType type;
        union {
            sda::FusionType fusion;
            float f;
            bool b;
            sda::AccelPose pose;
            sda::CalProcedure proc;
            struct { float window; sda::PrecisionQuantity qty; } precision;
            struct { float hint[3]; bool bitDown; } mount;
        } u;
    };

private:

    static void imuTaskEntry(void* arg);
    static void fusionTaskEntry(void* arg);
    void imuTask();
    void fusionTask();
    bool post(const Command& c);
    void execute(const Command& c);
    bool request(const Command& c, uint32_t timeoutMs);

    QMI8658Driver* imu_ = nullptr;
    I2CBus* bus_ = nullptr;
    bool imuOk_ = false;
    volatile bool suspended_ = false;
    sda::MeasurementPipeline pipeline_;
    SharedState<sda::OrientationSnapshot> shared_;
    QueueHandle_t sampleQueue_ = nullptr;
    QueueHandle_t cmdQueue_ = nullptr;
    SemaphoreHandle_t done_ = nullptr;
    SemaphoreHandle_t requestMutex_ = nullptr;
    TaskHandle_t imuTask_ = nullptr;
    TaskHandle_t fusionTask_ = nullptr;
    Stats stats_;

    // staging areas for objects too large for the queue (guarded by requestMutex_)
    sda::SensorCalibration stagedCal_;
    sda::MountModel stagedMount_;
    sda::PipelineConfig stagedCfg_;
    sda::AccelCalPoint accelResult_;
    sda::MountFitResult mountResult_;
    sda::GyroCalPoint gyroResult_;
    bool gyroResultOk_ = false;
    bool requestOk_ = false;
};
