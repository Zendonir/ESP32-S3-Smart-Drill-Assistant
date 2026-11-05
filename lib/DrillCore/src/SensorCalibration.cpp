// SPDX-License-Identifier: MIT
#include "SensorCalibration.h"

#include "LinearAlgebra.h"

namespace sda {

// ============================================================================
// Temperature table helpers
// ============================================================================
namespace {

/// Finds interpolation partners for temperature t in a sorted array of temps.
/// Returns weight for the upper point (0..1) and indices lo/hi.
template <typename P>
void findBracket(const P* pts, int n, float t, int& lo, int& hi, float& w, TempLookup& info) {
    info.points = n;
    info.valid = n > 0;
    lo = hi = 0;
    w = 0.0f;
    if (n <= 0) return;
    if (n == 1 || !std::isfinite(t)) {
        // single point or no valid temperature: constant correction, nearest/most robust
        lo = hi = 0;
        info.compensated = false;
        info.distanceC = (n == 1 && std::isfinite(t) && std::isfinite(pts[0].tempC))
                             ? std::fabs(t - pts[0].tempC)
                             : 0.0f;
        return;
    }
    if (t <= pts[0].tempC) {
        lo = hi = 0;
        info.distanceC = pts[0].tempC - t;
    } else if (t >= pts[n - 1].tempC) {
        lo = hi = n - 1;
        info.distanceC = t - pts[n - 1].tempC;
    } else {
        for (int i = 0; i < n - 1; ++i) {
            if (t >= pts[i].tempC && t <= pts[i + 1].tempC) {
                lo = i;
                hi = i + 1;
                const float span = pts[hi].tempC - pts[lo].tempC;
                w = span > 1e-3f ? (t - pts[lo].tempC) / span : 0.0f;
                break;
            }
        }
        info.distanceC = 0.0f;
    }
    // Interpolation is only meaningful with a span of >= 3 K. Extrapolation is not
    // performed (clamped); more than 5 K outside the range counts as uncompensated.
    const float span = pts[n - 1].tempC - pts[0].tempC;
    info.compensated = span >= 3.0f && info.distanceC <= 5.0f;
}

template <typename P>
void insertPoint(P* pts, int& count, int maxCount, const P& p, float mergeC) {
    // replace point at similar temperature
    for (int i = 0; i < count; ++i) {
        const bool bothNan = !std::isfinite(pts[i].tempC) && !std::isfinite(p.tempC);
        if (bothNan || (std::isfinite(pts[i].tempC) && std::isfinite(p.tempC) &&
                        std::fabs(pts[i].tempC - p.tempC) < mergeC)) {
            pts[i] = p;
            return;
        }
    }
    if (!std::isfinite(p.tempC)) {
        // without a temperature the point cannot be part of a table: keep only it
        pts[0] = p;
        count = 1;
        return;
    }
    // drop points without temperature when a table is built
    int w = 0;
    for (int i = 0; i < count; ++i)
        if (std::isfinite(pts[i].tempC)) pts[w++] = pts[i];
    count = w;
    if (count < maxCount) {
        pts[count++] = p;
        return;
    }
    // full: replace the point nearest in temperature
    int best = 0;
    float bestD = 1e9f;
    for (int i = 0; i < count; ++i) {
        const float d = std::fabs(pts[i].tempC - p.tempC);
        if (d < bestD) { bestD = d; best = i; }
    }
    pts[best] = p;
}

template <typename P>
void sortByTemp(P* pts, int n) {
    for (int i = 1; i < n; ++i) {
        P key = pts[i];
        int j = i - 1;
        while (j >= 0 && pts[j].tempC > key.tempC) {
            pts[j + 1] = pts[j];
            --j;
        }
        pts[j + 1] = key;
    }
}

inline bool finiteVec(const Vec3& v, float maxAbs) {
    return v.isFinite() && std::fabs(v.x) <= maxAbs && std::fabs(v.y) <= maxAbs &&
           std::fabs(v.z) <= maxAbs;
}

}  // namespace

// ============================================================================
// SensorCalibration
// ============================================================================

void SensorCalibration::addGyroPoint(const GyroCalPoint& p, float mergeC) {
    insertPoint(gyro_, gyroCount_, kMaxGyroCalPoints, p, mergeC);
    sortGyro();
}

void SensorCalibration::addAccelPoint(const AccelCalPoint& p, float mergeC) {
    insertPoint(accel_, accelCount_, kMaxAccelCalPoints, p, mergeC);
    sortAccel();
}

void SensorCalibration::sortGyro() { sortByTemp(gyro_, gyroCount_); }
void SensorCalibration::sortAccel() { sortByTemp(accel_, accelCount_); }

Vec3 SensorCalibration::gyroBias(float tempC, TempLookup* info) const {
    TempLookup li;
    int lo, hi;
    float w;
    findBracket(gyro_, gyroCount_, tempC, lo, hi, w, li);
    if (info) *info = li;
    if (gyroCount_ == 0) return Vec3();
    return gyro_[lo].biasDps * (1.0f - w) + gyro_[hi].biasDps * w;
}

Vec3 SensorCalibration::gyroNoise() const {
    if (gyroCount_ == 0) return Vec3(0.05f, 0.05f, 0.05f);  // conservative default
    return gyro_[gyroCount_ - 1].noiseDps;
}

void SensorCalibration::accelCorrection(float tempC, Mat3& m, Vec3& c, TempLookup* info) const {
    TempLookup li;
    int lo, hi;
    float w;
    findBracket(accel_, accelCount_, tempC, lo, hi, w, li);
    if (info) *info = li;
    if (accelCount_ == 0) {
        m = Mat3::identity();
        c = Vec3();
        return;
    }
    m = accel_[lo].matrix * (1.0f - w) + accel_[hi].matrix * w;
    c = accel_[lo].offset * (1.0f - w) + accel_[hi].offset * w;
}

Vec3 SensorCalibration::applyGyro(const Vec3& rawDps, float tempC) const {
    return rawDps - gyroBias(tempC);
}

Vec3 SensorCalibration::applyAccel(const Vec3& rawG, float tempC) const {
    Mat3 m;
    Vec3 c;
    accelCorrection(tempC, m, c);
    return m * rawG + c;
}

bool SensorCalibration::gyroTempCompActive() const {
    return gyroCount_ >= 2 && (gyro_[gyroCount_ - 1].tempC - gyro_[0].tempC) >= 3.0f;
}

bool SensorCalibration::accelTempCompActive() const {
    return accelCount_ >= 2 && (accel_[accelCount_ - 1].tempC - accel_[0].tempC) >= 3.0f;
}

void SensorCalibration::remap(const Mat3& R) {
    const Mat3 Rt = R.transposed();
    for (int i = 0; i < gyroCount_; ++i) {
        gyro_[i].biasDps = R * gyro_[i].biasDps;
        const Vec3 n = R * gyro_[i].noiseDps;  // per-axis magnitudes follow the permutation
        gyro_[i].noiseDps = Vec3(std::fabs(n.x), std::fabs(n.y), std::fabs(n.z));
    }
    for (int i = 0; i < accelCount_; ++i) {
        // a_cal = M a_raw + c  in the old frame  ->  R a_cal = (R M R^T)(R a_raw) + R c
        accel_[i].matrix = R * accel_[i].matrix * Rt;
        accel_[i].offset = R * accel_[i].offset;
    }
}

bool SensorCalibration::validate() const {
    if (gyroCount_ < 0 || gyroCount_ > kMaxGyroCalPoints) return false;
    if (accelCount_ < 0 || accelCount_ > kMaxAccelCalPoints) return false;
    for (int i = 0; i < gyroCount_; ++i) {
        if (!finiteVec(gyro_[i].biasDps, 50.0f)) return false;
        if (!finiteVec(gyro_[i].noiseDps, 10.0f)) return false;
        if (std::isfinite(gyro_[i].tempC) && (gyro_[i].tempC < -40 || gyro_[i].tempC > 100))
            return false;
    }
    for (int i = 0; i < accelCount_; ++i) {
        const AccelCalPoint& p = accel_[i];
        if (!finiteVec(p.offset, 0.5f)) return false;
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 3; ++k) {
                const float v = p.matrix.m[r][k];
                if (!std::isfinite(v)) return false;
                if (r == k && (v < 0.8f || v > 1.25f)) return false;   // scale within +/-20 %
                if (r != k && std::fabs(v) > 0.1f) return false;      // < ~6 deg misalignment
            }
    }
    return true;
}

void SensorCalibration::setGyroPoints(const GyroCalPoint* p, int n) {
    gyroCount_ = 0;
    for (int i = 0; i < n && i < kMaxGyroCalPoints; ++i) gyro_[gyroCount_++] = p[i];
    sortGyro();
}

void SensorCalibration::setAccelPoints(const AccelCalPoint* p, int n) {
    accelCount_ = 0;
    for (int i = 0; i < n && i < kMaxAccelCalPoints; ++i) accel_[accelCount_++] = p[i];
    sortAccel();
}

// ============================================================================
// GyroCalibrator
// ============================================================================

void GyroCalibrator::start(const GyroCalConfig& cfg) {
    cfg_ = cfg;
    restarts_ = 0;
    failure_ = Failure::None;
    state_ = State::Collecting;
    restart();
    restarts_ = 0;
}

void GyroCalibrator::restart() {
    n_ = 0;
    for (int i = 0; i < 3; ++i) sum_[i] = sumSq_[i] = 0.0;
    tempSum_ = 0;
    tempN_ = 0;
    ++restarts_;
}

float GyroCalibrator::progress() const {
    if (state_ == State::Done) return 1.0f;
    if (state_ != State::Collecting || cfg_.samples == 0) return 0.0f;
    return static_cast<float>(n_) / static_cast<float>(cfg_.samples);
}

void GyroCalibrator::addSample(const Vec3& g, const Vec3& a, float tempC) {
    motionFlag_ = false;
    if (state_ != State::Collecting) return;
    if (!g.isFinite() || !a.isFinite()) {
        state_ = State::Failed;
        failure_ = Failure::InvalidData;
        return;
    }
    if (n_ == 0) {
        accelRef_ = a;
    } else {
        const Vec3 mean(static_cast<float>(sum_[0] / n_), static_cast<float>(sum_[1] / n_),
                        static_cast<float>(sum_[2] / n_));
        // motion check: rate deviation from running mean (bias itself may be large)
        // and change of the gravity vector
        if ((g - mean).norm() > cfg_.maxMotionDps || (a - accelRef_).norm() > cfg_.maxAccelDevG) {
            motionFlag_ = true;
            if (restarts_ >= cfg_.maxRestarts) {
                state_ = State::Failed;
                failure_ = Failure::TooMuchMotion;
                return;
            }
            restart();
            return;
        }
    }
    for (int i = 0; i < 3; ++i) {
        sum_[i] += g[i];
        sumSq_[i] += double(g[i]) * double(g[i]);
    }
    if (std::isfinite(tempC)) {
        tempSum_ += tempC;
        tempN_++;
    }
    ++n_;
    if (n_ >= cfg_.samples) {
        GyroCalPoint r;
        for (int i = 0; i < 3; ++i) {
            const double mean = sum_[i] / n_;
            const double var = sumSq_[i] / n_ - mean * mean;
            r.biasDps[i] = static_cast<float>(mean);
            r.noiseDps[i] = static_cast<float>(var > 0 ? std::sqrt(var) : 0.0);
        }
        r.samples = n_;
        r.tempC = tempN_ > 0 ? static_cast<float>(tempSum_ / tempN_) : NAN;
        result_ = r;
        if (r.noiseDps.x > cfg_.maxNoiseDps || r.noiseDps.y > cfg_.maxNoiseDps ||
            r.noiseDps.z > cfg_.maxNoiseDps) {
            state_ = State::Failed;
            failure_ = Failure::TooNoisy;
        } else if (std::fabs(r.biasDps.x) > cfg_.maxAbsBiasDps ||
                   std::fabs(r.biasDps.y) > cfg_.maxAbsBiasDps ||
                   std::fabs(r.biasDps.z) > cfg_.maxAbsBiasDps) {
            state_ = State::Failed;
            failure_ = Failure::Implausible;
        } else {
            state_ = State::Done;
        }
    }
}

// ============================================================================
// AccelCalibrator
// ============================================================================

Vec3 accelPoseExpected(AccelPose p) {
    switch (p) {
        case AccelPose::PosX: return {1, 0, 0};
        case AccelPose::NegX: return {-1, 0, 0};
        case AccelPose::PosY: return {0, 1, 0};
        case AccelPose::NegY: return {0, -1, 0};
        case AccelPose::PosZ: return {0, 0, 1};
        case AccelPose::NegZ: return {0, 0, -1};
    }
    return {0, 0, 1};
}

const char* accelPoseName(AccelPose p) {
    static const char* names[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
    return names[int(p) % 6];
}

void AccelCalibrator::start(const AccelCalConfig& cfg) {
    cfg_ = cfg;
    for (auto& p : poses_) p = Pose{};
    active_ = -1;
    failure_ = Failure::None;
    resetAccumulators();
}

void AccelCalibrator::resetAccumulators() {
    n_ = 0;
    for (int i = 0; i < 3; ++i) sum_[i] = sumSq_[i] = 0;
    tempSum_ = 0;
    tempN_ = 0;
}

void AccelCalibrator::beginPose(AccelPose p) {
    active_ = int(p);
    poses_[active_].state = PoseState::Collecting;
    failure_ = Failure::None;
    resetAccumulators();
}

float AccelCalibrator::poseProgress() const {
    if (active_ < 0 || cfg_.samplesPerPose == 0) return 0.0f;
    return static_cast<float>(n_) / static_cast<float>(cfg_.samplesPerPose);
}

bool AccelCalibrator::allPosesAccepted() const {
    for (const auto& p : poses_)
        if (p.state != PoseState::Accepted) return false;
    return true;
}

int AccelCalibrator::detectPose(const Vec3& a, float maxTiltDeg) {
    const float n = a.norm();
    if (n < 0.7f || n > 1.3f) return -1;
    const Vec3 u = a / n;
    const float cosLim = std::cos(deg2rad(maxTiltDeg));
    for (int i = 0; i < kAccelPoseCount; ++i)
        if (u.dot(accelPoseExpected(AccelPose(i))) >= cosLim) return i;
    return -1;
}

void AccelCalibrator::addSample(const Vec3& a, const Vec3& g, float tempC) {
    if (active_ < 0) return;
    if (!a.isFinite() || !g.isFinite()) return;
    Pose& pose = poses_[active_];
    // orientation check
    const float n = a.norm();
    const Vec3 expected = accelPoseExpected(AccelPose(active_));
    const float cosLim = std::cos(deg2rad(cfg_.maxPoseTiltDeg));
    if (n < 0.7f || n > 1.3f || a.dot(expected) / n < cosLim) {
        failure_ = Failure::WrongOrientation;
        resetAccumulators();
        return;
    }
    // stillness check (gyro is uncalibrated here; use a generous threshold)
    if (n_ > 0) {
        if ((a - runMean_).norm() > cfg_.maxSampleDevG || g.norm() > cfg_.maxGyroDps + 20.0f) {
            failure_ = Failure::Unstable;
            resetAccumulators();
            return;
        }
    }
    for (int i = 0; i < 3; ++i) {
        sum_[i] += a[i];
        sumSq_[i] += double(a[i]) * double(a[i]);
    }
    if (std::isfinite(tempC)) {
        tempSum_ += tempC;
        tempN_++;
    }
    ++n_;
    runMean_ = Vec3(float(sum_[0] / n_), float(sum_[1] / n_), float(sum_[2] / n_));
    failure_ = Failure::None;
    if (n_ >= cfg_.samplesPerPose) {
        pose.mean = runMean_;
        for (int i = 0; i < 3; ++i) {
            const double m = sum_[i] / n_;
            const double var = sumSq_[i] / n_ - m * m;
            pose.stddev[i] = float(var > 0 ? std::sqrt(var) : 0);
        }
        pose.tempC = tempN_ ? float(tempSum_ / tempN_) : NAN;
        pose.state = PoseState::Accepted;
        active_ = -1;
        resetAccumulators();
    }
}

bool AccelCalibrator::solve(AccelCalPoint& out) {
    if (!allPosesAccepted()) {
        failure_ = Failure::SolveFailed;
        return false;
    }
    AccelCalPoint r;
    // temperature: mean over poses
    double ts = 0;
    int tn = 0;
    for (const auto& p : poses_)
        if (std::isfinite(p.tempC)) { ts += p.tempC; ++tn; }
    r.tempC = tn ? float(ts / tn) : NAN;

    if (!cfg_.extended) {
        // Offset/scale per axis: with axis i up the reading is  s_i*1g + o_i, with axis
        // i down  -s_i*1g + o_i. Small placement tilts only enter in 2nd order (cos).
        Mat3 m = Mat3::identity();
        Vec3 c;
        for (int i = 0; i < 3; ++i) {
            const float pos = poses_[2 * i].mean[i];
            const float neg = poses_[2 * i + 1].mean[i];
            const float offset = 0.5f * (pos + neg);
            const float scale = 0.5f * (pos - neg);
            if (!(scale > 0.5f && scale < 1.5f)) {
                failure_ = Failure::Implausible;
                return false;
            }
            m.m[i][i] = 1.0f / scale;
            c[i] = -offset / scale;
        }
        r.matrix = m;
        r.offset = c;
        r.extended = false;
    } else {
        // Full affine model a_cal = M a_raw + c solved per output row by linear least
        // squares over the 6 pose means (6 equations, 4 unknowns per row).
        // Requires the poses to be accurately orthogonal (precision square block).
        Mat3 m;
        Vec3 c;
        for (int row = 0; row < 3; ++row) {
            LeastSquares ls(4);
            for (int pidx = 0; pidx < kAccelPoseCount; ++pidx) {
                const Vec3& a = poses_[pidx].mean;
                const float h[4] = {a.x, a.y, a.z, 1.0f};
                ls.add(h, accelPoseExpected(AccelPose(pidx))[row]);
            }
            float x[4];
            if (!ls.solve(x)) {
                failure_ = Failure::SolveFailed;
                return false;
            }
            m.m[row][0] = x[0];
            m.m[row][1] = x[1];
            m.m[row][2] = x[2];
            c[row] = x[3];
        }
        r.matrix = m;
        r.offset = c;
        r.extended = true;
    }

    // Quality: magnitude residual over all poses after correction.
    double sq = 0;
    for (const auto& p : poses_) {
        const Vec3 cal = r.matrix * p.mean + r.offset;
        const double e = (cal.norm() - 1.0) * 1000.0;
        sq += e * e;
    }
    r.residualMg = float(std::sqrt(sq / kAccelPoseCount));

    // Plausibility (same limits as SensorCalibration::validate)
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) {
            const float v = r.matrix.m[i][k];
            if (i == k && (v < 0.8f || v > 1.25f)) { failure_ = Failure::Implausible; return false; }
            if (i != k && std::fabs(v) > 0.1f) { failure_ = Failure::Implausible; return false; }
        }
    if (std::fabs(r.offset.x) > 0.5f || std::fabs(r.offset.y) > 0.5f || std::fabs(r.offset.z) > 0.5f) {
        failure_ = Failure::Implausible;
        return false;
    }
    out = r;
    failure_ = Failure::None;
    return true;
}

}  // namespace sda
