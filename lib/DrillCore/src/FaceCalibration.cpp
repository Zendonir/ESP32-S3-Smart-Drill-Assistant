// SPDX-License-Identifier: MIT
#include "FaceCalibration.h"

namespace sda {

const char* supportFaceName(SupportFace f) { return f == SupportFace::Back ? "Rückseite" : "Unterseite"; }

Vec3 supportFaceNominalUp(SupportFace f) { return f == SupportFace::Back ? Vec3(0, 0, 1) : Vec3(0, 1, 0); }

void faceAngles(SupportFace face, const Vec3& upIn, float& a, float& b) {
    const Vec3 u = upIn.normalized();
    a = rad2deg(safeAsin(u.x));
    b = rad2deg(safeAsin(face == SupportFace::Back ? u.y : u.z));
}

FlipResult solveFlip(SupportFace face, const Vec3& u1, const Vec3& u2) {
    FlipResult r;
    const Vec3 nominal = supportFaceNominalUp(face);
    if (rad2deg(angleBetween(u1, nominal)) > 15.0f || rad2deg(angleBetween(u2, nominal)) > 15.0f) {
        r.error = "Gerät liegt nicht auf dieser Fläche";
        return r;
    }
    float a1, b1, a2, b2;
    faceAngles(face, u1, a1, b1);
    faceAngles(face, u2, a2, b2);
    FaceCalibration c;
    c.errADeg = 0.5f * (a1 + a2);
    c.errBDeg = 0.5f * (b1 + b2);
    const float ta = 0.5f * (a1 - a2), tb = 0.5f * (b1 - b2);
    c.tableTiltDeg = std::sqrt(ta * ta + tb * tb);
    c.pairs = 1;
    if (c.tableTiltDeg > 5.0f) {
        r.error = "Fläche zu schräg (> 5°)";
        return r;
    }
    if (std::fabs(c.errADeg) > 5.0f || std::fabs(c.errBDeg) > 5.0f) {
        r.error = "Gerätefehler unplausibel (> 5°)";
        return r;
    }
    c.valid = true;
    r.cal = c;
    r.ok = true;
    return r;
}

FaceCalibration combineFlips(const FaceCalibration* p, int n) {
    FaceCalibration c;
    if (n <= 0) return c;
    float minA = 1e9f, maxA = -1e9f, minB = 1e9f, maxB = -1e9f, tilt = 0;
    for (int i = 0; i < n; ++i) {
        c.errADeg += p[i].errADeg / float(n);
        c.errBDeg += p[i].errBDeg / float(n);
        tilt += p[i].tableTiltDeg / float(n);
        minA = std::fmin(minA, p[i].errADeg);
        maxA = std::fmax(maxA, p[i].errADeg);
        minB = std::fmin(minB, p[i].errBDeg);
        maxB = std::fmax(maxB, p[i].errBDeg);
    }
    c.tableTiltDeg = tilt;
    c.spreadDeg = n > 1 ? std::fmax(maxA - minA, maxB - minB) : -1.0f;
    c.pairs = static_cast<uint8_t>(n);
    c.valid = true;
    return c;
}

bool applyFaceCorrections(LevelAngles& l, const Vec3& upD, const FaceCalibration& back,
                          const FaceCalibration& bottom) {
    const Vec3 u = upD.normalized();
    if (l.flat && u.z > 0.0f && back.valid) {
        l.tiltXDeg -= back.errADeg;
        l.tiltYDeg -= back.errBDeg;
        l.faceCorrected = true;
        return true;
    }
    // standing on the bottom face: device +Y up (edge index 0)
    if (!l.flat && l.edgeIndex == 0 && u.y > 0.0f && bottom.valid) {
        l.edgeRollDeg -= bottom.errADeg;
        l.tiltXDeg -= bottom.errADeg;
        l.edgePitchDeg -= bottom.errBDeg;
        l.faceCorrected = true;
        return true;
    }
    return false;
}

}  // namespace sda
