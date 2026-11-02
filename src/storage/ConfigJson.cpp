// SPDX-License-Identifier: MIT
#include "ConfigJson.h"

#include <cmath>
#include <cstring>

using namespace sda;

namespace cfgjson {

namespace {

void vecTo(JsonArray a, const Vec3& v) {
    a.add(v.x);
    a.add(v.y);
    a.add(v.z);
}

bool vecFrom(JsonVariantConst v, Vec3& out) {
    if (!v.is<JsonArrayConst>()) return false;
    JsonArrayConst a = v.as<JsonArrayConst>();
    if (a.size() != 3) return false;
    for (int i = 0; i < 3; ++i) {
        if (!a[i].is<float>()) return false;
        out[i] = a[i].as<float>();
    }
    return true;
}

template <typename T>
bool getNum(JsonObjectConst o, const char* key, T& out) {
    JsonVariantConst v = o[key];
    if (v.isNull()) return true;  // absent: keep
    if (!v.is<double>() && !v.is<long>() && !v.is<bool>()) return false;
    const double d = v.as<double>();
    if (!std::isfinite(d) || d < -1e7 || d > 1e9) return false;
    out = static_cast<T>(d);
    return true;
}

bool getBool(JsonObjectConst o, const char* key, bool& out) {
    JsonVariantConst v = o[key];
    if (v.isNull()) return true;
    if (!v.is<bool>()) return false;
    out = v.as<bool>();
    return true;
}

template <typename E>
bool getEnum(JsonObjectConst o, const char* key, E& out) {
    int tmp = static_cast<int>(out);
    if (!getNum(o, key, tmp)) return false;
    if (tmp < 0 || tmp > 255) return false;
    out = static_cast<E>(tmp);
    return true;
}

bool getStr(JsonObjectConst o, const char* key, char* out, size_t n) {
    JsonVariantConst v = o[key];
    if (v.isNull()) return true;
    if (!v.is<const char*>()) return false;
    const char* s = v.as<const char*>();
    if (std::strlen(s) >= n) return false;  // reject instead of truncating silently
    copyString(out, n, s);
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------
void settingsToJson(const AppSettings& s, JsonObject o, bool secrets) {
    o["version"] = s.version;
    JsonObject m = o["measurement"].to<JsonObject>();
    m["filterStrength"] = s.measurement.filterStrength;
    m["fusion"] = static_cast<int>(s.measurement.fusion);
    m["precisionWindowSec"] = s.measurement.precisionWindowSec;
    m["holdTimeSec"] = s.measurement.holdTimeSec;
    m["hemisphere"] = s.measurement.hemisphere;
    m["zeroKind"] = static_cast<int>(s.measurement.zeroKind);
    m["reference3D"] = s.measurement.reference3D;
    m["customAngleDeg"] = s.measurement.customAngleDeg;
    m["tempCoeffAssumedDpsPerK"] = s.measurement.tempCoeffAssumedDpsPerK;
    m["onlineBiasTracking"] = s.measurement.onlineBiasTracking;
    m["startupGyroCal"] = static_cast<int>(s.measurement.startupGyroCal);
    m["lastMode"] = static_cast<int>(s.measurement.lastMode);

    JsonObject d = o["display"].to<JsonObject>();
    d["brightness"] = s.display.brightness;
    d["dimAfterSec"] = s.display.dimAfterSec;
    d["offAfterSec"] = s.display.offAfterSec;
    d["rotation"] = s.display.rotation;
    d["theme"] = s.display.theme;
    d["colorMode"] = s.display.colorMode;
    d["largeFont"] = s.display.largeFont;
    d["decimals"] = s.display.decimals;
    d["touchLockAuto"] = s.display.touchLockAuto;
    d["showHistory"] = s.display.showHistory;
    d["historyWindowSec"] = s.display.historyWindowSec;

    JsonObject y = o["system"].to<JsonObject>();
    y["wifiMode"] = static_cast<int>(s.system.wifiMode);
    y["ssid"] = s.system.ssid;
    y["hostname"] = s.system.hostname;
    y["mdns"] = s.system.mdns;
    y["sleepAfterMin"] = s.system.sleepAfterMin;
    y["bootButton"] = static_cast<int>(s.system.bootButton);
    char axis[8];
    s.system.axisMap.format(axis);
    y["axisMap"] = axis;
    y["logLevel"] = s.system.logLevel;
    y["batteryDividerFactor"] = s.system.batteryDividerFactor;
    y["lowBatteryV"] = s.system.lowBatteryV;
    y["criticalBatteryV"] = s.system.criticalBatteryV;
    y["activeProfile"] = s.system.activeProfile;
    y["liveRateHz"] = s.system.liveRateHz;
    if (secrets) {
        y["password"] = s.system.password;
        y["apPassword"] = s.system.apPassword;
        y["adminPassword"] = s.system.adminPassword;
    } else {
        y["passwordSet"] = s.system.password[0] != '\0';
    }
}

bool settingsFromJson(JsonObjectConst o, AppSettings& out) {
    AppSettings s = out;
    bool ok = true;
    JsonObjectConst m = o["measurement"];
    if (!m.isNull()) {
        ok &= getNum(m, "filterStrength", s.measurement.filterStrength);
        ok &= getEnum(m, "fusion", s.measurement.fusion);
        ok &= getNum(m, "precisionWindowSec", s.measurement.precisionWindowSec);
        ok &= getNum(m, "holdTimeSec", s.measurement.holdTimeSec);
        ok &= getNum(m, "hemisphere", s.measurement.hemisphere);
        ok &= getEnum(m, "zeroKind", s.measurement.zeroKind);
        ok &= getBool(m, "reference3D", s.measurement.reference3D);
        ok &= getNum(m, "customAngleDeg", s.measurement.customAngleDeg);
        ok &= getNum(m, "tempCoeffAssumedDpsPerK", s.measurement.tempCoeffAssumedDpsPerK);
        ok &= getBool(m, "onlineBiasTracking", s.measurement.onlineBiasTracking);
        ok &= getEnum(m, "startupGyroCal", s.measurement.startupGyroCal);
        ok &= getEnum(m, "lastMode", s.measurement.lastMode);
    }
    JsonObjectConst d = o["display"];
    if (!d.isNull()) {
        ok &= getNum(d, "brightness", s.display.brightness);
        ok &= getNum(d, "dimAfterSec", s.display.dimAfterSec);
        ok &= getNum(d, "offAfterSec", s.display.offAfterSec);
        ok &= getNum(d, "rotation", s.display.rotation);
        ok &= getNum(d, "theme", s.display.theme);
        ok &= getNum(d, "colorMode", s.display.colorMode);
        ok &= getNum(d, "largeFont", s.display.largeFont);
        ok &= getNum(d, "decimals", s.display.decimals);
        ok &= getBool(d, "touchLockAuto", s.display.touchLockAuto);
        ok &= getBool(d, "showHistory", s.display.showHistory);
        ok &= getNum(d, "historyWindowSec", s.display.historyWindowSec);
    }
    JsonObjectConst y = o["system"];
    if (!y.isNull()) {
        ok &= getEnum(y, "wifiMode", s.system.wifiMode);
        ok &= getStr(y, "ssid", s.system.ssid, sizeof(s.system.ssid));
        ok &= getStr(y, "password", s.system.password, sizeof(s.system.password));
        ok &= getStr(y, "apPassword", s.system.apPassword, sizeof(s.system.apPassword));
        ok &= getStr(y, "adminPassword", s.system.adminPassword, sizeof(s.system.adminPassword));
        // credentials must never become empty/short through the API (WPA2 + auth need >= 8)
        if (!y["apPassword"].isNull() && std::strlen(s.system.apPassword) < 8) ok = false;
        if (!y["adminPassword"].isNull() && std::strlen(s.system.adminPassword) < 8) ok = false;
        ok &= getStr(y, "hostname", s.system.hostname, sizeof(s.system.hostname));
        ok &= getBool(y, "mdns", s.system.mdns);
        ok &= getNum(y, "sleepAfterMin", s.system.sleepAfterMin);
        ok &= getEnum(y, "bootButton", s.system.bootButton);
        JsonVariantConst ax = y["axisMap"];
        if (!ax.isNull()) {
            AxisMap am;
            if (!ax.is<const char*>() || !am.parse(ax.as<const char*>())) ok = false;
            else s.system.axisMap = am;
        }
        ok &= getNum(y, "logLevel", s.system.logLevel);
        ok &= getNum(y, "batteryDividerFactor", s.system.batteryDividerFactor);
        ok &= getNum(y, "lowBatteryV", s.system.lowBatteryV);
        ok &= getNum(y, "criticalBatteryV", s.system.criticalBatteryV);
        ok &= getNum(y, "activeProfile", s.system.activeProfile);
        ok &= getNum(y, "liveRateHz", s.system.liveRateHz);
    }
    if (!ok) return false;
    validateSettings(s);
    out = s;
    return true;
}

// ---------------------------------------------------------------------------
void profileToJson(const Profile& p, JsonObject o) {
    o["used"] = p.used;
    o["name"] = p.name;
    JsonObject m = o["mount"].to<JsonObject>();
    vecTo(m["axis"].to<JsonArray>(), p.mount.drillAxisD);
    m["method"] = static_cast<int>(p.mount.method);
    m["qualityDeg"] = p.mount.qualityDeg;
    m["angleCorrectionDeg"] = p.mount.angleCorrectionDeg;
    o["targetAngleDeg"] = p.targetAngleDeg;
    o["toleranceDeg"] = p.toleranceDeg;
    o["scale"] = static_cast<int>(p.scale);
    o["invertMarker"] = p.invertMarker;
    o["showNumbers"] = p.showNumbers;
    o["mountCalTime"] = p.mountCalTime;
}

bool profileFromJson(JsonObjectConst o, Profile& out) {
    Profile p = out;
    bool ok = true;
    ok &= getBool(o, "used", p.used);
    ok &= getStr(o, "name", p.name, sizeof(p.name));
    JsonObjectConst m = o["mount"];
    if (!m.isNull()) {
        if (!m["axis"].isNull() && !vecFrom(m["axis"], p.mount.drillAxisD)) ok = false;
        ok &= getEnum(m, "method", p.mount.method);
        ok &= getNum(m, "qualityDeg", p.mount.qualityDeg);
        ok &= getNum(m, "angleCorrectionDeg", p.mount.angleCorrectionDeg);
    }
    ok &= getNum(o, "targetAngleDeg", p.targetAngleDeg);
    ok &= getNum(o, "toleranceDeg", p.toleranceDeg);
    ok &= getEnum(o, "scale", p.scale);
    ok &= getBool(o, "invertMarker", p.invertMarker);
    ok &= getBool(o, "showNumbers", p.showNumbers);
    ok &= getNum(o, "mountCalTime", p.mountCalTime);
    if (!ok) return false;
    validateProfile(p);
    out = p;
    return true;
}

void profilesToJson(const ProfileSet& ps, JsonObject o) {
    o["version"] = ps.version;
    JsonArray a = o["profiles"].to<JsonArray>();
    for (const Profile& p : ps.profiles) profileToJson(p, a.add<JsonObject>());
}

bool profilesFromJson(JsonObjectConst o, ProfileSet& out) {
    JsonArrayConst a = o["profiles"];
    if (a.isNull()) return false;
    ProfileSet ps;
    int i = 0;
    for (JsonObjectConst po : a) {
        if (i >= kMaxProfiles) break;
        if (!profileFromJson(po, ps.profiles[i])) return false;
        ++i;
    }
    out = ps;
    return true;
}

// ---------------------------------------------------------------------------
void calibrationToJson(const SensorCalibration& c, JsonObject o) {
    o["version"] = kCalibrationVersion;
    JsonArray g = o["gyro"].to<JsonArray>();
    for (int i = 0; i < c.gyroPointCount(); ++i) {
        const GyroCalPoint& p = c.gyroPoint(i);
        JsonObject e = g.add<JsonObject>();
        if (std::isfinite(p.tempC)) e["tempC"] = p.tempC;
        vecTo(e["bias"].to<JsonArray>(), p.biasDps);
        vecTo(e["noise"].to<JsonArray>(), p.noiseDps);
        e["samples"] = p.samples;
    }
    JsonArray a = o["accel"].to<JsonArray>();
    for (int i = 0; i < c.accelPointCount(); ++i) {
        const AccelCalPoint& p = c.accelPoint(i);
        JsonObject e = a.add<JsonObject>();
        if (std::isfinite(p.tempC)) e["tempC"] = p.tempC;
        JsonArray m = e["matrix"].to<JsonArray>();
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 3; ++k) m.add(p.matrix.m[r][k]);
        vecTo(e["offset"].to<JsonArray>(), p.offset);
        e["residualMg"] = p.residualMg;
        e["extended"] = p.extended;
    }
}

bool calibrationFromJson(JsonObjectConst o, SensorCalibration& out) {
    if (o["version"].as<int>() != kCalibrationVersion) return false;  // no migration needed yet
    GyroCalPoint gp[kMaxGyroCalPoints];
    AccelCalPoint ap[kMaxAccelCalPoints];
    int ng = 0, na = 0;
    for (JsonObjectConst e : o["gyro"].as<JsonArrayConst>()) {
        if (ng >= kMaxGyroCalPoints) break;
        GyroCalPoint& p = gp[ng];
        p.tempC = e["tempC"].isNull() ? NAN : e["tempC"].as<float>();
        if (!vecFrom(e["bias"], p.biasDps) || !vecFrom(e["noise"], p.noiseDps)) return false;
        p.samples = e["samples"].as<uint32_t>();
        ++ng;
    }
    for (JsonObjectConst e : o["accel"].as<JsonArrayConst>()) {
        if (na >= kMaxAccelCalPoints) break;
        AccelCalPoint& p = ap[na];
        p.tempC = e["tempC"].isNull() ? NAN : e["tempC"].as<float>();
        JsonArrayConst m = e["matrix"];
        if (m.size() != 9) return false;
        for (int i = 0; i < 9; ++i) p.matrix.m[i / 3][i % 3] = m[i].as<float>();
        if (!vecFrom(e["offset"], p.offset)) return false;
        p.residualMg = e["residualMg"] | -1.0f;
        p.extended = e["extended"] | false;
        ++na;
    }
    SensorCalibration c;
    c.setGyroPoints(gp, ng);
    c.setAccelPoints(ap, na);
    if (!c.validate()) return false;
    out = c;
    return true;
}

}  // namespace cfgjson
