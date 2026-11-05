// SPDX-License-Identifier: MIT
#include "Settings.h"

#include <cstring>

namespace sda {

const char* appModeName(AppMode m) {
    switch (m) {
        case AppMode::Drill: return "Bohrassistent";
        case AppMode::Level: return "Wasserwaage";
        case AppMode::Angle: return "Winkelmesser";
        case AppMode::Zero: return "Nullpunkt";
        case AppMode::Reference: return "Referenzfläche";
        case AppMode::Precision: return "Präzision";
        default: return "?";
    }
}

const char* appModeShort(AppMode m) {
    switch (m) {
        case AppMode::Drill: return "BOHREN";
        case AppMode::Level: return "WAAGE";
        case AppMode::Angle: return "WINKEL";
        case AppMode::Zero: return "NULL";
        case AppMode::Reference: return "REFERENZ";
        case AppMode::Precision: return "PRÄZISION";
        default: return "?";
    }
}

const char* buttonActionName(ButtonAction a) {
    switch (a) {
        case ButtonAction::Hold: return "Hold";
        case ButtonAction::Zero: return "Nullpunkt";
        case ButtonAction::NextMode: return "Nächster Modus";
        case ButtonAction::TouchLock: return "Touchsperre";
        default: return "?";
    }
}

const char* wifiModeName(WifiMode m) {
    switch (m) {
        case WifiMode::Off: return "Aus";
        case WifiMode::AccessPoint: return "Access Point";
        case WifiMode::Client: return "Client";
    }
    return "?";
}

void copyString(char* dst, size_t n, const char* src) {
    if (!dst || n == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    std::strncpy(dst, src, n - 1);
    dst[n - 1] = '\0';
}

void defaultSettings(AppSettings& s) { s = AppSettings{}; }

void defaultProfiles(ProfileSet& ps) {
    ps = ProfileSet{};
    static const char* names[] = {"Bohrmaschine 1", "Bohrmaschine 2", "Akkuschrauber",
                                  "Freie Wasserwaage", "Benutzerdefiniert"};
    for (int i = 0; i < 5; ++i) {
        Profile& p = ps.profiles[i];
        p.used = true;
        copyString(p.name, sizeof(p.name), names[i]);
        p.mount = MountModel{};
        p.mount.drillAxisD = mountPresetAxis(MountPreset::ScreenFacingUser);
        p.mount.method = MountModel::Method::Default;
        p.targetAngleDeg = 90.0f;
        p.toleranceDeg = 0.5f;
        p.scale = ScaleMode::Auto;
    }
    // free level: measurement axis = device X (long edge), coarser tolerance
    ps.profiles[3].mount.drillAxisD = mountPresetAxis(MountPreset::RightEdgeForward);
    ps.profiles[3].targetAngleDeg = 0.0f;
    ps.profiles[3].toleranceDeg = 0.2f;
    ps.profiles[2].toleranceDeg = 1.0f;
}

namespace {

template <typename T>
int fixRange(T& v, T lo, T hi, T def) {
    if (!(v >= lo && v <= hi)) {  // also catches NaN for floats
        v = def;
        return 1;
    }
    return 0;
}

int fixString(char* s, size_t n, const char* def, bool allowEmpty = true) {
    // ensure termination and printable content
    s[n - 1] = '\0';
    bool ok = true;
    for (size_t i = 0; s[i]; ++i) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        if (c < 0x20 || c == 0x7F) { ok = false; break; }
    }
    if (!allowEmpty && s[0] == '\0') ok = false;
    if (!ok) {
        copyString(s, n, def);
        return 1;
    }
    return 0;
}

int fixHostname(char* s, size_t n) {
    s[n - 1] = '\0';
    bool ok = s[0] != '\0';
    for (size_t i = 0; s[i] && ok; ++i) {
        const char c = s[i];
        const bool valid = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                           (c >= '0' && c <= '9') || (c == '-' && i > 0);
        if (!valid) ok = false;
    }
    if (!ok) {
        copyString(s, n, "smartdrill");
        return 1;
    }
    return 0;
}

}  // namespace

int validateSettings(AppSettings& s) {
    const AppSettings d;
    int n = 0;
    MeasurementSettings& m = s.measurement;
    n += fixRange<uint8_t>(m.filterStrength, 0, 4, d.measurement.filterStrength);
    n += fixRange<uint8_t>(reinterpret_cast<uint8_t&>(m.fusion), 0, 2, 0);
    n += fixRange(m.precisionWindowSec, 0.5f, 10.0f, d.measurement.precisionWindowSec);
    n += fixRange(m.holdTimeSec, 0.0f, 3.0f, d.measurement.holdTimeSec);
    n += fixRange<int8_t>(m.hemisphere, -1, 1, 0);
    n += fixRange<uint8_t>(reinterpret_cast<uint8_t&>(m.zeroKind), 0, 1, 0);
    n += fixRange(m.customAngleDeg, 0.0f, 90.0f, d.measurement.customAngleDeg);
    n += fixRange(m.tempCoeffAssumedDpsPerK, 0.0f, 0.5f, d.measurement.tempCoeffAssumedDpsPerK);
    n += fixRange<uint8_t>(reinterpret_cast<uint8_t&>(m.startupGyroCal), 0, 2, 1);
    n += fixRange<uint8_t>(reinterpret_cast<uint8_t&>(m.lastMode), 0, uint8_t(AppMode::Count) - 1, 0);

    DisplaySettings& ds = s.display;
    n += fixRange<uint8_t>(ds.brightness, 5, 100, d.display.brightness);
    n += fixRange<uint16_t>(ds.dimAfterSec, 0, 3600, d.display.dimAfterSec);
    n += fixRange<uint16_t>(ds.offAfterSec, 0, 7200, d.display.offAfterSec);
    if (ds.offAfterSec != 0 && ds.dimAfterSec != 0 && ds.offAfterSec < ds.dimAfterSec) {
        ds.offAfterSec = ds.dimAfterSec;
        ++n;
    }
    n += fixRange<uint8_t>(ds.rotation, 0, 3, 0);
    n += fixRange<uint8_t>(ds.theme, 0, 1, 0);
    n += fixRange<uint8_t>(ds.colorMode, 0, 1, 0);
    n += fixRange<uint8_t>(ds.largeFont, 0, 1, 0);
    n += fixRange<uint8_t>(ds.decimals, 1, 2, 1);
    if (ds.historyWindowSec != 5 && ds.historyWindowSec != 15 && ds.historyWindowSec != 30) {
        ds.historyWindowSec = 15;
        ++n;
    }

    SystemSettings& ss = s.system;
    n += fixRange<uint8_t>(reinterpret_cast<uint8_t&>(ss.wifiMode), 0, 2, 0);
    n += fixString(ss.ssid, sizeof(ss.ssid), "");
    n += fixString(ss.password, sizeof(ss.password), "");
    n += fixString(ss.apPassword, sizeof(ss.apPassword), "");
    n += fixString(ss.adminPassword, sizeof(ss.adminPassword), "");
    // WPA2 requires >= 8 characters; empty means "generate on boot"
    if (ss.apPassword[0] && std::strlen(ss.apPassword) < 8) { ss.apPassword[0] = '\0'; ++n; }
    if (ss.adminPassword[0] && std::strlen(ss.adminPassword) < 8) { ss.adminPassword[0] = '\0'; ++n; }
    if (ss.wifiMode == WifiMode::Client && ss.ssid[0] == '\0') { ss.wifiMode = WifiMode::AccessPoint; ++n; }
    n += fixHostname(ss.hostname, sizeof(ss.hostname));
    n += fixRange<uint16_t>(ss.sleepAfterMin, 0, 240, d.system.sleepAfterMin);
    n += fixRange<uint8_t>(reinterpret_cast<uint8_t&>(ss.bootButton), 0, uint8_t(ButtonAction::Count) - 1, 0);
    if (!ss.axisMap.isValid()) { ss.axisMap = AxisMap{}; ++n; }
    n += fixRange<uint8_t>(ss.logLevel, 0, 3, 2);
    n += fixRange(ss.batteryDividerFactor, 1.0f, 6.0f, d.system.batteryDividerFactor);
    n += fixRange(ss.lowBatteryV, 3.0f, 4.0f, d.system.lowBatteryV);
    n += fixRange(ss.criticalBatteryV, 2.9f, 3.9f, d.system.criticalBatteryV);
    if (ss.criticalBatteryV >= ss.lowBatteryV) {
        ss.criticalBatteryV = d.system.criticalBatteryV;
        ss.lowBatteryV = d.system.lowBatteryV;
        ++n;
    }
    n += fixRange<uint8_t>(ss.activeProfile, 0, kMaxProfiles - 1, 0);
    n += fixRange<uint8_t>(ss.liveRateHz, 1, 25, 10);
    n += fixRange<uint8_t>(ss.chargeScreen, 0, 2, 2);
    n += fixRange<uint8_t>(ss.performance, 0, 1, 0);
    n += fixRange<uint8_t>(ss.apIdleOffMin, 0, 120, 10);
    s.version = kSettingsVersion;
    return n;
}

int validateProfile(Profile& p) {
    int n = 0;
    n += fixString(p.name, sizeof(p.name), "Profil", false);
    if (!p.mount.drillAxisD.isFinite() || p.mount.drillAxisD.norm() < 0.5f) {
        p.mount.drillAxisD = mountPresetAxis(MountPreset::ScreenFacingUser);
        p.mount.method = MountModel::Method::Default;
        ++n;
    } else {
        const float len = p.mount.drillAxisD.norm();
        if (std::fabs(len - 1.0f) > 1e-3f) {
            p.mount.drillAxisD = p.mount.drillAxisD / len;
            ++n;
        }
    }
    n += fixRange<uint8_t>(reinterpret_cast<uint8_t&>(p.mount.method), 0, 3, 0);
    n += fixRange(p.mount.angleCorrectionDeg, -10.0f, 10.0f, 0.0f);
    if (!std::isfinite(p.mount.qualityDeg)) { p.mount.qualityDeg = -1.0f; ++n; }
    n += fixRange(p.targetAngleDeg, 0.0f, 90.0f, 90.0f);
    n += fixRange<uint8_t>(reinterpret_cast<uint8_t&>(p.angleRef), 0, uint8_t(AngleReference::Count) - 1, 0);
    n += fixRange(p.toleranceDeg, 0.05f, 10.0f, 0.5f);
    n += fixRange<uint8_t>(reinterpret_cast<uint8_t&>(p.scale), 0, 4, 4);
    return n;
}

int validateProfiles(ProfileSet& ps, uint8_t& active) {
    int n = 0;
    int used = 0;
    for (auto& p : ps.profiles)
        if (p.used) {
            n += validateProfile(p);
            ++used;
        }
    if (used == 0) {
        defaultProfiles(ps);
        n += 1;
    }
    if (active >= kMaxProfiles || !ps.profiles[active].used) {
        for (int i = 0; i < kMaxProfiles; ++i)
            if (ps.profiles[i].used) { active = uint8_t(i); break; }
        ++n;
    }
    ps.version = kProfilesVersion;
    return n;
}

float quickAngle(int idx, const MeasurementSettings& m) {
    static const float angles[] = {0, 15, 30, 45, 60, 90};
    if (idx >= 0 && idx < 6) return angles[idx];
    return m.customAngleDeg;
}

FilterParams filterParams(uint8_t strength) {
    static const FilterParams table[] = {
        {4.0f, 0.60f, 0.05f},  // 0: schnell
        {2.0f, 0.40f, 0.10f},  // 1
        {1.0f, 0.25f, 0.20f},  // 2: standard
        {0.5f, 0.12f, 0.50f},  // 3
        {0.25f, 0.06f, 1.00f}, // 4: sehr ruhig
    };
    return table[strength > 4 ? 4 : strength];
}

}  // namespace sda
