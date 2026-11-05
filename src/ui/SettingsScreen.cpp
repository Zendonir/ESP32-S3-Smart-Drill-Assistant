// SPDX-License-Identifier: MIT
// Settings menu tree (requirement 6.5). Every entry reads and writes the real
// configuration (SettingsManager / ProfileManager); there are no dummy entries.
#include <Arduino.h>
#include <esp_ota_ops.h>

#include <cstdio>

#include "../system/OtaGuard.h"
#include "../system/SystemTask.h"
#include "MeasurementPipeline.h"
#include "Theme.h"
#include "UIManager.h"
#include "Version.h"

using namespace sda;

namespace {
std::string fmt(const char* f, double v) {
    char b[48];
    snprintf(b, sizeof(b), f, v);
    return b;
}

/// All 24 proper signed axis permutations, as "+X+Y+Z" strings.
std::vector<std::string> axisMapOptions() {
    std::vector<std::string> out;
    const int perms[6][3] = {{1, 2, 3}, {1, 3, 2}, {2, 1, 3}, {2, 3, 1}, {3, 1, 2}, {3, 2, 1}};
    for (const auto& p : perms)
        for (int signs = 0; signs < 8; ++signs) {
            AxisMap m;
            for (int i = 0; i < 3; ++i) m.code[i] = static_cast<int8_t>((signs & (1 << i)) ? -p[i] : p[i]);
            if (!m.isValid()) continue;
            char b[8];
            m.format(b);
            out.emplace_back(b);
        }
    return out;
}

std::string joinOptions(const std::vector<std::string>& v) {
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += "\n";
        s += v[i];
    }
    return s;
}
}  // namespace

void UIManager::openMenuRoot() {
    view_ = View::Menu;
    menu_.open("Einstellungen",
               {
                   MenuItem::submenu("Messung", [this] { menuMeasurement(); }),
                   MenuItem::submenu("Kalibrierung", [this] { menuCalibration(); }),
                   MenuItem::submenu("Anzeige", [this] { menuDisplay(); }),
                   MenuItem::submenu("Profile", [this] { menuProfiles(); }),
                   MenuItem::submenu("System", [this] { menuSystem(); }),
               },
               [this] { backToMain(); });
}

// ---------------------------------------------------------------------------
void UIManager::menuMeasurement() {
    SettingsManager& sm = app_->settings;
    ProfileManager& pm = app_->profiles;
    auto active = [this] { return app_->settings.get().system.activeProfile; };
    std::string modes;
    for (int i = 0; i < static_cast<int>(AppMode::Count); ++i) {
        if (i) modes += "\n";
        modes += appModeName(static_cast<AppMode>(i));
    }
    menu_.open(
        "Messung",
        {
            MenuItem::choice("Modus", modes, [this] { return static_cast<int>(main_.mode()); },
                             [this](int i) { setMode(static_cast<AppMode>(i)); }),
            MenuItem::number("Sollwinkel", 0, 90, 0.5f, 1, "°", [&pm, active] { return pm.get(active()).targetAngleDeg; },
                             [&pm, active](float v) { pm.update(active(), [v](Profile& p) { p.targetAngleDeg = v; }); }),
            MenuItem::choice("Winkelbezug",
                             "zur Horizontalen\nzur Vertikalen\nNullpunkt + oben\nNullpunkt + unten\n"
                             "Nullpunkt + links\nNullpunkt + rechts",
                             [&pm, active] { return static_cast<int>(pm.get(active()).angleRef); },
                             [&pm, active](int i) {
                                 pm.update(active(), [i](Profile& p) { p.angleRef = static_cast<AngleReference>(i); });
                             }),
            MenuItem::number("Toleranz", 0.05f, 10, 0.05f, 2, "°", [&pm, active] { return pm.get(active()).toleranceDeg; },
                             [&pm, active](float v) { pm.update(active(), [v](Profile& p) { p.toleranceDeg = v; }); }),
            MenuItem::choice("Skala", "Grob ±20°\nNormal ±10°\nFein ±5°\nPräzision ±2°\nAutomatisch",
                             [&pm, active] { return static_cast<int>(pm.get(active()).scale); },
                             [&pm, active](int i) { pm.update(active(), [i](Profile& p) { p.scale = static_cast<ScaleMode>(i); }); }),
            MenuItem::choice("Punkt", "Achsposition\nLibelle (invers)",
                             [&pm, active] { return pm.get(active()).invertMarker ? 1 : 0; },
                             [&pm, active](int i) { pm.update(active(), [i](Profile& p) { p.invertMarker = i == 1; }); }),
            MenuItem::choice("Filterstärke", "0 schnell\n1\n2 Standard\n3\n4 sehr ruhig",
                             [&sm] { return int(sm.get().measurement.filterStrength); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.measurement.filterStrength = uint8_t(i); }); }),
            MenuItem::choice("Fusion", "Mahony\nKomplementär\nMadgwick", [&sm] { return int(sm.get().measurement.fusion); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.measurement.fusion = FusionType(i); }); }),
            MenuItem::number("Haltezeit", 0, 3, 0.1f, 1, " s", [&sm] { return sm.get().measurement.holdTimeSec; },
                             [&sm](float v) { sm.update([v](AppSettings& s) { s.measurement.holdTimeSec = v; }); }),
            MenuItem::number("Präz.-Dauer", 0.5f, 10, 0.5f, 1, " s", [&sm] { return sm.get().measurement.precisionWindowSec; },
                             [&sm](float v) { sm.update([v](AppSettings& s) { s.measurement.precisionWindowSec = v; }); }),
            MenuItem::choice("Richtung", "Automatisch\nAbwärts\nAufwärts",
                             [&sm] {
                                 const int h = sm.get().measurement.hemisphere;
                                 return h == 0 ? 0 : (h < 0 ? 1 : 2);
                             },
                             [&sm](int i) {
                                 sm.update([i](AppSettings& s) { s.measurement.hemisphere = int8_t(i == 0 ? 0 : (i == 1 ? -1 : 1)); });
                             }),
            MenuItem::choice("Nullpunkt", "Neigung (driftfrei)\n3D (Gyro, driftet)",
                             [&sm] { return int(sm.get().measurement.zeroKind); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.measurement.zeroKind = ZeroKind(i); }); }),
            MenuItem::choice("Ref.-Fläche", "Nur Neigung\n3D (Gyro)", [&sm] { return sm.get().measurement.reference3D ? 1 : 0; },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.measurement.reference3D = i == 1; }); }),
            MenuItem::number("Eigener Winkel", 0, 90, 0.5f, 1, "°", [&sm] { return sm.get().measurement.customAngleDeg; },
                             [&sm](float v) { sm.update([v](AppSettings& s) { s.measurement.customAngleDeg = v; }); }),
            MenuItem::toggle("Bias-Nachführ.", [&sm] { return sm.get().measurement.onlineBiasTracking; },
                             [&sm](bool b) { sm.update([b](AppSettings& s) { s.measurement.onlineBiasTracking = b; }); }),
            MenuItem::choice("Gyro-Kal. Start", "Nie\nWenn ruhig\nImmer", [&sm] { return int(sm.get().measurement.startupGyroCal); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.measurement.startupGyroCal = StartupGyroCal(i); }); }),
        },
        [this] { openMenuRoot(); });
}

// ---------------------------------------------------------------------------
void UIManager::menuCalibration() {
    view_ = View::Menu;
    auto active = [this] { return app_->settings.get().system.activeProfile; };
    ProfileManager& pm = app_->profiles;
    menu_.open(
        "Kalibrierung",
        {
            MenuItem::act("Achsen erkennen", [this] { openCalibration(CalibrationScreen::Kind::AxisDetect); },
                          [this] { return std::string(app_->settings.get().system.axisMapVerified ? "ok" : "!"); }),
            MenuItem::act("Gyroskop", [this] { openCalibration(CalibrationScreen::Kind::Gyro); }),
            MenuItem::act("Sensor 6 Lagen", [this] { openCalibration(CalibrationScreen::Kind::Accel); }),
            MenuItem::act("6 Lagen erweitert", [this] { openCalibration(CalibrationScreen::Kind::AccelExtended); }),
            MenuItem::act("Montage V-Nut", [this] { openCalibration(CalibrationScreen::Kind::MountRoll); }),
            MenuItem::act("Montage Ständer", [this] { openCalibration(CalibrationScreen::Kind::MountVertical); }),
            MenuItem::choice("Montage grob", "Display zum Nutzer\nOberkante vorn\nRechte Kante vorn\nDisplay zum Bohrer",
                             [&pm, active] {
                                 const Vec3 a = pm.get(active()).mount.drillAxisD;
                                 for (int i = 0; i < 4; ++i)
                                     if (a.dot(mountPresetAxis(MountPreset(i))) > 0.98f) return i;
                                 return 0;
                             },
                             [this, &pm, active](int i) {
                                 pm.update(active(), [i](Profile& p) {
                                     p.mount.drillAxisD = mountPresetAxis(MountPreset(i));
                                     p.mount.method = MountModel::Method::AxisPreset;
                                     p.mount.qualityDeg = -1;
                                 });
                                 app_->meas.setMount(pm.get(active()).mount);
                             }),
            MenuItem::act("Nullpunkt setzen", [this] {
                app_->meas.captureTiltZero();
                showMessage("Neigungs-Nullpunkt gesetzt");
            }),
            MenuItem::submenu("Daten anzeigen", [this] { menuCalibrationData(); }),
            MenuItem::confirm("Kalibr. löschen", "Sensorkalibrierung löschen?", [this] {
                app_->calStore.clear();
                app_->meas.setCalibration(SensorCalibration());
                showMessage("Kalibrierung gelöscht");
            }),
        },
        [this] { openMenuRoot(); });
}

void UIManager::menuCalibrationData() {
    auto snap = [this] {
        OrientationSnapshot s;
        app_->meas.snapshot(s);
        return s;
    };
    auto cal = [this] {
        SensorCalibration c;
        app_->meas.currentCalibration(c);
        return c;
    };
    auto prof = [this] { return app_->profiles.get(app_->settings.get().system.activeProfile); };
    menu_.open(
        "Kalibrierdaten",
        {
            MenuItem::info("Gyro-Punkte", [cal] { return std::to_string(cal().gyroPointCount()); }),
            MenuItem::info("Gyro-Offset", [cal] {
                const SensorCalibration c = cal();
                if (!c.hasGyro()) return std::string("-");
                const Vec3 b = c.gyroPoint(c.gyroPointCount() - 1).biasDps;
                char t[48];
                snprintf(t, sizeof(t), "%.2f %.2f %.2f", double(b.x), double(b.y), double(b.z));
                return std::string(t);
            }),
            MenuItem::info("Gyro-TK", [cal] { return std::string(cal().gyroTempCompActive() ? "aktiv" : "inaktiv"); }),
            MenuItem::info("Accel-Punkte", [cal] { return std::to_string(cal().accelPointCount()); }),
            MenuItem::info("Accel-Rest", [cal] {
                const SensorCalibration c = cal();
                if (!c.hasAccel()) return std::string("-");
                return fmt("%.1f mg", double(c.accelPoint(c.accelPointCount() - 1).residualMg));
            }),
            MenuItem::info("Accel-TK", [cal] { return std::string(cal().accelTempCompActive() ? "aktiv" : "inaktiv"); }),
            MenuItem::info("Temperatur", [snap] { return fmt("%.1f °C", double(snap().temperatureC)); }),
            MenuItem::info("Online-Bias", [snap] { return fmt("%.3f °/s", double(snap().onlineBiasDps.norm())); }),
            MenuItem::info("Montage", [prof] {
                static const char* m[] = {"Standard", "Voreinst.", "V-Nut", "Ständer"};
                return std::string(m[int(prof().mount.method) & 3]);
            }),
            MenuItem::info("Achse", [prof] {
                const Vec3 a = prof().mount.drillAxisD;
                char t[48];
                snprintf(t, sizeof(t), "%.3f %.3f %.3f", double(a.x), double(a.y), double(a.z));
                return std::string(t);
            }),
            MenuItem::info("Montage-Rest", [prof] {
                const float q = prof().mount.qualityDeg;
                return q < 0 ? std::string("unbekannt") : fmt("%.2f°", double(q));
            }),
        },
        [this] { menuCalibration(); });
}

// ---------------------------------------------------------------------------
void UIManager::menuDisplay() {
    SettingsManager& sm = app_->settings;
    menu_.open(
        "Anzeige",
        {
            MenuItem::number("Helligkeit", 5, 100, 5, 0, " %", [&sm] { return float(sm.get().display.brightness); },
                             [&sm](float v) { sm.update([v](AppSettings& s) { s.display.brightness = uint8_t(v); }); }),
            MenuItem::choice("Design", "Dunkel\nHoher Kontrast", [&sm] { return int(sm.get().display.theme); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.display.theme = uint8_t(i); }); }),
            MenuItem::choice("Farbmodus", "Standard\nFarbenblind", [&sm] { return int(sm.get().display.colorMode); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.display.colorMode = uint8_t(i); }); }),
            MenuItem::choice("Rotation", "0°\n90°\n180°\n270°", [&sm] { return int(sm.get().display.rotation); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.display.rotation = uint8_t(i); }); }),
            MenuItem::choice("Schrift", "Normal\nGroß", [&sm] { return int(sm.get().display.largeFont); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.display.largeFont = uint8_t(i); }); }),
            MenuItem::choice("Nachkomma", "1 (0,1°)\n2 (0,01°)", [&sm] { return int(sm.get().display.decimals) - 1; },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.display.decimals = uint8_t(i + 1); }); }),
            MenuItem::number("Dimmen nach", 0, 3600, 10, 0, " s", [&sm] { return float(sm.get().display.dimAfterSec); },
                             [&sm](float v) { sm.update([v](AppSettings& s) { s.display.dimAfterSec = uint16_t(v); }); }),
            MenuItem::number("Aus nach", 0, 7200, 30, 0, " s", [&sm] { return float(sm.get().display.offAfterSec); },
                             [&sm](float v) { sm.update([v](AppSettings& s) { s.display.offAfterSec = uint16_t(v); }); }),
            MenuItem::toggle("Sperre b. Vibr.", [&sm] { return sm.get().display.touchLockAuto; },
                             [&sm](bool b) { sm.update([b](AppSettings& s) { s.display.touchLockAuto = b; }); }),
            MenuItem::toggle("Verlauf", [&sm] { return sm.get().display.showHistory; },
                             [&sm](bool b) { sm.update([b](AppSettings& s) { s.display.showHistory = b; }); }),
            MenuItem::choice("Verlaufsfenster", "5 s\n15 s\n30 s",
                             [&sm] {
                                 const int w = sm.get().display.historyWindowSec;
                                 return w == 5 ? 0 : (w == 15 ? 1 : 2);
                             },
                             [&sm](int i) {
                                 static const uint8_t v[] = {5, 15, 30};
                                 sm.update([i](AppSettings& s) { s.display.historyWindowSec = v[i % 3]; });
                             }),
        },
        [this] { openMenuRoot(); });
}

// ---------------------------------------------------------------------------
void UIManager::menuWifi() {
    SettingsManager& sm = app_->settings;
    auto net = [this] {
        NetStatus n;
        app_->net.read(n);
        return n;
    };
    menu_.open(
        "WLAN",
        {
            MenuItem::choice("Modus", "Aus\nAccess Point\nClient", [&sm] { return int(sm.get().system.wifiMode); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.system.wifiMode = WifiMode(i); }); }),
            MenuItem::info("Status", [net] {
                const NetStatus n = net();
                return std::string(n.connected ? (n.apFallback ? "AP (Fallback)" : "verbunden") : "getrennt");
            }),
            MenuItem::info("IP", [net] { return std::string(net().ip[0] ? net().ip : "-"); }),
            MenuItem::info("SSID", [net] { return std::string(net().ssid[0] ? net().ssid : "-"); }),
            MenuItem::info("AP-Passwort", [&sm] { return std::string(sm.get().system.apPassword); }),
            MenuItem::info("Web-Login", [] { return std::string("admin"); }),
            MenuItem::info("Web-Passwort", [&sm] { return std::string(sm.get().system.adminPassword); }),
            MenuItem::info("Hostname", [&sm] { return std::string(sm.get().system.hostname) + ".local"; }),
            MenuItem::confirm("Zugang löschen", "WLAN-Zugangsdaten löschen?", [&sm] {
                sm.update([](AppSettings& s) {
                    s.system.ssid[0] = '\0';
                    s.system.password[0] = '\0';
                    if (s.system.wifiMode == WifiMode::Client) s.system.wifiMode = WifiMode::AccessPoint;
                });
            }),
        },
        [this] { menuSystem(); });
}

void UIManager::menuSystem() {
    view_ = View::Menu;
    SettingsManager& sm = app_->settings;
    static std::vector<std::string> axes = axisMapOptions();
    menu_.open(
        "System",
        {
            MenuItem::submenu("WLAN", [this] { menuWifi(); }),
            MenuItem::info("OTA", [this] {
                NetStatus n;
                app_->net.read(n);
                return n.connected ? std::string("http://") + n.ip + "/" : std::string("WLAN aktivieren");
            }),
            MenuItem::act("Akku", [this] { menuBattery(); }, [this] {
                PowerStatus p;
                app_->power.read(p);
                char b[32];
                if (p.batteryPercent < 0) snprintf(b, sizeof(b), "--");
                else snprintf(b, sizeof(b), "ca. %d %%%s", p.batteryPercent,
                              p.charge == ChargeState::Charging ? " " LV_SYMBOL_CHARGE : "");
                return std::string(b);
            }),
            MenuItem::info("Firmware", [] { return std::string(SDA_FW_VERSION_STR " (" SDA_FW_COMMIT ")"); }),
            MenuItem::info("Partition", [] { return std::string(ota_guard::runningPartition()); }),
            MenuItem::act("Sensorstatus", [this] { openDiagnostics(); },
                          [this] { return std::string(app_->imu.isHealthy() ? "ok" : "FEHLER"); }),
            MenuItem::act("Diagnose", [this] { openDiagnostics(); }),
            MenuItem::act("Farbtest", [this] { showColorTest(); }),
            MenuItem::choice("Ladeanzeige", "Aus\nNur wenn an\nAuch wenn aus",
                             [&sm] { return int(sm.get().system.chargeScreen); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.system.chargeScreen = uint8_t(i); }); }),
            MenuItem::choice("Einschalten", "BOOT kurz drücken\nBOOT 1 s halten",
                             [&sm] { return sm.get().system.powerOnHold ? 1 : 0; },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.system.powerOnHold = i == 1; }); }),
            MenuItem::choice("BOOT kurz", "Hold\nNullpunkt\nNächster Modus\nTouchsperre",
                             [&sm] { return int(sm.get().system.bootButton); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.system.bootButton = ButtonAction(i); }); }),
            MenuItem::number("Ruhezust. nach", 0, 240, 5, 0, " min", [&sm] { return float(sm.get().system.sleepAfterMin); },
                             [&sm](float v) { sm.update([v](AppSettings& s) { s.system.sleepAfterMin = uint16_t(v); }); }),
            MenuItem::choice("Achsen S→D", joinOptions(axes),
                             [&sm] {
                                 char b[8];
                                 sm.get().system.axisMap.format(b);
                                 for (size_t i = 0; i < axes.size(); ++i)
                                     if (axes[i] == b) return int(i);
                                 return 0;
                             },
                             [&sm](int i) {
                                 AxisMap m;
                                 if (i >= 0 && size_t(i) < axes.size() && m.parse(axes[i].c_str()))
                                     sm.update([m](AppSettings& s) { s.system.axisMap = m; });
                             }),
            MenuItem::number("Akku-Teiler", 1, 6, 0.01f, 2, "", [&sm] { return sm.get().system.batteryDividerFactor; },
                             [&sm](float v) { sm.update([v](AppSettings& s) { s.system.batteryDividerFactor = v; }); }),
            MenuItem::choice("Log-Level", "Fehler\nWarnung\nInfo\nDebug", [&sm] { return int(sm.get().system.logLevel); },
                             [&sm](int i) { sm.update([i](AppSettings& s) { s.system.logLevel = uint8_t(i); }); }),
            MenuItem::confirm("Ausschalten", "Gerät ausschalten?\nEinschalten: BOOT-Taste", [this] {
                system_task::requestPowerOff(*app_);
            }),
            MenuItem::confirm("Neustart", "Gerät neu starten?", [this] {
                app_->settings.tick(millis(), true);
                app_->profiles.tick(millis(), true);
                delay(100);
                ESP.restart();
            }),
            MenuItem::confirm("Werkseinst.", "Alle Einstellungen und Profile zurücksetzen? (Kalibrierung bleibt)", [this] {
                app_->settings.factoryReset();
                app_->profiles.factoryReset();
                app_->profiles.tick(millis(), true);
                delay(100);
                ESP.restart();
            }),
        },
        [this] { openMenuRoot(); });
}

// ---------------------------------------------------------------------------
// Colour test pattern to verify the panel configuration on the real hardware
// (inversion, RGB order, black level). Tap anywhere to close.
void UIManager::showColorTest() {
    lv_obj_t* o = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, 240, 240);
    lv_obj_set_style_bg_color(o, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_add_flag(o, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    struct Bar {
        uint32_t rgb;
        const char* name;
        uint32_t text;
    };
    static const Bar bars[] = {
        {0x000000, "Schwarz", 0xFFFFFF}, {0x202020, "Grau 12%", 0xFFFFFF}, {0x606060, "Grau 38%", 0xFFFFFF},
        {0xFFFFFF, "Weiß", 0x000000},    {0xFF0000, "Rot", 0xFFFFFF},      {0x00FF00, "Grün", 0x000000},
        {0x0000FF, "Blau", 0xFFFFFF},
    };
    const int n = sizeof(bars) / sizeof(bars[0]);
    const int h = 180 / n;
    for (int i = 0; i < n; ++i) {
        lv_obj_t* b = lv_obj_create(o);
        lv_obj_remove_style_all(b);
        lv_obj_set_size(b, 240, h);
        lv_obj_set_pos(b, 0, 30 + i * h);
        lv_obj_set_style_bg_color(b, lv_color_hex(bars[i].rgb), 0);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_clear_flag(b, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t* l = theme::label(b, theme::small(), lv_color_hex(bars[i].text), bars[i].name);
        lv_obj_center(l);
    }
    lv_obj_t* hint = theme::label(o, theme::small(), lv_color_hex(0xFFFFFF), "Tippen = schließen");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_add_event_cb(o, [](lv_event_t* e) { lv_obj_del_async(lv_event_get_current_target(e)); }, LV_EVENT_CLICKED,
                        nullptr);
}

void UIManager::menuBattery() {
    auto ps = [this] {
        PowerStatus p;
        app_->power.read(p);
        return p;
    };
    menu_.open(
        "Akku",
        {
            MenuItem::info("Ladestand", [ps] {
                const PowerStatus p = ps();
                return p.batteryPercent < 0 ? std::string("--") : fmt("ca. %.0f %%", double(p.batteryPercent));
            }),
            MenuItem::info("Zustand", [ps] { return std::string(chargeStateName(ps().charge)); }),
            MenuItem::info("Spannung", [ps] { return fmt("%.2f V", double(ps().batteryV)); }),
            MenuItem::info("Trend", [ps] { return fmt("%+.1f mV/min", double(ps().trendMvPerMin)); }),
            MenuItem::info("USB am PC", [ps] { return std::string(ps().usbHost ? "ja" : "nein"); }),
            MenuItem::info("Hinweis", [] { return std::string("Schätzung ±10-20%"); }),
        },
        [this] { menuSystem(); });
}

std::string UIManager::profileOptions() {
    const ProfileSet ps = app_->profiles.all();
    std::string o;
    profileNames_.clear();
    for (int i = 0; i < kMaxProfiles; ++i) {
        if (!ps.profiles[i].used) continue;
        if (!o.empty()) o += "\n";
        o += ps.profiles[i].name;
        profileNames_.push_back(std::to_string(i));
    }
    return o;
}

void UIManager::menuProfiles() {
    SettingsManager& sm = app_->settings;
    const std::string opts = profileOptions();
    menu_.open(
        "Profile",
        {
            MenuItem::choice("Aktiv", opts,
                             [this, &sm] {
                                 const std::string a = std::to_string(sm.get().system.activeProfile);
                                 for (size_t i = 0; i < profileNames_.size(); ++i)
                                     if (profileNames_[i] == a) return int(i);
                                 return 0;
                             },
                             [this, &sm](int i) {
                                 if (i < 0 || size_t(i) >= profileNames_.size()) return;
                                 const int idx = std::stoi(profileNames_[i]);
                                 sm.update([idx](AppSettings& s) { s.system.activeProfile = uint8_t(idx); });
                             }),
            MenuItem::act("Neues Profil", [this, &sm] {
                char name[24];
                snprintf(name, sizeof(name), "Profil %u", unsigned(millis() / 1000 % 1000));
                const int idx = app_->profiles.create(name, sm.get().system.activeProfile);
                if (idx < 0) {
                    showMessage("Maximal 8 Profile");
                    return;
                }
                sm.update([idx](AppSettings& s) { s.system.activeProfile = uint8_t(idx); });
                menuProfiles();
            }),
            MenuItem::submenu("Bearbeiten", [this] { menuProfileEdit(); }),
            MenuItem::confirm("Löschen", "Aktives Profil löschen?", [this, &sm] {
                const uint8_t a = sm.get().system.activeProfile;
                if (!app_->profiles.remove(a)) {
                    showMessage("Letztes Profil");
                    return;
                }
                uint8_t act = 0;
                ProfileSet ps = app_->profiles.all();
                validateProfiles(ps, act);
                sm.update([act](AppSettings& s) { s.system.activeProfile = act; });
                menuProfiles();
            }),
        },
        [this] { openMenuRoot(); });
}

void UIManager::menuProfileEdit() {
    ProfileManager& pm = app_->profiles;
    auto active = [this] { return app_->settings.get().system.activeProfile; };
    static const char* names[] = {"Bohrmaschine 1", "Bohrmaschine 2", "Bohrmaschine 3", "Akkuschrauber",
                                  "Schlagbohrer",   "Bohrhammer",     "Freie Wasserwaage", "Benutzerdefiniert"};
    std::string nameOpts;
    for (int i = 0; i < 8; ++i) {
        if (i) nameOpts += "\n";
        nameOpts += names[i];
    }
    menu_.open(
        "Profil",
        {
            MenuItem::choice("Name", nameOpts,
                             [&pm, active] {
                                 const Profile p = pm.get(active());
                                 for (int i = 0; i < 8; ++i)
                                     if (std::strcmp(p.name, names[i]) == 0) return i;
                                 return 7;
                             },
                             [&pm, active](int i) {
                                 pm.update(active(), [i](Profile& p) { copyString(p.name, sizeof(p.name), names[i & 7]); });
                             }),
            MenuItem::number("Winkelkorr.", -10, 10, 0.05f, 2, "°",
                             [&pm, active] { return pm.get(active()).mount.angleCorrectionDeg; },
                             [this, &pm, active](float v) {
                                 pm.update(active(), [v](Profile& p) { p.mount.angleCorrectionDeg = v; });
                                 app_->meas.setMount(pm.get(active()).mount);
                             }),
            MenuItem::toggle("Werte zeigen", [&pm, active] { return pm.get(active()).showNumbers; },
                             [&pm, active](bool b) { pm.update(active(), [b](Profile& p) { p.showNumbers = b; }); }),
            MenuItem::info("Freier Name", [] { return std::string("über Web-UI"); }),
        },
        [this] { menuProfiles(); });
}
