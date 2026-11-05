// SPDX-License-Identifier: MIT
#include "WebApi.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_ota_ops.h>
#include <esp_random.h>
#include <uri/UriBraces.h>

#include <cstring>

#include "../storage/ConfigJson.h"
#include "../system/AppContext.h"
#include "../system/Log.h"
#include "../system/OtaGuard.h"
#include "AppConfig.h"
#include "Version.h"
#include "WebAssets.h"  // generated from data/web by scripts/embed_web.py

using namespace sda;

static const char* TAG = "web";

namespace {
constexpr size_t kMaxBody = 12 * 1024;
constexpr uint8_t kMaxAuthFailures = 5;
constexpr uint32_t kLockoutMs = 30000;

void vec(JsonArray a, const Vec3& v) {
    a.add(v.x);
    a.add(v.y);
    a.add(v.z);
}

bool parseBody(WebServer& http, JsonDocument& doc) {
    if (!http.hasArg("plain")) return false;
    const String& body = http.arg("plain");
    if (body.length() == 0 || body.length() > kMaxBody) return false;
    return deserializeJson(doc, body) == DeserializationError::Ok && doc.is<JsonObject>();
}
}  // namespace

WebApi::WebApi(AppContext& app) : app_(app) {
    for (int i = 0; i < 16; ++i) {
        static const char hex[] = "0123456789abcdef";
        const uint32_t r = esp_random();
        token_[2 * i] = hex[r & 15];
        token_[2 * i + 1] = hex[(r >> 4) & 15];
    }
    token_[32] = '\0';
}

void WebApi::begin() {
    http_ = new WebServer(appcfg::kHttpPort);
    ws_ = new WebSocketsServer(appcfg::kWebSocketPort);
    routes();
    http_->begin();
    ws_->begin();
    ws_->onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
        onWsEvent(num, static_cast<int>(type), payload, len);
    });
    LOGI(TAG, "HTTP :%u, WebSocket :%u", appcfg::kHttpPort, appcfg::kWebSocketPort);
}

void WebApi::loop() {
    http_->handleClient();
    ws_->loop();
    const uint8_t rate = app_.settings.get().system.liveRateHz;
    const uint32_t period = 1000u / (rate ? rate : 10);
    if (millis() - lastBroadcastMs_ >= period) {
        lastBroadcastMs_ = millis();
        broadcast();
    }
}

void WebApi::fillStatus(NetStatus& n) const {
    uint8_t c = 0;
    for (uint8_t a : wsAuth_) c += a ? 1 : 0;
    n.wsClients = c;
    n.httpRequests = requests_;
    n.authFailures = authFailures_;
    n.otaInProgress = otaRunning_;
    n.otaProgress = otaProgress_;
}

// ---------------------------------------------------------------------------
bool WebApi::auth() {
    requests_++;
    if (lockoutUntilMs_ && millis() < lockoutUntilMs_) {
        sendError(429, "locked", "Zu viele Fehlversuche, bitte 30 s warten");
        return false;
    }
    const AppSettings s = app_.settings.get();
    if (std::strlen(s.system.adminPassword) >= 8 && http_->authenticate("admin", s.system.adminPassword)) {
        recentFailures_ = 0;
        return true;
    }
    if (http_->hasHeader("Authorization")) {
        authFailures_++;
        if (++recentFailures_ >= kMaxAuthFailures) {
            lockoutUntilMs_ = millis() + kLockoutMs;
            recentFailures_ = 0;
            LOGW(TAG, "authentication lock-out");
        }
    }
    http_->requestAuthentication(BASIC_AUTH, "SmartDrill", "{\"error\":{\"code\":\"unauthorized\"}}");
    return false;
}

void WebApi::sendJson(int code, const char* body) {
    http_->sendHeader("Cache-Control", "no-store");
    http_->send(code, "application/json", body);
}

void WebApi::sendError(int code, const char* errCode, const char* msg) {
    JsonDocument d;
    d["error"]["code"] = errCode;
    d["error"]["message"] = msg;
    String s;
    serializeJson(d, s);
    sendJson(code, s.c_str());
}

#define ROUTE(path, method, fn) http_->on(path, method, [this] { if (auth()) fn(); })

void WebApi::routes() {
    http_->on("/", HTTP_GET, [this] { hIndex(); });
    http_->on("/index.html", HTTP_GET, [this] { hIndex(); });
    ROUTE("/api/status", HTTP_GET, hStatus);
    ROUTE("/api/imu", HTTP_GET, hImu);
    ROUTE("/api/orientation", HTTP_GET, hOrientation);
    ROUTE("/api/config", HTTP_GET, hConfigGet);
    ROUTE("/api/config", HTTP_PUT, hConfigPut);
    ROUTE("/api/calibration", HTTP_GET, hCalibrationGet);
    ROUTE("/api/calibration", HTTP_DELETE, hCalDelete);
    ROUTE("/api/calibration/gyro", HTTP_POST, hCalGyro);
    ROUTE("/api/calibration/accel", HTTP_POST, hCalAccel);
    ROUTE("/api/calibration/save", HTTP_POST, hCalSave);
    ROUTE("/api/reference", HTTP_POST, hReference);
    ROUTE("/api/profiles", HTTP_GET, hProfilesGet);
    ROUTE("/api/profiles", HTTP_POST, hProfilesPost);
    ROUTE(UriBraces("/api/profiles/{}"), HTTP_PUT, hProfilePut);
    ROUTE(UriBraces("/api/profiles/{}"), HTTP_DELETE, hProfileDelete);
    ROUTE(UriBraces("/api/profiles/{}/activate"), HTTP_POST, hProfileActivate);
    ROUTE("/api/mode", HTTP_POST, hMode);
    ROUTE("/api/hold", HTTP_POST, hHold);
    ROUTE("/api/target", HTTP_POST, hTarget);
    ROUTE("/api/reboot", HTTP_POST, hReboot);
    ROUTE("/api/token", HTTP_GET, hToken);
    ROUTE("/api/log", HTTP_GET, hLog);
    ROUTE("/api/export", HTTP_GET, hExport);
    ROUTE("/api/import", HTTP_POST, hImport);
    http_->on("/api/update", HTTP_POST, [this] { hUpdateDone(); }, [this] { hUpdateUpload(); });
    http_->onNotFound([this] { sendError(404, "not_found", "Unbekannter Endpunkt"); });
}

// ---------------------------------------------------------------------------
void WebApi::hIndex() {
    requests_++;
    http_->sendHeader("Content-Encoding", "gzip");
    http_->sendHeader("Cache-Control", "no-cache");
    http_->send_P(200, "text/html; charset=utf-8", reinterpret_cast<const char*>(web_assets::kIndexHtmlGz),
                  web_assets::kIndexHtmlGzLen);
}

void WebApi::hStatus() {
    OrientationSnapshot s;
    app_.meas.snapshot(s);
    PowerStatus p;
    app_.power.read(p);
    UiStatus u;
    app_.uiStatus.read(u);
    NetStatus n;
    app_.net.read(n);
    JsonDocument d;
    d["firmware"] = SDA_FW_VERSION_STR;
    d["build"] = SDA_FW_BUILD_DATE;
    d["board"] = SDA_HW_NAME;
    d["partition"] = ota_guard::runningPartition();
    d["imageConfirmed"] = ota_guard::confirmed();
    d["uptimeS"] = p.uptimeS;
    d["freeHeap"] = p.freeHeap;
    d["minFreeHeap"] = p.minFreeHeap;
    d["freePsram"] = p.freePsram;
    JsonArray cpu = d["cpuLoadEstimatePct"].to<JsonArray>();
    cpu.add(p.cpuLoad[0]);
    cpu.add(p.cpuLoad[1]);
    d["mode"] = static_cast<int>(u.mode);
    d["modeName"] = appModeName(u.mode);
    d["hold"] = u.hold;
    d["touchLocked"] = u.touchLocked;
    d["uiFps"] = u.fps;
    d["profile"] = u.profile;
    d["quality"] = static_cast<int>(s.quality.state);
    d["qualityName"] = qualityStateName(s.quality.state);
    d["qualityFlags"] = s.quality.flags;
    JsonObject b = d["battery"].to<JsonObject>();
    b["voltage"] = p.batteryV;
    b["percentEstimate"] = p.batteryPercent;
    b["level"] = static_cast<int>(p.level);
    b["flashWritesAllowed"] = p.flashWritesAllowed;
    b["chargeState"] = static_cast<int>(p.charge);
    b["chargeStateName"] = chargeStateName(p.charge);
    b["trendMvPerMin"] = p.trendMvPerMin;
    b["usbHost"] = p.usbHost;
    JsonObject w = d["wifi"].to<JsonObject>();
    w["mode"] = static_cast<int>(n.mode);
    w["ip"] = n.ip;
    w["ssid"] = n.ssid;
    w["rssi"] = n.rssi;
    w["apFallback"] = n.apFallback;
    w["wsClients"] = n.wsClients;
    JsonObject hw = d["selfTest"].to<JsonObject>();
    hw["imu"] = app_.boot.imu;
    hw["touch"] = app_.boot.touch;
    hw["display"] = app_.boot.display;
    hw["fs"] = app_.boot.fs;
    hw["calibrationLoaded"] = app_.boot.calibrationLoaded;
    hw["resetReason"] = app_.boot.resetReason;
    String out;
    serializeJson(d, out);
    sendJson(200, out.c_str());
}

void WebApi::hImu() {
    OrientationSnapshot s;
    app_.meas.snapshot(s);
    const MeasurementService::Stats ms = app_.meas.stats();
    JsonDocument d;
    d["valid"] = s.valid;
    vec(d["accelG"].to<JsonArray>(), s.accelG);
    vec(d["gyroDps"].to<JsonArray>(), s.gyroDps);
    vec(d["rawAccelG"].to<JsonArray>(), s.rawAccelG);
    vec(d["rawGyroDps"].to<JsonArray>(), s.rawGyroDps);
    vec(d["onlineBiasDps"].to<JsonArray>(), s.onlineBiasDps);
    if (std::isfinite(s.temperatureC)) d["temperatureC"] = s.temperatureC;
    d["rateHz"] = s.timing.estimatedRateHz;
    d["nominalRateHz"] = s.timing.nominalRateHz;
    d["counterMode"] = s.timing.counterMode;
    d["lostSamples"] = s.timing.lostSamples;
    d["maxIntervalMs"] = s.timing.maxIntervalMs;
    d["samples"] = ms.samples;
    d["busErrors"] = ms.busErrors;
    d["queueOverflows"] = ms.queueOverflows;
    d["clipCount"] = s.clipCount;
    d["vibrationRmsMg"] = s.vibrationRmsMg;
    d["vibrationLevel"] = static_cast<int>(s.vibration);
    d["motion"] = static_cast<int>(s.motion);
    d["activity"] = drillActivityName(s.activity);
    d["accelNoiseG"] = s.accelNoiseG;
    String out;
    serializeJson(d, out);
    sendJson(200, out.c_str());
}

void WebApi::hOrientation() {
    OrientationSnapshot s;
    app_.meas.snapshot(s);
    const AppSettings st = app_.settings.get();
    const Profile prof = app_.profiles.get(st.system.activeProfile);
    UiStatus u;
    app_.uiStatus.read(u);
    JsonDocument d;
    d["valid"] = s.valid;
    JsonArray q = d["quaternion"].to<JsonArray>();
    q.add(s.qWD.w);
    q.add(s.qWD.x);
    q.add(s.qWD.y);
    q.add(s.qWD.z);
    vec(d["upDevice"].to<JsonArray>(), s.upD);
    JsonObject l = d["level"].to<JsonObject>();
    l["tiltXDeg"] = s.level.tiltXDeg;
    l["tiltYDeg"] = s.level.tiltYDeg;
    l["flat"] = s.level.flat;
    l["edgeRollDeg"] = s.level.edgeRollDeg;
    const DrillGeometry g = drillGeometry(s.qWD, prof.mount);
    JsonObject dr = d["drill"].to<JsonObject>();
    dr["elevationDeg"] = g.elevationDeg;
    dr["inclinationDeg"] = g.inclinationDeg;
    dr["rollDeg"] = g.rollDeg;
    dr["headingDegDrifting"] = g.headingDeg;
    JsonObject dv = d["deviation"].to<JsonObject>();
    dv["xDeg"] = u.devXDeg;
    dv["yDeg"] = u.devYDeg;
    dv["totalDeg"] = u.devTotalDeg;
    dv["targetState"] = static_cast<int>(u.targetState);
    dv["lateralFree"] = u.lateralFree;
    dv["lateralFromGyro"] = u.lateralFromGyro;
    dv["targetAngleDeg"] = prof.targetAngleDeg;
    dv["angleReference"] = static_cast<int>(prof.angleRef);
    dv["angleReferenceName"] = angleReferenceName(prof.angleRef);
    {
        const AngleTargetResult at =
            makeAngleTarget(prof.angleRef, prof.targetAngleDeg, st.measurement.hemisphere, s.reference, prof.mount);
        dv["targetValid"] = at.valid;
        dv["needsZero"] = at.needsZero;
        dv["targetUsesGyro"] = at.usesGyro;
        dv["targetElevationDeg"] = at.targetElevationDeg;
        if (!at.valid) dv["problem"] = at.problem;
    }
    dv["toleranceDeg"] = prof.toleranceDeg;
    JsonObject un = d["uncertaintyEstimate"].to<JsonObject>();
    un["tiltSigmaDeg"] = s.tiltSigmaDeg;
    un["headingSigmaDeg"] = s.headingSigmaDeg;
    un["biasSigmaDps"] = s.biasSigmaDps;
    JsonObject r = d["reference"].to<JsonObject>();
    const ReferenceInfo& ri = s.reference.info();
    r["kind"] = static_cast<int>(ri.kind);
    r["name"] = referenceKindName(ri.kind);
    r["ageSec"] = ri.ageSec;
    r["usesGyro"] = ri.usesGyro;
    r["headingSigmaDeg"] = ri.headingSigmaDeg;
    r["surfaceSlopeDeg"] = ri.surfaceSlopeDeg;
    r["azimuthFree"] = ri.azimuthFree;
    d["quality"] = static_cast<int>(s.quality.state);
    d["qualityName"] = qualityStateName(s.quality.state);
    if (s.precision.valid) {
        JsonObject p = d["precision"].to<JsonObject>();
        p["valueDeg"] = s.precision.valueDeg;
        p["meanDeg"] = s.precision.meanDeg;
        p["stdDeg"] = s.precision.stdDeg;
        p["semDeg"] = s.precision.semDeg;
        p["slopeDegPerS"] = s.precision.slopeDegPerS;
        p["samples"] = s.precision.samples;
        p["stable"] = s.precision.stable;
    }
    String out;
    serializeJson(d, out);
    sendJson(200, out.c_str());
}

void WebApi::hConfigGet() {
    JsonDocument d;
    cfgjson::settingsToJson(app_.settings.get(), d.to<JsonObject>(), false);
    String out;
    serializeJson(d, out);
    sendJson(200, out.c_str());
}

void WebApi::hConfigPut() {
    JsonDocument d;
    if (!parseBody(*http_, d)) return sendError(400, "invalid_json", "JSON-Objekt erwartet");
    AppSettings s = app_.settings.get();
    if (!cfgjson::settingsFromJson(d.as<JsonObjectConst>(), s))
        return sendError(422, "invalid_value", "Ungültiger Wert oder Typ");
    app_.settings.replace(s);  // validated again inside
    hConfigGet();
}

void WebApi::hCalibrationGet() {
    OrientationSnapshot s;
    app_.meas.snapshot(s);
    SensorCalibration c;
    app_.meas.currentCalibration(c);
    JsonDocument d;
    JsonObject st = d["status"].to<JsonObject>();
    st["procedure"] = static_cast<int>(s.cal.procedure);
    st["state"] = s.cal.state;
    st["failure"] = s.cal.failure;
    st["progress"] = s.cal.progress;
    st["detectedPose"] = s.cal.detectedPose;
    st["activePose"] = s.cal.activePose;
    st["acceptedPoseMask"] = s.cal.acceptedPoseMask;
    st["resultSeq"] = s.cal.resultSeq;
    st["gyroTempCompActive"] = c.gyroTempCompActive();
    st["accelTempCompActive"] = c.accelTempCompActive();
    cfgjson::calibrationToJson(c, d["data"].to<JsonObject>());
    String out;
    serializeJson(d, out);
    sendJson(200, out.c_str());
}

void WebApi::hCalGyro() {
    if (!app_.meas.startGyroCalibration()) return sendError(503, "busy", "Messsystem beschäftigt");
    sendJson(202, "{\"started\":true,\"hint\":\"Gerät ruhig hinlegen; Ergebnis über GET /api/calibration\"}");
}

void WebApi::hCalAccel() {
    JsonDocument d;
    if (!parseBody(*http_, d)) return sendError(400, "invalid_json", "JSON-Objekt erwartet");
    const char* action = d["action"] | "";
    if (!std::strcmp(action, "start")) {
        app_.meas.startAccelCalibration(d["extended"] | false);
        return sendJson(202, "{\"started\":true}");
    }
    if (!std::strcmp(action, "pose")) {
        const int p = d["pose"] | -1;
        if (p < 0 || p > 5) return sendError(422, "invalid_value", "pose 0..5");
        app_.meas.accelBeginPose(AccelPose(p));
        return sendJson(202, "{\"collecting\":true}");
    }
    if (!std::strcmp(action, "solve")) {
        AccelCalPoint r;
        if (!app_.meas.accelSolve(r)) return sendError(409, "calibration_failed", "Lösung nicht möglich/plausibel");
        const bool save = d["save"] | false;
        bool saved = false;
        if (save) {
            SensorCalibration c;
            if (app_.meas.currentCalibration(c)) {
                c.addAccelPoint(r);
                saved = app_.meas.setCalibration(c) && app_.calStore.save(c);
            }
            app_.meas.cancelCalibration();
        }
        JsonDocument o;
        o["residualMg"] = r.residualMg;
        o["extended"] = r.extended;
        o["saved"] = saved;
        JsonArray m = o["matrix"].to<JsonArray>();
        for (int i = 0; i < 9; ++i) m.add(r.matrix.m[i / 3][i % 3]);
        vec(o["offset"].to<JsonArray>(), r.offset);
        String out;
        serializeJson(o, out);
        return sendJson(200, out.c_str());
    }
    if (!std::strcmp(action, "cancel")) {
        app_.meas.cancelCalibration();
        return sendJson(200, "{\"cancelled\":true}");
    }
    sendError(422, "invalid_value", "action: start|pose|solve|cancel");
}

void WebApi::hCalSave() {
    SensorCalibration c;
    if (!app_.meas.currentCalibration(c)) return sendError(503, "busy", "Messsystem beschäftigt");
    if (!app_.calStore.save(c)) return sendError(503, "write_blocked", "Speichern nicht möglich (Akku kritisch?)");
    sendJson(200, "{\"saved\":true}");
}

void WebApi::hCalDelete() {
    app_.calStore.clear();
    app_.meas.setCalibration(SensorCalibration());
    sendJson(200, "{\"cleared\":true}");
}

void WebApi::hReference() {
    JsonDocument d;
    if (!parseBody(*http_, d)) return sendError(400, "invalid_json", "JSON-Objekt erwartet");
    const char* t = d["type"] | "";
    if (!std::strcmp(t, "tilt")) app_.meas.captureTiltZero();
    else if (!std::strcmp(t, "axis")) app_.meas.captureFull3D();  // reference axis for angle drilling
    else if (!std::strcmp(t, "3d")) app_.meas.captureFull3D();
    else if (!std::strcmp(t, "surface")) app_.meas.captureSurface(false);
    else if (!std::strcmp(t, "surface3d")) app_.meas.captureSurface(true);
    else if (!std::strcmp(t, "clear")) app_.meas.clearReference();
    else return sendError(422, "invalid_value", "type: tilt|3d|axis|surface|surface3d|clear");
    sendJson(200, "{\"ok\":true}");
}

void WebApi::hProfilesGet() {
    JsonDocument d;
    cfgjson::profilesToJson(app_.profiles.all(), d.to<JsonObject>());
    d["active"] = app_.settings.get().system.activeProfile;
    String out;
    serializeJson(d, out);
    sendJson(200, out.c_str());
}

void WebApi::hProfilesPost() {
    JsonDocument d;
    if (!parseBody(*http_, d)) return sendError(400, "invalid_json", "JSON-Objekt erwartet");
    const char* name = d["name"] | "Neues Profil";
    if (std::strlen(name) == 0 || std::strlen(name) > kProfileNameLen) return sendError(422, "invalid_value", "name 1..20 Zeichen");
    const int from = d["from"] | int(app_.settings.get().system.activeProfile);
    const int idx = app_.profiles.create(name, uint8_t(from));
    if (idx < 0) return sendError(409, "full", "Maximal 8 Profile");
    char b[32];
    snprintf(b, sizeof(b), "{\"index\":%d}", idx);
    sendJson(201, b);
}

void WebApi::hProfilePut() {
    const int idx = http_->pathArg(0).toInt();
    if (idx < 0 || idx >= kMaxProfiles || !app_.profiles.get(uint8_t(idx)).used)
        return sendError(404, "not_found", "Profil existiert nicht");
    JsonDocument d;
    if (!parseBody(*http_, d)) return sendError(400, "invalid_json", "JSON-Objekt erwartet");
    Profile p = app_.profiles.get(uint8_t(idx));
    if (!cfgjson::profileFromJson(d.as<JsonObjectConst>(), p)) return sendError(422, "invalid_value", "Ungültiger Wert");
    p.used = true;
    app_.profiles.update(uint8_t(idx), [&p](Profile& t) { t = p; });
    if (idx == app_.settings.get().system.activeProfile) app_.meas.setMount(app_.profiles.get(uint8_t(idx)).mount);
    hProfilesGet();
}

void WebApi::hProfileDelete() {
    const int idx = http_->pathArg(0).toInt();
    if (idx < 0 || idx >= kMaxProfiles) return sendError(404, "not_found", "Profil existiert nicht");
    if (!app_.profiles.remove(uint8_t(idx))) return sendError(409, "last_profile", "Letztes Profil oder unbenutzt");
    uint8_t act = app_.settings.get().system.activeProfile;
    ProfileSet ps = app_.profiles.all();
    validateProfiles(ps, act);
    app_.settings.update([act](AppSettings& s) { s.system.activeProfile = act; });
    sendJson(200, "{\"deleted\":true}");
}

void WebApi::hProfileActivate() {
    const int idx = http_->pathArg(0).toInt();
    if (idx < 0 || idx >= kMaxProfiles || !app_.profiles.get(uint8_t(idx)).used)
        return sendError(404, "not_found", "Profil existiert nicht");
    app_.settings.update([idx](AppSettings& s) { s.system.activeProfile = uint8_t(idx); });
    sendJson(200, "{\"active\":true}");
}

void WebApi::hMode() {
    JsonDocument d;
    if (!parseBody(*http_, d)) return sendError(400, "invalid_json", "JSON-Objekt erwartet");
    const int m = d["mode"] | -1;
    if (m < 0 || m >= int(AppMode::Count)) return sendError(422, "invalid_value", "mode 0..5");
    UiCommand c{};
    c.type = UiCommand::Type::SetMode;
    c.i = m;
    if (!app_.postUi(c)) return sendError(503, "busy", "UI beschäftigt");
    sendJson(200, "{\"ok\":true}");
}

void WebApi::hHold() {
    UiCommand c{};
    c.type = UiCommand::Type::ToggleHold;
    app_.postUi(c);
    sendJson(200, "{\"ok\":true}");
}

void WebApi::hTarget() {
    JsonDocument d;
    if (!parseBody(*http_, d)) return sendError(400, "invalid_json", "JSON-Objekt erwartet");
    const uint8_t active = app_.settings.get().system.activeProfile;
    Profile p = app_.profiles.get(active);
    if (d["angle"].is<float>()) {
        const float a = d["angle"].as<float>();
        if (!(a >= 0 && a <= 90)) return sendError(422, "invalid_value", "angle 0..90");
        p.targetAngleDeg = a;
    }
    if (!d["reference"].isNull()) {
        const int r = d["reference"] | -1;
        if (r < 0 || r >= int(AngleReference::Count)) return sendError(422, "invalid_value", "reference 0..5");
        p.angleRef = static_cast<AngleReference>(r);
    }
    if (d["tolerance"].is<float>()) {
        const float t = d["tolerance"].as<float>();
        if (!(t >= 0.05f && t <= 10)) return sendError(422, "invalid_value", "tolerance 0.05..10");
        p.toleranceDeg = t;
    }
    app_.profiles.update(active, [&p](Profile& t) { t = p; });
    sendJson(200, "{\"ok\":true}");
}

void WebApi::hReboot() {
    sendJson(200, "{\"rebooting\":true}");
    app_.settings.tick(millis(), true);
    app_.profiles.tick(millis(), true);
    delay(300);
    ESP.restart();
}

void WebApi::hToken() {
    char b[64];
    snprintf(b, sizeof(b), "{\"token\":\"%s\",\"port\":%u}", token_, appcfg::kWebSocketPort);
    sendJson(200, b);
}

void WebApi::hLog() {
    static char buf[2048];
    sda_log::recent(buf, sizeof(buf));
    http_->send(200, "text/plain; charset=utf-8", buf);
}

void WebApi::hExport() {
    JsonDocument d;
    d["firmware"] = SDA_FW_VERSION_STR;
    cfgjson::settingsToJson(app_.settings.get(), d["settings"].to<JsonObject>(), false);
    cfgjson::profilesToJson(app_.profiles.all(), d["profiles"].to<JsonObject>());
    SensorCalibration c;
    app_.meas.currentCalibration(c);
    cfgjson::calibrationToJson(c, d["calibration"].to<JsonObject>());
    String out;
    serializeJson(d, out);
    http_->sendHeader("Content-Disposition", "attachment; filename=smartdrill-config.json");
    sendJson(200, out.c_str());
}

void WebApi::hImport() {
    JsonDocument d;
    if (!parseBody(*http_, d)) return sendError(400, "invalid_json", "JSON-Objekt erwartet");
    // validate everything first, apply only if all parts are valid
    AppSettings s = app_.settings.get();
    ProfileSet ps;
    SensorCalibration c;
    const bool hasS = !d["settings"].isNull();
    const bool hasP = !d["profiles"].isNull();
    const bool hasC = !d["calibration"].isNull() && (d["importCalibration"] | false);
    if (hasS && !cfgjson::settingsFromJson(d["settings"].as<JsonObjectConst>(), s))
        return sendError(422, "invalid_value", "settings ungültig");
    if (hasP && !cfgjson::profilesFromJson(d["profiles"].as<JsonObjectConst>(), ps))
        return sendError(422, "invalid_value", "profiles ungültig");
    if (hasC && !cfgjson::calibrationFromJson(d["calibration"].as<JsonObjectConst>(), c))
        return sendError(422, "invalid_value", "calibration ungültig");
    if (hasS) app_.settings.replace(s);
    if (hasP) app_.profiles.replaceAll(ps);
    if (hasC) {
        app_.meas.setCalibration(c);
        app_.calStore.save(c);
    }
    char b[96];
    snprintf(b, sizeof(b), "{\"settings\":%s,\"profiles\":%s,\"calibration\":%s}", hasS ? "true" : "false",
             hasP ? "true" : "false", hasC ? "true" : "false");
    sendJson(200, b);
}

// ---------------------------------------------------------------------------
// OTA: multipart upload of firmware.bin. The image is written to the inactive
// OTA slot; Update.end() verifies size and the image checksum before the boot
// partition is switched. After reboot the new image must confirm itself
// (OtaGuard), otherwise the bootloader rolls back.
// ---------------------------------------------------------------------------
void WebApi::hUpdateUpload() {
    HTTPUpload& up = http_->upload();
    if (up.status == UPLOAD_FILE_START) {
        otaOk_ = false;
        otaError_[0] = '\0';
        otaProgress_ = 0;
        otaRunning_ = false;
        const AppSettings s = app_.settings.get();
        if (!http_->authenticate("admin", s.system.adminPassword)) {
            sda::copyString(otaError_, sizeof(otaError_), "unauthorized");
            return;
        }
        PowerStatus p;
        app_.power.read(p);
        if (p.level == BatteryLevel::Critical || p.level == BatteryLevel::Low) {
            sda::copyString(otaError_, sizeof(otaError_), "Akku zu schwach für Update");
            return;
        }
        const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
        if (!next || !Update.begin(next->size, U_FLASH)) {
            sda::copyString(otaError_, sizeof(otaError_), "Update.begin fehlgeschlagen");
            return;
        }
        otaRunning_ = true;
        LOGI(TAG, "OTA start: %s -> %s", up.filename.c_str(), next->label);
    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (!otaRunning_) return;
        if (up.totalSize == 0 && up.currentSize > 0 && up.buf[0] != 0xE9) {
            // ESP32 application images start with magic byte 0xE9
            sda::copyString(otaError_, sizeof(otaError_), "keine gültige Firmware-Datei");
            Update.abort();
            otaRunning_ = false;
            return;
        }
        if (Update.write(up.buf, up.currentSize) != up.currentSize) {
            sda::copyString(otaError_, sizeof(otaError_), Update.errorString());
            Update.abort();
            otaRunning_ = false;
            return;
        }
        const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
        if (next) otaProgress_ = static_cast<uint8_t>(100ULL * (up.totalSize + up.currentSize) / next->size);
    } else if (up.status == UPLOAD_FILE_END) {
        if (!otaRunning_) return;
        if (Update.end(true)) {
            otaOk_ = true;
            otaProgress_ = 100;
            LOGI(TAG, "OTA finished: %u bytes", static_cast<unsigned>(up.totalSize));
        } else {
            sda::copyString(otaError_, sizeof(otaError_), Update.errorString());
        }
        otaRunning_ = false;
    } else if (up.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        otaRunning_ = false;
        sda::copyString(otaError_, sizeof(otaError_), "Upload abgebrochen");
    }
}

void WebApi::hUpdateDone() {
    if (!auth()) return;
    if (!otaOk_) return sendError(400, "update_failed", otaError_[0] ? otaError_ : "kein Firmware-Upload");
    sendJson(200, "{\"updated\":true,\"rebooting\":true}");
    app_.settings.tick(millis(), true);
    app_.profiles.tick(millis(), true);
    delay(500);
    ESP.restart();
}

// ---------------------------------------------------------------------------
// WebSocket live data
// ---------------------------------------------------------------------------
void WebApi::onWsEvent(uint8_t num, int type, uint8_t* payload, size_t len) {
    if (num >= sizeof(wsAuth_)) {
        ws_->disconnect(num);
        return;
    }
    switch (static_cast<WStype_t>(type)) {
        case WStype_CONNECTED: {
            // URL must contain ?t=<token>
            char url[96];
            const size_t n = len < sizeof(url) - 1 ? len : sizeof(url) - 1;
            std::memcpy(url, payload, n);
            url[n] = '\0';
            const char* t = std::strstr(url, "t=");
            if (!t || std::strncmp(t + 2, token_, 32) != 0) {
                LOGW(TAG, "websocket client %u rejected (token)", num);
                ws_->disconnect(num);
                return;
            }
            uint8_t authed = 0;
            for (uint8_t a : wsAuth_) authed += a;
            if (authed >= appcfg::kMaxWsClients) {
                ws_->disconnect(num);
                return;
            }
            wsAuth_[num] = 1;
            break;
        }
        case WStype_DISCONNECTED: wsAuth_[num] = 0; break;
        default: break;  // the live channel is read-only
    }
}

void WebApi::broadcast() {
    bool any = false;
    for (uint8_t a : wsAuth_) any |= a != 0;
    if (!any) return;
    OrientationSnapshot s;
    app_.meas.snapshot(s);
    UiStatus u;
    app_.uiStatus.read(u);
    PowerStatus p;
    app_.power.read(p);
    char b[720];
    const int n = snprintf(
        b, sizeof(b),
        "{\"t\":%lu,\"mode\":%d,\"hold\":%d,\"q\":%d,\"qn\":\"%s\",\"dev\":[%.3f,%.3f,%.3f],\"ts\":%d,"
        "\"target\":%.2f,\"tol\":%.2f,\"val\":%.3f,\"tilt\":[%.3f,%.3f],\"acc\":[%.4f,%.4f,%.4f],"
        "\"gyr\":[%.3f,%.3f,%.3f],\"temp\":%.2f,\"vib\":%.1f,\"vl\":%d,\"motion\":%d,\"sig\":%.3f,"
        "\"hsig\":%.2f,\"rate\":%.1f,\"lost\":%lu,\"bat\":%.2f,\"pct\":%d,\"gcal\":%d,\"acal\":%d,"
        "\"chg\":%d,\"cal\":{\"p\":%d,\"pr\":%.2f,\"mask\":%d,\"det\":%d,\"act\":%d,\"seq\":%lu},\"fps\":%.0f}",
        static_cast<unsigned long>(millis()), int(u.mode), u.hold ? 1 : 0, int(s.quality.state),
        qualityStateShort(s.quality.state), double(u.devXDeg), double(u.devYDeg), double(u.devTotalDeg),
        int(u.targetState), double(u.targetAngleDeg), double(u.toleranceDeg), double(u.valueDeg),
        double(s.level.tiltXDeg), double(s.level.tiltYDeg), double(s.accelG.x), double(s.accelG.y),
        double(s.accelG.z), double(s.gyroDps.x), double(s.gyroDps.y), double(s.gyroDps.z),
        double(std::isfinite(s.temperatureC) ? s.temperatureC : 0.0f), double(s.vibrationRmsMg), int(s.vibration),
        int(s.motion), double(s.tiltSigmaDeg), double(s.reference.info().headingSigmaDeg),
        double(s.timing.estimatedRateHz), static_cast<unsigned long>(s.timing.lostSamples), double(p.batteryV),
        p.batteryPercent, s.hasGyroCal ? 1 : 0, s.hasAccelCal ? 1 : 0, int(p.charge), int(s.cal.procedure),
        double(s.cal.progress), int(s.cal.acceptedPoseMask), int(s.cal.detectedPose), int(s.cal.activePose),
        static_cast<unsigned long>(s.cal.resultSeq), double(u.fps));
    if (n <= 0 || n >= int(sizeof(b))) return;
    for (uint8_t i = 0; i < sizeof(wsAuth_); ++i)
        if (wsAuth_[i]) ws_->sendTXT(i, b, static_cast<size_t>(n));
}
