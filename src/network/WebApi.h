// SPDX-License-Identifier: MIT
// HTTP server (port 80): embedded web UI, REST API (docs/API.md), firmware
// upload; WebSocket server (port 81) for rate limited live data.
//
// Security: every /api/* endpoint requires HTTP Basic authentication
// (user "admin", device generated password shown on the device under
// System > WLAN) - except for devices connected directly to the device's own
// access point (captive portal), unless "Login im AP" is enabled. The access
// point has to be switched on at the device itself. The WebSocket requires a session token obtained from the
// authenticated endpoint GET /api/token. Repeated authentication failures
// trigger a temporary lock-out.
#pragma once

#include <cstddef>
#include <cstdint>

struct AppContext;
struct NetStatus;
class WebServer;
class WebSocketsServer;

class WebApi {
public:
    explicit WebApi(AppContext& app);
    void begin();
    void loop();
    void fillStatus(NetStatus& n) const;

private:
    AppContext& app_;
    WebServer* http_ = nullptr;
    WebSocketsServer* ws_ = nullptr;
    char token_[33] = "";
    uint8_t wsAuth_[8] = {};      ///< authenticated websocket client ids
    uint32_t lastBroadcastMs_ = 0;
    uint32_t requests_ = 0;
    uint32_t authFailures_ = 0;
    uint8_t recentFailures_ = 0;
    uint32_t lockoutUntilMs_ = 0;
    bool otaRunning_ = false;
    bool otaOk_ = false;
    uint8_t otaProgress_ = 0;
    char otaError_[64] = "";

    bool auth();
    /// Request comes from a device connected to our own access point.
    bool fromAccessPoint();
    /// Authorized without sending a response (used by the upload handler).
    bool authorized();
    void captiveRedirect();
    void hWifiScan();
    void sendJson(int code, const char* body);
    void sendError(int code, const char* errCode, const char* msg);
    void routes();
    void broadcast();
    void onWsEvent(uint8_t num, int type, uint8_t* payload, size_t len);

    // handlers
    void hIndex();
    void hStatus();
    void hImu();
    void hOrientation();
    void hConfigGet();
    void hConfigPut();
    void hCalibrationGet();
    void hCalGyro();
    void hCalAccel();
    void hCalSave();
    void hCalDelete();
    void hReference();
    void hProfilesGet();
    void hProfilesPost();
    void hProfilePut();
    void hProfileDelete();
    void hProfileActivate();
    void hMode();
    void hHold();
    void hTarget();
    void hReboot();
    void hToken();
    void hLog();
    void hExport();
    void hImport();
    void hUpdateDone();
    void hUpdateUpload();
};
