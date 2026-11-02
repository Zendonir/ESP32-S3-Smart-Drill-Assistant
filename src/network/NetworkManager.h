// SPDX-License-Identifier: MIT
// WiFi (off / access point / client with AP fallback), mDNS and the network task
// that runs the web server, REST API, WebSocket live data and OTA updates.
// The device works fully without network; WiFi is off by default.
#pragma once

#include <cstddef>
#include <cstdint>

struct AppContext;
class WebApi;

class NetworkManager {
public:
    explicit NetworkManager(AppContext& app) : app_(app) {}
    void start();

private:
    AppContext& app_;
    WebApi* api_ = nullptr;
    uint32_t settingsVersion_ = 0;
    uint8_t mode_ = 0;            ///< applied WifiMode
    char ssid_[33] = "";
    char pass_[65] = "";
    char apPass_[17] = "";
    char host_[25] = "";
    bool mdns_ = false;
    bool mdnsStarted_ = false;
    bool serverStarted_ = false;
    bool apFallback_ = false;
    uint32_t clientStartMs_ = 0;
    uint32_t lastReconnectMs_ = 0;

    static void taskEntry(void* arg);
    void task();
    void applySettings(bool force);
    void startAp();
    void startClient();
    void stopAll();
    void ensureServices();
    void updateStatus();
    void apSsid(char* out, size_t n) const;
};
