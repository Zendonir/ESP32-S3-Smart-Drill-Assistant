// SPDX-License-Identifier: MIT
#include "NetworkManager.h"

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include <cstring>

#include "../system/AppContext.h"
#include "../system/LoadEvents.h"
#include "../system/Log.h"
#include "AppConfig.h"
#include "WebApi.h"

static const char* TAG = "net";

void NetworkManager::start() {
    xTaskCreatePinnedToCore(taskEntry, "net", appcfg::kNetTaskStack, this, appcfg::kNetTaskPrio, nullptr,
                            appcfg::kNetTaskCore);
}

void NetworkManager::taskEntry(void* arg) { static_cast<NetworkManager*>(arg)->task(); }

void NetworkManager::apSsid(char* out, size_t n) const {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(out, n, "SmartDrill-%02X%02X", mac[4], mac[5]);
}

void NetworkManager::stopAll() {
    if (mdnsStarted_) {
        MDNS.end();
        mdnsStarted_ = false;
    }
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
    apFallback_ = false;
    app_.networkActive = false;
}

void NetworkManager::startAp() {
    char ssid[33];
    apSsid(ssid, sizeof(ssid));
    WiFi.mode(apFallback_ ? WIFI_AP_STA : WIFI_AP);
    const bool ok = WiFi.softAP(ssid, apPass_);
    LOGI(TAG, "access point %s %s, IP %s", ssid, ok ? "started" : "FAILED", WiFi.softAPIP().toString().c_str());
}

void NetworkManager::startClient() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(host_);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid_, pass_);
    clientStartMs_ = millis();
    apFallback_ = false;
    LOGI(TAG, "connecting to '%s'", ssid_);
}

void NetworkManager::applySettings(bool force) {
    settingsVersion_ = app_.settings.version();
    const sda::AppSettings s = app_.settings.get();
    const bool changed = force || mode_ != uint8_t(s.system.wifiMode) || std::strcmp(ssid_, s.system.ssid) != 0 ||
                         std::strcmp(pass_, s.system.password) != 0 || std::strcmp(apPass_, s.system.apPassword) != 0 ||
                         std::strcmp(host_, s.system.hostname) != 0 || mdns_ != s.system.mdns;
    if (!changed) return;
    mode_ = uint8_t(s.system.wifiMode);
    sda::copyString(ssid_, sizeof(ssid_), s.system.ssid);
    sda::copyString(pass_, sizeof(pass_), s.system.password);
    sda::copyString(apPass_, sizeof(apPass_), s.system.apPassword);
    sda::copyString(host_, sizeof(host_), s.system.hostname);
    mdns_ = s.system.mdns;
    load_events::mark();  // WiFi changes the current draw considerably
    stopAll();
    switch (s.system.wifiMode) {
        case sda::WifiMode::Off: LOGI(TAG, "WiFi off"); break;
        case sda::WifiMode::AccessPoint: startAp(); break;
        case sda::WifiMode::Client: startClient(); break;
    }
}

void NetworkManager::ensureServices() {
    const bool up = WiFi.getMode() != WIFI_OFF;
    if (up && !serverStarted_) {
        api_->begin();
        serverStarted_ = true;
    }
    const bool haveIp = (WiFi.getMode() & WIFI_AP) || WiFi.status() == WL_CONNECTED;
    if (up && haveIp && mdns_ && !mdnsStarted_) {
        if (MDNS.begin(host_)) {
            MDNS.addService("http", "tcp", appcfg::kHttpPort);
            mdnsStarted_ = true;
            LOGI(TAG, "mDNS %s.local", host_);
        }
    }
}

void NetworkManager::updateStatus() {
    NetStatus n;
    n.mode = sda::WifiMode(mode_);
    n.apFallback = apFallback_;
    const wifi_mode_t m = WiFi.getMode();
    if (m & WIFI_AP) {
        n.connected = true;
        sda::copyString(n.ip, sizeof(n.ip), WiFi.softAPIP().toString().c_str());
        apSsid(n.ssid, sizeof(n.ssid));
    }
    if ((m & WIFI_STA) && WiFi.status() == WL_CONNECTED) {
        n.connected = true;
        sda::copyString(n.ip, sizeof(n.ip), WiFi.localIP().toString().c_str());
        sda::copyString(n.ssid, sizeof(n.ssid), ssid_);
        n.rssi = static_cast<int8_t>(WiFi.RSSI());
    }
    if (api_) api_->fillStatus(n);
    app_.networkActive = n.connected && n.wsClients > 0;
    app_.net.write(n);
}

void NetworkManager::task() {
    api_ = new WebApi(app_);  // created once, lives forever (network task heap, not the measurement path)
    WiFi.persistent(false);   // credentials are stored by SettingsManager, not by the WiFi driver
    applySettings(true);
    uint32_t lastSlow = 0;
    for (;;) {
        const uint32_t now = millis();
        if (app_.settings.version() != settingsVersion_) applySettings(false);
        if (now - lastSlow >= 500) {
            lastSlow = now;
            if (mode_ == uint8_t(sda::WifiMode::Client)) {
                const bool conn = WiFi.status() == WL_CONNECTED;
                if (!conn && !apFallback_ && now - clientStartMs_ > appcfg::kClientFallbackToApMs) {
                    // client connection failed -> keep trying, but open the config AP
                    LOGW(TAG, "no connection to '%s' -> configuration AP", ssid_);
                    apFallback_ = true;
                    startAp();
                    WiFi.begin(ssid_, pass_);
                    lastReconnectMs_ = now;
                } else if (!conn && now - lastReconnectMs_ > appcfg::kWifiReconnectMs) {
                    lastReconnectMs_ = now;
                    WiFi.reconnect();
                } else if (conn && apFallback_) {
                    LOGI(TAG, "connected, closing fallback AP");
                    WiFi.softAPdisconnect(true);
                    WiFi.mode(WIFI_STA);
                    apFallback_ = false;
                }
            }
            ensureServices();
            updateStatus();
        }
        if (serverStarted_ && WiFi.getMode() != WIFI_OFF) {
            api_->loop();
            vTaskDelay(pdMS_TO_TICKS(2));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
