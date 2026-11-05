// SPDX-License-Identifier: MIT
// UI task and screen management. This is the ONLY place where LVGL is called
// after start-up (LVGL is not thread safe). Other tasks communicate via the
// UiCommand queue and read UiStatus through SharedState.
//
// Touch concept (requirement 6.4):
//   tap            -> quick menu
//   swipe left/right -> previous / next operating mode
//   long press     -> zero point / capture (mode dependent)
//   double tap     -> hold on/off
//   BOOT short     -> configurable (hold, zero, next mode, touch lock)
//   BOOT long      -> touch lock on/off
// Swipes are ignored while vibration is detected or the touch lock is active
// (unlock: press and hold for 2.5 s, or BOOT button).
#pragma once

#include <lvgl.h>

#include <string>
#include <vector>

#include "../system/AppContext.h"
#include "CalibrationScreen.h"
#include "DiagnosticsScreen.h"
#include "MainScreen.h"
#include "MenuScreen.h"

class UIManager {
public:
    /// Called from setup(): boot screen while the hardware initializes.
    void bootBegin(AppContext& app);
    void bootStep(const char* text, bool ok);
    /// Starts the UI task (takes over LVGL from the setup task).
    void start();

    // used by the settings tree
    AppContext& app() { return *app_; }
    void openMenuRoot();
    void backToMain();
    void showMessage(const char* text, uint32_t ms = 1800);

private:
    enum class View { Main, Menu, Calibration, Diagnostics };
    AppContext* app_ = nullptr;
    View view_ = View::Main;
    MainScreen main_;
    MenuScreen menu_;
    CalibrationScreen cal_;
    DiagnosticsScreen diag_;
    lv_obj_t* bootScr_ = nullptr;
    lv_obj_t* bootList_ = nullptr;
    lv_obj_t* quick_ = nullptr;      ///< quick menu overlay (on main screen)
    lv_obj_t* picker_ = nullptr;     ///< quick picker overlay
    lv_obj_t* pickerRoller_ = nullptr;
    int pickerKind_ = 0;
    bool touchLocked_ = false;
    bool hold_ = false;
    uint32_t lastTapMs_ = 0;
    lv_timer_t* tapTimer_ = nullptr;
    uint32_t unlockPressMs_ = 0;
    uint32_t settingsVersion_ = 0;
    uint32_t profilesVersion_ = 0;
    uint32_t calResultSeq_ = 0;
    sda::AppSettings settings_;
    sda::OrientationSnapshot snap_;
    float fps_ = 0;
    bool dimmed_ = false;
    bool displayOff_ = false;
    float savedValues_[10] = {};
    uint8_t savedCount_ = 0;
    uint8_t savedHead_ = 0;
    std::vector<std::string> profileNames_;

    static void taskEntry(void* arg);
    void task();
    void handleCommand(const UiCommand& c);
    void applySettings(bool initial);
    void applyProfile();
    void setMode(sda::AppMode m, bool persist = true);
    void nextMode(int dir);
    void toggleHold();
    void setTouchLock(bool on);
    void buttonAction(sda::ButtonAction a);
    void wakeDisplay();
    void checkStartupGyro();
    // power button feedback (drawn on the top layer, above every screen)
    lv_obj_t* powerOverlay_ = nullptr;
    lv_obj_t* powerArc_ = nullptr;
    uint32_t sleepAtMs_ = 0;
    void showPowerHold(float progress);
    void hidePowerOverlay();
    void showGoodbye();
    std::string batteryText(bool longForm);
    void menuBattery();
    void showColorTest();
    // charge screen (top layer)
    lv_obj_t* chargeOverlay_ = nullptr;
    lv_obj_t* chargeFill_ = nullptr;
    lv_obj_t* chargePct_ = nullptr;
    lv_obj_t* chargeState_ = nullptr;
    lv_obj_t* chargeHint_ = nullptr;
    uint32_t chargeHideAtMs_ = 0;
    uint32_t chargeLastUpdate_ = 0;
    void showChargeScreen(bool fromOff);
    void updateChargeScreen();
    void closeChargeScreen(bool leaveChargeMode);

    // main screen events
    static void mainEvent(lv_event_t* e);
    static void tapTimerCb(lv_timer_t* t);
    void onTap();
    void onDoubleTap();
    void onLongPress();
    void onGesture(lv_dir_t dir);

    // quick menu
    void openQuickMenu();
    void closeQuickMenu();
    void openPicker(int kind);
    void closePicker();
    static void quickEvent(lv_event_t* e);
    static void pickerOk(lv_event_t* e);

    // settings tree (SettingsScreen.cpp)
    void menuMeasurement();
    void menuCalibration();
    void menuCalibrationData();
    void menuDisplay();
    void menuSystem();
    void menuWifi();
    void menuProfiles();
    void menuProfileEdit();
    void openCalibration(CalibrationScreen::Kind k);
    void openDiagnostics();
    std::string profileOptions();
    std::string faceText(sda::SupportFace f);
};
