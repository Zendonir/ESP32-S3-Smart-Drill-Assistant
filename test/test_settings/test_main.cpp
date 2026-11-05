// SPDX-License-Identifier: MIT
// Unit tests: configuration defaults and validation/repair.
#include <unity.h>

#include <cstring>
#include <initializer_list>
#include <utility>

#include "BatteryModel.h"
#include "ButtonLogic.h"
#include "Settings.h"

using namespace sda;

void setUp() {}
void tearDown() {}

void test_defaults_are_valid() {
    AppSettings s;
    defaultSettings(s);
    TEST_ASSERT_EQUAL(0, validateSettings(s));
    ProfileSet ps;
    defaultProfiles(ps);
    uint8_t active = 0;
    TEST_ASSERT_EQUAL(0, validateProfiles(ps, active));
    TEST_ASSERT_TRUE(ps.profiles[0].used);
    TEST_ASSERT_TRUE(ps.profiles[4].used);
    TEST_ASSERT_FALSE(ps.profiles[5].used);
    TEST_ASSERT_EQUAL_STRING("Akkuschrauber", ps.profiles[2].name);
}

void test_invalid_values_are_repaired() {
    AppSettings s;
    s.measurement.filterStrength = 9;
    s.measurement.precisionWindowSec = NAN;
    s.display.brightness = 0;
    s.display.historyWindowSec = 7;
    s.system.wifiMode = WifiMode::Client;  // without SSID
    std::strcpy(s.system.hostname, "bad host!");
    std::strcpy(s.system.adminPassword, "short");
    s.system.axisMap.code[0] = 1;
    s.system.axisMap.code[1] = 1;  // duplicate -> invalid
    const int n = validateSettings(s);
    TEST_ASSERT_TRUE(n >= 8);
    TEST_ASSERT_EQUAL(2, s.measurement.filterStrength);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, s.measurement.precisionWindowSec);
    TEST_ASSERT_EQUAL(80, s.display.brightness);
    TEST_ASSERT_EQUAL(15, s.display.historyWindowSec);
    TEST_ASSERT_EQUAL(int(WifiMode::AccessPoint), int(s.system.wifiMode));
    TEST_ASSERT_EQUAL_STRING("smartdrill", s.system.hostname);
    TEST_ASSERT_EQUAL_STRING("", s.system.adminPassword);
    TEST_ASSERT_TRUE(s.system.axisMap.isValid());
    TEST_ASSERT_EQUAL(0, validateSettings(s));  // idempotent
}

void test_profile_validation() {
    ProfileSet ps;
    defaultProfiles(ps);
    ps.profiles[1].toleranceDeg = -3;
    ps.profiles[1].mount.drillAxisD = Vec3(0, 0, -2);  // not normalized
    ps.profiles[1].targetAngleDeg = 120;
    uint8_t active = 7;  // unused slot
    TEST_ASSERT_TRUE(validateProfiles(ps, active) >= 4);
    TEST_ASSERT_EQUAL(0, active);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, ps.profiles[1].mount.drillAxisD.norm());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, ps.profiles[1].toleranceDeg);
}

void test_no_profiles_restores_defaults() {
    ProfileSet ps;
    uint8_t a = 0;
    validateProfiles(ps, a);
    TEST_ASSERT_TRUE(ps.profiles[0].used);
}

void test_quick_angles_and_filter_params() {
    MeasurementSettings m;
    m.customAngleDeg = 33.0f;
    TEST_ASSERT_EQUAL_FLOAT(45.0f, quickAngle(3, m));
    TEST_ASSERT_EQUAL_FLOAT(33.0f, quickAngle(6, m));
    TEST_ASSERT_TRUE(filterParams(4).displayMinCutoffHz < filterParams(0).displayMinCutoffHz);
}

void test_battery_model() {
    BatteryModel b;
    b.configure(3.5f, 3.35f);
    b.update(3.85f, 0.5f);
    TEST_ASSERT_EQUAL(int(BatteryLevel::Ok), int(b.level()));
    TEST_ASSERT_EQUAL(60, b.percentEstimate());
    BatteryModel low;
    low.configure(3.5f, 3.35f);
    low.update(3.30f, 0.5f);
    TEST_ASSERT_EQUAL(int(BatteryLevel::Critical), int(low.level()));
    BatteryModel none;
    none.update(0.3f, 0.5f);
    TEST_ASSERT_EQUAL(-1, none.percentEstimate());
    TEST_ASSERT_EQUAL_FLOAT(100.0f, BatteryModel::socFromVoltage(4.2f));
}

// runs the button logic with a press pattern: list of (pressed, durationMs)
static ButtonLogic::Event runPattern(ButtonLogic& b, uint32_t& t, std::initializer_list<std::pair<bool, uint32_t>> p,
                                     int* count = nullptr, ButtonLogic::Event want = ButtonLogic::Event::None) {
    ButtonLogic::Event last = ButtonLogic::Event::None;
    for (const auto& seg : p)
        for (uint32_t i = 0; i < seg.second; i += 5) {
            t += 5;
            const ButtonLogic::Event e = b.update(seg.first, t);
            if (e != ButtonLogic::Event::None) {
                last = e;
                if (count && e == want) ++*count;
            }
        }
    return last;
}

void test_button_short_double_hold_poweroff() {
    using E = ButtonLogic::Event;
    ButtonLogic b;
    uint32_t t = 0;
    b.begin(false, t);
    TEST_ASSERT_EQUAL(int(E::Short), int(runPattern(b, t, {{true, 150}, {false, 600}})));
    TEST_ASSERT_EQUAL(int(E::Double), int(runPattern(b, t, {{true, 100}, {false, 150}, {true, 100}, {false, 600}})));
    int shorts = 0;
    runPattern(b, t, {{true, 100}, {false, 150}, {true, 100}, {false, 600}}, &shorts, E::Short);
    TEST_ASSERT_EQUAL(0, shorts);  // a double press must not also trigger a short press
    // hold 1.5 s and release: countdown cancelled, no short press
    TEST_ASSERT_EQUAL(int(E::HoldCancel), int(runPattern(b, t, {{true, 1500}, {false, 600}})));
    // hold 3.2 s: power off
    int off = 0;
    runPattern(b, t, {{true, 3200}}, &off, E::PowerOff);
    TEST_ASSERT_EQUAL(1, off);
    TEST_ASSERT_EQUAL(int(E::None), int(runPattern(b, t, {{false, 600}})));
}

void test_button_ignores_power_on_press() {
    using E = ButtonLogic::Event;
    ButtonLogic b;
    uint32_t t = 1000;
    b.begin(true, t);  // still held from waking the device
    int any = 0;
    for (auto e : {E::Short, E::HoldStart, E::PowerOff}) {
        uint32_t tt = t;
        ButtonLogic c = b;
        runPattern(c, tt, {{true, 3500}, {false, 600}}, &any, e);
    }
    TEST_ASSERT_EQUAL(0, any);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, b.powerOffProgress(t + 2000));
}

void test_battery_charge_detection_and_percent_hysteresis() {
    BatteryModel b;
    b.configure(3.5f, 3.35f);
    // discharging slowly: 3.80 V, -1 mV/min
    float v = 3.80f;
    for (int i = 0; i < 400; ++i) {  // 200 s at 0.5 s
        v -= 0.001f / 120.0f;
        b.update(v, 0.5f);
    }
    TEST_ASSERT_EQUAL(int(ChargeState::Discharging), int(b.chargeState()));
    const int p0 = b.percentEstimate();
    // small load step (+15 mV, backlight off) must not change the shown percentage
    b.update(v + 0.015f, 0.5f);
    TEST_ASSERT_EQUAL(p0, b.percentEstimate());
    // charging: +10 mV/min with a wall charger (no USB host)
    for (int i = 0; i < 480; ++i) {
        v += 0.010f / 120.0f;
        b.update(v, 0.5f);
    }
    TEST_ASSERT_EQUAL(int(ChargeState::Charging), int(b.chargeState()));
    // USB host at full voltage -> full
    BatteryModel f;
    f.setUsbHost(true);
    for (int i = 0; i < 300; ++i) f.update(4.18f, 0.5f);
    TEST_ASSERT_EQUAL(int(ChargeState::Full), int(f.chargeState()));
    BatteryModel n;
    n.update(0.2f, 0.5f);
    TEST_ASSERT_EQUAL(int(ChargeState::NoBattery), int(n.chargeState()));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_defaults_are_valid);
    RUN_TEST(test_invalid_values_are_repaired);
    RUN_TEST(test_profile_validation);
    RUN_TEST(test_no_profiles_restores_defaults);
    RUN_TEST(test_quick_angles_and_filter_params);
    RUN_TEST(test_battery_model);
    RUN_TEST(test_button_short_double_hold_poweroff);
    RUN_TEST(test_button_ignores_power_on_press);
    RUN_TEST(test_battery_charge_detection_and_percent_hysteresis);
    return UNITY_END();
}
