// SPDX-License-Identifier: MIT
// Unit tests: configuration defaults and validation/repair.
#include <unity.h>

#include <cstring>

#include "BatteryModel.h"
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

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_defaults_are_valid);
    RUN_TEST(test_invalid_values_are_repaired);
    RUN_TEST(test_profile_validation);
    RUN_TEST(test_no_profiles_restores_defaults);
    RUN_TEST(test_quick_angles_and_filter_params);
    RUN_TEST(test_battery_model);
    return UNITY_END();
}
