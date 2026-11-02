// SPDX-License-Identifier: MIT
// JSON (de)serialization of settings, profiles and calibration. Shared by the
// persistent storage (NVS / LittleFS) and the REST API so that both use exactly
// the same field names and validation (docs/API.md).
//
// fromJson functions apply only the keys that are present (partial update) and
// never trust the input: every result passes validate*() afterwards.
#pragma once

#include <ArduinoJson.h>

#include "SensorCalibration.h"
#include "Settings.h"

namespace cfgjson {

void settingsToJson(const sda::AppSettings& s, JsonObject o, bool includeSecrets);
/// Returns false if the JSON has a wrong type for a known key (nothing applied then).
bool settingsFromJson(JsonObjectConst o, sda::AppSettings& s);

void profileToJson(const sda::Profile& p, JsonObject o);
bool profileFromJson(JsonObjectConst o, sda::Profile& p);
void profilesToJson(const sda::ProfileSet& ps, JsonObject o);
bool profilesFromJson(JsonObjectConst o, sda::ProfileSet& ps);

void calibrationToJson(const sda::SensorCalibration& c, JsonObject o);
bool calibrationFromJson(JsonObjectConst o, sda::SensorCalibration& c);

}  // namespace cfgjson
