#pragma once
// Command datagram parsing (PROTOCOL.md section 3). Header-only so the host tests cover it too.
//   {"t":123456,"vf":0.20,"vs":0.00,"yr":0.10,"vz":0.00,"arm":1}
// vf is required (numeric); vs/yr/vz default to 0; arm defaults to 0; unknown keys are ignored; all clamped to [-1, 1].
#include <ArduinoJson.h>
#include "mixer.h"

static inline bool parseCommand(const char *json, Setpoint &sp, uint32_t nowMs) {
  JsonDocument doc;
  if (deserializeJson(doc, json) != DeserializationError::Ok || !doc["vf"].is<float>()) return false;
  sp.vf   = clampf(doc["vf"].as<float>(), -1.f, 1.f);
  sp.vs   = clampf(doc["vs"] | 0.f, -1.f, 1.f);
  sp.yr   = clampf(doc["yr"] | 0.f, -1.f, 1.f);
  sp.vz   = clampf(doc["vz"] | 0.f, -1.f, 1.f);
  sp.arm  = (doc["arm"] | 0) == 1;
  sp.rxMs = nowMs;
  return true;
}
