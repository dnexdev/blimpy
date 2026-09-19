#pragma once
// Pure control logic: no Arduino dependencies, so it compiles on the host (pio test -e native) and on the C3.
// Must stay identical to laptop/control/protocol.py (mix, FAILSAFE_MS) and PROTOCOL.md sections 7-8.
#include <stdint.h>
#include "config.h"

struct Setpoint { float vf = 0, vs = 0, yr = 0, vz = 0; bool arm = false; uint32_t rxMs = 0; };

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Motors may run only with arm == 1 and a command younger than FAILSAFE_MS. rxMs == 0 means "never commanded".
// Unsigned subtraction keeps this correct across the millis() wrap at 49.7 days.
static inline bool failsafeOk(const Setpoint &sp, uint32_t nowMs) {
  return sp.arm && sp.rxMs != 0 && (uint32_t)(nowMs - sp.rxMs) < (uint32_t)FAILSAFE_MS;
}

static inline float slewTo(float cur, float tgt) { return cur + clampf(tgt - cur, -SLEW_PER_TICK, SLEW_PER_TICK); }

// Yaw-rate integrator: cancels steady torques (e.g. the S motor a few cm off-centre). Reset on disarm / no IMU.
static float mixYawI = 0.f;
static inline void mixReset() { mixYawI = 0.f; }

// One 50 Hz mixer tick: setpoint + measured yaw rate (rad/s, 0 when no IMU) -> motor duties m[L,R,S,V] in [-CAP, CAP].
static inline void mixStep(const Setpoint &sp, float gzRadS, bool haveImu, float m[4]) {
  float gzNorm = haveImu ? gzRadS / YR_MAX_RAD_S : 0.f;
  float yawErr = sp.yr - gzNorm;
  if (haveImu) mixYawI = clampf(mixYawI + KI_YR * yawErr * 0.02f, -I_YR_MAX, I_YR_MAX);
  else mixYawI = 0.f;
  float diff = K_YR * yawErr + mixYawI;
  m[0] = slewTo(m[0], clampf(sp.vf - diff, -CAP, CAP));  // L
  m[1] = slewTo(m[1], clampf(sp.vf + diff, -CAP, CAP));  // R
  m[2] = slewTo(m[2], clampf(sp.vs,        -CAP, CAP));  // S
  m[3] = slewTo(m[3], clampf(sp.vz,        -CAP, CAP));  // V
}

// DRV8833 channel driven as IN1 = PWM, IN2 = DIR:
//   v >= 0: DIR low,  duty = v        (fast decay, forward)
//   v <  0: DIR high, duty = 1 - |v|  (slow decay, reverse)
static inline void pwmFor(float v, bool &dirHigh, float &duty) {
  v = clampf(v, -1.f, 1.f);
  if (v >= 0) { dirHigh = false; duty = v; }
  else        { dirHigh = true;  duty = 1.f + v; }
}
