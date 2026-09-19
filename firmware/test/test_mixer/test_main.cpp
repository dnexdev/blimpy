// Host-side tests of the firmware control logic (no board needed):
//   cd firmware && pio test -e native
// Golden numbers come from laptop/control/protocol.py::mix, so firmware and simulator cannot drift apart silently.
#include <unity.h>
#include <string.h>
#include "mixer.h"
#include "command.h"

static const float EPS = 1e-5f;

static void run_ticks(const Setpoint &sp, float gz, bool imu, float m[4], int n) {
  for (int i = 0; i < n; i++) mixStep(sp, gz, imu, m);
}

// ---------------------------------------------------------------- mixer vs protocol.py golden values
void test_mix_golden_sequence() {              // mix((0.2,-0.1,0.1,0.05), gz 0) from rest: ticks 1, 4, 12
  Setpoint sp; sp.vf = 0.2f; sp.vs = -0.1f; sp.yr = 0.1f; sp.vz = 0.05f;
  float m[4] = {0, 0, 0, 0};
  run_ticks(sp, 0, false, m, 1);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.05f, m[0]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.05f, m[1]);
  TEST_ASSERT_FLOAT_WITHIN(EPS, -0.05f, m[2]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.05f, m[3]);
  run_ticks(sp, 0, false, m, 3);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.1f, m[0]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.2f, m[1]);
  TEST_ASSERT_FLOAT_WITHIN(EPS, -0.1f, m[2]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.05f, m[3]);
  run_ticks(sp, 0, false, m, 8);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.1f, m[0]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.3f, m[1]);
  TEST_ASSERT_FLOAT_WITHIN(EPS, -0.1f, m[2]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.05f, m[3]);
}

void test_mix_cap_and_yaw_priority() {         // mix((1,0,1,0)) -> L 0, R 0.5 (CAP)
  Setpoint sp; sp.vf = 1.f; sp.yr = 1.f;
  float m[4] = {0, 0, 0, 0};
  run_ticks(sp, 0, false, m, 15);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, m[0]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.5f, m[1]);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, m[2]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, m[3]);
}

void test_mix_never_exceeds_cap() {
  Setpoint sp; sp.vf = 1.f; sp.vs = -1.f; sp.yr = -1.f; sp.vz = 1.f;
  float m[4] = {0, 0, 0, 0};
  for (int i = 0; i < 40; i++) {
    mixStep(sp, 0, false, m);
    for (int k = 0; k < 4; k++) TEST_ASSERT_TRUE(m[k] <= CAP + EPS && m[k] >= -CAP - EPS);
  }
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.5f, m[0]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, m[1]);
  TEST_ASSERT_FLOAT_WITHIN(EPS, -0.5f, m[2]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.5f, m[3]);
}

void test_mix_imu_feedback() {                 // gz 0.5 rad/s, yr 0, from (0.3, 0.3): L 0.35, R 0.25 (damps the spin)
  mixReset();                                  // the static yaw integrator survives between tests
  Setpoint sp; sp.vf = 0.3f;
  float m[4] = {0.3f, 0.3f, 0, 0};
  mixStep(sp, 0.5f, true, m);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.35f, m[0]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.25f, m[1]);
  float m2[4] = {0.3f, 0.3f, 0, 0};            // same gz but no IMU -> ignored (open loop)
  mixStep(sp, 0.5f, false, m2);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.3f, m2[0]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.3f, m2[1]);
}

void test_slew_rate() {                        // 0 -> 0.5 takes exactly 10 ticks (0.2 s), 0.05 per tick
  Setpoint sp; sp.vf = 0.5f;
  float m[4] = {0, 0, 0, 0};
  for (int i = 1; i <= 10; i++) { mixStep(sp, 0, false, m); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.05f * i, m[0]); }
  mixStep(sp, 0, false, m); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.5f, m[0]);
  sp.vf = -0.5f;                               // reversing is also slewed, through zero
  mixStep(sp, 0, false, m); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.45f, m[0]);
}

// ---------------------------------------------------------------- yaw-rate integrator (KI_YR) over many ticks
// err = 0.02: the I term adds KI_YR*err*0.02 = 0.0004 per tick, far below SLEW, so the outputs track their targets
// exactly and the integrator is visible: R = K_YR*err + 0.0004*n, L = -R. (One tick at err 0.5 saturates SLEW and
// hides it, which is why test_mix_imu_feedback passes with KI_YR = 0.)
void test_yaw_integrator_winds_up_and_clamps() {
  mixReset();
  Setpoint sp; sp.yr = 0.02f;
  float m[4] = {0, 0, 0, 0};
  for (int n = 1; n <= 10; n++) {
    mixStep(sp, 0.f, true, m);
    float want = K_YR * 0.02f + KI_YR * 0.02f * 0.02f * n;     // 0.0204, 0.0208, ..., 0.0240
    TEST_ASSERT_FLOAT_WITHIN(EPS, want, m[1]); TEST_ASSERT_FLOAT_WITHIN(EPS, -want, m[0]);
  }
  run_ticks(sp, 0.f, true, m, 390);                  // 400 ticks = 8 s: I would be 0.16 -> clamped at I_YR_MAX
  TEST_ASSERT_FLOAT_WITHIN(EPS, K_YR * 0.02f + I_YR_MAX, m[1]);          // 0.17
  TEST_ASSERT_FLOAT_WITHIN(EPS, -(K_YR * 0.02f + I_YR_MAX), m[0]);
  run_ticks(sp, 0.f, true, m, 100);
  TEST_ASSERT_FLOAT_WITHIN(EPS, K_YR * 0.02f + I_YR_MAX, m[1]);          // stays clamped
  float fresh[4] = {0, 0, 0, 0};                     // the wound-up integrator persists into a fresh output array...
  mixStep(sp, 0.f, true, fresh);
  TEST_ASSERT_FLOAT_WITHIN(EPS, SLEW_PER_TICK, fresh[1]);                // target 0.1704 -> slew-limited to 0.05
  mixReset();                                        // ...until mixReset() (controlTick on disarm, main.cpp)
  float after[4] = {0, 0, 0, 0};
  mixStep(sp, 0.f, true, after);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0204f, after[1]);
  run_ticks(sp, 0.f, true, after, 400);              // wind up again; one tick without the IMU must zero it too
  mixStep(sp, 0.f, false, after);
  float noimu[4] = {0, 0, 0, 0};
  mixStep(sp, 0.f, true, noimu);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0204f, noimu[1]);
  mixReset();
}

void test_yaw_integrator_negative_clamp() {
  mixReset();
  Setpoint sp; sp.yr = -0.02f;
  float m[4] = {0, 0, 0, 0};
  run_ticks(sp, 0.f, true, m, 400);
  TEST_ASSERT_FLOAT_WITHIN(EPS, -(K_YR * 0.02f + I_YR_MAX), m[1]); TEST_ASSERT_FLOAT_WITHIN(EPS, K_YR * 0.02f + I_YR_MAX, m[0]);
  mixReset();
}

// ---------------------------------------------------------------- failsafe
void test_failsafe_window() {
  Setpoint sp; sp.arm = true; sp.rxMs = 10000;
  TEST_ASSERT_TRUE(failsafeOk(sp, 10000));
  TEST_ASSERT_TRUE(failsafeOk(sp, 10000 + FAILSAFE_MS - 1));
  TEST_ASSERT_FALSE(failsafeOk(sp, 10000 + FAILSAFE_MS));
  TEST_ASSERT_FALSE(failsafeOk(sp, 10000 + 60000));
  sp.arm = false; TEST_ASSERT_FALSE(failsafeOk(sp, 10001));
  sp.arm = true; sp.rxMs = 0; TEST_ASSERT_FALSE(failsafeOk(sp, 100));      // never commanded
}

void test_failsafe_millis_wrap() {
  Setpoint sp; sp.arm = true; sp.rxMs = 0xFFFFFF00u;                       // 256 ms before the 32-bit wrap
  TEST_ASSERT_TRUE(failsafeOk(sp, 0x00000010u));                            // 272 ms later, wrapped
  TEST_ASSERT_FALSE(failsafeOk(sp, 0x00000200u));                           // 768 ms later
}

// ---------------------------------------------------------------- PWM + DIR mapping
void test_pwm_dir_mapping() {
  bool dir; float duty;
  pwmFor(0.3f, dir, duty);  TEST_ASSERT_FALSE(dir); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.3f, duty);
  pwmFor(-0.3f, dir, duty); TEST_ASSERT_TRUE(dir);  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.7f, duty);
  pwmFor(0.0f, dir, duty);  TEST_ASSERT_FALSE(dir); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, duty);
  pwmFor(-1.0f, dir, duty); TEST_ASSERT_TRUE(dir);  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, duty);
  pwmFor(2.0f, dir, duty);  TEST_ASSERT_FALSE(dir); TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, duty);   // clamped
}

// ---------------------------------------------------------------- command parsing
void test_parse_full_command() {
  Setpoint sp;
  TEST_ASSERT_TRUE(parseCommand("{\"t\":123456,\"vf\":0.20,\"vs\":-0.1,\"yr\":0.10,\"vz\":0.05,\"arm\":1}", sp, 777));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.2f, sp.vf); TEST_ASSERT_FLOAT_WITHIN(EPS, -0.1f, sp.vs);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.1f, sp.yr); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.05f, sp.vz);
  TEST_ASSERT_TRUE(sp.arm); TEST_ASSERT_EQUAL_UINT32(777, sp.rxMs);
}

void test_parse_defaults_and_clamp() {
  Setpoint sp;
  TEST_ASSERT_TRUE(parseCommand("{\"vf\":5,\"yr\":-3,\"arm\":2,\"foo\":\"bar\"}", sp, 5));   // unknown key ignored
  TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, sp.vf); TEST_ASSERT_FLOAT_WITHIN(EPS, -1.0f, sp.yr);
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, sp.vs); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, sp.vz);
  TEST_ASSERT_FALSE(sp.arm);                                                                    // arm must be exactly 1
  TEST_ASSERT_TRUE(parseCommand("{\"vf\":0,\"arm\":1}", sp, 6)); TEST_ASSERT_TRUE(sp.arm);
  TEST_ASSERT_TRUE(parseCommand("{\"vf\":0}", sp, 7)); TEST_ASSERT_FALSE(sp.arm);             // no arm key -> disarm
}

void test_parse_rejects_garbage() {
  Setpoint sp; sp.vf = 0.4f; sp.arm = true; sp.rxMs = 42;
  TEST_ASSERT_FALSE(parseCommand("not json", sp, 100));
  TEST_ASSERT_FALSE(parseCommand("{\"arm\":1}", sp, 100));                  // vf missing
  TEST_ASSERT_FALSE(parseCommand("{\"vf\":\"fast\",\"arm\":1}", sp, 100));  // vf not numeric
  TEST_ASSERT_FALSE(parseCommand("", sp, 100));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.4f, sp.vf); TEST_ASSERT_EQUAL_UINT32(42, sp.rxMs);   // untouched: a bad packet
  TEST_ASSERT_TRUE(sp.arm);                                                              // does not refresh the failsafe
}

// ---------------------------------------------------------------- end-to-end: teleop exit and failsafe
void test_scenario_arm_drive_lose_link() {
  Setpoint sp; float m[4] = {0, 0, 0, 0}; bool armed = false;
  uint32_t now = 1000;
  for (int tick = 0; tick < 50; tick++) {                    // 1 s of commands at 20 Hz, mixer at 50 Hz
    if (tick % 3 == 0) parseCommand("{\"vf\":0.3,\"arm\":1}", sp, now);
    if (failsafeOk(sp, now)) { armed = true; mixStep(sp, 0, false, m); }
    else { armed = false; for (int k = 0; k < 4; k++) m[k] = 0; }
    now += 20;
  }
  TEST_ASSERT_TRUE(armed); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.3f, m[0]); TEST_ASSERT_FLOAT_WITHIN(EPS, 0.3f, m[1]);
  uint32_t lastCmd = sp.rxMs, tOff = 0;
  for (int tick = 0; tick < 100 && armed; tick++) {          // link lost: no more commands
    if (failsafeOk(sp, now)) { mixStep(sp, 0, false, m); }
    else { armed = false; for (int k = 0; k < 4; k++) m[k] = 0; tOff = now - lastCmd; }
    now += 20;
  }
  TEST_ASSERT_FALSE(armed);
  TEST_ASSERT_TRUE(tOff >= FAILSAFE_MS && tOff < FAILSAFE_MS + 20);   // off within one tick of the deadline
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, m[0]);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_mix_golden_sequence);
  RUN_TEST(test_mix_cap_and_yaw_priority);
  RUN_TEST(test_mix_never_exceeds_cap);
  RUN_TEST(test_mix_imu_feedback);
  RUN_TEST(test_slew_rate);
  RUN_TEST(test_yaw_integrator_winds_up_and_clamps);
  RUN_TEST(test_yaw_integrator_negative_clamp);
  RUN_TEST(test_failsafe_window);
  RUN_TEST(test_failsafe_millis_wrap);
  RUN_TEST(test_pwm_dir_mapping);
  RUN_TEST(test_parse_full_command);
  RUN_TEST(test_parse_defaults_and_clamp);
  RUN_TEST(test_parse_rejects_garbage);
  RUN_TEST(test_scenario_arm_drive_lose_link);
  return UNITY_END();
}
