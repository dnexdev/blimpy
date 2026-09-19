#pragma once
// ---------------- Pins (ESP32-C3 SuperMini) — verify against silkscreen ----------------
// Four reversible motors, each driven as PWM + DIR on one DRV8833 channel (IN1 = PWM, IN2 = DIR).
// The C3 has only 6 PWM channels, so 4 motors x 2 PWM inputs is not possible; PWM+DIR needs 4 PWM + 4 GPIO.
//   Board A: AIN1/AIN2 = motor L (PWM_L/DIR_L)   BIN1/BIN2 = motor R
//   Board B: AIN1/AIN2 = motor S                 BIN1/BIN2 = motor V
#define PIN_PWM_L    0
#define PIN_PWM_R    1
#define PIN_PWM_S    3
#define PIN_PWM_V    10
#define PIN_DIR_L    4
#define PIN_DIR_R    5
#define PIN_DIR_S    20
#define PIN_DIR_V    21
#define PIN_SDA      6
#define PIN_SCL      7
#define PIN_NSLEEP   8      // DRV8833 STBY/EEP/nSLEEP on BOTH boards. HIGH = drivers awake.
                            // Shares the onboard LED (active LOW). Armed = LED off; disarmed = blinking
                            // (slow: laptop talking, fast: no commands / no WiFi). See ledTick() in main.cpp.
// Vbat sense: no pin left. Telemetry vbat = -1. (If wanted later: GPIO 2 via 100k/100k, it is a strapping pin.)

// ---------------- Network ----------------
#define CMD_PORT     5005
#define TELEM_PORT   5006
#define WIFI_TIMEOUT_MS 40000
#define AP_SSID      "wisp-gondola"   // fallback access point if the hotspot is not found
#define AP_PASS      "wispwisp"

// ---------------- Control (must match laptop/control/protocol.py) ----------------
#define CAP            0.5f    // max motor duty
#define K_YR           1.0f    // yaw-rate P gain
#define KI_YR          1.0f    // yaw-rate I gain per second (cancels steady torques, e.g. the S motor off-centre)
#define I_YR_MAX       0.15f   // yaw integrator clamp (motor duty)
#define YR_MAX_RAD_S   1.0f    // yr = 1.0 means this many rad/s
#define SLEW_PER_TICK  0.05f   // per 50 Hz tick
#define FAILSAFE_MS    500
#define PWM_FREQ_HZ    25000   // above hearing
#define PWM_RES_BITS   10

// ---------------- Hardware present? (set 0 to bench-test without it) ----------------
#define HAS_IMU  0
#define HAS_TOF  0
