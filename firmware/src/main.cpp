// Blimpy gondola firmware — ESP32-C3 SuperMini. REFLEXES ONLY.
//   WiFi/UDP command in -> IMU yaw-rate mixer -> 4 reversible motors (PWM+DIR on DRV8833) -> telemetry out.
//   Failsafe: no valid command for FAILSAFE_MS -> motors off, drivers asleep.
// Contract: ../../PROTOCOL.md   Pins: include/config.h   WiFi creds: include/secrets.h
// Compiles and runs with no sensors attached (set HAS_IMU/HAS_TOF to 0, or it just reports missing).

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "config.h"
#include "secrets.h"
#include "mixer.h"      // pure logic shared with test/test_mixer (pio test -e native)
#include "command.h"
#if HAS_TOF
#include <VL53L0X.h>
static VL53L0X tof;
static bool tofOk = false;
#endif

// ------------------------------------------------------------------ state
struct ImuState { float yaw = 0, gz = 0, pitch = 0, roll = 0, gzBias = 0; bool ok = false; };

static Setpoint  sp;
static ImuState  imu;
static float     m[4] = {0, 0, 0, 0};          // current motor outputs L, R, S, V in [-CAP, CAP]
static bool      armed = false;
static float     altM = -1;
static WiFiUDP   udp;
static IPAddress telemIp;
static bool      haveTelemIp = false;
static uint32_t  cmdCount = 0;
static bool      staMode = false;              // true = on the hotspot (reconnect on drop); false = fallback AP

// ------------------------------------------------------------------ motors: PWM + DIR (Arduino core 2.x and 3.x)
static const int      PWM_PINS[4] = {PIN_PWM_L, PIN_PWM_R, PIN_PWM_S, PIN_PWM_V};
static const int      DIR_PINS[4] = {PIN_DIR_L, PIN_DIR_R, PIN_DIR_S, PIN_DIR_V};
static const uint32_t PWM_MAX = (1u << PWM_RES_BITS) - 1;

static void pwmInit() {
  for (int i = 0; i < 4; i++) {
    pinMode(DIR_PINS[i], OUTPUT); digitalWrite(DIR_PINS[i], LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PWM_PINS[i], PWM_FREQ_HZ, PWM_RES_BITS);
#else
    ledcSetup(i, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PWM_PINS[i], i);
#endif
  }
}
static void pwmWrite(int i, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWM_PINS[i], duty);
#else
  ledcWrite(i, duty);
#endif
}
// DRV8833 with IN1 = PWM, IN2 = DIR:
//   DIR low : IN1 high drives forward, IN1 low coasts        -> forward speed = duty       (fast decay)
//   DIR high: IN1 low drives reverse, IN1 high brakes         -> reverse speed = 1 - duty  (slow decay)
static void driveMotor(int i, float v) {
  bool dirHigh; float duty;
  pwmFor(v, dirHigh, duty);                    // mapping lives in mixer.h (tested on the host)
  digitalWrite(DIR_PINS[i], dirHigh ? HIGH : LOW);
  pwmWrite(i, (uint32_t)(duty * PWM_MAX));
}
static void pwmZero() {                        // all channels coasting; does not touch nSLEEP (the LED owns it while disarmed)
  for (int i = 0; i < 4; i++) { m[i] = 0; pwmWrite(i, 0); digitalWrite(DIR_PINS[i], LOW); }
}
static void motorsOff() { pwmZero(); digitalWrite(PIN_NSLEEP, LOW); }

// ------------------------------------------------------------------ IMU: MPU6050 via raw registers
#define MPU_ADDR 0x68
static const float ACC_LSB = 8192.0f;   // +/-4 g
static const float GYR_LSB = 65.5f;     // +/-500 deg/s

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool mpuRead14(int16_t *out) {           // accel xyz, temp, gyro xyz
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  for (int i = 0; i < 7; i++) out[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}
static bool imuInit() {
  if (!mpuWrite(0x6B, 0x01)) return false;   // wake up, PLL clock
  mpuWrite(0x1A, 0x03);                       // low-pass ~44 Hz
  mpuWrite(0x1B, 0x08);                       // gyro +/-500 dps
  mpuWrite(0x1C, 0x08);                       // accel +/-4 g
  Serial.println("[imu] hold still: calibrating gyro bias for 2 s");
  double sum = 0; int n = 0; int16_t r[7]; uint32_t t0 = millis();
  while (millis() - t0 < 2000) { if (mpuRead14(r)) { sum += r[6] / GYR_LSB; n++; } delay(5); }
  if (n < 50) return false;
  imu.gzBias = sum / n;
  Serial.printf("[imu] gz bias %.3f deg/s over %d samples\n", imu.gzBias, n);
  return true;
}
static void imuStep(float dt) {
  int16_t r[7];
  if (!mpuRead14(r)) return;
  float ax = r[0] / ACC_LSB, ay = r[1] / ACC_LSB, az = r[2] / ACC_LSB;
  float gx = r[4] / GYR_LSB * DEG_TO_RAD, gy = r[5] / GYR_LSB * DEG_TO_RAD;
  float gz = (r[6] / GYR_LSB - imu.gzBias) * DEG_TO_RAD;
  imu.gz = gz;
  imu.yaw += gz * dt;
  while (imu.yaw >  PI) imu.yaw -= 2 * PI;
  while (imu.yaw <= -PI) imu.yaw += 2 * PI;
  float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float accRoll  = atan2f(ay, az);
  const float a = 0.98f;
  imu.pitch = a * (imu.pitch + gy * dt) + (1 - a) * accPitch;
  imu.roll  = a * (imu.roll  + gx * dt) + (1 - a) * accRoll;
}

static void readTof() {
#if HAS_TOF
  if (!tofOk) return;
  uint16_t mm = tof.readRangeContinuousMillimeters();
  altM = (tof.timeoutOccurred() || mm > 2000) ? -1 : mm / 1000.0f;
#endif
}

// ------------------------------------------------------------------ network
static String hostName() {                    // "blimpy-" + last 2 MAC bytes, e.g. blimpy-9910 -> reachable as blimpy-9910.local
  uint8_t mac[6]; WiFi.macAddress(mac);
  char b[16]; snprintf(b, sizeof(b), "blimpy-%02x%02x", mac[4], mac[5]);
  return String(b);
}

static void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostName().c_str());
  WiFi.setSleep(false);                       // lower latency
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);         // SuperMini clones have a bad antenna match; full power fails to connect
  Serial.printf("[wifi] connecting to '%s' ", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) { delay(250); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[wifi] connected. IP = %s   name = %s.local   <-- use the NAME in laptop/config.py ESP32_IP\n",
                  WiFi.localIP().toString().c_str(), hostName().c_str());
    if (!MDNS.begin(hostName().c_str())) Serial.println("[wifi] mDNS failed (IP still works)");
  } else {
    Serial.printf("\n[wifi] hotspot not found. Fallback AP '%s' pass '%s', my IP 192.168.4.1\n", AP_SSID, AP_PASS);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
  }
  staMode = WiFi.getMode() == WIFI_STA;
  udp.begin(CMD_PORT);
}

// Hotspot dropped (phone rebooted, laptop hotspot toggled): keep trying every 5 s instead of staying dead until a
// power cycle. Motors are already off by then (no commands -> failsafe). mDNS is restarted after each reconnect.
static void wifiTick(uint32_t now) {
  static uint32_t tRetry = 0; static bool wasUp = true;
  if (!staMode || now - tRetry < 5000) return;
  tRetry = now;
  bool up = WiFi.status() == WL_CONNECTED;
  if (!up) {
    if (wasUp) Serial.println("[wifi] link lost -> reconnecting every 5 s");
    WiFi.disconnect(false, false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  } else if (!wasUp) {
    Serial.printf("[wifi] reconnected. IP = %s\n", WiFi.localIP().toString().c_str());
    MDNS.end();
    if (!MDNS.begin(hostName().c_str())) Serial.println("[wifi] mDNS restart failed (IP still works)");
    udp.stop(); udp.begin(CMD_PORT);
  }
  wasUp = up;
}

// Status LED = the nSLEEP line (active-low LED on GPIO 8). While DISARMED the PWM outputs are all zero, so toggling
// nSLEEP only wakes/sleeps idle drivers and is safe. PROTOCOL.md section 8:
//   armed                        LED OFF (nSLEEP high, steady)
//   idle, laptop talking (<2 s)  slow blink 1 Hz
//   no commands / no WiFi        fast blink 4 Hz
static void ledTick(uint32_t now) {
  if (armed) return;
  bool talking = sp.rxMs != 0 && now - sp.rxMs < 2000;
  bool wifiUp = !staMode || WiFi.status() == WL_CONNECTED;
  uint32_t half = (talking && wifiUp) ? 500 : 125;
  bool lit = (now / half) % 2 == 0;
  digitalWrite(PIN_NSLEEP, lit ? LOW : HIGH);
}

static void pollCommands() {
  int n = udp.parsePacket();
  while (n > 0) {
    char buf[256];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len > 0 ? len : 0] = 0;
    if (parseCommand(buf, sp, millis())) {       // command.h: validates, clamps, stamps rxMs
      telemIp = udp.remoteIP(); haveTelemIp = true;
      cmdCount++;
    }
    n = udp.parsePacket();
  }
}

static float r3(float x) { return roundf(x * 1000) / 1000; }
static void sendTelemetry() {
  if (!haveTelemIp || millis() - sp.rxMs > 2000) return;   // only while someone is actually commanding us
  JsonDocument d;
  d["t"] = millis();
  d["yaw"] = r3(imu.yaw);     d["yr"] = r3(imu.gz);
  d["pitch"] = r3(imu.pitch); d["roll"] = r3(imu.roll);
  d["alt"] = r3(altM);        d["vbat"] = -1;
  d["armed"] = armed ? 1 : 0;
  d["age"] = sp.rxMs ? (int32_t)(millis() - sp.rxMs) : -1;
  d["mL"] = r3(m[0]); d["mR"] = r3(m[1]); d["mS"] = r3(m[2]); d["mV"] = r3(m[3]);
  char buf[320];
  size_t n = serializeJson(d, buf, sizeof(buf));
  udp.beginPacket(telemIp, TELEM_PORT); udp.write((uint8_t *)buf, n); udp.endPacket();
}

// ------------------------------------------------------------------ 50 Hz control tick
static void controlTick() {
  uint32_t now = millis();
  if (!failsafeOk(sp, now)) {                              // mixer.h: arm == 1 and age < FAILSAFE_MS
    if (armed) { Serial.printf("[ctl] DISARM (arm=%d age=%lu)\n", sp.arm, (unsigned long)(now - sp.rxMs)); motorsOff(); }
    armed = false; pwmZero(); mixReset();
    return;
  }
  if (!armed) { Serial.println("[ctl] ARMED"); armed = true; digitalWrite(PIN_NSLEEP, HIGH); delay(1); }
  mixStep(sp, imu.gz, imu.ok, m);                          // mixer.h: yaw-rate PI + clamp + slew (no IMU -> open loop)
  for (int i = 0; i < 4; i++) driveMotor(i, m[i]);
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[boot] blimpy gondola (4 motors, PWM+DIR)");
  pinMode(PIN_NSLEEP, OUTPUT); digitalWrite(PIN_NSLEEP, LOW);   // drivers asleep (LED on) until armed
  pwmInit(); motorsOff();
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
#if HAS_IMU
  imu.ok = imuInit();
  if (!imu.ok) Serial.println("[imu] NOT FOUND (check SDA/SCL wiring, addr 0x68). Yaw will be open-loop.");
#endif
#if HAS_TOF
  tof.setTimeout(50);
  tofOk = tof.init();
  if (tofOk) tof.startContinuous(50); else Serial.println("[tof] not found");
#endif
  wifiConnect();
}

void loop() {
  static uint32_t tImu = 0, tCtl = 0, tTel = 0, tSen = 0, tLog = 0;
  uint32_t now = millis();
  pollCommands();
  if (now - tImu >= 10) {                          // 100 Hz
    float dt = (now - tImu) * 1e-3f; if (dt > 0.1f) dt = 0.01f;
    tImu = now;
#if HAS_IMU
    if (imu.ok) imuStep(dt);
#endif
  }
  if (now - tCtl >= 20)  { tCtl = now; controlTick(); }   // 50 Hz
  if (now - tSen >= 100) { tSen = now; readTof(); }       // 10 Hz
  if (now - tTel >= 50)  { tTel = now; sendTelemetry(); } // 20 Hz
  wifiTick(now); ledTick(now);
  if (now - tLog >= 1000) {
    tLog = now;
    Serial.printf("[st] armed=%d age=%ld yaw=%+.2f gz=%+.2f m=(%+.2f %+.2f %+.2f %+.2f) alt=%.2f cmds=%lu\n",
                  armed, sp.rxMs ? (long)(now - sp.rxMs) : -1L, imu.yaw, imu.gz, m[0], m[1], m[2], m[3], altM,
                  (unsigned long)cmdCount);
  }
}
