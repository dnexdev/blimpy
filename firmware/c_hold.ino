#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <atomic>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

/*
  C_HOLD TEST VERSION

  Existing commands:
    C 40
    D -30
    E 50
    F 75
    ALL 30
    MOTORS 10 20 30 40
    STOP
    SCAN

  New commands:
    C_HOLD          Downward estimated velocity -> C 100; upward/zero -> C 0.
    C_HOLD STATUS   Print motion estimates and output to Serial.
    C_HOLD P        Read the same diagnostics over BLE (legacy command name).
    C_HOLD OFF      Stop.

  Starts with C off for a one-second stationary gravity/gyro-bias calibration.
  Vertical velocity resets to zero per run, then accumulates vertical acceleration.
  The starting readings are simply averaged; no noise scoring or rejection.
  Each control update switches directly to 100 or 0: no P tuning, ramp,
  acceleration latch, or minimum-output bias. IMU velocity can still drift.

  Only C runs during C_HOLD. D/E/F are held OFF.
  No active-run speed, tilt, rotation, acceleration-magnitude, or time cutoff.
  STOP, disconnect, invalid/failed sensor data, I2C/command errors, or a manual
  command still exits C_HOLD. Hold output is either 0 or +100%.
  C_HOLD can only start through BLE and requires the same connection throughout.

  This is experimental IMU-only vertical-motion damping, not measured
  altitude hold. An IMU cannot detect constant drift or guarantee height.
  This version has not been physically validated on your balloon.
*/

// ============================================================
// HARDWARE
// ============================================================

constexpr uint8_t MOTOR_SLAVE_ADDR = 0x12;
constexpr int SLAVE_SDA = 8;
constexpr int SLAVE_SCL = 9;

constexpr int MPU_SDA = 0;
constexpr int MPU_SCL = 1;
constexpr uint8_t MPU_ADDR = 0x68;
constexpr unsigned int SOFT_I2C_DELAY_US = 5;

constexpr int EIN1 = 2;
constexpr int EIN2 = 3;
constexpr int EPWM = 4;

constexpr int FIN1 = 10;
constexpr int FIN2 = 20;
constexpr int FPWM = 21;

// STBY remains connected to 3.3V.

// ============================================================
// BLE
// ============================================================

#define SERVICE_UUID   "12345678-1234-1234-1234-123456789000"
#define COMMAND_UUID   "12345678-1234-1234-1234-123456789001"
#define TELEMETRY_UUID "12345678-1234-1234-1234-123456789002"
#define P_REPORT_UUID  "12345678-1234-1234-1234-123456789003"

BLECharacteristic* telemetryCharacteristic = nullptr;
BLECharacteristic* pReportCharacteristic = nullptr;

std::atomic<bool> deviceConnected{false};
std::atomic<bool> connectPending{false};
std::atomic<bool> disconnectPending{false};
std::atomic<bool> commandOverflow{false};
std::atomic<bool> stopPending{false};
std::atomic<bool> telemetrySubscribed{false};
std::atomic<uint32_t> connectionGeneration{0};
std::atomic<uint32_t> receivedCommandSequence{0};
std::atomic<uint32_t> stopCommandSequence{0};
uint32_t processedCommandSequence = 0;

struct QueuedCommand {
  uint32_t generation;
  uint32_t sequence;
  char text[128];
};

QueueHandle_t commandQueue = nullptr;
bool restartAdvertisingPending = false;
unsigned long restartAdvertisingAt = 0;

constexpr unsigned long TELEMETRY_INTERVAL_MS = 200;
unsigned long lastTelemetry = 0;

// ============================================================
// MOTOR / IMU STATE
// ============================================================

int8_t motorCValue = 0;
int8_t motorDValue = 0;
float motorEValue = 0;
float motorFValue = 0;
int lastSlaveI2CStatus = -1;
int lastHoldI2CStatus = -1;
int lastAcknowledgedC = 0;

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float temperature;
};

// ============================================================
// SOFTWARE I2C — SAME MPU PINS
// ============================================================

void softSDAHigh() {
  pinMode(MPU_SDA, INPUT_PULLUP);
}

void softSDALow() {
  pinMode(MPU_SDA, OUTPUT);
  digitalWrite(MPU_SDA, LOW);
}

void softSCLHigh() {
  pinMode(MPU_SCL, INPUT_PULLUP);
}

void softSCLLow() {
  pinMode(MPU_SCL, OUTPUT);
  digitalWrite(MPU_SCL, LOW);
}

void softI2CDelay() {
  delayMicroseconds(SOFT_I2C_DELAY_US);
}

void softI2CStart() {
  softSDAHigh();
  softSCLHigh();
  softI2CDelay();
  softSDALow();
  softI2CDelay();
  softSCLLow();
  softI2CDelay();
}

void softI2CStop() {
  softSDALow();
  softI2CDelay();
  softSCLHigh();
  softI2CDelay();
  softSDAHigh();
  softI2CDelay();
}

bool softI2CWriteByte(uint8_t value) {
  for (int bit = 7; bit >= 0; --bit) {
    if (value & (1 << bit)) softSDAHigh();
    else softSDALow();

    softI2CDelay();
    softSCLHigh();
    softI2CDelay();
    softSCLLow();
    softI2CDelay();
  }

  softSDAHigh();
  softI2CDelay();
  softSCLHigh();
  softI2CDelay();

  bool ack = digitalRead(MPU_SDA) == LOW;

  softSCLLow();
  softI2CDelay();
  return ack;
}

uint8_t softI2CReadByte(bool sendAck) {
  uint8_t value = 0;
  softSDAHigh();

  for (int bit = 7; bit >= 0; --bit) {
    softSCLHigh();
    softI2CDelay();

    if (digitalRead(MPU_SDA)) value |= (1 << bit);

    softSCLLow();
    softI2CDelay();
  }

  if (sendAck) softSDALow();
  else softSDAHigh();

  softI2CDelay();
  softSCLHigh();
  softI2CDelay();
  softSCLLow();
  softI2CDelay();
  softSDAHigh();

  return value;
}

bool mpuWriteByte(uint8_t reg, uint8_t value) {
  softI2CStart();

  if (!softI2CWriteByte((MPU_ADDR << 1) | 0) ||
      !softI2CWriteByte(reg) ||
      !softI2CWriteByte(value)) {
    softI2CStop();
    return false;
  }

  softI2CStop();
  return true;
}

bool mpuReadBytes(uint8_t reg, uint8_t* buffer, size_t count) {
  softI2CStart();

  if (!softI2CWriteByte((MPU_ADDR << 1) | 0) ||
      !softI2CWriteByte(reg)) {
    softI2CStop();
    return false;
  }

  softI2CStart();

  if (!softI2CWriteByte((MPU_ADDR << 1) | 1)) {
    softI2CStop();
    return false;
  }

  for (size_t i = 0; i < count; ++i) {
    buffer[i] = softI2CReadByte(i < count - 1);
  }

  softI2CStop();
  return true;
}

bool initMPU() {
  softSDAHigh();
  softSCLHigh();
  delay(100);

  if (!mpuWriteByte(0x6B, 0x00)) return false;
  if (!mpuWriteByte(0x1C, 0x00)) return false; // +/-2g
  if (!mpuWriteByte(0x1B, 0x00)) return false; // +/-250 deg/s
  if (!mpuWriteByte(0x1A, 0x03)) return false; // Low-pass filter

  return true;
}

bool readIMU(IMUData& data) {
  uint8_t buffer[14];

  if (!mpuReadBytes(0x3B, buffer, sizeof(buffer))) return false;

  int16_t rawAx = (buffer[0] << 8) | buffer[1];
  int16_t rawAy = (buffer[2] << 8) | buffer[3];
  int16_t rawAz = (buffer[4] << 8) | buffer[5];
  int16_t rawTemp = (buffer[6] << 8) | buffer[7];
  int16_t rawGx = (buffer[8] << 8) | buffer[9];
  int16_t rawGy = (buffer[10] << 8) | buffer[11];
  int16_t rawGz = (buffer[12] << 8) | buffer[13];

  data.ax = rawAx / 16384.0f;
  data.ay = rawAy / 16384.0f;
  data.az = rawAz / 16384.0f;

  data.gx = rawGx / 131.0f;
  data.gy = rawGy / 131.0f;
  data.gz = rawGz / 131.0f;

  data.temperature = rawTemp / 340.0f + 36.53f;
  return true;
}

// ============================================================
// ORIGINAL MOTOR CONTROL
// ============================================================

int percentToPWM(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  return (int)(percent * 255.0f / 100.0f);
}

void setLocalMotor(int in1, int in2, int pwmPin, float percent) {
  percent = constrain(percent, -100.0f, 100.0f);

  if (fabs(percent) < 0.01f) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    return;
  }

  int pwm = percentToPWM(fabs(percent));

  if (percent > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  analogWrite(pwmPin, pwm);
}

void motorE(float percent) {
  motorEValue = constrain(percent, -100.0f, 100.0f);
  setLocalMotor(EIN1, EIN2, EPWM, motorEValue);
}

void motorF(float percent) {
  motorFValue = constrain(percent, -100.0f, 100.0f);
  setLocalMotor(FIN1, FIN2, FPWM, motorFValue);
}

bool sendSlaveMotors() {
  Wire.beginTransmission(MOTOR_SLAVE_ADDR);
  Wire.write((uint8_t)motorCValue);
  Wire.write((uint8_t)motorDValue);
  uint8_t error = Wire.endTransmission();
  lastSlaveI2CStatus = error;
  if (error == 0) lastAcknowledgedC = motorCValue;

  Serial.print("I2C -> slave C=");
  Serial.print(motorCValue);
  Serial.print(" D=");
  Serial.print(motorDValue);
  Serial.print(" status=");
  Serial.println(error);

  return error == 0;
}

void motorC(float percent) {
  percent = constrain(percent, -100.0f, 100.0f);
  motorCValue = (int8_t)percent;
  sendSlaveMotors();
}

void motorD(float percent) {
  percent = constrain(percent, -100.0f, 100.0f);
  motorDValue = (int8_t)percent;
  sendSlaveMotors();
}

void allOff() {
  motorE(0);
  motorF(0);
  motorCValue = 0;
  motorDValue = 0;
  sendSlaveMotors();
}

// ============================================================
// C_HOLD CONTROLLER
// ============================================================

// Direct on/off control using estimated vertical velocity.
class CHoldController {
public:
  enum Phase { OFF, BASELINE, ACTIVE };
  Phase phase = OFF;
  const char* reason = "not started";
  float output = 0;
  float velocity = 0;
  float verticalAcceleration = 0;

  bool startCalibrated(uint32_t now, const IMUData& imu) {
    phase = BASELINE;
    reason = "averaging 1 second; hold still; C off";
    output = velocity = verticalAcceleration = 0;
    phaseAt = lastSample = now;
    samples = 0;
    q[0] = 1;
    q[1] = q[2] = q[3] = 0;
    for (int i = 0; i < 3; ++i) sumA[i] = sumG[i] = bias[i] = 0;
    const float values[] = {imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz};
    for (float value : values) {
      if (!isfinite(value)) {
        stop("non-finite startup IMU");
        return false;
      }
    }
    return true;
  }

  void stop(const char* why) {
    phase = OFF;
    reason = why;
    output = 0;
  }

  bool running() const { return phase != OFF; }

  const char* stateName() const {
    if (phase == BASELINE) return "CALIBRATING";
    if (phase == ACTIVE) return "ON/OFF DAMPING";
    return "OFF";
  }

  void update(uint32_t now, const IMUData& imu) {
    if (!running()) return;
    float a[3] = {imu.ax * 9.80665f, imu.ay * 9.80665f, imu.az * 9.80665f};
    float g[3] = {imu.gx * DEG_TO_RAD, imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD};
    for (int i = 0; i < 3; ++i) {
      if (!isfinite(a[i]) || !isfinite(g[i])) {
        stop("non-finite IMU");
        return;
      }
    }
    uint32_t elapsed = now - lastSample;
    if (elapsed == 0) return;
    lastSample = now;
    if (phase == BASELINE) {
      collectBaseline(now, a, g);
      return;
    }

    float dt = elapsed * 0.001f;
    for (int i = 0; i < 3; ++i) g[i] -= bias[i];
    integrateOrientation(g, dt);
    float worldA[3];
    rotate(q, a, worldA);
    verticalAcceleration = worldA[2] - gravity;
    velocity += verticalAcceleration * dt;
    output = velocity < 0 ? 100.0f : 0.0f;
    reason = velocity < 0 ? "downward estimate; C 100"
                          : "upward/zero estimate; C 0";
  }

private:
  uint32_t phaseAt = 0;
  uint32_t lastSample = 0;
  unsigned samples = 0;
  float sumA[3] = {};
  float sumG[3] = {};
  float bias[3] = {};
  float q[4] = {1, 0, 0, 0};
  float gravity = 9.80665f;

  static void multiply(const float* a, const float* b, float* r) {
    r[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    r[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    r[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    r[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
  }

  static void rotate(const float* rotation, const float* v, float* r) {
    float tx = 2 * (rotation[2]*v[2] - rotation[3]*v[1]);
    float ty = 2 * (rotation[3]*v[0] - rotation[1]*v[2]);
    float tz = 2 * (rotation[1]*v[1] - rotation[2]*v[0]);

    r[0] = v[0] + rotation[0]*tx + rotation[2]*tz - rotation[3]*ty;
    r[1] = v[1] + rotation[0]*ty + rotation[3]*tx - rotation[1]*tz;
    r[2] = v[2] + rotation[0]*tz + rotation[1]*ty - rotation[2]*tx;
  }

  void integrateOrientation(const float* g, float dt) {
    float rate[4] = {0, g[0], g[1], g[2]};
    float derivative[4];
    multiply(q, rate, derivative);

    float magnitude = 0;
    for (int i = 0; i < 4; ++i) {
      q[i] += 0.5f * dt * derivative[i];
      magnitude += q[i] * q[i];
    }

    magnitude = sqrtf(magnitude);
    for (int i = 0; i < 4; ++i) q[i] /= magnitude;
  }

  void collectBaseline(uint32_t now, const float* a, const float* g) {
    for (int i = 0; i < 3; ++i) {
      sumA[i] += a[i];
      sumG[i] += g[i];
    }
    ++samples;
    if ((uint32_t)(now - phaseAt) < 1000) return;
    float averageA[3];
    for (int i = 0; i < 3; ++i) {
      averageA[i] = sumA[i] / samples;
      bias[i] = sumG[i] / samples;
    }
    gravity = sqrtf(
      averageA[0] * averageA[0] +
      averageA[1] * averageA[1] +
      averageA[2] * averageA[2]
    );

    if (!isfinite(gravity) || gravity < 0.001f) {
      stop("invalid calibration gravity reference");
      return;
    }

    q[0] = sqrtf(fmaxf(0, (1 + averageA[2] / gravity) * 0.5f));

    if (q[0] < 0.001f) {
      q[0] = 0;
      q[1] = 1;
      q[2] = q[3] = 0;
    } else {
      q[1] = averageA[1] / (2 * gravity * q[0]);
      q[2] = -averageA[0] / (2 * gravity * q[0]);
      q[3] = 0;
    }


    // Calibration defines stationary velocity; retain it across active updates.
    // No release delay or motor-response probes follow this baseline.
    phase = ACTIVE;
    phaseAt = lastSample = now;
    velocity = 0;
    verticalAcceleration = 0;
    reason = "calibrated; release; vertical velocity starts at zero";
    output = 0;
  }

};

CHoldController cHold;
uint32_t cHoldGeneration = 0;
uint32_t lastHoldRead = 0;
uint32_t holdIMUSamples = 0;
bool holdOwnsMotors = false;
bool holdOutputValid = false;

// ============================================================
// C_HOLD HARDWARE / COMMAND HELPERS
// ============================================================

void publishPReport(bool printToSerial) {
  char report[512];
  snprintf(report, sizeof(report),
    "fw=hold-ack1; rx=%lu; done=%lu; tick=%lu; ble=%d; "
    "mode=ON_OFF; vz_est=%.3f; az_est=%.3f; "
    "phase=%s; reason=%s; hold IMU reads=%lu; "
    "C_request=%.1f; C_packet=%d; C_last_ACK=%d; "
    "I2C=%d; hold_I2C=%d; %s",
    (unsigned long)receivedCommandSequence.load(),
    (unsigned long)processedCommandSequence, (unsigned long)millis(),
    (int)deviceConnected.load(),
    cHold.velocity, cHold.verticalAcceleration,
    cHold.stateName(), cHold.reason,
    (unsigned long)holdIMUSamples,
    cHold.output, (int)motorCValue, lastAcknowledgedC,
    lastSlaveI2CStatus, lastHoldI2CStatus,
    cHold.running() ? "running" : "stopped");
  if (printToSerial) Serial.println(report);
  if (pReportCharacteristic) pReportCharacteristic->setValue(report);
}

void printCHoldStatus() {
  Serial.printf(
    "C_HOLD: %s | %s | C=%.1f%% | vz_est=%.3f m/s | "
    "az_est=%.3f m/s^2\n",
    cHold.stateName(),
    cHold.reason,
    cHold.output,
    cHold.velocity,
    cHold.verticalAcceleration
  );
}

void stopCHold(const char* reason) {
  bool wasRunning = cHold.running() || holdOwnsMotors;
  cHold.stop(reason);
  holdOwnsMotors = false;
  holdOutputValid = false;

  if (wasRunning) {
    allOff();
    printCHoldStatus();
    publishPReport(false);
  }
}

bool holdLinkValid() {
  return deviceConnected.load() &&
         !disconnectPending.load() &&
         !stopPending.load() &&
         !commandOverflow.load() &&
         cHoldGeneration == connectionGeneration.load();
}

bool writeCHoldOutput() {
  if (!cHold.running() || !holdLinkValid() || !isfinite(cHold.output)) return false;

  // Hardware boundary: this mode can command ONLY C.
  if (!holdOutputValid || motorEValue != 0) motorE(0);
  if (!holdOutputValid || motorFValue != 0) motorF(0);

  float requestedC = constrain(cHold.output, 0.0f, 100.0f);
  int8_t nextC = (int8_t)requestedC;

  if (!holdOutputValid || nextC != motorCValue || motorDValue != 0) {
    motorCValue = nextC;
    motorDValue = 0;

    // Quiet transaction: avoid printing at the control-loop rate.
    Wire.beginTransmission(MOTOR_SLAVE_ADDR);
    Wire.write((uint8_t)motorCValue);
    Wire.write((uint8_t)0);

    lastHoldI2CStatus = Wire.endTransmission();
    lastSlaveI2CStatus = lastHoldI2CStatus;
    if (lastHoldI2CStatus != 0) return false;
    lastAcknowledgedC = motorCValue;
  }

  // STOP/disconnect may have arrived during the transaction.
  if (!holdLinkValid()) return false;

  holdOutputValid = true;
  return true;
}

void serviceCHold() {
  if (!cHold.running()) return;

  if (!holdLinkValid()) {
    stopCHold("BLE/command fault");
    return;
  }

  uint32_t now = millis();
  if ((uint32_t)(now - lastHoldRead) < 20) return;
  lastHoldRead = now;

  IMUData imu;
  if (!readIMU(imu)) {
    stopCHold("IMU read failed");
    return;
  }
  ++holdIMUSamples;

  CHoldController::Phase previousPhase = cHold.phase;
  cHold.update(millis(), imu);

  if (!cHold.running()) {
    stopCHold(cHold.reason);
    return;
  }

  if (!writeCHoldOutput()) {
    // C/D stopping still depends on a working I2C link to the slave.
    stopCHold("I2C/output/link failure");
    return;
  }

  // Print transitions only, not a continuous stream.
  if (cHold.phase != previousPhase) printCHoldStatus();
}

bool handleCHoldCommand(const String& command, bool fromBLE) {
  if (command == "C_HOLD P") {
    publishPReport(true);
    return true;
  }
  if (command == "C_HOLD STATUS") {
    printCHoldStatus();
    return true;
  }

  if (command == "C_HOLD OFF" || command == "C_HOLD CANCEL") {
    stopCHold("cancelled");
    return true;
  }

  if (command == "C_HOLD" || command.startsWith("C_HOLD ")) {
    if (!fromBLE) {
      Serial.println("C_HOLD must be started over BLE");
      return true;
    }
    if (cHold.running()) {
      Serial.println("C_HOLD already running; STOP first");
      return true;
    }

    float maximum = 100;
    const char* p = command.c_str() + 6;
    while (*p && isspace((unsigned char)*p)) ++p;

    if (*p) {
      char* end = nullptr;
      maximum = strtof(p, &end);

      if (end == p) {
        Serial.println("Use C_HOLD (fixed 0/100 output)");
        return true;
      }

      while (*end && isspace((unsigned char)*end)) ++end;

      if (*end || !isfinite(maximum) || maximum != 100) {
        Serial.println("Use C_HOLD (fixed 0/100 output)");
        return true;
      }
    }

    if (!deviceConnected.load() ||
        disconnectPending.load() ||
        stopPending.load() ||
        commandOverflow.load()) {
      cHold.stop("connection lost or STOP/command fault pending");
      publishPReport(false);
      Serial.println("C_HOLD rejected: connection lost or STOP/command fault pending");
      return true;
    }

    allOff();
    xQueueReset(commandQueue);
    holdIMUSamples = 0;
    lastHoldI2CStatus = -1;

    // Fail before calibration with an actionable explanation. C lives on the
    // slave, so a working master BLE connection alone cannot make it move.
    Wire.beginTransmission(MOTOR_SLAVE_ADDR);
    uint8_t slaveStatus = Wire.endTransmission();
    lastSlaveI2CStatus = slaveStatus;
    if (slaveStatus != 0) {
      cHold.stop("motor slave 0x12 not responding");
      publishPReport(false);
      Serial.printf("C_HOLD rejected: slave 0x12 I2C status=%u; check slave power, SDA/SCL and common GND\n", slaveStatus);
      return true;
    }
    IMUData imu;
    if (!readIMU(imu)) {
      cHold.stop("IMU read failed before calibration");
      printCHoldStatus();
      publishPReport(false);
      return true;
    }
    ++holdIMUSamples;

    cHoldGeneration = connectionGeneration.load();
    if (!cHold.startCalibrated(millis(), imu)) {
      printCHoldStatus();
      publishPReport(false);
      return true;
    }
    lastHoldRead = millis();
    holdOwnsMotors = true;
    holdOutputValid = false;

    if (!writeCHoldOutput()) {
      stopCHold("I2C/output/link failure at startup");
      return true;
    }
    Serial.printf("C_HOLD: hold still for 1 second; then release. Downward = %.0f%%; upward/zero = off\n", maximum);
    printCHoldStatus();
    publishPReport(false);
    return true;
  }

  return false;
}

// ============================================================
// ORIGINAL SCAN / COMMAND INTERFACE
// ============================================================

void scanMotorSlave() {
  Serial.println("Scanning hardware I2C...");
  bool found = false;

  for (uint8_t addr = 1; addr < 127; ++addr) {
    if (disconnectPending.load() || stopPending.load()) return;

    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found I2C device at 0x");
      if (addr < 0x10) Serial.print("0");
      Serial.println(addr, HEX);
      found = true;
    }
  }

  if (!found) Serial.println("No hardware I2C devices found");
}

void processCommand(String command, bool fromBLE) {
  command.trim();
  command.toUpperCase();
  if (command.length() == 0) return;

  if (handleCHoldCommand(command, fromBLE)) return;

  // A manual command takes ownership back from C_HOLD.
  if (cHold.running()) {
    stopCHold(command == "STOP" ? "stopped" : "manual override");
  }

  Serial.print("Command: ");
  Serial.println(command);

  if (command == "STOP") {
    allOff();
    Serial.println("ALL MOTORS OFF");
    return;
  }

  if (command == "SCAN") {
    scanMotorSlave();
    return;
  }

  if (command.startsWith("C ")) {
    motorC(command.substring(2).toFloat());
    return;
  }

  if (command.startsWith("D ")) {
    motorD(command.substring(2).toFloat());
    return;
  }

  if (command.startsWith("E ")) {
    motorE(command.substring(2).toFloat());
    return;
  }

  if (command.startsWith("F ")) {
    motorF(command.substring(2).toFloat());
    return;
  }

  if (command.startsWith("ALL ")) {
    float p = command.substring(4).toFloat();

    motorCValue = (int8_t)constrain(p, -100.0f, 100.0f);
    motorDValue = motorCValue;
    sendSlaveMotors();

    motorE(p);
    motorF(p);
    return;
  }

  if (command.startsWith("MOTORS ")) {
    float values[4] = {0, 0, 0, 0};
    String remaining = command.substring(7);

    for (int i = 0; i < 4; ++i) {
      int space = remaining.indexOf(' ');

      if (space == -1) {
        values[i] = remaining.toFloat();
        break;
      }

      values[i] = remaining.substring(0, space).toFloat();
      remaining = remaining.substring(space + 1);
    }

    motorCValue = (int8_t)constrain(values[0], -100.0f, 100.0f);
    motorDValue = (int8_t)constrain(values[1], -100.0f, 100.0f);
    sendSlaveMotors();

    motorE(values[2]);
    motorF(values[3]);
    return;
  }

  Serial.println("Unknown command");
}

// ============================================================
// BLE CALLBACKS — NO MOTOR / I2C WORK HERE
// ============================================================

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    connectionGeneration.fetch_add(1);
    deviceConnected.store(true);
    connectPending.store(true);
  }

  void onDisconnect(BLEServer* server) override {
    deviceConnected.store(false);
    telemetrySubscribed.store(false);
    connectionGeneration.fetch_add(1);
    disconnectPending.store(true);
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    uint32_t sequence = receivedCommandSequence.fetch_add(1) + 1;
    if (!deviceConnected.load() || commandQueue == nullptr) return;

    auto value = characteristic->getValue();
    if (value.length() == 0) return;

    QueuedCommand command{};

    if (value.length() >= sizeof(command.text)) {
      commandOverflow.store(true);
      return;
    }

    for (size_t i = 0; i < value.length(); ++i) {
      if (value[i] == '\0') {
        commandOverflow.store(true);
        return;
      }
    }

    String normalized(value.c_str());
    normalized.trim();
    normalized.toUpperCase();

    if (normalized == "STOP") {
      stopCommandSequence.store(sequence);
      stopPending.store(true);
      return;
    }

    command.generation = connectionGeneration.load();
    command.sequence = sequence;
    memcpy(command.text, value.c_str(), value.length());

    if (xQueueSend(commandQueue, &command, 0) != pdTRUE) {
      commandOverflow.store(true);
    }
  }
};

class TelemetryCallbacks : public BLECharacteristicCallbacks {
#if defined(CONFIG_NIMBLE_ENABLED)
  void onSubscribe(
    BLECharacteristic* characteristic,
    ble_gap_conn_desc* connection,
    uint16_t value
  ) override {
    telemetrySubscribed.store((value & 1) != 0);
  }
#endif
};

void handleBLEEvents() {
  if (stopPending.exchange(false)) {
    xQueueReset(commandQueue);

    bool wasHolding = cHold.running() || holdOwnsMotors;
    stopCHold("STOP");
    if (!wasHolding) allOff();

    processedCommandSequence = stopCommandSequence.load();
    publishPReport(false);
    Serial.println("ALL MOTORS OFF");
  }

  if (disconnectPending.exchange(false)) {
    xQueueReset(commandQueue);

    bool wasHolding = cHold.running() || holdOwnsMotors;
    stopCHold("BLE disconnected");
    if (!wasHolding) allOff();

    Serial.println("BLE disconnected -> ALL MOTORS OFF");
    restartAdvertisingPending = true;
    restartAdvertisingAt = millis() + 200;
  }

  if (commandOverflow.exchange(false)) {
    xQueueReset(commandQueue);

    bool wasHolding = cHold.running() || holdOwnsMotors;
    stopCHold("command overflow");
    if (!wasHolding) allOff();

    Serial.println("BLE command invalid/too long/queue full -> ALL MOTORS OFF");
  }

  if (connectPending.exchange(false)) {
    Serial.println("BLE connected");
  }

  if (restartAdvertisingPending &&
      (int32_t)(millis() - restartAdvertisingAt) >= 0) {
    restartAdvertisingPending = false;

    if (!deviceConnected.load()) {
      BLEDevice::startAdvertising();
      Serial.println("BLE advertising restart requested");
    }
  }
}

void handleSerialCommands() {
  static char buffer[128];
  static size_t length = 0;
  static bool overflow = false;

  for (int count = 0; count < 64 && Serial.available(); ++count) {
    char ch = (char)Serial.read();

    if (ch == '\r' || ch == '\n') {
      if (overflow) {
        stopCHold("Serial command overflow");
        Serial.println("Serial command too long; discarded");
      } else if (length) {
        buffer[length] = '\0';
        processCommand(String(buffer), false);
      }

      length = 0;
      overflow = false;
      return;
    }

    if (!overflow) {
      if (length < sizeof(buffer) - 1) buffer[length++] = ch;
      else overflow = true;
    }
  }
}

void initBLE() {
  BLEDevice::init("BalloonRobot");
  BLEDevice::setMTU(185);

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  BLECharacteristic* commandCharacteristic =
    service->createCharacteristic(
      COMMAND_UUID,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR
    );

  commandCharacteristic->setCallbacks(new CommandCallbacks());

  telemetryCharacteristic = service->createCharacteristic(
    TELEMETRY_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  telemetryCharacteristic->addDescriptor(new BLE2902());
  telemetryCharacteristic->setCallbacks(new TelemetryCallbacks());
  telemetryCharacteristic->setValue("starting");

  pReportCharacteristic = service->createCharacteristic(
    P_REPORT_UUID, BLECharacteristic::PROPERTY_READ
  );
  publishPReport(false);
  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising as BalloonRobot");
}

// ============================================================
// TELEMETRY — ORIGINAL FORMAT, 200 ms
// ============================================================

void sendTelemetry() {
  if (!deviceConnected.load() || telemetryCharacteristic == nullptr) return;

#if defined(CONFIG_NIMBLE_ENABLED)
  if (!telemetrySubscribed.load()) return;
#else
  BLE2902* subscription = static_cast<BLE2902*>(
    telemetryCharacteristic->getDescriptorByUUID((uint16_t)0x2902)
  );
  if (subscription == nullptr || !subscription->getNotifications()) return;
#endif

  IMUData imu;

  if (!readIMU(imu)) {
    telemetryCharacteristic->setValue("IMU_ERROR");
    telemetryCharacteristic->notify();
    return;
  }

  char buffer[140];

  snprintf(
    buffer,
    sizeof(buffer),
    "A:%.3f,%.3f,%.3f;G:%.2f,%.2f,%.2f;T:%.1f",
    imu.ax, imu.ay, imu.az,
    imu.gx, imu.gy, imu.gz,
    imu.temperature
  );

  telemetryCharacteristic->setValue(
    (uint8_t*)buffer,
    strlen(buffer)
  );
  telemetryCharacteristic->notify();
}

// ============================================================
// SETUP / LOOP
// ============================================================

void printResetReason() {
  esp_reset_reason_t reset = esp_reset_reason();
  const char* label = "OTHER";

  switch (reset) {
    case ESP_RST_POWERON:  label = "POWER ON"; break;
    case ESP_RST_SW:       label = "SOFTWARE"; break;
    case ESP_RST_PANIC:    label = "PANIC / CRASH"; break;
    case ESP_RST_INT_WDT:  label = "INTERRUPT WATCHDOG"; break;
    case ESP_RST_TASK_WDT: label = "TASK WATCHDOG"; break;
    case ESP_RST_WDT:      label = "WATCHDOG"; break;
    case ESP_RST_BROWNOUT: label = "BROWNOUT"; break;
    default: break;
  }

  Serial.printf("RESET REASON: %s (%d)\n", label, (int)reset);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  printResetReason();

  pinMode(EIN1, OUTPUT);
  pinMode(EIN2, OUTPUT);
  pinMode(EPWM, OUTPUT);
  pinMode(FIN1, OUTPUT);
  pinMode(FIN2, OUTPUT);
  pinMode(FPWM, OUTPUT);

  motorE(0);
  motorF(0);

  Wire.begin(SLAVE_SDA, SLAVE_SCL, 100000);
  Wire.setTimeOut(25);

  Serial.println("Hardware I2C master started");
  Serial.printf("SDA = GPIO%d, SCL = GPIO%d\n", SLAVE_SDA, SLAVE_SCL);

  Wire.beginTransmission(MOTOR_SLAVE_ADDR);
  uint8_t status = Wire.endTransmission();
  Serial.printf("Motor slave 0x12 status: %u\n", status);

  allOff();

  Serial.println("Starting MPU6050 software I2C...");
  Serial.println(initMPU() ? "MPU6050 OK" : "MPU6050 FAILED");

  commandQueue = xQueueCreate(12, sizeof(QueuedCommand));

  if (commandQueue == nullptr) {
    allOff();
    Serial.println("Cannot allocate BLE command queue");
    while (true) delay(1000);
  }

  initBLE();

  Serial.println();
  Serial.println("MASTER READY");
  Serial.println("C/D -> I2C slave; E/F -> local driver");
  Serial.println("IMU BLE telemetry: 200 ms when subscribed");
  Serial.println("Commands: C/D/E/F percent, ALL, MOTORS, STOP, SCAN");
  Serial.println("New: C_HOLD, C_HOLD STATUS, C_HOLD P, C_HOLD OFF");
}

void loop() {
  handleBLEEvents();

  QueuedCommand command{};

  if (xQueueReceive(commandQueue, &command, 0) == pdTRUE &&
      deviceConnected.load() &&
      !disconnectPending.load() &&
      !stopPending.load() &&
      command.generation == connectionGeneration.load()) {
    processCommand(String(command.text), true);
    processedCommandSequence = command.sequence;
    publishPReport(false);
  }

  handleBLEEvents();
  handleSerialCommands();
  handleBLEEvents();

  serviceCHold();

  unsigned long now = millis();

  if (now - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
    lastTelemetry = now;
    publishPReport(false);
    sendTelemetry();
  }

  delay(1);
}
