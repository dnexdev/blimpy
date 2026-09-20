#include <Arduino.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>


// ============================================================
// HARDWARE I2C -> MOTOR SLAVE
// ============================================================

constexpr uint8_t MOTOR_SLAVE_ADDR = 0x12;

constexpr int SLAVE_SDA = 8;
constexpr int SLAVE_SCL = 9;


// ============================================================
// SOFTWARE I2C -> MPU6050
//
// ESP32-C3 only has one hardware I2C peripheral.
// Hardware Wire is being used for the motor slave on 8/9.
//
// Therefore MPU6050 on GPIO0/1 is implemented with software I2C.
// ============================================================

constexpr int MPU_SDA = 0;
constexpr int MPU_SCL = 1;

constexpr uint8_t MPU_ADDR = 0x68;


// ============================================================
// LOCAL MOTOR PINS
// ============================================================

// Motor E
constexpr int EIN1 = 2;
constexpr int EIN2 = 3;
constexpr int EPWM = 4;

// Motor F
constexpr int FIN1 = 10;
constexpr int FIN2 = 20;
constexpr int FPWM = 21;

// Motor driver STBY -> 3.3V


// ============================================================
// BLE UUIDS
// ============================================================

#define SERVICE_UUID \
  "12345678-1234-1234-1234-123456789000"

#define COMMAND_UUID \
  "12345678-1234-1234-1234-123456789001"

#define TELEMETRY_UUID \
  "12345678-1234-1234-1234-123456789002"


BLECharacteristic* telemetryCharacteristic = nullptr;

bool deviceConnected = false;


// ============================================================
// MOTOR STATE
//
// C/D live on slave.
// We retain their state on the master because the slave packet
// always contains BOTH C and D values.
// ============================================================

int8_t motorCValue = 0;
int8_t motorDValue = 0;

float motorEValue = 0;
float motorFValue = 0;


// ============================================================
// IMU DATA
// ============================================================

struct IMUData {
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  float temperature;
};


// ============================================================
// ONBOARD FLIGHT MIXER  (Blimpy, 2026-09-20)
//
// The laptop used to run the mixer itself and write MOTORS 20
// times a second, closing the yaw loop on the IMU stream. With
// telemetry every 200 ms that loop is blind, so the fast part
// moved here. The laptop now sends SETPOINTS:
//
//   CMD vf vs yr vz      percent, -100..100
//                        vf forward   vs sideways (+ = left)
//                        yr yaw rate  (+ = counter-clockwise seen
//                           from above, 100 = 1 rad/s)
//                        vz up
//   MAP LC+ RD- SE+ VF+ G+
//                        which motor LETTER plays which ROLE:
//                        L/R rear left/right, S sideways, V vertical.
//                        Sign = direction for a positive command.
//                        G = gyro sign (G- if turning CCW gives a
//                        negative gz). Saved in flash; the laptop
//                        also sends it on every connection.
//   MAP                  print the map     STATUS  print everything
//
// Every 20 ms: read the gyro, yaw-rate PI -> L/R difference, slew,
// write the four motors. No CMD for CMD_TIMEOUT_MS -> motors off.
// The bench commands (C 30 / MOTORS / ALL / STOP) keep working:
// any of them switches the mixer off first.
// Same maths as laptop/control/protocol.py::mix (PROTOCOL.md s7).
// ============================================================

constexpr float MIX_CAP    = 0.5f;    // max duty after mixing (50 %)
constexpr float MIX_TOTAL  = 1.2f;    // max SUM of |duty| over the four motors: they share one supply, and four
                                      // at 0.4 together sag it toward a brown-out. Over budget = all scaled down
constexpr float MIX_K_YR   = 1.0f;    // yaw-rate P gain
constexpr float MIX_KI_YR  = 1.0f;    // yaw-rate I gain, per second
constexpr float MIX_I_MAX  = 0.15f;   // integrator clamp (duty)
constexpr float MIX_SLEW   = 0.05f;   // max duty change per tick (0 -> 0.5 in 0.2 s)
constexpr float MIX_DT     = 0.02f;   // s, control tick
constexpr float YR_MAX_DPS = 57.2958f;  // yr = 100 means 1 rad/s

constexpr unsigned long CONTROL_INTERVAL_MS = 20;
constexpr unsigned long CMD_TIMEOUT_MS      = 500;   // no CMD for this long -> motors off
constexpr unsigned long SLAVE_HEARTBEAT_MS  = 200;   // C/D resent at least this often (the slave stops at 600 ms)
constexpr unsigned long ADVERTISE_EVERY_MS  = 3000;  // while nobody is connected, re-assert advertising

// role -> motor letter (0 = C, 1 = D, 2 = E, 3 = F) and sign
int8_t roleLetter[4] = {0, 1, 2, 3};   // L R S V
int8_t roleSign[4]   = {1, 1, 1, 1};
int8_t gyroSign      = 1;

bool  cmdMode     = false;   // true after a CMD; false after any raw motor command, STOP or a link loss
bool  cmdTimedOut = false;
unsigned long lastCmdMs = 0;
float sp[4]  = {0, 0, 0, 0};  // setpoints vf vs yr vz, -1..1
float cur[4] = {0, 0, 0, 0};  // current duties L R S V, -1..1
float yawI   = 0;

IMUData lastImu;
bool  imuOk  = false;
float yawDeg = 0;      // heading integrated here at 50 Hz, CCW positive, wrapped to +-180
float gzDps  = 0;      // gyro z after zero and sign, deg/s
float gzBias = 0;      // gyro zero, learnt while the motors are off and the box is still

unsigned long lastControl   = 0;
unsigned long lastSlaveSend = 0;
unsigned long lastAdvertise = 0;
uint8_t slaveStatus = 255;

Preferences prefs;

// BLE writes are queued here and handled in loop(): the Bluetooth
// stack has its own task, and the motors / I2C must be touched
// from one place only.
constexpr int CMD_QUEUE_N = 8;
char cmdQueue[CMD_QUEUE_N][80];
volatile int cmdHead = 0;
volatile int cmdTail = 0;
volatile bool linkLost = false;


// ============================================================
// SOFTWARE I2C HELPERS
// ============================================================

constexpr unsigned int SOFT_I2C_DELAY_US = 5;


void softSDAHigh() {
  // Open-drain behavior:
  // releasing the line lets the pull-up make it HIGH.
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
  delayMicroseconds(
    SOFT_I2C_DELAY_US
  );
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


// ============================================================
// SOFTWARE I2C WRITE BYTE
//
// Returns true if slave ACKs.
// ============================================================

bool softI2CWriteByte(
  uint8_t value
) {
  for (
    int bit = 7;
    bit >= 0;
    bit--
  ) {
    if (
      value &
      (1 << bit)
    ) {
      softSDAHigh();
    } else {
      softSDALow();
    }

    softI2CDelay();

    softSCLHigh();

    softI2CDelay();

    softSCLLow();

    softI2CDelay();
  }


  // Release SDA for ACK
  softSDAHigh();

  softI2CDelay();

  softSCLHigh();

  softI2CDelay();

  bool ack =
    digitalRead(
      MPU_SDA
    ) == LOW;

  softSCLLow();

  softI2CDelay();

  return ack;
}


// ============================================================
// SOFTWARE I2C READ BYTE
// ============================================================

uint8_t softI2CReadByte(
  bool sendAck
) {
  uint8_t value = 0;

  softSDAHigh();


  for (
    int bit = 7;
    bit >= 0;
    bit--
  ) {
    softSCLHigh();

    softI2CDelay();

    if (
      digitalRead(
        MPU_SDA
      )
    ) {
      value |=
        (1 << bit);
    }

    softSCLLow();

    softI2CDelay();
  }


  // ACK = SDA LOW
  // NACK = SDA HIGH

  if (sendAck) {
    softSDALow();
  } else {
    softSDAHigh();
  }

  softI2CDelay();

  softSCLHigh();

  softI2CDelay();

  softSCLLow();

  softI2CDelay();

  softSDAHigh();

  return value;
}


// ============================================================
// MPU REGISTER WRITE
// ============================================================

bool mpuWriteByte(
  uint8_t reg,
  uint8_t value
) {
  softI2CStart();


  if (
    !softI2CWriteByte(
      (MPU_ADDR << 1) | 0
    )
  ) {
    softI2CStop();
    return false;
  }


  if (
    !softI2CWriteByte(
      reg
    )
  ) {
    softI2CStop();
    return false;
  }


  if (
    !softI2CWriteByte(
      value
    )
  ) {
    softI2CStop();
    return false;
  }


  softI2CStop();

  return true;
}


// ============================================================
// MPU MULTI-BYTE READ
// ============================================================

bool mpuReadBytes(
  uint8_t reg,
  uint8_t* buffer,
  size_t count
) {
  softI2CStart();


  // Write address
  if (
    !softI2CWriteByte(
      (MPU_ADDR << 1) | 0
    )
  ) {
    softI2CStop();
    return false;
  }


  // Register
  if (
    !softI2CWriteByte(
      reg
    )
  ) {
    softI2CStop();
    return false;
  }


  // Repeated START
  softI2CStart();


  // Read address
  if (
    !softI2CWriteByte(
      (MPU_ADDR << 1) | 1
    )
  ) {
    softI2CStop();
    return false;
  }


  for (
    size_t i = 0;
    i < count;
    i++
  ) {
    bool ack =
      i < (
        count - 1
      );

    buffer[i] =
      softI2CReadByte(
        ack
      );
  }


  softI2CStop();

  return true;
}


// ============================================================
// MPU INIT
// ============================================================

bool initMPU() {
  // Release software I2C lines
  softSDAHigh();
  softSCLHigh();

  delay(100);


  // Wake MPU6050
  if (
    !mpuWriteByte(
      0x6B,
      0x00
    )
  ) {
    return false;
  }


  // Accelerometer +/-2g
  mpuWriteByte(
    0x1C,
    0x00
  );


  // Gyroscope +/-250 deg/s
  mpuWriteByte(
    0x1B,
    0x00
  );


  // Digital low-pass filter
  mpuWriteByte(
    0x1A,
    0x03
  );


  return true;
}


// ============================================================
// READ MPU
// ============================================================

bool readIMU(
  IMUData& data
) {
  uint8_t buffer[14];


  if (
    !mpuReadBytes(
      0x3B,
      buffer,
      14
    )
  ) {
    return false;
  }


  int16_t rawAx =
    (buffer[0] << 8) |
    buffer[1];

  int16_t rawAy =
    (buffer[2] << 8) |
    buffer[3];

  int16_t rawAz =
    (buffer[4] << 8) |
    buffer[5];

  int16_t rawTemp =
    (buffer[6] << 8) |
    buffer[7];

  int16_t rawGx =
    (buffer[8] << 8) |
    buffer[9];

  int16_t rawGy =
    (buffer[10] << 8) |
    buffer[11];

  int16_t rawGz =
    (buffer[12] << 8) |
    buffer[13];


  data.ax =
    rawAx / 16384.0f;

  data.ay =
    rawAy / 16384.0f;

  data.az =
    rawAz / 16384.0f;


  data.gx =
    rawGx / 131.0f;

  data.gy =
    rawGy / 131.0f;

  data.gz =
    rawGz / 131.0f;


  data.temperature =
    rawTemp / 340.0f +
    36.53f;


  return true;
}


// ============================================================
// MOTOR PWM HELPER
// ============================================================

int percentToPWM(
  float percent
) {
  percent = constrain(
    percent,
    0.0f,
    100.0f
  );

  return (int)(
    percent *
    255.0f /
    100.0f
  );
}


// ============================================================
// LOCAL MOTOR CONTROL
//
// E/F use IN1 + IN2 + separate PWM.
// ============================================================

void setLocalMotor(
  int in1,
  int in2,
  int pwmPin,
  float percent
) {
  percent = constrain(
    percent,
    -100.0f,
    100.0f
  );


  if (
    fabs(percent) <
    0.01f
  ) {
    analogWrite(
      pwmPin,
      0
    );

    digitalWrite(
      in1,
      LOW
    );

    digitalWrite(
      in2,
      LOW
    );

    return;
  }


  int pwm =
    percentToPWM(
      fabs(percent)
    );


  if (
    percent > 0
  ) {
    digitalWrite(
      in1,
      HIGH
    );

    digitalWrite(
      in2,
      LOW
    );

    analogWrite(
      pwmPin,
      pwm
    );
  }

  else {
    digitalWrite(
      in1,
      LOW
    );

    digitalWrite(
      in2,
      HIGH
    );

    analogWrite(
      pwmPin,
      pwm
    );
  }
}


// ============================================================
// LOCAL MOTOR WRAPPERS
// ============================================================

void motorE(
  float percent
) {
  motorEValue =
    constrain(
      percent,
      -100.0f,
      100.0f
    );

  setLocalMotor(
    EIN1,
    EIN2,
    EPWM,
    motorEValue
  );
}


void motorF(
  float percent
) {
  motorFValue =
    constrain(
      percent,
      -100.0f,
      100.0f
    );

  setLocalMotor(
    FIN1,
    FIN2,
    FPWM,
    motorFValue
  );
}


// ============================================================
// SEND C / D TO SLAVE
//
// Packet:
// byte 0 = C (-100..100)
// byte 1 = D (-100..100)
// ============================================================

bool sendSlaveMotorsV(
  bool verbose
) {
  Wire.beginTransmission(
    MOTOR_SLAVE_ADDR
  );


  Wire.write(
    (uint8_t)motorCValue
  );

  Wire.write(
    (uint8_t)motorDValue
  );


  uint8_t error =
    Wire.endTransmission();

  slaveStatus = error;
  lastSlaveSend = millis();


  // ==========================================================
  // I2C DEBUG  (the mixer calls this 50 times a second: quiet
  // unless asked, errors at most once a second)
  // ==========================================================

  static unsigned long lastErrPrint = 0;

  if (
    verbose ||
    (error != 0 && millis() - lastErrPrint > 1000)
  ) {
    if (error != 0) lastErrPrint = millis();

    Serial.print(
      "I2C -> slave C="
    );

    Serial.print(
      motorCValue
    );

    Serial.print(
      " D="
    );

    Serial.print(
      motorDValue
    );

    Serial.print(
      " status="
    );

    Serial.println(
      error
    );
  }


  return (
    error == 0
  );
}


bool sendSlaveMotors() {
  return sendSlaveMotorsV(true);
}


// ============================================================
// SLAVE MOTOR WRAPPERS
// ============================================================

void motorC(
  float percent
) {
  percent =
    constrain(
      percent,
      -100.0f,
      100.0f
    );

  motorCValue =
    (int8_t)percent;

  sendSlaveMotors();
}


void motorD(
  float percent
) {
  percent =
    constrain(
      percent,
      -100.0f,
      100.0f
    );

  motorDValue =
    (int8_t)percent;

  sendSlaveMotors();
}


// ============================================================
// ALL OFF
// ============================================================

void allOff() {
  motorCValue = 0;
  motorDValue = 0;

  sendSlaveMotors();

  motorE(0);
  motorF(0);
}


// ============================================================
// I2C SLAVE SCAN / DEBUG
// ============================================================

void scanMotorSlave() {
  Serial.println(
    "Scanning hardware I2C..."
  );


  bool found = false;


  for (
    uint8_t addr = 1;
    addr < 127;
    addr++
  ) {
    Wire.beginTransmission(
      addr
    );

    uint8_t error =
      Wire.endTransmission();


    if (
      error == 0
    ) {
      Serial.print(
        "Found I2C device at 0x"
      );

      if (
        addr < 0x10
      ) {
        Serial.print(
          "0"
        );
      }

      Serial.println(
        addr,
        HEX
      );

      found = true;
    }
  }


  if (!found) {
    Serial.println(
      "No hardware I2C devices found"
    );
  }
}


// ============================================================
// ONBOARD FLIGHT MIXER: map, status, control tick
// ============================================================

String mapString() {
  const char roles[5]   = "LRSV";
  const char letters[5] = "CDEF";
  String s;

  for (int r = 0; r < 4; r++) {
    if (r) s += ' ';
    s += roles[r];
    s += letters[roleLetter[r]];
    s += (roleSign[r] < 0) ? '-' : '+';
  }

  s += " G";
  s += (gyroSign < 0) ? '-' : '+';
  return s;
}


// "LC+ RD- SE+ VF+ G+"  (also accepts L=C+, L:C-, LC). All four
// roles must be given, on four different letters; G is optional.
bool parseMap(
  String s
) {
  int8_t letter[4] = {-1, -1, -1, -1};
  int8_t sign[4]   = {1, 1, 1, 1};
  int8_t g = gyroSign;

  s.trim();
  s.toUpperCase();

  int start = 0;

  while (start < (int)s.length()) {
    int space = s.indexOf(' ', start);
    if (space < 0) space = s.length();

    String tok = s.substring(start, space);
    start = space + 1;

    tok.replace("=", "");
    tok.replace(":", "");
    tok.trim();

    if (tok.length() == 0) continue;

    char first = tok.charAt(0);

    if (first == 'G') {
      g = (tok.indexOf('-') >= 0) ? -1 : 1;
      continue;
    }

    int ri = String("LRSV").indexOf(first);
    if (ri < 0 || tok.length() < 2) return false;

    int li = String("CDEF").indexOf(tok.charAt(1));
    if (li < 0) return false;

    letter[ri] = li;
    sign[ri]   = (tok.indexOf('-') >= 0) ? -1 : 1;
  }

  for (int i = 0; i < 4; i++) {
    if (letter[i] < 0) return false;

    for (int j = 0; j < i; j++) {
      if (letter[i] == letter[j]) return false;
    }
  }

  for (int i = 0; i < 4; i++) {
    roleLetter[i] = letter[i];
    roleSign[i]   = sign[i];
  }

  gyroSign = g;
  return true;
}


void saveMap() {
  prefs.putString("map", mapString());
}


void loadMap() {
  String m = prefs.getString("map", "");

  if (m.length() && parseMap(m)) {
    Serial.print("Motor map from flash: ");
  } else {
    Serial.print("Motor map default (send MAP ... to set): ");
  }

  Serial.println(mapString());
}


// Mixer off, everything zero, motors off. Called for STOP, a raw
// motor command, a link loss.
void mixerOff(
  const char* why
) {
  bool was = cmdMode;

  cmdMode = false;
  cmdTimedOut = false;
  yawI = 0;

  for (int i = 0; i < 4; i++) {
    sp[i] = 0;
    cur[i] = 0;
  }

  allOff();

  if (was) {
    Serial.print("mixer off: ");
    Serial.println(why);
  }
}


// Duties L R S V -> percent per letter -> motors. C/D go to the
// slave when they changed or as a heartbeat; E/F when changed.
void writeRoles(
  unsigned long now,
  bool force
) {
  int8_t pct[4] = {0, 0, 0, 0};

  for (int r = 0; r < 4; r++) {
    pct[roleLetter[r]] =
      (int8_t)lroundf(cur[r] * roleSign[r] * 100.0f);
  }

  bool cdChanged =
    (pct[0] != motorCValue) ||
    (pct[1] != motorDValue);

  motorCValue = pct[0];
  motorDValue = pct[1];

  if (
    force ||
    cdChanged ||
    now - lastSlaveSend >= SLAVE_HEARTBEAT_MS
  ) {
    sendSlaveMotorsV(false);
  }

  if (force || pct[2] != (int8_t)lroundf(motorEValue)) motorE(pct[2]);
  if (force || pct[3] != (int8_t)lroundf(motorFValue)) motorF(pct[3]);
}


// CMD vf vs yr vz  (percent). Any number of values; missing = 0.
void handleCmd(
  String args
) {
  float v[4] = {0, 0, 0, 0};
  int n = 0;

  args.trim();
  int start = 0;

  while (n < 4 && start < (int)args.length()) {
    int space = args.indexOf(' ', start);
    if (space < 0) space = args.length();

    String tok = args.substring(start, space);
    start = space + 1;

    if (tok.length() == 0) continue;

    v[n++] = tok.toFloat();
  }

  for (int i = 0; i < 4; i++) {
    sp[i] = constrain(v[i] / 100.0f, -1.0f, 1.0f);
  }

  if (!cmdMode) {
    Serial.println("mixer on (CMD)");
    yawI = 0;
    for (int i = 0; i < 4; i++) cur[i] = 0;
  }

  cmdMode = true;
  cmdTimedOut = false;
  lastCmdMs = millis();
}


void printStatus() {
  Serial.print("MAP ");
  Serial.println(mapString());

  Serial.print("mode: ");
  Serial.print(cmdMode ? (cmdTimedOut ? "CMD timed out (motors off)" : "CMD flying") : "raw (bench commands)");

  Serial.print("   age ");
  Serial.print(cmdMode ? (long)(millis() - lastCmdMs) : -1L);
  Serial.println(" ms");

  Serial.print("setpoints vf vs yr vz: ");
  for (int i = 0; i < 4; i++) { Serial.print(sp[i], 2); Serial.print(' '); }
  Serial.println();

  Serial.print("duties L R S V: ");
  for (int i = 0; i < 4; i++) { Serial.print(cur[i], 2); Serial.print(' '); }
  Serial.println();

  Serial.print("percent C D E F: ");
  Serial.print(motorCValue); Serial.print(' ');
  Serial.print(motorDValue); Serial.print(' ');
  Serial.print((int)lroundf(motorEValue)); Serial.print(' ');
  Serial.println((int)lroundf(motorFValue));

  Serial.print("imu ");
  Serial.print(imuOk ? "ok" : "ERROR");
  Serial.print("   yaw ");
  Serial.print(yawDeg, 1);
  Serial.print(" deg   gz ");
  Serial.print(gzDps, 2);
  Serial.print(" deg/s   gyro zero ");
  Serial.print(gzBias, 2);
  Serial.print("   slave status ");
  Serial.print(slaveStatus);
  Serial.print("   ble ");
  Serial.println(deviceConnected ? "connected" : "advertising");
}


// One 50 Hz tick: gyro, heading, gyro zero, then the mixer.
void controlTick(
  unsigned long now
) {
  IMUData imu;

  if (readIMU(imu)) {
    lastImu = imu;
    imuOk = true;
  } else {
    imuOk = false;
  }

  bool motorsIdle =
    motorCValue == 0 &&
    motorDValue == 0 &&
    fabs(motorEValue) < 0.5f &&
    fabs(motorFValue) < 0.5f;

  if (imuOk) {
    // Gyro zero: a MEMS gyro at rest does not read 0 (bench: -0.36
    // deg/s = 20 degrees of heading a minute). Learn it while the
    // motors are off and the box is still; wide gate for the first
    // seconds after boot, then only small corrections.
    float gate = (now < 8000) ? 5.0f : 0.6f;

    if (motorsIdle && fabs(lastImu.gz - gzBias) < gate) {
      gzBias += 0.01f * (lastImu.gz - gzBias);
    }

    gzDps = (lastImu.gz - gzBias) * gyroSign;
    yawDeg += gzDps * MIX_DT;

    if (yawDeg > 180.0f) yawDeg -= 360.0f;
    else if (yawDeg <= -180.0f) yawDeg += 360.0f;
  }

  if (!cmdMode) {
    // Bench mode: keep the slave's 600 ms timeout fed while C or D
    // is running from a typed command.
    if (
      (motorCValue != 0 || motorDValue != 0) &&
      now - lastSlaveSend >= SLAVE_HEARTBEAT_MS
    ) {
      sendSlaveMotorsV(false);
    }

    return;
  }

  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    bool first = !cmdTimedOut;
    cmdTimedOut = true;

    if (first) Serial.println("mixer: no CMD for 500 ms -> motors off");

    yawI = 0;
    for (int i = 0; i < 4; i++) { sp[i] = 0; cur[i] = 0; }

    writeRoles(now, first);
    return;
  }

  float gzNorm = imuOk ? (gzDps / YR_MAX_DPS) : 0.0f;
  float err = sp[2] - gzNorm;

  if (imuOk) {
    yawI = constrain(yawI + MIX_KI_YR * err * MIX_DT, -MIX_I_MAX, MIX_I_MAX);
  } else {
    yawI = 0;       // no gyro: open loop, P on the setpoint only
  }

  float diff = MIX_K_YR * err + yawI;

  float tgt[4] = {
    constrain(sp[0] - diff, -MIX_CAP, MIX_CAP),   // L
    constrain(sp[0] + diff, -MIX_CAP, MIX_CAP),   // R
    constrain(sp[1],        -MIX_CAP, MIX_CAP),   // S
    constrain(sp[3],        -MIX_CAP, MIX_CAP)    // V
  };

  float total = fabs(tgt[0]) + fabs(tgt[1]) + fabs(tgt[2]) + fabs(tgt[3]);

  if (total > MIX_TOTAL) {
    float scale = MIX_TOTAL / total;
    for (int i = 0; i < 4; i++) tgt[i] *= scale;
  }

  for (int i = 0; i < 4; i++) {
    cur[i] += constrain(tgt[i] - cur[i], -MIX_SLEW, MIX_SLEW);
  }

  writeRoles(now, false);
}


// ============================================================
// COMMAND PARSER
//
// Seamless:
//
// C 40
// D -30
// E 50
// F 75
//
// ALL 30
//
// MOTORS 10 20 30 40
//
// STOP
//
// SCAN
//
// Flight (onboard mixer, see the section above):
//
// CMD 20 0 0 0
// MAP LC+ RD+ SE+ VF+ G+
// MAP
// STATUS
// ============================================================

void processCommand(
  String command
) {
  command.trim();
  command.toUpperCase();


  if (
    command.length() == 0
  ) {
    return;
  }


  if (
    !command.startsWith("CMD ")     // 10 a second in flight: not echoed
  ) {
    Serial.print(
      "Command: "
    );

    Serial.println(
      command
    );
  }


  // ==========================================================
  // STOP
  // ==========================================================

  if (
    command == "STOP"
  ) {
    mixerOff("STOP");

    Serial.println(
      "ALL MOTORS OFF"
    );

    return;
  }


  // ==========================================================
  // I2C SCAN
  // ==========================================================

  if (
    command == "SCAN"
  ) {
    scanMotorSlave();

    return;
  }


  // ==========================================================
  // ONBOARD MIXER: CMD / MAP / STATUS
  // ==========================================================

  if (
    command.startsWith(
      "CMD "
    )
  ) {
    handleCmd(
      command.substring(4)
    );

    return;
  }


  if (
    command.startsWith(
      "MAP "
    )
  ) {
    if (
      parseMap(
        command.substring(4)
      )
    ) {
      saveMap();

      Serial.print(
        "MAP saved: "
      );

      Serial.println(
        mapString()
      );
    }

    else {
      Serial.println(
        "MAP rejected: give all of L R S V on four different letters, e.g. MAP LC+ RD+ SE+ VF+ G+"
      );
    }

    return;
  }


  if (
    command == "MAP"
  ) {
    Serial.print(
      "MAP "
    );

    Serial.println(
      mapString()
    );

    return;
  }


  if (
    command == "STATUS"
  ) {
    printStatus();

    return;
  }


  // A raw motor command below = bench mode: the mixer lets go first
  // (otherwise the motors it does not name would keep spinning).

  if (
    cmdMode &&
    (
      command.startsWith("C ") ||
      command.startsWith("D ") ||
      command.startsWith("E ") ||
      command.startsWith("F ") ||
      command.startsWith("ALL ") ||
      command.startsWith("MOTORS ")
    )
  ) {
    mixerOff("raw motor command");
  }


  // ==========================================================
  // C
  // ==========================================================

  if (
    command.startsWith(
      "C "
    )
  ) {
    motorC(
      command
        .substring(2)
        .toFloat()
    );

    return;
  }


  // ==========================================================
  // D
  // ==========================================================

  if (
    command.startsWith(
      "D "
    )
  ) {
    motorD(
      command
        .substring(2)
        .toFloat()
    );

    return;
  }


  // ==========================================================
  // E
  // ==========================================================

  if (
    command.startsWith(
      "E "
    )
  ) {
    motorE(
      command
        .substring(2)
        .toFloat()
    );

    return;
  }


  // ==========================================================
  // F
  // ==========================================================

  if (
    command.startsWith(
      "F "
    )
  ) {
    motorF(
      command
        .substring(2)
        .toFloat()
    );

    return;
  }


  // ==========================================================
  // ALL
  // ==========================================================

  if (
    command.startsWith(
      "ALL "
    )
  ) {
    float p =
      command
        .substring(4)
        .toFloat();


    // Set C/D together before sending one packet
    motorCValue =
      (int8_t)constrain(
        p,
        -100.0f,
        100.0f
      );

    motorDValue =
      motorCValue;


    sendSlaveMotors();


    motorE(
      p
    );

    motorF(
      p
    );


    return;
  }


  // ==========================================================
  // MOTORS C D E F
  //
  // Example:
  //
  // MOTORS 10 20 30 40
  // ==========================================================

  if (
    command.startsWith(
      "MOTORS "
    )
  ) {
    float values[4] = {
      0,
      0,
      0,
      0
    };


    String remaining =
      command.substring(7);


    for (
      int i = 0;
      i < 4;
      i++
    ) {
      int space =
        remaining.indexOf(
          ' '
        );


      if (
        space == -1
      ) {
        values[i] =
          remaining.toFloat();

        break;
      }


      values[i] =
        remaining
          .substring(
            0,
            space
          )
          .toFloat();


      remaining =
        remaining.substring(
          space + 1
        );
    }


    motorCValue =
      (int8_t)constrain(
        values[0],
        -100.0f,
        100.0f
      );

    motorDValue =
      (int8_t)constrain(
        values[1],
        -100.0f,
        100.0f
      );


    sendSlaveMotors();


    motorE(
      values[2]
    );

    motorF(
      values[3]
    );


    return;
  }


  Serial.println(
    "Unknown command"
  );
}


// ============================================================
// BLE SERVER CALLBACKS
// ============================================================

class ServerCallbacks
  : public BLEServerCallbacks {

  void onConnect(
    BLEServer* server
  ) override {
    deviceConnected = true;

    Serial.println(
      "BLE connected"
    );
  }


  void onDisconnect(
    BLEServer* server
  ) override {
    deviceConnected = false;


    // Flags only. loop() stops the motors and restarts advertising:
    // this runs on the Bluetooth stack's task, and an advertising
    // restart from here failed silently on the bench (2026-09-19):
    // the box then stayed invisible until it was power-cycled.
    linkLost = true;
  }
};


// ============================================================
// BLE COMMAND CALLBACK
// ============================================================

class CommandCallbacks
  : public BLECharacteristicCallbacks {

  void onWrite(
    BLECharacteristic* characteristic
  ) override {
    String command =
      characteristic
        ->getValue();


    if (
      command.length() == 0
    ) {
      return;
    }


    // Queue it for loop(): BLE and Serial use the exact same command
    // system, but the motors and the I2C bus are touched from loop()
    // only (this callback runs on the Bluetooth task). A full queue
    // drops the newest command; at 10 CMD a second it never fills.
    int next = (cmdHead + 1) % CMD_QUEUE_N;

    if (
      next == cmdTail
    ) {
      return;
    }

    strncpy(
      cmdQueue[cmdHead],
      command.c_str(),
      sizeof(cmdQueue[0]) - 1
    );

    cmdQueue[cmdHead][sizeof(cmdQueue[0]) - 1] = 0;

    cmdHead = next;
  }
};


// Called from loop(): every command the Bluetooth task queued.
void drainCommandQueue() {
  while (
    cmdTail != cmdHead
  ) {
    String command(
      cmdQueue[cmdTail]
    );

    cmdTail = (cmdTail + 1) % CMD_QUEUE_N;

    if (
      !command.startsWith("CMD ")     // 10 a second: keep the serial monitor readable
    ) {
      Serial.print(
        "BLE -> "
      );

      Serial.println(
        command
      );
    }

    processCommand(
      command
    );
  }
}


// ============================================================
// BLE INIT
// ============================================================

void initBLE() {
  BLEDevice::init(
    "BalloonRobot"
  );


  // Telemetry lines are ~100 characters: allow the laptop to
  // negotiate a bigger packet than the 20-byte default.
  BLEDevice::setMTU(
    185
  );


  BLEServer* server =
    BLEDevice::createServer();


  server->setCallbacks(
    new ServerCallbacks()
  );


  BLEService* service =
    server->createService(
      SERVICE_UUID
    );


  // ==========================================================
  // COMMAND
  // ==========================================================

  BLECharacteristic*
    commandCharacteristic =
      service->createCharacteristic(
        COMMAND_UUID,

        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
      );


  commandCharacteristic
    ->setCallbacks(
      new CommandCallbacks()
    );


  // ==========================================================
  // TELEMETRY
  // ==========================================================

  telemetryCharacteristic =
    service->createCharacteristic(
      TELEMETRY_UUID,

      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
    );


  telemetryCharacteristic
    ->addDescriptor(
      new BLE2902()
    );


  telemetryCharacteristic
    ->setValue(
      "starting"
    );


  service->start();


  BLEAdvertising* advertising =
    BLEDevice::getAdvertising();


  advertising->addServiceUUID(
    SERVICE_UUID
  );


  advertising->setScanResponse(
    true
  );


  BLEDevice::startAdvertising();


  Serial.println(
    "BLE advertising as BalloonRobot"
  );
}


// ============================================================
// TELEMETRY
//
// 200 ms = 5 Hz. The IMU itself is read every 20 ms by the
// control tick (heading integrated there); this only reports.
//
// One line:
//   A:ax,ay,az;G:gx,gy,gz;yaw:d;gzc:d;st:s;age:ms;mc:p;md:p;me:p;mf:p;bias:d
//   A g, G deg/s raw, yaw deg (CCW+, integrated on the box, drifts
//   slowly: the laptop learns the offset), gzc deg/s after zero and
//   sign, st 0 = raw/bench 1 = CMD flying 2 = CMD timed out,
//   age ms since the last CMD (-1 none), mc..mf percent per letter,
//   bias = the gyro zero in use.
// ============================================================

constexpr unsigned long
  TELEMETRY_INTERVAL_MS = 200;

unsigned long lastTelemetry = 0;


// ============================================================
// SEND TELEMETRY
// ============================================================

void sendTelemetry() {
  char buffer[160];

  int st = cmdMode ? (cmdTimedOut ? 2 : 1) : 0;
  long age = cmdMode ? (long)(millis() - lastCmdMs) : -1L;


  if (
    imuOk
  ) {
    snprintf(
      buffer,
      sizeof(buffer),

      "A:%.2f,%.2f,%.2f;"
      "G:%.1f,%.1f,%.1f;"
      "yaw:%.1f;gzc:%.2f;st:%d;age:%ld;"
      "mc:%d;md:%d;me:%d;mf:%d;bias:%.2f",

      lastImu.ax,
      lastImu.ay,
      lastImu.az,

      lastImu.gx,
      lastImu.gy,
      lastImu.gz,

      yawDeg,
      gzDps,
      st,
      age,

      (int)motorCValue,
      (int)motorDValue,
      (int)lroundf(motorEValue),
      (int)lroundf(motorFValue),
      gzBias
    );
  }

  else {
    static unsigned long lastImuErr = 0;

    if (millis() - lastImuErr > 1000) {
      lastImuErr = millis();
      Serial.println("IMU_ERROR");
    }

    snprintf(
      buffer,
      sizeof(buffer),

      "IMU_ERROR;st:%d;age:%ld;mc:%d;md:%d;me:%d;mf:%d",

      st,
      age,

      (int)motorCValue,
      (int)motorDValue,
      (int)lroundf(motorEValue),
      (int)lroundf(motorFValue)
    );
  }


  if (
    telemetryCharacteristic != nullptr
  ) {
    telemetryCharacteristic
      ->setValue(
        (uint8_t*)buffer,
        strlen(buffer)
      );


    if (
      deviceConnected
    ) {
      telemetryCharacteristic
        ->notify();
    }
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(
    115200
  );

  delay(
    500
  );


  // ==========================================================
  // LOCAL MOTORS
  // ==========================================================

  pinMode(
    EIN1,
    OUTPUT
  );

  pinMode(
    EIN2,
    OUTPUT
  );

  pinMode(
    EPWM,
    OUTPUT
  );


  pinMode(
    FIN1,
    OUTPUT
  );

  pinMode(
    FIN2,
    OUTPUT
  );

  pinMode(
    FPWM,
    OUTPUT
  );


  motorE(
    0
  );

  motorF(
    0
  );


  // ==========================================================
  // HARDWARE I2C -> SLAVE
  // ==========================================================

  Wire.begin(
    SLAVE_SDA,
    SLAVE_SCL,
    100000
  );


  Serial.println(
    "Hardware I2C master started"
  );

  Serial.print(
    "SDA = GPIO"
  );

  Serial.println(
    SLAVE_SDA
  );

  Serial.print(
    "SCL = GPIO"
  );

  Serial.println(
    SLAVE_SCL
  );


  // ==========================================================
  // FIND SLAVE
  // ==========================================================

  Wire.beginTransmission(
    MOTOR_SLAVE_ADDR
  );

  uint8_t status =
    Wire.endTransmission();


  Serial.print(
    "Motor slave 0x12 status: "
  );

  Serial.println(
    status
  );


  // ==========================================================
  // MPU
  // ==========================================================

  Serial.println(
    "Starting MPU6050 software I2C..."
  );


  if (
    initMPU()
  ) {
    Serial.println(
      "MPU6050 OK"
    );
  }

  else {
    Serial.println(
      "MPU6050 FAILED"
    );
  }


  // ==========================================================
  // MOTOR MAP (flash)
  // ==========================================================

  prefs.begin(
    "blimpy",
    false
  );

  loadMap();


  // ==========================================================
  // BLE
  // ==========================================================

  initBLE();


  // ==========================================================
  // READY
  // ==========================================================

  Serial.println();

  Serial.println(
    "MASTER READY"
  );

  Serial.println(
    "C/D -> I2C slave"
  );

  Serial.println(
    "E/F -> local driver"
  );

  Serial.println(
    "IMU -> BLE every 200 ms (heading integrated here at 50 Hz)"
  );

  Serial.println();


  Serial.println(
    "Commands:"
  );

  Serial.println(
    "CMD 20 0 0 0            flight setpoints vf vs yr vz (percent), motors off 500 ms after the last one"
  );

  Serial.println(
    "MAP LC+ RD+ SE+ VF+ G+  which letter is L/R/S/V (+ sign), G = gyro sign; saved"
  );

  Serial.println(
    "STATUS"
  );

  Serial.println(
    "C 40"
  );

  Serial.println(
    "D 40"
  );

  Serial.println(
    "E 40"
  );

  Serial.println(
    "F 40"
  );

  Serial.println(
    "ALL 40"
  );

  Serial.println(
    "MOTORS 10 20 30 40"
  );

  Serial.println(
    "STOP"
  );

  Serial.println(
    "SCAN"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop() {
  unsigned long now =
    millis();


  // ==========================================================
  // SERIAL COMMANDS
  // ==========================================================

  if (
    Serial.available()
  ) {
    String command =
      Serial.readStringUntil(
        '\n'
      );


    processCommand(
      command
    );
  }


  // ==========================================================
  // BLE COMMANDS (queued by the Bluetooth task)
  // ==========================================================

  drainCommandQueue();


  // ==========================================================
  // LINK LOST -> motors off, mixer off
  // ==========================================================

  if (
    linkLost
  ) {
    linkLost = false;

    mixerOff("BLE disconnected");

    Serial.println(
      "BLE disconnected -> ALL MOTORS OFF"
    );

    lastAdvertise = 0;      // advertise again right away
  }


  // ==========================================================
  // ADVERTISING: re-asserted from here while nobody is
  // connected (harmless when it is already running)
  // ==========================================================

  if (
    !deviceConnected &&
    now - lastAdvertise >= ADVERTISE_EVERY_MS
  ) {
    lastAdvertise = now;

    BLEDevice::startAdvertising();
  }


  // ==========================================================
  // CONTROL TICK @ 50 Hz: gyro, heading, mixer, motors
  // ==========================================================

  if (
    now -
      lastControl >=
    CONTROL_INTERVAL_MS
  ) {
    lastControl =
      now;


    controlTick(
      now
    );
  }


  // ==========================================================
  // TELEMETRY @ 5 Hz
  // ==========================================================

  if (
    now -
      lastTelemetry >=
    TELEMETRY_INTERVAL_MS
  ) {
    lastTelemetry =
      now;


    sendTelemetry();
  }


  delay(
    1
  );
}