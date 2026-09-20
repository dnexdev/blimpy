#include <Arduino.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>


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

std::atomic<bool> deviceConnected{false};
std::atomic<bool> connectPending{false};
std::atomic<bool> disconnectPending{false};
std::atomic<bool> commandOverflow{false};
std::atomic<uint32_t> connectionGeneration{0};

struct QueuedCommand {
  uint32_t generation;
  char text[128];
};
QueueHandle_t commandQueue = nullptr;
bool restartAdvertisingPending = false;
unsigned long restartAdvertisingAt = 0;


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

bool sendSlaveMotors() {
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


  // ==========================================================
  // I2C DEBUG
  // ==========================================================

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


  return (
    error == 0
  );
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
  // Stop local motors before attempting I2C.
  motorE(0);
  motorF(0);
  motorCValue = 0;
  motorDValue = 0;

  sendSlaveMotors();
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
    if (disconnectPending.load()) return;
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


  Serial.print(
    "Command: "
  );

  Serial.println(
    command
  );


  // ==========================================================
  // STOP
  // ==========================================================

  if (
    command == "STOP"
  ) {
    allOff();

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

// BLE callbacks only publish state or queue commands. Hardware and logging
// run in loop(), so an I2C transaction cannot hold up a Bluetooth callback.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    connectionGeneration.fetch_add(1);
    deviceConnected.store(true);
    connectPending.store(true);
  }

  void onDisconnect(BLEServer* server) override {
    deviceConnected.store(false);
    connectionGeneration.fetch_add(1);
    disconnectPending.store(true);
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    if (!deviceConnected.load() || commandQueue == nullptr) return;
    auto value = characteristic->getValue();
    if (value.length() == 0) return;

    QueuedCommand command{};
    if (value.length() >= sizeof(command.text)) {
      commandOverflow.store(true);
      return;
    }
    command.generation = connectionGeneration.load();
    memcpy(command.text, value.c_str(), value.length());
    if (xQueueSend(commandQueue, &command, 0) != pdTRUE) {
      commandOverflow.store(true);
    }
  }
};

void handleBLEEvents() {
  if (disconnectPending.exchange(false)) {
    xQueueReset(commandQueue);
    allOff();
    Serial.println("BLE disconnected -> ALL MOTORS OFF");
    restartAdvertisingPending = true;
    restartAdvertisingAt = millis() + 200;
  }
  if (commandOverflow.exchange(false)) {
    xQueueReset(commandQueue);
    allOff();
    Serial.println("BLE command too long or queue full -> ALL MOTORS OFF");
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

// Do not wait for a newline: loop() must keep servicing disconnect events.
void handleSerialCommands() {
  static char buffer[128];
  static size_t length = 0;
  static bool overflow = false;
  for (int count = 0; count < 64 && Serial.available(); ++count) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (overflow) {
        Serial.println("Serial command too long; discarded");
      } else if (length) {
        buffer[length] = '\0';
        processCommand(String(buffer));
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


// ============================================================
// BLE INIT
// ============================================================

void initBLE() {
  BLEDevice::init(
    "BalloonRobot"
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
// IMU TELEMETRY
//
// 20ms = 50 Hz
// ============================================================

constexpr unsigned long
  TELEMETRY_INTERVAL_MS = 200;

unsigned long lastTelemetry = 0;


// ============================================================
// SEND IMU TELEMETRY
// ============================================================

void sendTelemetry() {
  IMUData imu;


  if (
    !readIMU(
      imu
    )
  ) {
    Serial.println(
      "IMU_ERROR"
    );


    if (
      telemetryCharacteristic != nullptr
    ) {
      telemetryCharacteristic
        ->setValue(
          "IMU_ERROR"
        );


      if (
        deviceConnected
      ) {
        telemetryCharacteristic
          ->notify();
      }
    }


    return;
  }


  // ==========================================================
  // BLE STREAM
  // ==========================================================

  char buffer[140];


  snprintf(
    buffer,
    sizeof(buffer),

    "A:%.3f,%.3f,%.3f;"
    "G:%.2f,%.2f,%.2f;"
    "T:%.1f",

    imu.ax,
    imu.ay,
    imu.az,

    imu.gx,
    imu.gy,
    imu.gz,

    imu.temperature
  );


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
  Wire.setTimeOut(25);


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
  // BLE
  // ==========================================================

  commandQueue = xQueueCreate(12, sizeof(QueuedCommand));
  if (commandQueue == nullptr) {
    allOff();
    Serial.println("Cannot allocate BLE command queue");
    while (true) delay(1000);
  }
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
    "IMU BLE telemetry disabled for connection debugging"
  );

  Serial.println();


  Serial.println(
    "Commands:"
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
  handleBLEEvents();
  QueuedCommand command{};
  if (xQueueReceive(commandQueue, &command, 0) == pdTRUE &&
      deviceConnected.load() && !disconnectPending.load() &&
      command.generation == connectionGeneration.load()) {
    processCommand(String(command.text));
  }
  handleBLEEvents();
  handleSerialCommands();
  handleBLEEvents();

  // ==========================================================
  // IMU TELEMETRY @ 50 Hz
  // ==========================================================

  unsigned long now =
    millis();


  if (
    now -
      lastTelemetry >=
    TELEMETRY_INTERVAL_MS
  ) {
    lastTelemetry =
      now;


    // sendTelemetry();
  }


  delay(
    1
  );
}
