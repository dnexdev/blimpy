#include <Arduino.h>
#include <Wire.h>

// ============================================================
// I2C SLAVE CONFIG
// ============================================================

constexpr uint8_t SLAVE_ADDRESS = 0x12;

constexpr int I2C_SDA = 8;
constexpr int I2C_SCL = 9;


// ============================================================
// MOTOR PINS
// ============================================================

// Motor 1
constexpr int M1_IN1 = 0;
constexpr int M1_IN2 = 1;
constexpr int M1_PWM = 2;

// Motor 2
constexpr int M2_IN1 = 3;
constexpr int M2_IN2 = 4;
constexpr int M2_PWM = 5;

// Driver STBY -> 3.3V


// ============================================================
// COMMAND STATE
// ============================================================

volatile int8_t pendingMotor1 = 0;
volatile int8_t pendingMotor2 = 0;

volatile bool commandPending = false;


// ============================================================
// PWM HELPER
// ============================================================

int percentToPWM(float percent) {
  percent = constrain(
    percent,
    0.0f,
    100.0f
  );

  return (int)(
    (percent / 100.0f) * 255.0f
  );
}


// ============================================================
// MOTOR CONTROL
// ============================================================

void setMotor(
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

  // STOP
  if (fabs(percent) < 0.01f) {
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


  // FORWARD
  if (percent > 0.0f) {
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

    return;
  }


  // REVERSE
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


// ============================================================
// MOTOR WRAPPERS
// ============================================================

void motor1(float percent) {
  setMotor(
    M1_IN1,
    M1_IN2,
    M1_PWM,
    percent
  );
}


void motor2(float percent) {
  setMotor(
    M2_IN1,
    M2_IN2,
    M2_PWM,
    percent
  );
}


void stopAllMotors() {
  motor1(0);
  motor2(0);
}


// ============================================================
// I2C RECEIVE CALLBACK
// ============================================================

void onReceive(int byteCount) {
  if (byteCount < 2) {
    while (Wire.available()) {
      Wire.read();
    }

    return;
  }


  int8_t m1 =
    (int8_t)Wire.read();

  int8_t m2 =
    (int8_t)Wire.read();


  while (Wire.available()) {
    Wire.read();
  }


  m1 = constrain(
    m1,
    -100,
    100
  );

  m2 = constrain(
    m2,
    -100,
    100
  );


  pendingMotor1 = m1;
  pendingMotor2 = m2;

  commandPending = true;
}


// ============================================================
// SERIAL COMMAND PARSER
//
// M1 50
// M1 -50
//
// M2 50
// M2 -50
//
// BOTH 40
//
// MOTORS 30 -20
//
// STOP
// ============================================================

void processSerialCommand(
  String command
) {
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) {
    return;
  }


  Serial.print(
    "Serial command: "
  );

  Serial.println(
    command
  );


  // STOP
  if (command == "STOP") {
    stopAllMotors();

    Serial.println(
      "ALL MOTORS OFF"
    );

    return;
  }


  // MOTOR 1
  if (command.startsWith("M1 ")) {
    float value =
      command.substring(3).toFloat();

    motor1(value);

    Serial.print(
      "M1 = "
    );

    Serial.println(
      value
    );

    return;
  }


  // MOTOR 2
  if (command.startsWith("M2 ")) {
    float value =
      command.substring(3).toFloat();

    motor2(value);

    Serial.print(
      "M2 = "
    );

    Serial.println(
      value
    );

    return;
  }


  // BOTH SAME POWER
  if (command.startsWith("BOTH ")) {
    float value =
      command.substring(5).toFloat();

    motor1(value);
    motor2(value);

    Serial.print(
      "BOTH = "
    );

    Serial.println(
      value
    );

    return;
  }


  // INDEPENDENT
  //
  // MOTORS 30 -20
  if (command.startsWith("MOTORS ")) {
    String values =
      command.substring(7);

    int space =
      values.indexOf(' ');

    if (space == -1) {
      Serial.println(
        "Usage: MOTORS <M1> <M2>"
      );

      return;
    }


    float m1 =
      values
        .substring(
          0,
          space
        )
        .toFloat();

    float m2 =
      values
        .substring(
          space + 1
        )
        .toFloat();


    motor1(m1);
    motor2(m2);


    Serial.print(
      "M1 = "
    );

    Serial.print(
      m1
    );

    Serial.print(
      "   M2 = "
    );

    Serial.println(
      m2
    );

    return;
  }


  Serial.println(
    "Unknown command"
  );
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(
    115200
  );

  delay(500);


  // ==========================================================
  // MOTOR PINS
  // ==========================================================

  pinMode(
    M1_IN1,
    OUTPUT
  );

  pinMode(
    M1_IN2,
    OUTPUT
  );

  pinMode(
    M1_PWM,
    OUTPUT
  );


  pinMode(
    M2_IN1,
    OUTPUT
  );

  pinMode(
    M2_IN2,
    OUTPUT
  );

  pinMode(
    M2_PWM,
    OUTPUT
  );


  stopAllMotors();


  // ==========================================================
  // I2C SLAVE
  // ==========================================================

  Wire.begin(
    SLAVE_ADDRESS,
    I2C_SDA,
    I2C_SCL,
    100000
  );

  Wire.onReceive(
    onReceive
  );


  // ==========================================================
  // READY
  // ==========================================================

  Serial.println();
  Serial.println(
    "Motor slave ready"
  );

  Serial.print(
    "I2C address: 0x"
  );

  Serial.println(
    SLAVE_ADDRESS,
    HEX
  );

  Serial.print(
    "SDA: GPIO"
  );

  Serial.println(
    I2C_SDA
  );

  Serial.print(
    "SCL: GPIO"
  );

  Serial.println(
    I2C_SCL
  );


  Serial.println();
  Serial.println(
    "Serial commands:"
  );

  Serial.println(
    "M1 50"
  );

  Serial.println(
    "M1 -50"
  );

  Serial.println(
    "M2 50"
  );

  Serial.println(
    "M2 -50"
  );

  Serial.println(
    "BOTH 40"
  );

  Serial.println(
    "MOTORS 30 -20"
  );

  Serial.println(
    "STOP"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ==========================================================
  // SERIAL DEBUG CONTROL
  // ==========================================================

  if (Serial.available()) {
    String command =
      Serial.readStringUntil(
        '\n'
      );

    processSerialCommand(
      command
    );
  }


  // ==========================================================
  // I2C COMMAND
  // ==========================================================

  if (commandPending) {
    noInterrupts();

    int8_t m1 =
      pendingMotor1;

    int8_t m2 =
      pendingMotor2;

    commandPending = false;

    interrupts();


    motor1(
      m1
    );

    motor2(
      m2
    );


    Serial.print(
      "I2C -> M1: "
    );

    Serial.print(
      m1
    );

    Serial.print(
      "%   M2: "
    );

    Serial.print(
      m2
    );

    Serial.println(
      "%"
    );
  }


  delay(1);
}