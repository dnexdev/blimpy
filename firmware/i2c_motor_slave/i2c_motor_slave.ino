// Blimpy: a SECOND ESP32 that drives motors C and D, hanging off the main BalloonRobot ESP32 over I2C.
// The laptop still sees one robot: the main board keeps Bluetooth, the IMU and motors E / F, and forwards
// C / D to this board (see README.md next to this file for the main-board patch and the wiring).
//
// Driver on this board: a TB6612 in FOUR-WIRE mode (PWMA, PWMB, STBY and VCC tied to 3.3 V, VM to the motor
// battery), or a DRV8833 (nSLEEP tied to 3.3 V). Each motor = two inputs, driven in SLOW DECAY: the idle input
// is held high, the other carries the inverted duty. Only analogWrite touches the input pins (on arduino-esp32
// 3.x a pin that has been analogWrite'n ignores digitalWrite).
//
// Protocol: the main board writes 2 signed bytes, percent C and percent D (-100..100), whenever a command
// arrives and every 200 ms while either is non-zero. No message for TIMEOUT_MS -> both motors stop.
// Chain of failsafes: laptop bridge (500 ms) -> main board (500 ms) -> this board (500 ms).
//
// Board: any ESP32 with arduino-esp32 core 2.0.1 or newer (I2C slave mode); written and pin-numbered for an
// ESP32-C3 SuperMini on core 3.x. Change the pin numbers for another module.
// Serial (USB) prints one line at boot and one line each time the wanted percents change; nothing else.

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t I2C_ADDR = 0x10;          // must not collide with the MPU6050 (0x68 / 0x69) on the same bus
constexpr int SDA_PIN = 0, SCL_PIN = 1;     // joined to the main board's SDA / SCL (its GPIO 0 / 1)
constexpr int CIN1 = 3, CIN2 = 4;           // driver AIN1 / AIN2  -> motor C on AO1 / AO2
constexpr int DIN1 = 5, DIN2 = 6;           // driver BIN1 / BIN2  -> motor D on BO1 / BO2
constexpr int STBY_PIN = -1;                // -1 = STBY (TB6612) / nSLEEP (DRV8833) is wired to 3.3 V. Or wire it to GPIO 7 and put 7 here
constexpr int PWM_HZ = 20000;               // above hearing (1 kHz is the whine)
constexpr unsigned long TIMEOUT_MS = 500;

const int MOTOR_PINS[] = {CIN1, CIN2, DIN1, DIN2};

volatile int8_t wantC = 0, wantD = 0;
volatile unsigned long lastMsg = 0;

void setMotor(int in1, int in2, int percent) {
  percent = constrain(percent, -100, 100);
  if (percent == 0) { analogWrite(in1, 0); analogWrite(in2, 0); return; }      // coast
  int pwm = (abs(percent) * 255) / 100;                                       // 0..255; 255 = solid high
  if (percent > 0) { analogWrite(in1, 255); analogWrite(in2, 255 - pwm); }    // forward, slow decay
  else             { analogWrite(in1, 255 - pwm); analogWrite(in2, 255); }    // reverse, slow decay
}

void onReceive(int n) {
  if (n < 2) { while (Wire.available()) Wire.read(); return; }
  int8_t c = (int8_t)Wire.read();
  int8_t d = (int8_t)Wire.read();
  while (Wire.available()) Wire.read();                                       // ignore anything extra
  wantC = c; wantD = d; lastMsg = millis();
}

void setup() {
  Serial.begin(115200);
  for (int p : MOTOR_PINS) {
    pinMode(p, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    analogWriteFrequency(p, PWM_HZ);
#else
    analogWriteFrequency(PWM_HZ);
#endif
    analogWrite(p, 0);
  }
  if (STBY_PIN >= 0) { pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, HIGH); }   // never analogWrite this pin
  Wire.begin(I2C_ADDR, SDA_PIN, SCL_PIN, 400000);                             // slave mode
  Wire.onReceive(onReceive);
  Serial.printf("[slave] i2c motor slave 0x%02X ready: C on %d/%d, D on %d/%d, timeout %lu ms\n",
                I2C_ADDR, CIN1, CIN2, DIN1, DIN2, TIMEOUT_MS);
}

void loop() {
  static int8_t c = 0, d = 0;
  int8_t wc = wantC, wd = wantD;
  if (millis() - lastMsg > TIMEOUT_MS) { wc = 0; wd = 0; }                    // main board went quiet: stop
  if (wc != c || wd != d) {
    c = wc; d = wd;
    setMotor(CIN1, CIN2, c);
    setMotor(DIN1, DIN2, d);
    Serial.printf("[slave] C %d D %d\n", c, d);
  }
  delay(2);
}
