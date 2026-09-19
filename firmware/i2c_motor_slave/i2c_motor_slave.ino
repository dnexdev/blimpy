// Blimpy: a SECOND ESP32 that drives motors C and D, hanging off the main BalloonRobot ESP32 over I2C.
// The laptop still sees one robot: the main board keeps Bluetooth, the IMU and motors E / F, and forwards
// C / D to this board (see README.md next to this file for the main-board patch and the wiring).
//
// Driver on this board: a DRV8833, or a TB6612 with PWMA, PWMB and VCC tied to 3.3 V (four-wire mode).
// Each motor = two inputs, driven in SLOW DECAY: the idle input is held high, the other carries the
// inverted duty. Only analogWrite touches the input pins (on arduino-esp32 3.x a pin that has been
// analogWrite'n ignores digitalWrite).
//
// Protocol: the main board writes 2 signed bytes, percent C and percent D (-100..100), whenever a command
// arrives and every 200 ms while either is non-zero. No message for TIMEOUT_MS -> both motors stop.
// Chain of failsafes: laptop bridge (500 ms) -> main board (500 ms) -> this board (500 ms).
//
// Board: any ESP32 with arduino-esp32 core 3.x (slave mode needs >= 2.0.1). Pin numbers below are for an
// ESP32-C3 SuperMini; change them for another module.

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t I2C_ADDR = 0x10;          // must not collide with the MPU6050 (0x68 / 0x69) on the same bus
constexpr int SDA_PIN = 0, SCL_PIN = 1;     // joined to the main board's SDA / SCL
constexpr int CIN1 = 5, CIN2 = 6;           // driver AIN1 / AIN2  -> motor C on AO1 / AO2
constexpr int DIN1 = 7, DIN2 = 9;           // driver BIN1 / BIN2  -> motor D on BO1 / BO2
constexpr int STBY_PIN = 8;                 // driver STBY / nSLEEP. Or tie STBY to 3.3 V and set this to -1
constexpr int PWM_HZ = 20000;               // above hearing (1 kHz is the whine)
constexpr unsigned long TIMEOUT_MS = 500;

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
  for (int p : {CIN1, CIN2, DIN1, DIN2}) {
    pinMode(p, OUTPUT);
    analogWriteFrequency(p, PWM_HZ);
    analogWrite(p, 0);
  }
  if (STBY_PIN >= 0) { pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, HIGH); }   // never analogWrite this pin
  Wire.begin(I2C_ADDR, SDA_PIN, SCL_PIN, 400000);                             // slave mode
  Wire.onReceive(onReceive);
}

void loop() {
  static int8_t c = 0, d = 0;
  int8_t wc = wantC, wd = wantD;
  if (millis() - lastMsg > TIMEOUT_MS) { wc = 0; wd = 0; }                    // main board went quiet: stop
  if (wc != c) { c = wc; setMotor(CIN1, CIN2, c); }
  if (wd != d) { d = wd; setMotor(DIN1, DIN2, d); }
  delay(2);
}
