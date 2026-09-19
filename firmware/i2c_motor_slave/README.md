# Second ESP32 as an I2C motor slave (motors C and D)

Why: the main ESP32-C3 has no spare pins. Moving motors C and D to a second ESP32 over I2C frees five pins on the
main board (5, 6, 7, 9 and 8) and keeps ONE Bluetooth robot for the laptop: nothing changes on the laptop side.

`i2c_motor_slave.ino` is the whole sketch for the second board. Below: the wiring, and the patch to the main
BalloonRobot sketch.

## Wiring

| Second board | Goes to | Note |
|---|---|---|
| GPIO 0 (SDA) | main board SDA (GPIO 0), same row the MPU6050 SDA is on | the GY-521 board already has the pull-up resistors |
| GPIO 1 (SCL) | main board SCL (GPIO 1) | |
| GND | the common ground rail | both boards, both drivers, the IMU: ONE ground |
| 5V / VIN (or 3V3) | the same supply that feeds the main board | not the motor battery rail if that is more than one cell |
| GPIO 5, 6 | driver AIN1, AIN2 | motor C on AO1 / AO2 |
| GPIO 7, 9 | driver BIN1, BIN2 | motor D on BO1 / BO2 |
| GPIO 8 | driver STBY / nSLEEP | or tie STBY to 3.3 V and set `STBY_PIN = -1` |

Driver: a DRV8833 as is, or a TB6612 with PWMA, PWMB and VCC tied to 3.3 V. The driver's VM goes to the motor
battery rail as before. Change the pin numbers at the top of the sketch if the second board is not a C3 SuperMini.

## Patch for the main BalloonRobot sketch

```cpp
#include <Wire.h>                       // already there for the MPU6050; the slave shares that bus
constexpr uint8_t SLAVE_ADDR = 0x10;
int8_t slaveC = 0, slaveD = 0;
unsigned long lastSlaveSend = 0;

void sendSlave() {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write((uint8_t)slaveC);
  Wire.write((uint8_t)slaveD);
  Wire.endTransmission();
  lastSlaveSend = millis();
}

// REPLACE motorC / motorD (and drop CIN1, CIN2, DIN1, DIN2, DRV_STBY, setMotorDRV, hardStopMotorDRV,
// enableDRV, disableDRV: those pins are free now)
void motorC(float p) { slaveC = (int8_t)constrain(p, -100.0f, 100.0f); sendSlave(); }
void motorD(float p) { slaveD = (int8_t)constrain(p, -100.0f, 100.0f); sendSlave(); }

// in allOff(): add
//   slaveC = 0; slaveD = 0; sendSlave();

// in loop(): add (keeps the slave's 500 ms timeout fed while the main board wants a motor on)
//   if ((slaveC != 0 || slaveD != 0) && millis() - lastSlaveSend > 200) sendSlave();
```

Keep the main board's own 500 ms command timeout (from the earlier review): laptop quiet -> main board `allOff()`
-> zeros reach the slave -> and if the main board itself dies, the slave stops on its own 500 ms later.

## Test

1. Power both boards, `python -m laptop.control.ble_gondola --probe` still says OK (the IMU shares the bus).
2. `python -m laptop.control.ble_gondola --motor C 30 --secs 5`: motor C on the second board runs 5 s, then stops.
3. Pull the SDA wire while C runs: the motor must stop within about half a second (the slave timeout).
4. `--motor D -30`, `--motor C 100`: reverse and full power both work (they did not on the old code).
