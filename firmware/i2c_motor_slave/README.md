# Second ESP32 as an I2C motor slave (motors C and D)

Why: the main ESP32-C3 has no spare pins, and the DRV8833 that drove C and D is being replaced by a second TB6612.
Moving C and D to a second ESP32 over I2C frees five pins on the main board (5, 6, 7, 9 and 8) and keeps ONE
Bluetooth robot for the laptop: nothing changes on the laptop side. `python -m laptop.control.ble_gondola --motor C 30`
still runs motor C; the main board forwards it.

`i2c_motor_slave.ino` is the whole sketch for the second board. Below: the wiring, the patch to the main
BalloonRobot sketch, the bench test, and what to do when it does not work.

Decision of 2026-09-19 late: two TB6612 drivers, two ESP32-C3 boards. The one-board alternative (both TB6612s in
four-wire mode on the main ESP32: 8 pins for four motors plus 2 for I2C, 3 pins spare) is the fallback if the second
board fights you for more than two hours; the slow-decay recipe in `setMotor` is the same either way.

## Wiring

Five wires between the boards, then the driver hangs off the second board.

| Second board (C3 SuperMini) | Goes to | Note |
|---|---|---|
| GPIO 0 (SDA) | main board SDA (GPIO 0), same breadboard row the MPU6050 SDA is on | the GY-521 board already carries the pull-up resistors |
| GPIO 1 (SCL) | main board SCL (GPIO 1), same row as the MPU6050 SCL | keep both wires under 20 cm |
| GND | the common ground rail | both boards, both drivers, the IMU, both batteries' minus: ONE ground |
| 5V | the same row the main board's 5V pin is fed from | the ESP32 LiPo; NOT the motor battery |
| GPIO 3, 4 | second TB6612 AIN1, AIN2 | motor C on AO1 / AO2 |
| GPIO 5, 6 | second TB6612 BIN1, BIN2 | motor D on BO1 / BO2 |

Second TB6612, four-wire mode: PWMA, PWMB, STBY and VCC all to the 3.3 V rail (the ESP32's 3V3 pin, not the motor
battery); VM to the motor battery plus; GND to the common ground. Nothing else. (`STBY_PIN = -1` in the sketch; wire
STBY to GPIO 7 and set `STBY_PIN = 7` only if you want the second board able to sleep the driver.)

While you are there: the FIRST TB6612 (motors E and F, main board) had its STBY row floating on 2026-09-19. Jumper
that STBY to the 3.3 V rail too. A floating STBY is a driver that works until it does not.

A DRV8833 board works in the same slot (nSLEEP to 3.3 V, the same four inputs), but the plan is two TB6612s.

## Patch for the main BalloonRobot sketch

The I2C write happens in `loop()`, never inside the Bluetooth callback: the BLE stack runs its callbacks in another
task, and the MPU6050 read in `loop()` uses the same bus.

```cpp
#include <Wire.h>                       // already there for the MPU6050; the slave shares that bus
constexpr uint8_t SLAVE_ADDR = 0x10;
volatile int8_t slaveC = 0, slaveD = 0;
volatile bool slaveDirty = false;
unsigned long lastSlaveSend = 0;
bool slaveOk = true;

// REPLACE motorC / motorD. Drop CIN1, CIN2, DIN1, DIN2, DRV_STBY, setMotorDRV, hardStopMotorDRV, enableDRV,
// disableDRV and their pinMode / analogWrite lines in setup(): those pins are free now.
void motorC(float p) { slaveC = (int8_t)constrain(p, -100.0f, 100.0f); slaveDirty = true; }
void motorD(float p) { slaveD = (int8_t)constrain(p, -100.0f, 100.0f); slaveDirty = true; }

// in allOff(): add
//   slaveC = 0; slaveD = 0; slaveDirty = true;

// call this from loop(), next to the MPU6050 read (same task, same bus)
void serviceSlave() {
  bool due = (slaveC != 0 || slaveD != 0) && millis() - lastSlaveSend > 200;   // keeps the slave's 500 ms timeout fed
  if (!slaveDirty && !due) return;
  slaveDirty = false;
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write((uint8_t)slaveC);
  Wire.write((uint8_t)slaveD);
  uint8_t err = Wire.endTransmission();
  lastSlaveSend = millis();
  if ((err == 0) != slaveOk) {                                                // prints once per change, not per send
    slaveOk = (err == 0);
    Serial.printf("[i2c] slave 0x10 %s (err %u)\n", slaveOk ? "ok" : "NOT ANSWERING", err);
  }
}
```

Also on the main board, from the 2026-09-19 review: a 500 ms command timeout (`lastCmdMs` stamped in the command
handler; in `loop()`, if any motor is on and `millis() - lastCmdMs > 500`, call `allOff()`); `TELEMETRY_INTERVAL_MS`
20; and `analogWriteFrequency(EPWM, 20000)` / `(FPWM, 20000)` before the first `analogWrite` so E and F stop whining.
With the DRV8833 gone, the `digitalWrite`-after-`analogWrite` bug is gone with it (E and F never analogWrite their
IN pins).

Failsafe chain: laptop quiet -> main board `allOff()` after 500 ms -> zeros reach the slave -> and if the main board
itself dies, the slave stops on its own 500 ms later. The laptop bridge resends `--motor` commands every 200 ms for
exactly this reason.

## Bench test (this is the handoff test in `docs/BUILD_STEPS.md`, Phase H)

1. Second board on USB, Serial Monitor at 115200: `[slave] i2c motor slave 0x10 ready: C on 3/4, D on 5/6, timeout 500 ms`.
2. Main board on, laptop: `python -m laptop.control.ble_gondola --probe` still says OK (the IMU shares the bus).
   The main board's serial must NOT say `NOT ANSWERING`.
3. `python -m laptop.control.ble_gondola --motor C 30 --secs 5`: motor C runs 5 s, then stops. The slave's serial
   prints `[slave] C 30 D 0` then `[slave] C 0 D 0`.
4. `--motor D 30 --secs 5`, `--motor E 30 --secs 5`, `--motor F 30 --secs 5`: each letter spins exactly one motor.
5. `--motor C -30`, `--motor D -30`, `--motor E -30`, `--motor F -30`: each reverses.
6. `--motor C 100 --secs 3`: clearly harder than 30 (100 % used to do nothing on the old driver).
7. Timeout, slave: pull the SDA wire while `--motor C 30 --secs 10` runs. C must stop within about half a second.
8. Timeout, main board: run `python -m laptop.control.ble_gondola`, then `python -m laptop.control.teleop` in a
   second window, SPACE, `w`, then close the BRIDGE window (not teleop). Every motor must stop within a second.
9. For each letter find the lowest `--motor X NN` that starts the motor from rest (try 10, 15, 20). Write the four
   numbers in `calib/MEASUREMENTS.md`; if any is above 10, tell the software side (`FOLLOW DUTY_MIN`).

## When it does not work

| Symptom | Look at |
|---|---|
| main board serial: `[i2c] slave 0x10 NOT ANSWERING (err 2)` | slave not powered, no common ground, SDA and SCL swapped, or the slave sketch not running (its serial says nothing at boot) |
| `NOT ANSWERING (err 5)` (timeout) | a wire is off, or SDA/SCL are on the wrong main-board pins; the IMU probe also fails in that case |
| probe OK, slave serial never prints `C 30` | the main board is not calling `serviceSlave()` from `loop()`, or `motorC` still points at the old DRV code |
| slave prints `C 30 D 0` but nothing spins | driver side: VM has no motor battery, STBY / PWMA / PWMB / VCC not on 3.3 V, motor lead loose |
| motor spins the wrong way | swap that motor's two leads at the driver, or leave it: `SIGN` in `laptop/config.py` handles it at build step J7 |
| C stops for a moment every few seconds | the 200 ms resend is not in `loop()`, or `loop()` is blocked longer than 500 ms (a `delay`) |
| the IMU line stops while C/D run | ground loop or the motor battery sharing a wire with the ESP32 LiPo minus somewhere other than the ground rail; add a 470-1000 uF capacitor across the motor battery |
