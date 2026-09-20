# DETAILS 

## Flight commands (onboard mixer, 2026-09-20)

Telemetry now goes out every **200 ms**, too slow for a control loop on the laptop, so the fast part runs on the
master: the laptop sends setpoints, the box mixes on its own gyro at 50 Hz. Added to `esp32_master.ino` (section
"ONBOARD FLIGHT MIXER"), the bench commands below are unchanged:

| Command | What it does |
|---|---|
| `CMD vf vs yr vz` | flight setpoints, percent -100..100: forward, sideways (+ = left), yaw rate (+ = counter-clockwise from above, 100 = 1 rad/s), up. The box runs the yaw-rate PI on the gyro, slews 0.05 duty per 20 ms, caps each motor at 50 % and the four together at 120 %, and **stops every motor 500 ms after the last CMD**. Not echoed on Serial (10 a second). |
| `MAP LF- RD- SE+ VC+ G+` | which motor letter plays which role: L / R rear left / right, S sideways, V vertical; the sign is the direction for a positive command; `G-` if a counter-clockwise turn gives a negative gz. Saved in flash (Preferences). The laptop sends it on every connection from `calib/motor_map.json`. `MAP` alone prints it. |
| `STATUS` | map, mode (raw / CMD flying / CMD timed out), setpoints, duties, percent per letter, yaw, gyro zero, slave status, BLE state |
| `C 30` `D -30` `E 50` `F 100` `ALL 30` `MOTORS c d e f` `STOP` | as before. Any of them switches the mixer OFF first (motors it does not name would otherwise keep running). |

Telemetry line (one every 200 ms):
`A:ax,ay,az;G:gx,gy,gz;yaw:d;gzc:d;st:s;age:ms;mc:p;md:p;me:p;mf:p;bias:d` = accelerometer (g), gyro raw (deg/s),
heading integrated on the box (deg, CCW+, wrapped), yaw rate after the gyro zero and sign (deg/s), state (0 bench,
1 flying CMD, 2 CMD timed out), ms since the last CMD (-1 none), percent per letter, gyro zero in use (deg/s). The
gyro zero is learnt while the motors are off and the box is still: keep it still for a few seconds after power-up.

Also in this revision: BLE writes are queued and handled in `loop()` (the Bluetooth task no longer touches I2C or
the motors); `onDisconnect` only sets a flag, `loop()` stops the motors and re-asserts advertising every 3 s while
nobody is connected (the restart from inside the callback failed silently on 2026-09-19 and the box went dark until
a power cycle); `BLEDevice::setMTU(185)` so the ~100-character line is not truncated; the slave stops C/D 600 ms after
the last I2C packet (`esp32_slave.ino`), the master resends C/D at least every 200 ms; the per-packet I2C print is
quiet unless the values changed.

## Python BLE control reference

[`reference_control.py`](reference_control.py) is the reference for controlling
the master firmware in [`esp32_master.ino`](esp32_master.ino). Run the Python
script on your laptop with Bluetooth enabled and the master ESP32 powered on:

```sh
python firmware/reference_control.py
```

Run this command from the repository root. Install the required `bleak` package
first with `python -m pip install bleak` if it is not already installed.

The script finds the master by its advertised service UUID, connects, and sends
typed commands such as `C 40`, `ALL 30`, `MOTORS 20 30 -10 0` (C, D, E, F order),
or `STOP`. Motor values are signed percentages from -100 to 100.

Use this script's `SERVICE_UUID`, `COMMAND_UUID`, service discovery, and
`write_gatt_char(..., response=False)` call as the reference when integrating BLE
control into the main control code. Keep the UUIDs and command format consistent
with `esp32_master.ino`; the Python script runs on the laptop, while the master
firmware receives the commands and routes them to the appropriate motors.

## Hardware details

The project is a small ESP32-C3-based balloon/drone-like robot with four brushed DC motors split across two ESP32s. The user originally tried to run mixed motor drivers on one ESP32, including a DRV8833-style board and a TB6612FNG-style board, but ran into severe power-distribution and hardware-contact problems: motors affecting each other’s speed, BLE resets when multiple motors started, random cross-channel activation, unstable breadboard/jumper behavior, and DRV standby-related startup issues. External battery power and removing the DRV path improved things, so the current direction is to **ditch the DRV8833 entirely** and use PWMA/PWMB-style motor control on both ESP32s.

The current architecture is **one master ESP32-C3 + one slave ESP32-C3**. The master is the user-facing controller. It handles BLE, the IMU, two local motors, Serial debugging, and forwards commands for the other two motors to the slave over I2C. The slave has no BLE and no IMU; its only real job is to receive two motor commands over I2C and drive two motors.

The intended logical motor names are **C, D, E, F**. To the user, control should feel seamless regardless of which ESP32 physically owns a motor:
- `C` and `D` live on the slave.
- `E` and `F` live on the master.
- Commands like `C 40`, `D -20`, `E 50`, `F 10`, `ALL 30`, `MOTORS 10 20 30 40`, and `STOP` should all be accepted by the master through BLE or Serial.
- The master maintains C/D state and sends both values together to the slave so updating D does not accidentally zero C, and vice versa.

The slave’s current intended pinout is:
```text
I2C:
SDA = GPIO 8
SCL = GPIO 9
slave address = 0x12

Motor 1:
IN1 = GPIO 0
IN2 = GPIO 1
PWM = GPIO 2

Motor 2:
IN1 = GPIO 3
IN2 = GPIO 4
PWM = GPIO 5

Driver STBY -> hard-wired to 3.3V
```

The slave receives a **2-byte signed packet**:
```text
byte 0 = motor 1 command, int8_t, -100..100
byte 1 = motor 2 command, int8_t, -100..100
```
For example `[40, -25]` means motor 1 = +40%, motor 2 = -25%. The slave also supports Serial debug commands locally (`M1 50`, `M2 -30`, `BOTH 40`, `MOTORS 20 -10`, `STOP`) so the motor hardware can be tested independently of I2C. Serial is at 115200 baud. The user specifically wanted Serial debugging retained.

The master’s intended pinout is:
```text
I2C to slave:
SDA = GPIO 8
SCL = GPIO 9

IMU:
SDA = GPIO 0
SCL = GPIO 1

Local Motor E:
IN1 = GPIO 2
IN2 = GPIO 3
PWM = GPIO 4

Local Motor F:
IN1 = GPIO 10
IN2 = GPIO 20
PWM = GPIO 21
```

Because ESP32-C3 has one hardware I2C controller, the current master design uses hardware `Wire` on GPIO8/9 for the slave and a software/bit-banged I2C implementation on GPIO0/1 for the MPU6050. The MPU is read and streamed over BLE at **50 Hz** using a 20 ms telemetry interval. The user later asked to disable the IMU’s Serial spam while keeping BLE telemetry active, so Serial should ideally only show useful events such as startup, BLE connection state, motor commands, I2C status, scan/debug output, and errors.

BLE is only on the master. The intended BLE device name is:
```text
BalloonRobot
```
with service/characteristic UUIDs:
```text
SERVICE_UUID   = 12345678-1234-1234-1234-123456789000
COMMAND_UUID   = 12345678-1234-1234-1234-123456789001
TELEMETRY_UUID = 12345678-1234-1234-1234-123456789002
```
BLE command writes are fed into the same command parser as Serial, so BLE and Serial behavior should stay identical.

The master’s I2C behavior is intended to be:
```text
C/D commands -> update stored C/D values -> transmit both to slave address 0x12
E/F commands -> drive local motors directly
ALL/MOTORS -> update both remote and local motors
STOP -> zero C/D via I2C and stop E/F locally
```

A master-side I2C debug command like `SCAN` should scan the hardware I2C bus and report whether `0x12` is detected. The user previously failed to detect the slave because the two ESP32s were not sharing ground; the required interconnect is:
```text
Master GPIO8 SDA -> Slave GPIO8 SDA
Master GPIO9 SCL -> Slave GPIO9 SCL
Master GND       -> Slave GND
```
If needed, external pull-ups around 4.7 kΩ from SDA/SCL to 3.3 V may be required.

Important historical hardware context for debugging: the user had many symptoms that looked like software bugs but were likely electrical. These included one motor speeding up when another turned off, random other-channel spinning, BLE disconnects under motor load, behavior changing when GPIO jumpers were touched, and breadboard/jumper instability. The project has used solderless breadboards heavily. Power integrity, shared ground, bad contacts, and current draw have been recurring issues. Another battery improved the collapse issue, which reinforced that the original setup had power-path problems. So if a future issue looks like “commanding one motor changes another” or “BLE dies under load,” do not assume parser logic first; check supply sag, common ground, breadboard contacts, driver power, and wiring.

The current design goal is to make the motor layer much cleaner: both master and slave use a PWMA/PWMB-style dual H-bridge interface with separate direction pins and PWM pins, rather than the earlier mixed DRV8833/TB6612 scheme. The user does not need exact RPM control yet; commands are still percentage-based PWM. They understand that raw PWM is open-loop and motor speed can vary with load/supply voltage. If discussing control quality, distinguish between commanded PWM duty cycle and actual motor speed/thrust.

One additional BLE debugging note: the user’s Python/Bleak scanner was often seeing BLE devices with `None` names and failing to find `BalloonRobot` by advertised name. A more reliable approach is to discover the master by the advertised service UUID rather than by name alone.

The user’s communication style is very direct and they want practical code quickly. Prefer complete working snippets over long theory, but mention hardware caveats when they materially explain behavior.
