# DETAILS 

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
