# Blimpy — PROTOCOL.md

Principle: **thinking on the laptop; the gondola is a motor box.**
Sections 2-5, 7 and 8 are the contract between the control programs and `laptop/control/ble_gondola.py`, the
Bluetooth bridge, which is the only code that talks to the gondola. Section 1b is the gondola's own protocol. The
legacy WiFi firmware in `firmware/` implements sections 2-3 and 7-9 on the board itself.

## 1. Links

| Link      | Port | Direction              | Rate      | Payload            |
|-----------|------|------------------------|-----------|--------------------|
| command   | 5005 | control -> bridge (or legacy ESP32) | 20 Hz | one JSON / datagram|
| telemetry | 5006 | bridge (or legacy ESP32) -> control | 20 Hz | one JSON / datagram|
| gondola   | BLE  | bridge <-> gondola     | <= 20 Hz / IMU rate | text lines (1b) |
| state     | 5007 | vision or sim -> control (localhost) | ~15 Hz | one JSON / datagram |

All UDP. One JSON object per datagram. No newline needed. Unknown keys are ignored.
Telemetry is sent to whichever IP most recently sent a command.

## 1b. Gondola over Bluetooth LE (the hardware team's firmware)

Discover by advertised service UUID `12345678-1234-1234-1234-123456789000`, matching
`firmware/reference_control.py`; the device name `BalloonRobot` may be absent. `--name` optionally adds a name filter.
Two GATT characteristics (UUIDs in `config.BLE`):
- **command** (write without response), ASCII: `C 40` / `D -40` / `E 50` / `F 100` (one motor, percent, sign =
  direction), `ALL 30`, `MOTORS c d e f` (percent for motors C D E F), `STOP`.
- **telemetry** (notify): one IMU text line per sample. The layout is the firmware's; `config.BLE` `IMU_FIELDS` /
  `GYRO_UNITS` / `GYRO_SIGN` describe it and `laptop/control/imu_store.py` parses it (key=value, JSON or bare numbers).

The bridge maps our L/R/S/V to the letters (`config.BLE` `MOTORS`, `SIGN`), runs the section 7 mixer at 50 Hz on the
laptop with the yaw rate from the IMU, writes `MOTORS` at most 20 times a second (or every 250 ms unchanged), and
turns the IMU into section 3 telemetry: `alt` and `vbat` are -1, plus `imu_age` (ms) and `ble` (0/1). Percent = mixer
duty x 100, so normal flight stays within +-50. The latest sample is also served on `http://127.0.0.1:5008/imu`.

## 2. Command (5005)

```json
{"t":123456,"vf":0.20,"vs":0.00,"yr":0.10,"vz":0.00,"arm":1}
```
| key | meaning | range |
|-----|---------|-------|
| t   | sender timestamp, ms (int, any epoch) | |
| vf  | forward motor duty, + = forward (- = reverse thrust = braking) | -1..1 |
| vs  | sideways motor duty, + = left (body +y). Optional, default 0 | -1..1 |
| yr  | yaw-rate setpoint, + = CCW seen from above. 1.0 = YR_MAX = 1.0 rad/s | -1..1 |
| vz  | vertical motor duty, + = up | -1..1 |
| arm | 1 = motors allowed, 0 = motors off | 0/1 |

Values are motor DUTY fractions, not speeds: thrust goes roughly with duty². The laptop linearises (`lin()`).

The ESP32 clamps every value to [-1, 1], mixes, then clamps each motor to ±CAP (0.5).

## 3. Telemetry (5006)

```json
{"t":123470,"yaw":1.57,"yr":0.02,"pitch":0.01,"roll":0.00,"alt":1.62,"vbat":-1,"armed":1,"age":38,"mL":0.2,"mR":0.2,"mS":0.0,"mV":0.0}
```
| key   | meaning |
|-------|---------|
| yaw   | gyro-integrated heading, rad, wrapped to (-pi, pi], CCW+. **Drifts** — the laptop adds an offset learned from motion. |
| yr    | measured yaw rate, rad/s |
| pitch, roll | rad, complementary filter |
| alt   | m, lens to the surface below (downward VL53L0X on the gondola); -1 if no sensor / no return / > 2 m. The laptop adds `config.PHYS["TOF_BELOW"]` to get the balloon-centre height (`estimator.update_telem`). |
| vbat  | battery volts; -1 = not measured (no spare pin on the 4-motor build) |
| armed | actual motor-enable state (0/1) |
| age   | ms since last valid command; -1 if none yet. age > 500 -> failsafe |
| mL,mR,mS,mV | current motor duties L, R, sideways, vertical (debug) |

## 4. State (5007) — vision (or simulator) -> control

```json
{"t":123460,"balloon":[x,y,z],"person":[x,y,z],"person_id":3,"src":"vision"}
{"t":123470,"balloon":null,"person":null,"fpv":{"bearing":0.21,"elev":-0.05,"range":1.8,"conf":0.9,"box":[0.4,0.2,0.6,0.9]},"src":"fpv"}
```
The second form is the **eye on the balloon** (`laptop/vision/fpv.py`, or the simulator): where the person is RELATIVE
to the gondola camera. `bearing` rad, + = the person is to the LEFT (a counter-clockwise yaw centres them); `elev` rad
from the camera axis; `range` m from the person's height in the frame, `null` when the box is cut by the frame (closer
than ~1 m); `box` normalised. It carries no world position: `balloon`/`person` are null in such rows. Consumers
(`pilot.py`, `tools/scenarios.py`) call `Behaviors.on_fpv`. With no room camera at all the pilot runs `--relative`.
```
```
`null` when not seen in this frame. Metres, world frame. `src` is `"vision"` or `"sim"` (`"fused"` from laptop/positioning/fuse.py).
Recorded sessions (`laptop/positioning/session.py`) wrap each datagram verbatim in `{"kind","t_ms","wall","data"}`;
format in `laptop/positioning/schema.py`. Laptop-internal, not part of the firmware contract.

## 5. Frames and conventions

- **World**: origin = centre of the floor AprilTag (36h11, id 0). +X along the tag's left->right edge,
  +Y along the tag's bottom->top edge (as printed), **+Z up**. Metres.
- **Heading psi**: angle of the balloon's forward axis in the XY plane, measured from +X toward +Y,
  radians in (-pi, pi]. Positive yaw rate = CCW when viewed from above.
- **Body**: +x forward, +z up. `vf>0` forward, `yr>0` CCW, `vz>0` up.
- **IMU mounting**: MPU6050 chip Z axis pointing UP so gyro-Z positive = CCW from above.
  Bench check: rotate the gondola CCW by hand -> telemetry `yaw` must increase.

## 6. Motor layout (4 x 8520 + 75 mm prop, all reversible)

```
            balloon (1.1 m), gondola hangs under its centre
                     ^ forward (+x)
              L  o=======o  R      L, R: rear, axes pointing forward, ~25 cm apart. Both fwd = forward,
                    | S |                 both back = brake/reverse, difference = yaw.
                    |[V]|          S: sideways, axis through the gondola centre (no yaw coupling): strafe / kill drift.
                     ---           V: vertical, through the centre, prop below the gondola, pushing UP as +.
```
Why 4: a sphere has no keel. After a turn it keeps sliding the old way and quadratic drag will not stop it;
only thrust does. S cancels sideways velocity, L+R in reverse cancel forward velocity, V holds height
(lift changes by grams as the room warms and helium leaks; ballast alone drifts into the ceiling within a minute).
Trim the balloon ~1 gf HEAVY: on power loss it settles to the floor and V mostly pushes up (its efficient direction;
a fixed prop in reverse gives ~60 % thrust).

| normalised | meaning | constant |
|------------|---------|----------|
| yr = 1.0   | 1.0 rad/s CCW  | YR_MAX = 1.0 rad/s |
| vf, vs, vz | motor duty fractions; laptop caps them at 0.3–0.4 | CAP = 0.5 in the mixer |

## 7. Mixer (50 Hz) — `laptop/control/protocol.py::mix`, run by the bridge (legacy WiFi build: `firmware/include/mixer.h` on the board)

```
gzNorm = measured_yaw_rate / YR_MAX          (0 if no IMU -> open loop)
yawErr = yr_set - gzNorm
yawI   = clamp(yawI + KI_YR * yawErr * 0.02, -I_YR_MAX, I_YR_MAX)    KI_YR = 1.0 /s, I_YR_MAX = 0.15 (0 without IMU / when disarmed)
diff   = K_YR * yawErr + yawI                K_YR = 1.0   (the I term cancels steady torques, e.g. the S motor a few cm off-centre)
mL_t   = clamp(vf - diff, -CAP, CAP)         CAP  = 0.5
mR_t   = clamp(vf + diff, -CAP, CAP)
mS_t   = clamp(vs,        -CAP, CAP)
mV_t   = clamp(vz,        -CAP, CAP)
each motor moves toward its target by at most SLEW = 0.05 per tick (=> 0 to 0.5 in 0.2 s)
Legacy WiFi build only. Each motor = one DRV8833 channel driven as PWM + DIR (IN1 = PWM 25 kHz, IN2 = DIR):
  value >= 0: DIR low,  duty = value        (fast decay)
  value <  0: DIR high, duty = 1 - |value|  (slow decay; reverse)
The ESP32-C3 has only 6 PWM channels, which is why it is PWM+DIR and not two PWMs per motor.
```

## 8. Failsafe / arming

- Motors spin only if `arm == 1` **and** age < 500 ms **and** the gondola link is up. Otherwise the bridge sends `STOP`
  (repeated once a second while disarmed), resets the yaw integrator and reports `armed = 0`. Legacy WiFi build: all
  PWM 0, DRV8833 nSLEEP LOW.
- The Bluetooth firmware keeps the last percentages if the bridge process dies. It should add its own STOP after
  500 ms without a command (asked of the hardware team); until then the bench test in README 3 step 4 is mandatory.
- Laptop sends `arm:0` three times when a program exits.
- Legacy WiFi build: ESP32 status LED (shares GPIO 8 with nSLEEP, active low): **off = armed**, slow blink (1 Hz) = idle & talking
  to the laptop, fast blink (4 Hz) = no commands for 2 s / no WiFi.
- Legacy WiFi build: hotspot lost after boot: the ESP32 retries every 5 s and restarts mDNS when back. If the hotspot was never found
  at boot it stays on the fallback AP `blimpy-gondola` (192.168.4.1) until power-cycled.

## 9. Legacy WiFi build: ESP32-C3 SuperMini pin map (the Bluetooth gondola's wiring is the hardware team's)

| function | GPIO | DRV8833 pin | note |
|----------|------|-------------|------|
| Motor L PWM / DIR | 0 / 4   | board A: AIN1 / AIN2 | motor on AO1/AO2 |
| Motor R PWM / DIR | 1 / 5   | board A: BIN1 / BIN2 | motor on BO1/BO2 |
| Motor S PWM / DIR | 3 / 20  | board B: AIN1 / AIN2 | motor on AO1/AO2 |
| Motor V PWM / DIR | 10 / 21 | board B: BIN1 / BIN2 | motor on BO1/BO2 |
| I2C SDA / SCL | 6 / 7 | | MPU6050 @0x68, VL53L0X @0x29 |
| DRV8833 STBY (both boards) | 8 | STBY | HIGH = enabled. Shares the onboard LED: **LED lit = disarmed**. |
| Vbat sense | none | | out of pins; telemetry vbat = -1 |
| avoid | 2, 9 | | strapping pins; 18/19 = USB |
Both DRV8833 boards: VM = battery +, GND = battery - = ESP32 GND. 100 uF on each VM, 1000 uF on the battery.

Power: LiPo -> 3.3 V low-dropout regulator (>= 500 mA, e.g. HT7833 / XC6220 / ME6211 module) -> 3V3 pin.
DRV8833 VM straight from the LiPo with 100 uF on each board + 1000 uF bulk on the battery.
