# Measured numbers (the ones a tape measure or ruler produced; do not guess these)

| Date | What | Value | Lives in | Notes |
|---|---|---|---|---|
| 2026-09-19 | printed AprilTag, edge of the outer black square (tag 0, `targets/apriltag36h11_id0_150mm_A4.png`) | **136.7 mm** | `laptop/config.py` `TAG_SIZE_M = 0.1367` | the 150 mm file printed at ~91 % (letter paper, "fit to page"). All four mat pages came from the same print job. Reprint = re-measure. |
| 2026-09-19 | printed checkerboard, one square (`targets/checkerboard_9x6_25mm_A4.png`) | **22.8 mm** | `laptop/config.py` `SQUARE_M = 0.0228` | same 91 % scaling (25 x 0.912 = 22.8, consistent with the tag). |
| 2026-09-19 | laptop webcam intrinsics, 1280x720, 19 checkerboard shots | **RMS 0.22 px**, f = 836 px, centre (643, 375), k1 0.057 k2 -0.115 | `calib/laptop_intrinsics.npz` (committed) | redo only if the webcam or its resolution changes. |
| 2026-09-19 | mat board survey (webcam at 2.1 m, 61 frames) | tags at (0,0) (0.347,0.012) (0.339,0.359) (-0.007,0.349) m, twists <= 3.4 deg, **residual 0.67 px** | `calib/mat.json` (committed) | the board is ~0.35 m square. Re-survey only if a page moves or is re-glued. |

Still to measure (README section 3a table): balloon diameter inflated -> `PHYS["D"]`; motor plane below the balloon
centre -> `PHYS["ARM_BELOW"]` (`PHYS["TOF_BELOW"]` only if an ultrasonic is ever fitted). The mat tag spacing is NOT
measured by hand: `tools/calib/survey_mat.py` fits it from tag 0's size and writes `calib/mat.json`.

## The box (2026-09-19, answers from the hardware team, build step 3)

| Question | Answer | What it means for the build |
|---|---|---|
| On / off | a switch | fine: that is the "person on the switch" control (rule 2) |
| Battery | TWO: a LiPo for the ESP32 (the Bluetooth board) and a separate battery for the motors. Cells / voltage / mAh not given yet: read them off the labels and add them here | both fly: both go under the plate (step 17), both count in the weight (step 26); charge both before the demo |
| Motor drivers | DRV8833 and TB6612 | good: almost no voltage drop, both directions per motor |
| Firmware stops the motors by itself after 500 ms without a command? | **No** | the laptop bridge is the only failsafe. Step 25 test 2 WILL keep the motors running: a person stays on the switch whenever motors run, and the hardware team is asked to add the timeout (ROBOT_BUILD.md section 9, item 1) |
| IMU rate | **Measured 2026-09-19, step 4 probe after the firmware fix: 499 lines in 10 s = 49.9 per second, gap median 1 ms, max 119 ms** (the radio delivers them in bursts of 3-5 every ~60 ms; the first probe that evening got 10 in 10 s until the hardware team changed the notify rate). Line `A:-0.144,-0.001,1.070;G:-2.21,1.85,-0.34;T:42.7`, az +1.07 = chip up, gz -0.3 deg/s at rest | more than the 20 the software needs; the 119 ms worst gap is inside the 200 ms `IMU_FRESH_MS` window, so the yaw loop always has a fresh sample. Step 4 ticked |
| Gyro sign (step 5) | counter-clockwise turn seen from above: gz +20 to +85 deg/s; clockwise: -40 to -94; at rest -0.3 | `GYRO_SIGN = 1` (the default) is right. gx swung ~15 during the turns: the IMU sits ~10 deg off flat on the breadboard; make it flat when it goes on the plate (step 12) |
| Ultrasonic | **none**: the box ran out of pins (2026-09-19) | step 18 has nothing to fit; height comes from the room camera (~15 cm, enough for a 1.7 m hover). `PHYS TOF_BELOW` and `BLE ALT_*` stay unused; `pilot --relative` (flying with no room camera) needs an altimeter, so that mode is out for this box |

## The firmware (2026-09-19 late, review of the sketch the hardware team pasted; arduino-esp32 core 3.x, ESP32-C3-class pins)

| Fact | Detail | Consequence |
|---|---|---|
| Pins | C = DRV8833 A (IN 5, 6), D = DRV8833 B (IN 7, 9), DRV8833 nSLEEP = GPIO 8; E = TB6612 A (IN 2, 3, PWM 4), F = TB6612 B (IN 20, 21, PWM 10), TB6612 STBY tied to 3.3 V; MPU6050 on SDA 0 / SCL 1 | GPIO 8 also drives the onboard LED on a C3 SuperMini: LED state changes while C/D run |
| Bug: `C 100` / `D 100` do nothing | the sketch does `analogWrite(pin, 0)` then `digitalWrite(pin, HIGH)`; on core 3.x a pin owned by LEDC ignores `digitalWrite` (serial log: `IO 5 is not set as GPIO`) | every 100 % test on C/D was meaningless (both inputs low = coast). E/F never `digitalWrite` a PWM pin, so they are fine |
| Fast decay on the DRV8833, slow decay on the TB6612 | DRV: IN1 = PWM, IN2 = low (coast during the off time). TB6612: PWM pin gates, IN1/IN2 fixed (short brake during the off time) | C/D start later than E/F at the same percent; not enough on its own to stop a healthy motor at 60 % |
| PWM 1 kHz (analogWrite default) | audible: this is the "whirring" | ask for 20 kHz |
| Telemetry | the pasted copy still has `TELEMETRY_INTERVAL_MS = 1000` (one notify per second: the earlier 1/s was the firmware, not the laptop); the running copy must say 20 | keep the 20 ms copy under version control |
| No command timeout | `onDisconnect` runs `allOff()` (good), nothing else stops the motors if the laptop goes quiet | the bridge heartbeats ~4/s when steady, so a 500 ms firmware timeout is safe to add |
| Fix sent | slow-decay DRV8833 recipe using only `analogWrite` on both inputs (255 / 255-pwm), 20 kHz, 500 ms timeout | see the chat of 2026-09-19 evening; re-test C/D at 30 / 60 / 100 after flashing |
| STBY floating | both drivers' STBY rows had no other wire (Raymond, 2026-09-19 late): a DRV8833 nSLEEP left floating is asleep, which fits "D worked once then died" | jumper every STBY / nSLEEP to the 3.3 V rail |
| Decision 2026-09-19 late | the DRV8833 is replaced by a second TB6612 driven by a second ESP32-C3 over I2C (slave 0x10 on the IMU bus, 2 signed bytes C, D, 500 ms timeout, four-wire slow decay): `firmware/i2c_motor_slave/` has the slave sketch, the wiring, the main-board patch and the handoff test | frees GPIO 5, 6, 7, 8, 9 on the main board; laptop side unchanged; build guide reorganised into phases H / S / J so the software side builds the balloon, frame and room meanwhile |

## The balloon (2026-09-19, build phase S)

| Date | What | Value | Goes to | Notes |
|---|---|---|---|---|
| 2026-09-19 | balloon diameter after inflating (step S4) | **1.00 m** | `laptop/config.py` `PHYS D = 1.00` (drives `R_BALLOON`, `mono.BALLOON_DIAM_M`, the sim) | 100 cm on the dot; smaller than the 1.10 m the code assumed, so the camera's range to the balloon was 10 % long until this change. Net lift, gondola weight: next rows. Scenario suite at D = 1.00: 23 of 24 pass; `follow_eye_only_walk` (no room camera, eye only: a mode this box cannot fly, no eye camera and no altimeter) now grazes the walls at 0.33 m/s (limit 0.15) where it passed at 1.10. Left as is; the room-camera scenarios all pass. |
| 2026-09-20 | balloon diameter, re-inflated for the flight | **1.15 m** | `laptop/config.py` `PHYS D = 1.15` | supersedes the 1.00 m row. Archived scenario suite green at this size (26 of 26, 5 seeds on the follow / hover / rotate set). |

## The motors and the flight box (2026-09-20, rewired; firmware with the onboard mixer)

| Date | What | Value | Goes to | Notes |
|---|---|---|---|---|
| 2026-09-20 | which letter is which motor, and which way it flies for a positive percent (Raymond, by hand, box on the bench) | **C+ = UP, E+ = LEFT, D+ and F+ = BACKWARD; D is the rear RIGHT motor, F the rear LEFT, C is under the gondola, E on the side** | `calib/motor_map.json`: `L=F- R=D- S=E+ V=C+`, gyro sign +1 -> `config.BLE MOTORS / SIGN`, sent to the box as `MAP LF- RD- SE+ VC+ G+` on every connection | left / right as seen from behind the gondola looking forward (forward = the way D- / F- push). Confirm after flashing: `python tools/motor_map.py --check`. Re-run `python tools/motor_map.py` after ANY rewiring. |
| 2026-09-20 | the motors share one supply | switching on more motors slows the others (Raymond, bench) | mixer `TOTAL_CAP = 1.2` (sum of the four duties, both in the firmware and in `protocol.py`); `config.FOLLOW A_BRAKE 0.055 / A_WALL 0.045` (was 0.08 / 0.06), `K_Z 1.3`, `Z_KI 0.15`; sim `REAL supply_sag = 0.15` | four motors at 0.4 together would take ~24 % of the volts and ~42 % of the thrust; the yaw loop is closed on the gyro ON THE BOX so L/R take up the slack when S or V start. Not measured: the sag per unit of duty (put a meter on the motor battery while `MOTORS 40 40 40 40` runs; volts at rest / volts loaded) |
| 2026-09-20 | telemetry every 200 ms (hardware team's change) | 5 lines/s | firmware `TELEMETRY_INTERVAL_MS = 200`; the yaw loop, the slew, the heading integration and the 500 ms timeout moved INTO the firmware (`CMD` setpoints from the laptop, 10/s) | at 5 Hz the laptop's yaw loop was blind (`IMU_FRESH_MS`); `python -m laptop.control.ble_gondola --probe` now says "OK, onboard mixer" at 5 lines/s |
| 2026-09-20 | lowest percent each motor starts at | **not measured yet** | `config.FOLLOW DUTY_MIN` (0.1 = 10 %) | `python tools/motor_map.py --pct 15` etc.: the lowest percent at which each prop turns |
