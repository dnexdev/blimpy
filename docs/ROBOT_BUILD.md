# Blimpy: how to build the robot

Read out of the code on 2026-09-19 (`laptop/config.py` PHYS / BLE / FOLLOW, `PROTOCOL.md` section 6, `laptop/sim/world.py`,
`laptop/control/ble_gondola.py`, `estimator.py`, `laptop/vision/mono.py`, `detect.py`). Every number below is what the
software already assumes. The ones marked **MEASURE** are placeholders: measure them on the bench, write them in
`laptop/config.py` and log them in `calib/MEASUREMENTS.md`.

Who this is for: the two software people building the airframe, and the hardware team building the electronics
box. Words explained the first time they appear.

## 0. The whole robot in one paragraph

A plain **white latex balloon about 1.1 m across** with a small flat **gondola** (the thing that hangs under a
balloon) clipped tight under it. On the gondola: the hardware team's breadboard (Bluetooth board, two motor drivers,
the IMU), one battery, and **four little motors**: two at the back pointing forward (**L** and **R**), one sideways
through the middle (**S**), one pointing down under the middle (**V**). Nothing else flies: no camera, no mic, no
speaker. The laptop watches the room with its webcam (the AprilTag board on the floor tells it where the camera is),
talks to the gondola over Bluetooth, and does all the thinking. If the laptop goes quiet for half a second, every motor
stops.

```
                    .-'''''-.
                  .'         '.        white latex balloon, 1.10 m across
                 /             \       (48" balloon NOT fully inflated)
                |               |
                |       +       |      + = balloon centre. The software steers THIS point:
                |               |          it flies at 1.7 m above the floor
                 \             /
                  '.         .'
                    '-.___.-'
                       ||  <- neck, knot, keyring
                       || 10-15 cm of string (bridle: 3 strings to the frame)
        L o=======[ breadboard ]=======o R      rear bar, L and R 25 cm apart, both pointing FORWARD
                     [V]  (prop under the middle, pushes UP)
                      S   (sideways motor through the middle)
        ------------------------------------------- floor
```

## 1. What the software assumes (the contract between code and hardware)

| What | Value | Lives in | If it is different |
|---|---|---|---|
| Balloon diameter | **1.10 m** (MEASURE after inflating) | `PHYS["D"]`, `R_BALLOON`, `mono.BALLOON_DIAM_M` | the camera's range to the balloon scales with it: 10 % wrong diameter = 10 % wrong distance |
| Balloon colour | plain **white**, nothing stuck on the round part | `BALLOON_COLOR = "white"`, `detect.py` | the fallback detector finds "the biggest round white blob"; fins, stickers or a white cloth make the blob the wrong size |
| Gondola position | hanging **directly under the centre**, as tight as the props allow; motor plane ~0.55-0.65 m below the centre (MEASURE) | `PHYS["ARM_BELOW"]` | only the simulator's tilt model; a bigger number is fine once measured |
| Motors | **four, all reversible** (can spin both ways) | `PROTOCOL.md` s6, `protocol.mix`, `world.Balloon` | fewer motors = the code cannot brake or hold height; a balloon has no keel, drag never stops it |
| L and R | at the **rear**, thrust lines parallel to forward, **0.25 m apart** (MEASURE) | `PHYS["MOTOR_SPACING"]` | wider = more turning authority, less bending stiffness. Anything 0.2-0.3 m is fine |
| S | sideways, thrust line **through the gondola centre**, + pushes the balloon to its **left** | `protocol.mix` mS, `world` `s_yaw_arm` | a few cm off-centre gives a steady twist; the yaw integrator cancels up to 15 % duty of it. 10 cm off = trouble |
| V | vertical, prop **under the middle**, + pushes **up** | `protocol.mix` mV | the vertical axis is slow and heavy (period ~15 s); an off-centre V tilts the gondola a little, acceptable |
| Thrust per motor | ~50 gf (gram-force) at 100 %, ~12 gf at the 50 % cap, reverse gives ~60 % (MEASURE) | `PHYS["T_MAX"]`, `PHYS["REV_EFF"]`, `protocol.CAP` | the mixer never asks for more than 50 %; stronger motors are fine, the numbers go in PHYS |
| Motor start | a motor must turn by **10 % duty** | `FOLLOW["DUTY_MIN"]`, `world.DUTY_START` | commands below 10 % are sent as 0. If a motor only starts at 20 %, tell the software side |
| IMU | mounted **flat and rigid**, gyro Z axis **up**: turning the gondola counter-clockwise (seen from above) must give a **positive gz** | `BLE["GYRO_SIGN"]`, `imu_store.py`, `ble_gondola._on_imu` | wrong sign = the yaw loop fights itself and the balloon spins up. One config flag fixes it after the bench check |
| IMU line | one text line per sample, **at least 20 per second**: `A:ax,ay,az;G:gx,gy,gz;T:44.5` (gyro in deg/s, as seen on the bench 2026-09-19) | `imu_store.parse`, `BLE["GYRO_UNITS"]`, `BLE["IMU_FRESH_MS"]` | slower than 5 per second = the mixer runs open loop (no yaw feedback). Bench 2026-09-19: **one per second arrived**; being fixed (section 9, item 4) |
| Gyro at rest | the bridge learns the resting offset while disarmed: **hold the gondola still for 3 s before arming** | `BLE["GYRO_ZERO*"]`, `ble_gondola._zero_gyro` | if it is moving when you arm, the heading drifts (bench: 20 deg a minute uncorrected) |
| Motor letters | the firmware calls the motors **C D E F**; the software maps L R S V onto them | `BLE["MOTORS"]`, `BLE["SIGN"]` | set once on the bench with `ble_gondola --motor C 30` and friends (section 5, step 3) |
| Command | `MOTORS c d e f`, percent -100..100, sign = direction; `STOP`; at most 20 lines a second | `ble_gondola.to_pct`, `BLE["HZ"]` | percent = duty x 100. Normal flight stays within +-50 |
| Failsafe | no command for **500 ms**, or the Bluetooth link drops, or the laptop disarms: **every motor stops** | `protocol.FAILSAFE_MS`, the bridge | the bridge sends STOP. The FIRMWARE should also stop itself after 500 ms without a command (section 9) |
| Ultrasonic | **none on this box** (it ran out of pins, 2026-09-19). The code takes one appended to the IMU line as ` alt=123.4` (cm) if it is ever added | `BLE["ALT_KEYS"]`, `BLE["ALT_UNITS"]`, `PHYS["TOF_BELOW"]`, `estimator.update_telem` | without it, height comes from the room camera (~15 cm), enough for a 1.7 m hover; with one, ~2 cm. `pilot --relative` (no room camera) needs it, so that mode is out |
| Gondola weight | the model says **0.25 kg** (MEASURE) | `PHYS["M_GONDOLA"]` | heavier is fine as long as the balloon lifts it (section 4). Update the number |
| Trim | about **1 gram heavy** with the motors off | `world.FREE_LIFT_N` (-0.010 N) | a dead balloon sinks to the floor instead of the ceiling; V mostly pushes up, its efficient direction |
| Cruise height | balloon centre at **1.7 m**; allowed 0.9-2.4 m | `FOLLOW["Z_HOLD"]`, `AVOID["Z_MIN"/"Z_MAX"]` | the top of the balloon is at 2.25 m: the room needs a **ceiling of 2.5 m or more** and no vents above the demo area |
| Follow distance | 1.5 m centre to centre; the sim person keeps 0.9 m off the centre | `FOLLOW["D_FOLLOW"]`, `world.Person.KEEP` | the envelope edge ends up ~1 m from the person |

## 2. The balloon

- **48" white latex** (GALPADA, 50 g). Inflate to **1.10 m across**, not full size: tape measure around the fattest
  part reads **3.46 m**. Full 48" (1.22 m, tape 3.83 m) only if the gondola comes out heavy (section 4). Whatever you
  inflate to, measure it and put it in `PHYS["D"]`.
- Keep the round part **plain**: no fins, no tape, no stickers, no strings crossing it. The room camera measures the
  balloon's size in the picture to know how far away it is; anything stuck to it changes the size.
- Tie the neck, then tie a **short string loop with a keyring** around the knot. That ring is where the gondola clips
  on ("slap it on" = one clip). Keep the neck string short (5 cm): the gondola must hang tight under the balloon.
- Helium: the balloon at 1.1 m holds ~0.70 m3 (25 cubic feet). A disposable party tank is ~14 cubic feet, so you need
  two, and they are often an 80 % helium blend (less lift, see section 4). Optional Hi-Float slows the leak for a long
  demo day.
- Handle it like an egg: no rings or watches, no rough tables, keep it away from hot lights.

## 3. The gondola

The breadboard IS the gondola body. A stiff stick under its rear edge carries L and R. S sits on the right edge, V
under the middle. Battery under the middle, opposite S. Forward is the direction L and R push.

### Plan view (from above, forward = up the page)

```
                          FORWARD (+x)
                               ^
                               |
                 [front edge: nothing under it (no ultrasonic on this box)]
        +-----------------------------------------------+
        |   breadboard                                  |
        |   Bluetooth board   driver 1   driver 2       |     S motor: on the RIGHT edge, in line with the
        |   IMU flat, arrow forward, in the middle      |=== S  middle of the board. Axis left<->right.
        |                    (V under here)             |     Its prop sticks out to the right and blows
        |   battery underneath, on the LEFT half        |     air to the RIGHT -> gondola goes LEFT for "+".
        +-----------------------------------------------+
   L  o========================|========================o  R     rear bar (30 cm stick), zip-tied / hot-glued
   ^                        25 cm                       ^        under the rear edge of the board
   |                                                    |
   props BEHIND the bar, blowing BACKWARD                props: one clockwise, one counter-clockwise
   (motor axis points forward)                          (the kit has both; their twist cancels)

   LEFT = +y (S pushes this way for +)          "yaw +" = counter-clockwise seen from above = R pushes harder than L
```

### Side view (from the left, forward = left of the page)

```
   ~~~~~~~~ balloon belly ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    | neck + ring
                   /|\   three bridle strings, 10-15 cm, to: front-centre, rear-left, rear-right
                  / | \
   [us]  ===[ breadboard ]===== o R (L behind it)    <- board level: every prop tip at least 5 cm below the belly
   down     [batt] [V]                                   (a 75 mm prop reaches 3.75 cm above its motor)
                    v  V prop, 3-4 cm under the board on a standoff (a cork or foam block), blows DOWN = pushes up
```

### Rules that come straight from the code

1. **L and R point forward, parallel, 25 cm apart, at the rear.** Both forward = drive. Both backward = brake. One
   stronger than the other = turn. Put a clockwise prop on one and a counter-clockwise prop on the other.
2. **S goes through the middle.** Its thrust line must pass through the board's centre when seen from above (it may be
   at the edge of the board, that is a sideways offset and does not matter; a forward/back offset does).
3. **V under the middle, prop clear.** At least 2 cm of free air above and below the prop, nothing under it (no
   wires, no battery).
4. **IMU flat, chip facing up, glued or screwed to the board, not dangling on wires.** Its printed X arrow forward
   is nice but not required; only the gyro Z axis matters, and one config flag fixes its sign.
5. **Nothing within 3 cm of a prop disc**: strings, wires, the balloon skin, your fingers. Route the bridle strings
   to the rear bar 3 cm inboard of the motors and in front of the prop plane.
6. **Every motor on soft foam tape**, wires tied down with tape or zip ties. Vibration is the noise, and the IMU reads
   vibration as motion.
7. **Nothing else under the plate.** There is no ultrasonic on this box (it ran out of pins): the underside carries
   only the battery, the V standoff and taped leads, and nothing reaches lower than the V prop. If an ultrasonic is
   ever added: front edge, looking straight down, 10 cm or more from the V prop so the prop is not in its cone.
8. **Battery on the left half, under the board**, so the S motor on the right is balanced: the board must hang
   level from the ring. Move the battery until it does.
9. **Balance point under the ring.** Hang the finished gondola from a finger through the bridle ring: level in both
   directions. If it tips, shift the battery or shorten one string.

### Materials for the frame (any hackathon table has them)

- Rear bar: a 30 cm bamboo skewer, chopstick, carbon rod, or two straws taped together. Stiff beats light.
- Motor mounts: foam tape or hot glue; 8520 motors press into a short piece of drinking straw or a hot-glue blob.
- V standoff: a wine cork, a foam block, or a 4 cm stack of foam tape.
- Bridle: sewing thread or kite string, one keyring at the balloon, one mini carabiner (keychain clip) on the gondola.
- Zip ties, foam mounting tape, hot glue, a few coins or blu-tack for ballast.

## 4. Weight budget (weigh the real thing; do not trust this table)

The balloon at 1.10 m lifts about **630 g** with good helium (95 %), about **525 g** with a party-tank blend (80 %),
after its own 50 g skin. Inflated full (1.22 m): **880 g** / **735 g**.

| Part | grams (typical) | note |
|---|---:|---|
| breadboard, half size / full size | 40 / 85 | the biggest single weight after the battery; a half board is enough |
| Bluetooth board (Nano-class / ESP32 devkit / Uno-class) | 5 / 10 / 25 | |
| motor drivers x 2 | 5 (DRV8833 / TB6612) or 55 (L298N, big black heatsink) | see section 9: an L298N also wastes ~2 V |
| IMU (MPU6050 GY-521 style) | 2 | |
| four motors + props (8520 + 75 mm) | 20 | |
| battery: 1S LiPo 1800 mAh / 9 V block / 2S 1000 mAh | 35 / 45 / 55 | |
| wires, headers, switch | 15-25 | breadboard jumpers add up |
| frame, foam tape, glue, bridle, clip | 30-50 | |
| ballast to trim | 0-40 | coins at the centre |
| **total** | **150-330** | |

Rule: weigh the complete gondola with battery and ballast on a kitchen scale.

- **Under 450 g**: inflate to 1.10 m as planned.
- **450-700 g**: inflate to the full 1.22 m and set `PHYS["D"] = 1.22`.
- **Over 700 g**: cut weight before anything else (small protoboard instead of the breadboard, DRV8833 instead of
  L298N, one battery not two).

Write the weight into `PHYS["M_GONDOLA"]` (kg) and `calib/MEASUREMENTS.md`.

## 5. Build order (bench first, balloon last)

The full procedure, one step at a time with what you should see after each, is `docs/BUILD_STEPS.md` (36 steps in
three phases: H = the hardware team's motor box, S = everything the software people do meanwhile with no motors, J = the
join). Since 2026-09-19 late the box is two ESP32-C3 boards and two TB6612 drivers (`firmware/i2c_motor_slave/`).
This is the summary, in the old single-track order. Each step is a go / no-go for the next. Commands run from the repo root on the laptop with Bluetooth on. Section 3 of
the README has the same bench checklist with more detail.

1. **The box talks before anything is built around it.** Power the breadboard. On the laptop:
   ```powershell
   python -m laptop.control.ble_gondola --probe
   ```
   It must connect to `BalloonRobot` and print an IMU line with a parsed dict (`gz_rad` present) at least 20 times
   a second. Turn the board counter-clockwise by hand: `gz` positive. If negative, set `GYRO_SIGN = -1` in
   `config.BLE`.
2. **Motors spin with no props on.** Take the props off for this. Run each letter for 2 s:
   ```powershell
   python -m laptop.control.ble_gondola --motor C 30
   ```
   Repeat for D, E, F and write down which physical motor each letter is. Then `--motor C -30`: it must reverse.
3. **Fill in the map.** In `laptop/config.py`, `BLE["MOTORS"]` = which letter is L (rear left), R (rear right),
   S (sideways), V (vertical). Nothing to mount yet: decide which motor will be which and label them with tape.
4. **Build the frame around the board** (section 3): rear bar with L and R, S on the right edge, V under the middle,
   IMU flat, battery on the left under the board, bridle strings to a clip. Props on now, blades balanced (spin each
   on a pin; sand the heavy tip until it stops settling to one side).
5. **Teleop on the bench, gondola tied down**, props on:
   ```powershell
   python -m laptop.control.ble_gondola       # terminal 1: the bridge, leave it running
   python -m laptop.control.teleop            # terminal 2: SPACE arms
   ```
   Hold the gondola still for 3 s, SPACE, then: `w` = L and R both blow backward (gondola wants to go forward);
   `s` = both reverse; `j` = S blows to the right (gondola wants to go left); `q` = V blows down (gondola wants to go
   up); `a` = R harder than L and telemetry `yaw` rises. A motor pushing the wrong way: flip its `SIGN` to -1 in
   `config.BLE`. Every key must move exactly the motor the plan view says.
6. **Failsafe.** Ctrl+C the teleop: everything stops within half a second. Then kill the **bridge** while the motors
   run: if they keep spinning, the firmware has no timeout yet (section 9). Until it has one, keep a hand on the
   battery switch.
7. **Weigh it** (section 4). Write `M_GONDOLA` and the motor spacing into `PHYS`, both into `calib/MEASUREMENTS.md`.
8. **Measure thrust** (README 3b step 1): one motor taped upright on a kitchen scale, prop blowing up and away from
   the pan, teleop `w` x3 / x4 / x5 (30 / 40 / 50 %), read grams. Expect roughly 4.5 / 8 / 12 g for an 8520.
   `T_MAX = 9.81e-3 * grams_at_50 / 0.25` newtons. Then `s` x5 for reverse: `REV_EFF = reverse / forward`.
9. **Inflate** to the size the weight allows (section 2 and 4). Ring on the neck.
10. **Clip the gondola on.** Check that the board hangs level and every prop tip is 5 cm or more from the balloon
    skin. If not, lengthen the bridle strings a little (and measure again in step 12).
11. **Trim.** Motors off. Add coins or blu-tack at the gondola centre until, released at chest height, it takes
    5-10 s to reach the floor. That is about 1 g heavy. It will feel too heavy; it is not.
12. **Measure the hang**: tape from the floor to the balloon's equator and from the floor to the motor plane.
    `ARM_BELOW` = equator height minus motor-plane height. Balloon diameter into `D`. All into `PHYS` and
    `calib/MEASUREMENTS.md` (`TOF_BELOW` stays as it is: no ultrasonic on this box).
13. **Rerun the simulator suite** with the measured numbers, so the controller has been tested with the real vehicle:
    ```powershell
    python tools/scenarios.py --seeds 3
    ```
14. **First flight = hover only.** Laptop webcam looking at the room, mat board on the floor in view, balloon in view:
    ```powershell
    python -m laptop.vision.mono --auto-calib --show     # terminal 1: the room camera
    python -m laptop.control.ble_gondola                 # terminal 2: the bridge
    python -m laptop.control.pilot --no-voice            # terminal 3: SPACE arms; type: hover
    ```
    On arming it does a few short pushes to learn which way it faces, then holds position. Let it hover a minute
    before anyone says "follow me". Someone stands next to the battery switch the whole time.

## 6. The room (the half of the robot that stays on the ground)

```
        wall ______________________________________________
             |                                            |
             |     person   O                             |
             |              |         balloon at 1.7 m    |
             |             / \          (   )             |
             |                                            |
             |   [mat board on the floor, 0.35 m square]  |   <- tags 0-3 face up, the same way every time
             |                                            |
             |  [laptop, webcam looking across the room]  |   <- on a table 0.8-1.0 m up, mat 2-3 m in front
             |____________________________________________|
```

- The **laptop webcam is the only camera** (`SOURCES["A"] = "0"`, calibrated once: `calib/laptop_intrinsics.npz`).
  It sits on a table about 0.8-1.0 m up, looking across the room. The **mat board** (four AprilTags on one rigid
  board, surveyed once: `calib/mat.json`) lies flat on the floor 2-3 m in front of it. Auto-calibration solves where
  the camera is in 3 s; keep the lid still and nobody in front of the board for those 3 s.
- The balloon flies **3-5 m from the camera** and must be fully inside the picture (top of the balloon is at 2.25 m).
  If the top gets cut off, tilt the laptop lid back a touch, keeping the mat in the lower half of the frame.
- **Space**: ceiling 2.5 m or more, 3 x 4 m of floor, no HVAC vent or open door blowing across it (a vent gust of
  1 m/s beats the motors). The person leads and keeps 1 m off the walls.
- Voice is the laptop mic (or AirPods) and speakers. Nothing on the balloon listens or talks.
- The room's walls, table and judges' spot are numbers in `venues/default.json`, tape-measured from tag 0 on the day.

## 7. Every number to measure, and where it goes

| Measure | Put it in | Command that uses it |
|---|---|---|
| balloon diameter after inflating | `PHYS["D"]` | mono range, sim mass and drag |
| gondola weight with battery and ballast | `PHYS["M_GONDOLA"]` | sim tilt, estimator prediction |
| L to R axis spacing | `PHYS["MOTOR_SPACING"]` | sim yaw authority |
| equator to motor plane | `PHYS["ARM_BELOW"]` | sim tilt |
| grams at 50 % duty, forward and reverse | `PHYS["T_MAX"]`, `PHYS["REV_EFF"]` | estimator, sim, braking |
| which letter is which motor, and its sign | `BLE["MOTORS"]`, `BLE["SIGN"]` | the bridge |
| gyro sign | `BLE["GYRO_SIGN"]` | the bridge |
| walls, table, judges spot | `venues/default.json` | avoidance, go-to |

Every measured value also gets a dated row in `calib/MEASUREMENTS.md` so it is never lost. After changing PHYS run
`python tools/scenarios.py --seeds 3` and `python tools/control_test.py`.

## 8. Go / no-go before the first flight

- [ ] `ble_gondola --probe` shows IMU lines at 20+ per second, gz positive when turned counter-clockwise
- [ ] each teleop key moves exactly the motor the plan view says, in the direction it says
- [ ] Ctrl+C on the teleop stops every motor within half a second; the firmware timeout is confirmed or a person is on the switch
- [ ] every motor on foam tape, props balanced, nothing within 3 cm of a prop disc
- [ ] gondola weighed, PHYS updated, scenarios pass
- [ ] balloon plain white, at the diameter written in PHYS, ring on the neck
- [ ] gondola hangs level, every prop tip 5 cm or more from the skin, trimmed ~1 g heavy
- [ ] room: ceiling 2.5 m+, no vents, laptop webcam sees the mat and the whole balloon
- [ ] battery switch reachable, and someone standing next to it

## 9. Ask the hardware team (things the code cannot see)

1. **Command timeout in the firmware**: if no command arrives for 500 ms, run `STOP` yourself. Today the laptop bridge
   is the only failsafe; if the laptop process dies, the last percentages keep running.
2. **Which motor drivers**: an L298N drops about 2 V, so on a 3.7 V battery the motors see 1.7 V and barely turn.
   DRV8833 / TB6612 / MX1508 boards drop almost nothing. If it is an L298N, the battery must be 7.4 V (2S) and the
   motors must be rated for it, or swap the driver.
3. **Percent to PWM**: percent 30 should mean 30 % duty. If the firmware maps it differently (a minimum, a curve),
   tell the software side; `DUTY_MIN` and `T_MAX` will be adjusted.
4. **IMU line rate**: 20 or more notifications per second of `A:...;G:...;T:...` (gyro in deg/s). Bench 2026-09-19:
   one per second arrived. The laptop prints every notification the moment it comes in (bleak callback, no timer), so
   the slow part is between the IMU read and the radio. Quick test: add a counter to the line (`;N:123`, +1 per
   notify() call). N counting 1, 2, 3 at the laptop = the firmware notifies once a second; N jumping by ~100 = the
   radio drops them, so notify every 20-50 ms instead of every 10. `ble_gondola --probe` ends with a
   `[probe] N lines in 10 s = X per second` verdict: OK at 20+.
5. **One battery, one ground, one switch**: motors and the Bluetooth board from the same battery, a bulk capacitor
   (470-1000 uF) across the battery so four motors starting at once do not reset the board, and a physical switch
   the person next to the balloon can reach.
6. **Bluetooth link drops** were seen on the bench (2026-09-19). The bridge reconnects and sends STOP as its last
   word, but a firmware timeout (item 1) is what makes a drop safe.
