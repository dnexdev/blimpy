# Blimpy build guide (canonical)

One document for building the robot. It consolidates `docs/ROBOT_BUILD.md` (the design), `docs/BUILD_STEPS.md` (the 38
steps) and what the bench has taught us since, and it corrects both where they were wrong (section 11 lists every
correction). Where documents disagree, the order of truth is:

1. **`calib/MEASUREMENTS.md`**: anything a tape, a scale or a probe produced, with its date.
2. **`laptop/config.py`** (`PHYS`, `BLE`, `OMNI`, `FPV`): the numbers the software actually uses.
3. **This guide.**
4. `docs/ROBOT_BUILD.md`, `docs/BUILD_STEPS.md` and their HTML pages: kept for the drawings and the long "if not"
   texts; not updated any more.

Commands run from the repo root. On the Windows demo laptop that is plain `python` (no venv); anywhere else use the
venv from README section 0 (`.venv/bin/python` on macOS / Linux).

## 1. What we know, and how we know it (2026-09-19)

| Fact | Status | Source |
|---|---|---|
| Box = ESP32-C3-class board, Bluetooth LE device `BalloonRobot`, text commands `MOTORS c d e f` / `C 30` / `STOP` | measured | bench, `ble_gondola --probe` |
| Drivers: DRV8833 (letters C, D) and TB6612 (letters E, F). Both reverse. | told + sketch review | MEASUREMENTS "The box", "The firmware" |
| Two batteries: a LiPo for the ESP32, a separate one for the motors. Cells / mAh not recorded yet | told | MEASUREMENTS "The box" |
| IMU: MPU6050, line `A:ax,ay,az;G:gx,gy,gz;T:t`, gyro in **deg/s**, **49.9 lines/s** after the firmware fix (was 1/s) | measured | MEASUREMENTS "The box" |
| Gyro sign: counter-clockwise from above = positive gz, so `GYRO_SIGN = 1`. Rest offset gz about -0.3 deg/s (the bridge zeroes it while disarmed and still) | measured | same; `tools/ble_test.py` |
| IMU sat about 10 deg off flat on the breadboard | measured | same. Make it flat when it goes on the plate |
| No ultrasonic (out of pins). Height comes from the room camera; `pilot --relative` is out for this box | told | same |
| Firmware has **no command timeout**: if the laptop bridge dies, the last percentages keep running | measured + sketch | bench: STOP could not be delivered after a link loss |
| Sketch bug: `C 100` / `D 100` did nothing (digitalWrite on a PWM pin, core 3.x). Fix sent (slow decay, 20 kHz, 500 ms timeout). **Re-test C/D at 30 / 60 / 100 after flashing** | sketch review | MEASUREMENTS "The firmware" |
| Motor **E** dropped the Bluetooth link about 1.4 s into a 25 % run, twice, and the board stopped advertising | measured, **cause open** | bench. Treat as a power / wiring fault until proven otherwise (section 3) |
| Letter -> job map `L=C R=D S=E V=F`, all signs +1 | **assumed** | `config.BLE` defaults; settled in steps A6-A8 and B12 |
| Envelope 1.10 m, gondola 0.25 kg, motor spacing 0.25 m, arm 0.55 m, thrust 50 gf at 100 %, reverse 60 % | **placeholders** | `config.PHYS`, every one marked MEASURE |
| Laptop webcam intrinsics (RMS 0.22 px) and the mat board survey (0.67 px) | measured, committed | `calib/` |
| Eye on the balloon (ESP32-CAM): **not on this box** (`config.FPV SOURCE = None`) | config | README 1e describes it for when one is fitted |

## 2. The robot

A plain white 48-inch latex balloon inflated to about 1.10 m, with a small flat gondola clipped under it. On the
gondola: the hardware team's board (ESP32, two motor drivers, the IMU), the batteries, and four small reversible
motors: **L** and **R** at the rear pointing forward, **S** sideways through the middle, **V** vertical under the
middle. Nothing else flies. The laptop watches the room with its webcam, talks to the gondola over Bluetooth and does
all the thinking (mixer, failsafe, vision, voice).

Why four reversible motors: a sphere has no keel. It keeps sliding after a turn and drag will not stop it in time;
only thrust does. L + R drive, brake and steer; S kills sideways drift; V holds height, because lift changes by grams
as the hall warms and helium leaks.

Numbers the software assumes (all in `laptop/config.py`; MEASURE = placeholder until the bench says otherwise):

| Item | Value | Where |
|---|---|---|
| Envelope diameter | 1.10 m (tape round the fattest part: 3.46 m) | `PHYS D` MEASURE |
| Gondola, all in | 0.25 kg | `PHYS M_GONDOLA` MEASURE |
| L to R axis spacing | 0.25 m | `PHYS MOTOR_SPACING` MEASURE |
| Motor plane below the balloon's centre | 0.55 m in the config; **expect 0.75-0.80 m** with this hang (section 5) | `PHYS ARM_BELOW` MEASURE |
| Thrust per motor | about 50 gf at 100 %, about 12 gf at the 50 % cap (thrust goes with duty squared) | `PHYS T_MAX` MEASURE, `protocol.CAP` |
| Reverse thrust | about 60 % of forward | `PHYS REV_EFF` MEASURE |
| A motor must turn by | 10 % duty | `FOLLOW DUTY_MIN` |
| Cruise height, band | 1.7 m (balloon centre), 0.9-2.4 m | `FOLLOW Z_HOLD`, `AVOID Z_MIN / Z_MAX` |
| Follow distance | 1.5 m centre to centre | `FOLLOW D_FOLLOW` |

Lift (checked): 1.10 m is 0.70 m3. After a 50 g skin that lifts about **630 g with 95 % helium, 525 g with an 80/20
party blend** (disposable party tanks are the blend: plan on 525). Inflated full, 1.22 m: 880 / 735 g. Filling to
1.10 m takes about 25 cubic feet: two disposable tanks or one cylinder. The 50 g skin is a guess: **weigh the balloon
before inflating** (a 48-inch latex can be 70-100 g) and log it.

## 3. Power and wiring (this is where the bench has failed so far)

An 8520 coreless motor draws about 1 A at the 50 % cap and 2 A or more flat out. Breadboard contacts and jumper wires
are good for about 1 A; a few tenths of an ohm in the motor path sags the supply, heats the contacts and, with a shared
ground, glitches the ESP32. That is the first suspect for motor E killing the link.

1. **Motor current never goes through the breadboard.** Motor battery -> switch -> driver power pins -> motors, in
   soldered or screw-terminal wire, as short as the layout allows. Only signal wires (PWM, direction, I2C) use the
   breadboard.
2. **A 470-1000 uF capacitor across the motor supply, at the drivers**, not at the far end of a jumper.
3. **One common ground** between the two batteries, joined at ONE point near the drivers (a star), not daisy-chained
   through breadboard rails.
4. **The ESP32 keeps its own battery** (it has one: good). If it is ever fed from the motor battery, a single cell
   through a standard dev-board regulator browns out under motor load.
5. **One switch within reach** that cuts the MOTOR battery. Until the firmware has its 500 ms timeout that switch is
   the only failsafe when the laptop goes quiet.
6. **Trim the jumpers.** The ribbon bundles on the bench weigh tens of grams; cut every signal wire to length.
7. DRV8833 limits at about 1.5 A per channel, TB6612 at 1.2 A continuous: fine at the 50 % cap, not at a stalled motor.
   A prop that cannot turn (string, wire) trips the driver or cooks it: section 4, rule 5.

After rewiring, prove it: `--motor E 25`, then 40, then 50, each for its 2 s, link still up, `[ble] STOP` printed each
time. Then all four together through teleop (step B13). If E still drops the link with clean wiring, swap motors
between E and F: the fault follows the motor or stays with the channel, and that tells the hardware team which.

## 4. Gondola layout

```
                      FRONT (+x)
        +------------------------------------+
        |  o front bridle hole               |
        |        board (ESP32, drivers)      |
        |   [batteries: UNDER, left half]    |       S motor on the middle of the RIGHT edge,
 +y     |              (V)  <- under centre  |==[S]| shaft pointing out, prop outboard.
 left   |                        [IMU] flat  |       +S pushes the gondola LEFT (air blows right)
        |  o                              o  |
        +--+------------------------------+--+
   [L]=====|========= rear bar ===========|=====[R]      L, R: shafts point BACKWARD, props behind the bar,
   prop                                          prop     0.25 m axis to axis, one CW prop and one CCW prop
```

Rules (each comes from how the mixer and the estimator work):

1. **L and R point forward, parallel, 0.25 m apart, at the rear.** Both forward = drive, both reverse = brake, R harder
   than L = turn left (yaw +, counter-clockwise from above). A few degrees of toe-in is a steady sideways push.
2. **S's thrust line crosses the centre** seen from above. Up to 2 cm fore/aft is fine (the yaw loop cancels it); more
   is a steady twist.
3. **V under the centre, 3-4 cm below the plate on a standoff, 2 cm of free air above and below the prop**, nothing
   under it. Its thrust line then passes through the hang point: no tilt from V.
4. **IMU flat, chip up, stuck to the board, not hanging on wires.** Only the gyro's Z axis matters.
5. **Nothing within 3 cm of any prop disc**: strings, wires, skin, fingers, other motors.
6. **Every motor on soft foam tape**, every wire taped down every 5 cm. Vibration is the noise, and the IMU reads it
   as motion.
7. **Batteries under the left half** to balance S; slide them until the gondola hangs level from the clip.
8. **Check S against R before gluing.** On a full-size breadboard the plate is about 20 x 9.5 cm, S sits on the short
   right edge, and its prop disc ends up under 1 cm from the R motor at the 12.5 cm bar mark: rule 5 broken, and S
   blows across R. **Chosen fix for this build: L and R 0.30 m apart (step N4)**, which leaves about 4 cm; set
   `PHYS MOTOR_SPACING` to the measured distance. A half-size board (about 45 g lighter, 12.5 cm plate) makes 0.25 m
   fine again. (Worked out from typical part sizes: dry-fit the real parts, step B2.)

Frame materials: a stiff 30 cm stick (carbon rod, or two bamboo skewers taped together), foam board or stiff card for
the plate (the board's outline plus 2 cm all round), a cork or 4 cm foam block (V standoff), foam mounting tape,
masking tape, small zip ties, hot glue, kite string, one 2-3 cm keyring, one mini carabiner, coins and blu-tack.

## 5. The hang and the trim

The balloon keeps the ring, the gondola keeps the clip: knot the neck, capture a keyring on it with a zip tie; three
bridle strings (front-centre, rear-left, rear-right) run from the plate to one mini carabiner, plate 10-12 cm below
the clip. Rear strings pass in FRONT of the L / R prop discs. Level it by shortening the string on the low side.

- **Expected arm**: balloon radius 0.55 + neck, ring and clip about 0.12 + bridle 0.11 = about **0.75-0.80 m** from
  the balloon's centre to the motor plane. The older documents say 0.55-0.7 and "tight under the balloon"; they are
  optimistic. Measure it (step C4) and write the real number into `PHYS ARM_BELOW`.
- **Yaw**: a single-point hang lets the gondola turn on its own under the balloon. That is fine (a sphere has no
  heading; the gondola's heading is the one that matters, and the IMU rides on it), but the gondola's yaw inertia is
  around a tenth of what the simulator assumes for the whole vehicle. Expect yaw to be much snappier than in the sim.
  First hover: a finger on SPACE; if it hunts left-right, lower the yaw gain before anything else.
- **Trim target: motors off, released at chest height, it takes 12-20 s to touch the floor.** That is 1-2 g heavy.
  (The older documents say 5-10 s "about one gram": 5-10 s is really 4-8 g heavy, which is half or more of V's 12 gf
  at the cap, burnt continuously, with no margin left as helium leaks.) Heavy rather than light on purpose: on power
  loss it settles to the floor, not the ceiling, and V mostly pushes up, its efficient direction.
- **Helium leaks.** Untreated latex loses grams per hour; V has 12 gf. Re-trim right before EACH judging round, keep
  the ballast coins and the helium at hand. Hi-Float inside the balloon, if anyone can get it, buys hours.

## 6. Weight budget (typical grams; weigh the real thing)

| Part | grams | Note |
|---|---|---|
| Breadboard, half / full size | 40 / 85 | biggest item after the batteries; half is enough |
| ESP32-C3 board | 5 | |
| DRV8833 + TB6612 | 5 | |
| MPU6050 board | 2 | |
| Four 8520 motors + 75 mm props | 20 | |
| Motor battery + ESP32 LiPo | 35-55 + 10-35 | **read the labels, weigh both, log them** |
| Wires, headers, switch, capacitor | 20-40 | the jumper bundles are the variable: trim them |
| Frame, tape, glue, bridle, clip | 30-50 | |
| Ballast | 0-40 | coins at the centre |
| **Total** | **170-340** | lift 525 g (party blend) at 1.10 m |

Rule: weigh the complete gondola. Under 450 g: inflate to 1.10 m. 450-700 g: inflate to the full 1.22 m and set
`PHYS D = 1.22`. Over 700 g: cut weight first. Write the weight into `PHYS M_GONDOLA` and `calib/MEASUREMENTS.md`.

## 7. Build steps

Rules while building: a motor never runs on the bench unless the gondola is taped down and hands, hair and cables are
clear; whenever it flies, one person stands at the motor switch and does nothing else; never re-plug a motor or driver
wire with the battery on; a latex balloon pops on rings, watches, rough tables and hot lights; every measured number
goes into `calib/MEASUREMENTS.md` the moment it is measured.

Each step: **do** -> **see** -> *if not*. The long versions of the "if not" texts are in `docs/BUILD_STEPS.md`
(old step numbers in brackets).

### N. Start NOW: everything that does not need the box (one person, about 90 minutes)

The board and the motors are with the hardware team (photo 2026-09-19 17:34: full-size breadboard, tall jumper loops,
motors still wired to it). None of the steps below needs them. They end with a finished chassis waiting for the board,
an inflated balloon, and the one number nobody has yet: how many grams this balloon really lifts.

Sizes below are for the **full-size breadboard in the photo (about 16.5 x 5.5 cm)**. Measure it once with a ruler; if
it differs by more than 5 mm, plate length = board length + 4 cm, plate width = board width + 4 cm.

**Chassis (N1-N8, no helium yet)**
- **N1. Cut the plate.** Foam board or stiff card, **20.5 x 9.5 cm**. Floppy? Glue two layers. Weigh it, write the grams on it.
- **N2. Mark it (top side, marker).** One LONG edge is FRONT: draw an arrow to it and write FRONT. Draw both diagonals: the crossing is the CENTRE dot. Draw the centre line front-to-back through the dot. Write L on the rear-left corner and R on the rear-right corner (seen from above, FRONT pointing away from you). Draw the board's outline, 16.5 x 5.5, centred on the dot, long side parallel to FRONT.
- **N3. Three bridle holes**, 1 cm in from the edge, poked with a skewer: front edge on the centre line; rear-left corner; rear-right corner. Ring each hole with a drop of hot glue on both sides so the string cannot tear out.
- **N4. Rear bar.** A stiff stick **32-34 cm** long (30 cm works: see below). Mark its centre, then **15 cm to each side** (the motor seats, 30 cm apart: this is what keeps the S prop 4 cm clear of the R motor on this long plate). Only a 30 cm stick: mark 14 cm each side. Turn the plate over. Lay the stick along the REAR edge, its centre mark on the centre line, parallel to the edge, flush with it. Hot glue along the whole contact, hold 30 s. Poke two pairs of holes through the plate beside the stick and add two zip ties.
- **N5. Motor seats.** At each seat mark on TOP of the bar, stick a 2 cm strip of foam mounting tape, backing still on. On the middle of the RIGHT short edge (on the line through the CENTRE dot, perpendicular to the centre line) stick another strip on top of the plate: the S seat. The motors go on later (B4, B5): the seats are ready.
- **N6. V standoff.** Turn the plate over. Hot-glue the cork (or a 4 cm foam block) standing on the CENTRE dot's underside. Check from two sides that it is square to the plate. Foam-tape strip on its free end, backing on.
- **N7. Battery and board seats.** Underside, LEFT half, between the cork and the left edge: two 5 cm strips of velcro (hook side) for the batteries. Top side, inside the board outline: four strips of foam tape, backing on, for the board.
- **N8. Bridle.** Cut **three strings of 25 cm**. Tie one through each hole with a double knot. Gather the three free ends, and tie them together to the mini carabiner so the plate hangs **11 cm** below it. Do NOT cut the spare ends: levelling happens when the real parts are on (B11). Hang it from a finger: strings clear of the bar ends. Weigh the whole chassis; log it.

**Balloon (N9-N15). Two people for N11-N12.**
- **N9. Helium check.** Read the tank's label. Filling to 1.10 m takes about **25 cubic feet**; a jumbo disposable tank holds 14.9 (two needed, and nothing left for a top-up); a rental cylinder of 55 or more is plenty. Cylinder upright and strapped or held by a second person, valve opened slowly, nozzle pointed away from faces. Nobody inhales it.
- **N10. Prepare.** Rings, watches and lanyards off. Clear 2 m of floor and wall: nothing sharp, no hot lamp above, note where the ceiling sprinklers and lights are. Weigh the EMPTY balloon on the kitchen scale; log it (the lift budget assumes 50 g). Two tape marks on a wall at eye height, exactly **110 cm** apart.
- **N11. Inflate.** Neck over the nozzle, pinch it on with two fingers. Fill in bursts of 10-20 s. After each burst hold the balloon in front of the marks at eye height, looking straight on. **Stop when its sides just reach both marks.** A 48-inch balloon must never pass 122 cm.
- **N12. Close the neck so it can be re-opened** (it will need a top-up tomorrow): twist the neck 5 full turns, fold the twisted part back on itself, and clamp the fold with a small binder clip or a tightly wound rubber band. Then capture the keyring: a zip tie through the ring and round the neck ABOVE the clamp, snug but not cutting. Listen at the neck for 10 s: no hiss.
- **N13. Measure the real lift** (the most useful number of the evening). Clip a sandwich bag to the keyring with the carabiner of a spare keychain or a paperclip. Add coins (dime 1.75 g, nickel 3.95 g, quarter 4.4 g, loonie 6.3 g, toonie 6.9 g) and small objects until, released at chest height in still air, it neither rises nor sinks for 5 s. Take the bag off, weigh bag + contents + clip. **That is the net lift in grams**: the most the finished gondola, with batteries and ballast, may weigh. Log it with the time, and tell the hardware team the number. Expect about 525 g on a party blend, 630 g on good helium. *Under 400 g: inflate toward 122 cm (new wall marks), measure again, and `PHYS D = 1.22`.*
- **N14. Park it.** Leave the ballast bag on with 20 g extra so it sits on the floor, or tie 1 m of string from the ring to a chair leg. Away from lamps, vents, doors and foot traffic. Never outdoors, never left free under a ceiling.
- **N15. Leak rate.** One hour later repeat N13 with the same bag: the grams you had to take out = loss per hour. Log it. It tells us how often to re-trim (V has only 12 gf at the cap) and how much top-up tomorrow needs. *More than 15 g in the hour: the neck leaks. Re-do N12 further up the neck.*

**Meanwhile, ask the hardware team for three numbers** (N13 gives them their budget): the weight of the board as it will fly, wires trimmed; the weight and label of each battery; whether the full-size breadboard can become a half-size one (saves about 45 g and the long plate). Tall jumper loops snag the bridle strings that pass over the board: they need to lie flat, under 3 cm, before the board goes on the plate.

When the box comes back: section A on the bench, then B1 is already done (N1-N3, N7), B3 = N4, B6 = N6, B11 = N8: go straight to B2, B4, B5, B7 onward. With the 30 cm seats, set `PHYS MOTOR_SPACING` to the measured axis-to-axis distance (B15).

### A. The laptop and the box, nothing built yet
- **A1 (1-2)** `git pull`, then `python tools/ble_test.py` -> every line PASS. *A FAIL is a laptop problem: stop, fix that first.*
- **A2 (3)** Read the labels of BOTH batteries (cells, mAh) and the driver chips; log them. **Still open.**
- **A3** Rewire power per section 3 before any more motor tests.
- **A4 (4)** Box on, flat, still: `python -m laptop.control.ble_gondola --probe` -> connects, 10 s of `[imu] ... -> {...}` lines, verdict `... per second ...: OK` (about 50/s measured). *`not found`: box on? another phone or laptop connected to it (one connection only)? `-> None`: line format changed, send one raw line to the software side. SLOW / TOO SLOW: the firmware's notify interval went back to 1000 ms; it must be 20.*
- **A5 (5)** In the probe: `az` about +1.0 at rest (chip up); turn the board counter-clockwise from above: `gz` clearly positive, tens of deg/s. Done once already (`GYRO_SIGN = 1`); **repeat after the IMU is fixed flat to the plate**. *Negative: `GYRO_SIGN = -1`. About 0.8 where you expect about 45: the firmware switched to rad/s, set `GYRO_UNITS = "rad"`.*
- **A6 (6)** `python -m laptop.control.ble_gondola --motor C 30` (then D, E, F): one motor per letter spins 2 s, ends with `[ble] STOP`. Label each motor with its letter. *`STOP NOT DELIVERED: link is down`: cut the motor switch NOW, then it is section 3, not software. After the firmware fix also run C and D at 60 and 100.*
- **A7 (7)** `--motor C -30` etc.: every motor reverses. *One direction only: that channel is wired with one input; hardware team. No reverse = no braking and no descent.*
- **A8 (8-9)** The two motors with the longest leads become L and R. Write the map into `config.BLE MOTORS`. Check through the bridge: window 1 `python -m laptop.control.ble_gondola`, window 2 `python -m laptop.control.teleop`; hands off 3 s (gyro zero), SPACE, `w` = L and R, `j` = S only, `q` = V only, `x` = zero. The bridge line shows `BLE ARMED` and `imu` near 50 Hz.

### B. The gondola
- **B1 (10-12)** Plate = board outline + 2 cm all round. One LONG edge is FRONT: draw the arrow, the centre dot (diagonals), the centre line, three bridle holes (front-centre, two rear corners). Board on foam tape, centred, square. IMU flat, chip up, stuck down.
- **B2** Before any glue: lay L, R, S and the props on the plate and check rule 8 (S against R).
- **B3 (13)** Rear bar: mark its centre and 12.5 cm each side. Glue it under the REAR edge, centred, parallel; zip-tie through the plate as well.
- **B4 (14)** L and R on top of the bar ends, one turn of foam tape round each can, **shafts pointing backward**, axes parallel to the centre line, same height. Sight from behind.
- **B5 (15)** S on the middle of the right edge, shaft outward, its line through the centre dot. *If it has to move to the left edge, shaft pointing left: its sign will be -1 in B12.*
- **B6 (16)** V: cork on the centre dot UNDER the plate, V on the cork's free end, **shaft straight down**.
- **B7 (17)** Batteries under the left half on velcro (they come off to charge), leads taped, the motor switch reachable without passing a prop.
- **B8 (18-19)** Underside: only batteries, standoff, taped leads; nothing hangs lower than the V prop. Every wire taped every 5 cm, none within 3 cm of a prop disc, none under V. Tug each one. *A lead had to be re-plugged: repeat A6 for that letter.*
- **B9 (20)** Props. Rule: **the lettered face of the hub points the way the motor must PUSH the gondola; air leaves the unlettered side.** L = one CW prop, letters toward FRONT. R = one CCW prop, letters toward FRONT. S = letters toward the plate. V = letters UP. Each spins freely by hand. *This rule is the only thing that catches a prop fitted backwards: such a prop still passes the tissue test below, at reverse efficiency.*
- **B10 (22, optional)** Balance each prop on a pin; sand the heavy tip.
- **B11 (23)** Bridle: three 25 cm strings, double knots, to the carabiner so the plate hangs 10-12 cm below it. Level both ways from a finger through the clip. Strings clear of every prop; then cut the spare and glue the knots.
- **B12 (21)** Air direction, gondola taped down, a strip of tissue: `--motor <letter> 30` -> L and R blow BACKWARD, S blows to the RIGHT, V blows DOWN. *Wrong way: that motor's `1` becomes `-1` in `config.BLE SIGN`. Still wrong: the prop is on upside down; turn it over and put the sign back.*
- **B13 (24)** Full teleop, tied down: `www` L + R blow back · `sss` both forward · `jjj` S blows right · `lll` left · `qqq` V down · `eee` V up · `aaa` the bridge shows R's letter larger than L's. `x` between each. `BLE ARMED` throughout. *`a` makes L larger: L and R are swapped in `MOTORS`.*
- **B14 (25)** Failsafe, twice. (1) Ctrl+C the teleop with motors running: off within half a second, bridge prints `MOTORS OFF`. (2) Kill the BRIDGE process in Task Manager with motors running: **today they keep running** (no firmware timeout): cut the switch, and that is why someone stands at it. When the firmware has the 500 ms timeout, test 2 stops by itself: log the date.
- **B15 (26)** Weigh the complete gondola; measure L-R spacing. `PHYS M_GONDOLA`, `PHYS MOTOR_SPACING`, both logged. Choose the balloon size (section 6 rule).
- **B16** Thrust (README 3b): one motor taped upright on the scale blowing up, 30 / 40 / 50 % -> grams (expect about 4.5 / 8 / 12). `T_MAX = 9.81e-3 x grams_at_50 / 0.25` N; reverse at 50 % -> `REV_EFF`.

### C. The balloon
- **C1** Weigh the empty balloon; log it.
- **C2 (27-28)** Two tape marks on a wall 110 cm apart (122 if section 6 said so). Inflate slowly, checking against the marks at eye height. Twist the neck three times, fold, zip tie; a second zip tie through the keyring and round the neck. *Barely rises: party blend; go toward 122 and set `PHYS D`.*
- **C3 (29)** Clip the gondola on. Level, every prop tip 5 cm or more from the skin, strings touching nothing. *Sinks: more helium or less weight. Nose up or down: shorten the low string.*
- **C4 (30)** Trim to **12-20 s** from chest height, motors off, doors closed, no fan: dimes (1.75 g) at the centre dot for coarse steps, blu-tack for fine. *Different every try: the air is moving.*
- **C5 (31)** Gondola resting on the floor, balloon upright: floor to the equator E, floor to the motor bar M. `ARM_BELOW = E - M`; diameter against the wall marks -> `D`. Log both, then `python tools/scenarios.py --seeds 3` and `python tools/control_test.py` -> ALL PASS / PASS. *A scenario fails with the real numbers: send the table to the software side; hover-only flying is still allowed, following waits.*

### D. The room and the first flight
- **D1 (32)** Space of 3 x 4 m or more, ceiling 2.5 m or more, no vent or open door across it (a 1 m/s draught beats the motors). Laptop on a table 0.8-1.0 m up, lid still; mat board flat on the floor 2-3 m in front, the way it was surveyed; the whole mat in the lower half of the picture, headroom above for a balloon 3-5 m away up to 2.3 m high. Close every other app that holds the webcam.
- **D2 (33)** Window 1: `python -m laptop.vision.mono --auto-calib --show`, hands off 3 s -> `[auto-calib] camera laptop at world ... 4 tags reproj 0.xx px`, Z within 10 cm of the table height, then `[mono] 15 Hz ...`.
- **D3 (34)** Hold the balloon by the gondola mid-room at 1.5 m: a blue box and `balloon=[x, y, z]` with z about 1.5; a green box and `person=[...]` (feet must be in the picture). *No blue box, or it jumps to a lamp: the 10-minute site fine-tune, README section 2.*
- **D4 (35)** Window 2: `python -m laptop.control.ble_gondola`. Window 3: `python -m laptop.control.pilot --no-voice`. Balloon free, 1 m or more from every wall, untouched 3 s -> the pilot shows `safe DISARMED ... (x, y, z) ... z:cam` with real numbers.
- **D5 (36)** Person at the switch, everyone else 2 m back. SPACE: a few short pushes while it learns its heading (10-30 s), then HOVER. One full minute: within about 0.3 m of its spot, height near 1.7 m. SPACE or the switch stops it. *Spins up: SPACE at once, the yaw sign (A5, B12). Hunts left-right: yaw gain down (section 5). Climbs or sinks steadily: V's sign or the trim. Steady drift one way: a draught, not a bug. `BALLOON LOST -> disarm`: light, or it left the picture.*
- **D6 (37)** In the pilot press `t`, type `follow_me`, Enter. Stand 2 m away, fully in view, still for 10 s, then walk slowly. `t`, `hover`, Enter ends it. *It takes the largest person in the picture: others step out of view.*
- **D7 (38)** Voice. **The room camera holds webcam 0, and Windows gives a webcam to one process**, so give the voice another camera or none: `python -m laptop.control.pilot --omni-cam none` (ears only), or `--omni-cam 1` with a second camera, or `--voice local` (offline, `v` = push-to-talk). Say "Blimpy, follow me", then "Blimpy, stop". *Answers but does not move: the command safety net should catch "follow me"; send the `[omni]` lines to the software side. Not heard: `python -m laptop.voice.omni --meter`.* Rooms, the `l` key and press-to-talk: README 1g.

## 8. Every number to measure, and where it goes

| Measure | Put it in | Used by |
|---|---|---|
| battery labels and weights (both) | `calib/MEASUREMENTS.md` | weight budget, run time |
| empty balloon weight | `calib/MEASUREMENTS.md` | lift budget |
| balloon diameter after inflating | `PHYS D` | camera range to the balloon, sim mass and drag |
| gondola weight, complete with ballast | `PHYS M_GONDOLA` | sim tilt, estimator prediction |
| L to R axis spacing | `PHYS MOTOR_SPACING` | sim yaw authority |
| balloon centre to motor plane | `PHYS ARM_BELOW` | sim tilt |
| grams at 50 % duty, forward and reverse | `PHYS T_MAX`, `PHYS REV_EFF` | estimator, sim, braking |
| letter -> motor, and each sign | `BLE MOTORS`, `BLE SIGN` | the bridge |
| gyro sign and units (after the IMU is on the plate) | `BLE GYRO_SIGN`, `BLE GYRO_UNITS` | the bridge |
| walls, table, judges' spot | `venues/default.json` | avoidance, go-to |

Each gets a dated row in `calib/MEASUREMENTS.md`. After changing `PHYS`: `python tools/scenarios.py --seeds 3` and
`python tools/control_test.py`.

## 9. Go / no-go before the first flight

- [ ] Motor current runs in proper wire, capacitor at the drivers, one star ground (section 3); E runs at 50 % with the link up.
- [ ] `--probe` verdict OK (20+ lines a second), gz positive counter-clockwise, IMU flat on the plate.
- [ ] C and D work at 30 / 60 / 100 after the firmware fix.
- [ ] Every teleop key moves exactly the motor the layout says, the way it says.
- [ ] Ctrl+C on the teleop stops everything within half a second; firmware timeout confirmed, **or a person at the switch**.
- [ ] Motors on foam tape, nothing within 3 cm of a prop disc (S against R checked).
- [ ] Gondola weighed, `PHYS` updated, scenario suite passes.
- [ ] Balloon plain white, at the diameter written in `PHYS`, ring on the neck.
- [ ] Hangs level, prop tips 5 cm or more from the skin, trimmed to 12-20 s, re-trimmed within the last hour.
- [ ] Room: ceiling 2.5 m or more, no draught, the webcam sees the whole mat and the whole balloon.

## 10. Open items for the hardware team

1. **Command timeout**: no command for 500 ms = STOP. The bridge heartbeats about 4 times a second, so 500 ms is safe.
2. **Flash the fix** (slow-decay DRV8833 with `analogWrite` on both inputs, PWM 20 kHz, the timeout) and keep the copy with `TELEMETRY_INTERVAL_MS = 20` under version control; the pasted copy still said 1000.
3. **Motor E / power**: rewire per section 3, then the E test at 25 / 40 / 50 %.
4. **Battery facts**: cells, mAh, weight of both; how each is charged.
5. **Percent to PWM**: 30 means 30 % duty. A floor or a curve is fine if the software side is told (`DUTY_MIN`, `T_MAX`).
6. Bluetooth drops were seen on the bench with motors running. The bridge reconnects and its last word is STOP; the firmware timeout is what makes a drop safe.

## 11. What this guide corrects in the older documents

| Older text | Correct | Why |
|---|---|---|
| "The breadboard is the gondola body", power needs only a capacitor | motor current stays off the breadboard (section 3) | contact and jumper ratings; motor E killed the link on the bench |
| One battery | two: ESP32 LiPo + motor battery, one star ground | MEASUREMENTS "The box" |
| Trim: 5-10 s to the floor = "about one gram heavy" | 12-20 s (1-2 g); 5-10 s is 4-8 g | the fall time of about a kilogram of effective mass under 1 gf is about 15 s |
| Motor plane 0.55-0.7 m below the centre, "tight under the balloon" | expect 0.75-0.80 m; measure | radius + neck + ring + clip + bridle |
| S anywhere on the right edge | check S against R first (rule 8) | under 1 cm clearance on a full-size board |
| IMU "one line a second", go/no-go waits on it | 49.9/s measured after the firmware fix | MEASUREMENTS |
| Step 5 checks the gyro sign only | sign and units both settled (deg/s); repeat once the IMU is on the plate | MEASUREMENTS |
| 50 g balloon skin taken as given | weigh it | 48-inch latex varies |
| "Trim again before the demo" | before EACH judging round | latex leaks grams per hour against V's 12 gf |
| Step 31 points to "step 39" | the hover is step 36 (D5 here) | 38 steps |
| Step 38 starts the cloud voice next to `mono` | `--omni-cam none` (or a second camera) | one process per webcam on Windows |
| "No camera on the balloon" vs README 1e's eye | no eye on THIS box (`FPV SOURCE = None`); README 1e applies when one is fitted | config |
| Yaw behaves as simulated | snappier: the gondola turns alone under a single-point hang | inertia about a tenth of the sim's |
