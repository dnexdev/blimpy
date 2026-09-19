# Blimpy: build it, one step at a time

Sequential build of the Blimpy robot, one step at a time, for two software people and a breadboard the hardware team
has finished. Each step has **You need**, **Do**, **You should see** and **If not**. Do the steps in order. Do not
start a step while the previous step's **You should see** has not happened. Every command runs in PowerShell from the
repo root with the virtual environment active (`.\.venv\Scripts\Activate.ps1`).

The design behind these steps (what the software assumes and why) is in `docs/ROBOT_BUILD.md`.

## Rules while building

1. Whenever a motor runs on the bench, the gondola is taped or clamped to the table and hands, hair and cables are clear of the props.
2. Whenever the balloon flies, one person stands next to the battery switch and does nothing else.
3. Never re-plug a motor or driver wire while the battery is on. Switch off, change, switch on.
4. Hot glue burns. Let each joint set for 30 seconds before you let go.
5. A latex balloon pops on rings, watches, rough tables and hot lights. Handle it like an egg.
6. Write every measured number into `calib/MEASUREMENTS.md` the moment you measure it, with the date.

## What to have on the table

**From the hardware team**
- the finished breadboard: Bluetooth board, two motor drivers, IMU, four motors with leads, battery and its switch or connector
- four props: two clockwise, two counter-clockwise (the 8520 kit has both; they are marked A/B or CW/CCW)
- the ultrasonic sensor, only if their firmware already prints its reading

**Frame**
- a stiff 30 cm stick: a chopstick, a bamboo skewer pair taped together, or a carbon rod
- foam board or stiff corrugated cardboard, about 20 x 15 cm (the chassis plate)
- a wine cork, or a foam block about 4 cm tall (the standoff for the vertical motor)
- soft foam mounting tape (double-sided), masking tape, 6 small zip ties
- hot glue gun with glue, scissors or a craft knife, a marker

**Hang**
- kite string or strong sewing thread, about 1 m
- one keyring (2-3 cm) and one mini carabiner (keychain clip)
- two or three zip ties for the balloon neck

**Trim and measure**
- coins: dimes (1.75 g each), nickels (3.95 g), quarters (4.40 g); a pinch of blu-tack
- a kitchen scale, a tape measure (3 m or longer), a ruler

**Balloon**
- the 48-inch white latex balloon (plus a spare)
- helium: about 25 cubic feet for 1.10 m, so two disposable party tanks, or one cylinder

**Laptop**
- this repo, the virtual environment, Bluetooth on, the mat board (the four AprilTags on one rigid board)

## The steps

### Phase A. The laptop and the box, before building anything

#### Step 1. Get the laptop ready

**You need**
- the laptop, Bluetooth switched on in Windows Settings
- the repo at `C:\Users\raymo\Projects\blimpy`

**Do**
1. Open PowerShell in the repo folder.
2. Run `git pull`.
3. Run `.\.venv\Scripts\Activate.ps1`.
4. Open Windows Settings > Bluetooth & devices and make sure Bluetooth is On.

**You should see**
- The prompt starts with `(.venv)`.
- `git pull` says `Already up to date.` or lists the files it fetched.

**If not**
- No `.venv`: follow README section 0 (create the venv, install torch, `pip install -r requirements.txt`).
- `git pull` complains about local changes: run `git status`, ask the software side before discarding anything.

#### Step 2. Prove the laptop side works with no hardware

**You need**
- nothing but the laptop

**Do**
1. Run `python tools/ble_test.py`. It takes about 12 seconds.

**You should see**
- Every line says `PASS` and the last line reads `N/N checks passed` with the two numbers equal.

**If not**
- Any `FAIL`: stop here. It is a laptop-side problem; paste the output to the software side. Nothing in the next steps can work until this passes.

#### Step 3. Get the facts about the box from the hardware team

**You need**
- the hardware team, two minutes of their time
- `calib/MEASUREMENTS.md` open in an editor

**Do**
1. Ask: how is it switched on and off (switch, connector, which wire)?
2. Ask: what battery is it (cells, voltage, mAh) and how is it charged?
3. Ask: which motor driver chips (read the chip name off the driver boards: DRV8833, TB6612, MX1508, L298N...)?
4. Ask: does the firmware stop the motors by itself when no command arrives for 500 ms?
5. Ask: how many IMU lines per second does it send, and does it print the ultrasonic reading?
6. Write the five answers under a new heading `## The box (2026-09-19)` at the end of `calib/MEASUREMENTS.md`.

**You should see**
- Five answers written down. You know how to switch it off in a hurry.

**If not**
- Driver is an L298N on a single 3.7 V cell: the motors will barely turn (the chip eats about 2 V). Tell the hardware team now; the rest of the build still goes ahead.
- No firmware timeout: also fine for now. Until they add one, a person stays on the switch whenever motors run (rule 2).

#### Step 4. Power the box and connect to it

**You need**
- the breadboard with its battery
- the laptop

**Do**
1. Switch the box on. Keep it on the table, flat, chip side up, and do not move it.
2. Run `python -m laptop.control.ble_gondola --probe`.

**You should see**
- `[ble] scanning for 'BalloonRobot'...` then `[ble] connected to BalloonRobot at XX:XX:XX:XX:XX:XX`.
- Then a stream of lines like `[imu] 'A:-0.16,-0.01,1.08;G:-2.1,2.0,-0.4;T:44.5' -> {'ax': -0.16, ... 'gz_rad': -0.007}` for 10 seconds, then it exits.
- Several lines per second: at least 100 lines in the 10 seconds (the software wants 20 per second).

**If not**
- `not found`: is the box on? Is a phone or another laptop already connected to it (Bluetooth takes one connection)? Is Windows Bluetooth on? Move within 2 m and try again.
- Lines print but end in `-> None`: the line format is not understood. Copy one raw line to the software side.
- Fewer than 50 lines in 10 seconds: the IMU is sent too slowly. Ask the hardware team for 20 per second; the yaw loop runs open-loop below 5 per second.

#### Step 5. Check the IMU is the right way up and the right way round

**You need**
- the probe output from the previous step

**Do**
1. In the printed dicts, read `az` (the third number after `A:`). With the box flat and the IMU chip facing up it is about +1.0.
2. Run `python -m laptop.control.ble_gondola --probe` again. While it prints, turn the whole breadboard counter-clockwise, seen from above (the way you unscrew a jar), steadily, for about two seconds. Then turn it clockwise for two seconds.
3. Read `gz` (the third number after `G:`) during each turn.

**You should see**
- `az` about +1.0 at rest (between +0.9 and +1.1).
- `gz` clearly positive while turning counter-clockwise (tens of degrees per second), clearly negative while turning clockwise, back near 0 at rest.

**If not**
- `az` about -1.0: the IMU is mounted upside down. Best fix: turn the IMU board over so the chip faces up (ask the hardware team). If that is impossible, set `GYRO_SIGN=-1` in `config.BLE` (upside down flips the yaw sign).
- `gz` negative when turning counter-clockwise: open `laptop/config.py`, find `GYRO_SIGN=1` inside `BLE = dict(...)`, change it to `GYRO_SIGN=-1`, save.
- `gz` does not change when you turn: the gyro is not being read. Hardware team.

#### Step 6. Find out which letter is which motor

**You need**
- the box on the table, taped down so it cannot walk; four strips of masking tape and a marker
- props off if the hardware team has not fitted them; props on is fine as long as fingers stay clear

**Do**
1. Run `python -m laptop.control.ble_gondola --motor C 30`.
2. Watch which motor spins for 2 seconds. Stick a tape label on that motor's body: `C`.
3. Repeat with `--motor D 30`, `--motor E 30`, `--motor F 30`, labelling each motor.

**You should see**
- Each command ends with `[ble] C 30 for 2 s` then `[ble] STOP`.
- Each letter spins exactly one motor, and every motor has a letter.

**If not**
- A letter spins nothing: that driver channel or motor is dead or unplugged. Hardware team.
- Two letters spin the same motor, or one letter spins two: wiring at the drivers is crossed. Hardware team.
- `STOP NOT DELIVERED: link is down`: the Bluetooth link dropped mid-run. Switch the box off, then on, and repeat that letter.

#### Step 7. Check every motor reverses

**You need**
- the same setup

**Do**
1. Run `python -m laptop.control.ble_gondola --motor C -30`. Watch the shaft (or feel the air): it must spin the other way than with `+30`.
2. Repeat for D, E and F.

**You should see**
- All four motors spin both ways.

**If not**
- A motor spins the same way for `+30` and `-30`, or not at all for `-30`: that channel is wired for one direction only (both driver inputs are needed per motor). Hardware team. The software cannot brake or descend without reverse.

#### Step 8. Give the motors their jobs and write the map into the config

**You need**
- the four labelled motors
- `laptop/config.py` open in an editor

**Do**
1. Look at the motor leads. The two motors with the longest leads become L (rear left) and R (rear right): they sit 12.5 cm out on the bar. The other two become S (sideways) and V (vertical).
2. Add the job to each tape label: for example `C = L`, `D = R`, `E = S`, `F = V`.
3. In `laptop/config.py` find the line `MOTORS={"L": "C", "R": "D", "S": "E", "V": "F"},` inside `BLE = dict(...)`. Change the letters to match your labels. Save the file.

**You should see**
- Each motor carries a label with a letter and a job. The config line matches the labels.

**If not**
- Unsure which lead is longest: it does not matter much; leads can be extended in step 19.

#### Step 9. Check the map through the bridge

**You need**
- two PowerShell windows, both in the repo with the venv active

**Do**
1. Window 1: `python -m laptop.control.ble_gondola`. Leave it running.
2. Window 2: `python -m laptop.control.teleop`.
3. Do not touch the box for 3 seconds (the bridge learns the gyro's resting offset), then press SPACE in window 2.
4. Press `w` once. Watch the motors. Press `x`.
5. Press `j` once. Watch. Press `x`.
6. Press `q` once. Watch. Press `x`.
7. Press SPACE to disarm, then ESC. Leave window 1 running for the next steps, or Ctrl+C it.

**You should see**
- Window 1 shows `[bridge] BLE  ARMED age=  ...` and, on `w`, the L and R letters at `+10` (for example `C= +10 D= +10 E=  +0 F=  +0`). The motors labelled L and R spin.
- On `j` only the S motor spins; on `q` only the V motor spins.
- The bridge line shows `imu 20.0 Hz` or close to it.

**If not**
- A different motor spins than the label says: the map in step 8 is wrong; fix the letters and repeat.
- Window 1 says `MOTORS OFF (arm=1 age=... ble=0)`: the Bluetooth link dropped. Wait; it reconnects within a few seconds.
- Window 2 shows no telemetry (`armed=` never appears): something else is holding UDP port 5006 (an old python process). Close every other PowerShell window and try again.

### Phase B. Build the gondola around the breadboard

#### Step 10. Lay out the materials

**You need**
- everything in the materials list above

**Do**
1. Clear a table. Put the hot glue gun on to heat.
2. Cut the three bridle strings now: three pieces of 25 cm.

**You should see**
- Everything within reach; glue gun hot.

**If not**
- Missing a stick: two bamboo skewers taped together with masking tape along their whole length are stiff enough.

#### Step 11. Make the chassis plate

**You need**
- foam board or stiff cardboard, marker, ruler, knife or scissors

**Do**
1. Put the breadboard on the board and draw around it. Draw a second outline 2 cm outside the first on every side. Cut on the outer line. That is the plate.
2. Choose one LONG edge as the FRONT. Draw a big arrow on the plate pointing at it and write FRONT.
3. Draw both diagonals; their crossing is the CENTRE. Mark it with a dot. Draw the front-back centre line through the dot.
4. Poke three small holes 1 cm in from the edge: one at the front edge on the centre line, one at each rear corner. These are the bridle holes.
5. Write L on the left rear corner and R on the right rear corner, as seen from above with FRONT pointing away from you.

**You should see**
- A plate 4 cm longer and 4 cm wider than the breadboard, FRONT arrow, centre dot, centre line, three holes.

**If not**
- The board is floppy: laminate two layers with glue, or use the lid of a shoebox. A floppy plate turns thrust into wobble and the IMU reads wobble as motion.

#### Step 12. Fix the breadboard to the plate

**You need**
- the plate, the breadboard, foam tape

**Do**
1. Peel the breadboard's adhesive backing if it has one; otherwise put four strips of foam tape under it.
2. Press it onto the plate, centred on the centre dot, its long edges parallel to the plate's long edges. If the box has an ultrasonic sensor, put that end of the breadboard toward FRONT.
3. Look at the IMU. It must be flat, parallel to the plate, chip facing up, and fixed. If it hangs on jumper wires, stick it to the top of the breadboard with a square of foam tape, chip up.

**You should see**
- Breadboard centred and square on the plate. IMU flat, chip up, not moving when you tap it.

**If not**
- The IMU cannot be made flat without unplugging wires: switch the box off first, then move it, then switch on and repeat step 5.

#### Step 13. Glue the rear bar under the rear edge

**You need**
- the 30 cm stick, ruler, marker, hot glue, two zip ties

**Do**
1. Mark the stick's centre. Mark 12.5 cm to each side of the centre: the motor spots, 25 cm apart.
2. Turn the plate over. Lay the stick along the REAR edge, its centre mark on the plate's centre line, the stick parallel to the rear edge, sticking out equally on both sides.
3. Hot-glue it along its whole contact with the plate. Hold 30 seconds.
4. Optional but worth it: poke two holes through the plate either side of the stick, near each end of the plate, and zip-tie the stick to the plate.

**You should see**
- The bar is straight, centred, parallel to the rear edge, and the two motor marks are 25 cm apart and equally far from the centre line.

**If not**
- It sits crooked: reheat the glue with the gun tip, straighten, hold again.

#### Step 14. Mount L and R on the bar ends

**You need**
- the motors labelled L and R, foam tape, hot glue

**Do**
1. Wrap one turn of foam tape around the can of the L motor.
2. Turn the plate right side up. Place the L motor on TOP of the left end of the bar, its can centred on the left mark, its SHAFT POINTING BACKWARD (away from FRONT), its axis parallel to the plate's centre line.
3. Hot-glue the taped can to the bar. Hold 30 seconds.
4. Same for the R motor at the right mark: shaft backward, axis parallel to the centre line.
5. Sight along the plate from behind: both shafts point exactly backward and are parallel; both motors sit at the same height.

**You should see**
- From above, the two shafts are parallel to the FRONT arrow and point away from it, 25 cm apart. The props (later) will spin behind the bar.

**If not**
- A shaft points inward or outward: reheat and rotate the can. A few degrees of error is a steady sideways push that the software has to fight.

#### Step 15. Mount S on the right edge

**You need**
- the motor labelled S, foam tape, hot glue

**Do**
1. Find the middle of the plate's RIGHT edge (front to back). It lies on a line through the centre dot, perpendicular to the centre line.
2. Wrap the S can in foam tape. Glue it to the right edge at that middle, on top of the plate, shaft pointing OUTWARD (to the right), axis parallel to the plate's left-right direction.
3. Check from above: the shaft line passes through the centre dot when extended.

**You should see**
- The S shaft points straight out to the right, in line with the plate's centre. Its prop will spin outboard, clear of the plate.

**If not**
- It cannot sit at the exact middle because a component is in the way: up to 2 cm forward or back is acceptable (the yaw loop cancels the small twist). More than that: move it to the left edge instead, shaft pointing left; then its plus direction will need `SIGN=-1` in step 21.

#### Step 16. Mount V under the centre

**You need**
- the motor labelled V, the cork, foam tape, hot glue

**Do**
1. Turn the plate over. Hot-glue the cork standing on the centre dot (the wide end on the plate). Hold 30 seconds.
2. Wrap the V can in foam tape. Glue it to the free end of the cork with the SHAFT POINTING DOWN, straight, so the axis is vertical when the plate is level.
3. Check the shaft is plumb: stand the plate on its bar and look from two sides.

**You should see**
- V hangs 4 cm under the middle of the plate, shaft straight down. Nothing else is under the centre.

**If not**
- No cork: a stack of foam tape 3-4 cm tall, or a short piece of thick marker pen, works. The prop needs 2 cm of free air above and below it.

#### Step 17. Fit the battery

**You need**
- the battery, foam tape or velcro

**Do**
1. Switch the box off if it is on.
2. Stick the battery UNDER the plate on the LEFT half, flat, with velcro or two strips of foam tape, so it can come off for charging. The V prop hangs 4 cm lower, so the battery may sit right against the plate.
3. Route its lead to the breadboard along the plate's underside and tape it every 5 cm. The switch or connector must be reachable from outside without touching a prop.

**You should see**
- Battery firm, on the left, lead taped, switch reachable.

**If not**
- The battery is a 9 V block or a 2S pack: same rules; note its weight later in step 26.

#### Step 18. Fit the ultrasonic (only if the firmware prints its reading)

**You need**
- the ultrasonic sensor on its leads, foam tape

**Do**
1. Skip this step if step 3 said the firmware does not print an `alt` value.
2. Stick the sensor UNDER the plate at the FRONT edge on the centre line, the two round transducers pointing straight DOWN, at least 10 cm from the V motor.
3. Tape its leads along the plate to the breadboard. Nothing may hang below the sensor.

**You should see**
- Sensor face pointing at the floor, 10 cm or more ahead of V.

**If not**
- The firmware prints the reading in millimetres or metres instead of centimetres: change `ALT_UNITS="cm"` in `config.BLE` to `"mm"` or `"m"`.

#### Step 19. Route and tie down every wire

**You need**
- masking tape, zip ties

**Do**
1. Run each motor's leads along the bar or the plate to the driver, taped every 5 cm.
2. Extend a lead that does not reach: twist a jumper wire onto it and tape the joint, or have the hardware team solder it.
3. Check with the box off: no wire passes within 3 cm of where a prop will spin (behind L and R, right of S, under V). No wire crosses under the V motor.
4. Tug every wire gently: nothing moves.

**You should see**
- A tidy gondola: nothing dangles, nothing is near a prop disc.

**If not**
- A lead had to be re-plugged at the breadboard: the letter may have moved. Repeat step 6 for that letter after switching on.

#### Step 20. Fit the props the right way round

**You need**
- four props: two clockwise, two counter-clockwise (marked A/B or CW/CCW on the hub)

**Do**
1. Rule for every prop: the marked (lettered) face of the hub points in the direction the motor must PUSH the gondola; air is blown out the unmarked side.
2. L: one CW prop, lettered face toward FRONT (air will blow backward).
3. R: one CCW prop, lettered face toward FRONT. L and R must be one of each kind so their twist cancels.
4. S: any remaining prop, lettered face toward the plate (pointing LEFT); air will blow out to the right.
5. V: the last prop, lettered face UP toward the cork; air will blow down.
6. Press each prop straight down onto its shaft until it sits firm. Spin each by hand: it turns freely and touches nothing.

**You should see**
- Four props on, each spinning freely, letters facing front / front / left / up.

**If not**
- A prop hits the bar, the plate or a wire: move the wire, or slide the prop 1 mm further onto the shaft.
- A prop is loose: a drop of hot glue on the hub (not the shaft bearing).

#### Step 21. Check the air blows the right way and fix the signs

**You need**
- the gondola taped to the table by its plate, a strip of tissue paper, `laptop/config.py` open

**Do**
1. Switch the box on. Hold the tissue 10 cm BEHIND the L prop. Run `python -m laptop.control.ble_gondola --motor <L's letter> 30`.
2. Repeat for R (tissue behind the prop): air must blow BACKWARD.
3. S: tissue to the right of its prop: air must blow to the RIGHT (outboard).
4. V: tissue below its prop: air must blow DOWN.
5. For every motor whose air went the wrong way, open `laptop/config.py`, find `SIGN={"L": 1, "R": 1, "S": 1, "V": 1},` in `BLE = dict(...)` and change that motor's `1` to `-1`. Save.
6. Re-run the four commands. Now every one blows the right way.

**You should see**
- Tissue blown away from the gondola behind L and R, out to the right of S, down under V, all with a positive percent.

**If not**
- A motor blows the wrong way even after the sign flip: the prop is on upside down (lettered face the wrong way). Turn the prop over, and put the sign back to `1` if you had flipped it.
- Air is weak at 30 percent: fine, it is about 4 grams of push. Weak at 50 percent with a whine: battery low or the driver drops too much voltage (step 3).

#### Step 22. Balance the props (optional, five minutes, worth it)

**You need**
- a sewing needle or pin

**Do**
1. Pull each prop off, push the pin through the hub hole, hold the pin level and let the prop settle.
2. If one blade always drops, sand that blade's tip lightly with a fingernail file and try again, until it stays in any position.
3. Put the prop back the same way round as in step 20.

**You should see**
- Each prop stays where you leave it on the pin.

**If not**
- No time: skip. Unbalanced props buzz and shake the IMU; the software copes, the sound suffers.

#### Step 23. Tie the bridle and the clip

**You need**
- the three 25 cm strings, the mini carabiner

**Do**
1. Tie one string through the front hole, one through the rear-left hole, one through the rear-right hole. Double knots.
2. Gather the three free ends and tie them to the carabiner so that the plate hangs 10-12 cm below the carabiner. Do not cut the spare string yet.
3. Hang the gondola from a finger through the carabiner. Look from the front and from the side.
4. Not level? Re-tie the string on the LOW side a little shorter. Repeat until the plate is level both ways.
5. Check the rear strings run in FRONT of the L and R prop discs and at least 3 cm from them. Then cut the spare ends and dab the knots with hot glue.

**You should see**
- The gondola hangs level from the clip, 10-12 cm below it, strings clear of every prop.

**If not**
- It always tips to the left: the battery side is heavy; that is what the shorter string is for. Still tipping: move the battery 1 cm toward the centre.

#### Step 24. Full teleop on the bench, tied down

**You need**
- the gondola taped to the table by the plate edges, props clear of the table
- two PowerShell windows

**Do**
1. Window 1: `python -m laptop.control.ble_gondola`. Window 2: `python -m laptop.control.teleop`.
2. Hands off for 3 seconds. SPACE.
3. `w` `w` `w` (30 percent): L and R both blow backward. `x`.
4. `s` `s` `s`: both blow forward. `x`.
5. `j` `j` `j`: S blows to the right. `x`. `l` `l` `l`: S blows to the left. `x`.
6. `q` `q` `q`: V blows down. `x`. `e` `e` `e`: V blows up. `x`.
7. `a` `a` `a`: window 1 shows R's letter larger than L's (a turn to the left). `x`.
8. SPACE to disarm. Leave both windows open.

**You should see**
- Every key moves exactly the motors the plan says, in the direction it says. The bridge line stays `BLE ARMED` throughout with `imu` near 20 Hz.

**If not**
- `a` makes L larger than R: the map is mirrored; swap the L and R letters in `MOTORS` and repeat step 9 onward.
- The bridge drops to `no link` during the run: Bluetooth range or interference. Move phones away, stay within 3 m. It reconnects by itself; note it for the hardware team.

#### Step 25. Failsafe, twice

**You need**
- the same two windows, the gondola still tied down

**Do**
1. Arm with SPACE and press `w` twice. Motors run.
2. Press Ctrl+C in window 2 (teleop). Count: the motors must stop within half a second and window 1 prints `MOTORS OFF (arm=0 ...)`.
3. Restart teleop, arm, `w` twice again.
4. Now kill the BRIDGE the hard way: open Task Manager, end the `python` process of window 1 (or close its window). Watch the motors.
5. Switch the box off with the switch if the motors are still running after two seconds.

**You should see**
- Test 1: motors off within half a second of Ctrl+C.
- Test 2: motors off within about half a second on their own (the firmware timeout works), or you switched them off by hand.

**If not**
- Test 2 kept the motors running: the firmware has no timeout yet. Tell the hardware team (STOP after 500 ms without a command). Until then rule 2 is law: a person on the switch whenever it flies.

#### Step 26. Weigh the gondola and pick the balloon size

**You need**
- the kitchen scale

**Do**
1. Put the complete gondola (battery, props, bridle, clip) on the scale. Read grams.
2. Write it in `calib/MEASUREMENTS.md`: `| date | gondola, complete, before ballast | NNN g | PHYS M_GONDOLA | |`.
3. In `laptop/config.py` set `M_GONDOLA=` to the weight in kilograms (320 g = `0.32`).
4. Measure the distance between the L and R shafts with the ruler. Write `MOTOR_SPACING=` in metres (25 cm = `0.25`) and log it too.
5. Decide the balloon size: under 450 g -> inflate to 1.10 m (tape marks 110 cm apart). 450-700 g -> inflate to 1.22 m (marks 122 cm apart) and set `D=1.22`. Over 700 g -> stop and cut weight first.

**You should see**
- A weight in grams, a spacing in metres, both in the config and in MEASUREMENTS.md, and a target diameter.

**If not**
- Over 700 g: the usual culprits are a full-size breadboard (85 g), two L298N boards (55 g) and a second battery. Swap or remove before inflating; helium is not free.

### Phase C. The balloon

#### Step 27. Mark the target size

**You need**
- masking tape, the tape measure, a bare wall

**Do**
1. Stick two small pieces of tape on the wall at about eye height, exactly 110 cm apart (or 122 cm if step 26 said so). These are the size gauge.
2. Clear the floor under them: no rough edges, nothing sharp.

**You should see**
- Two marks at the chosen distance.

**If not**
- No wall: two chairs 110 cm apart, back to back, work as the gauge.

#### Step 28. Inflate and tie

**You need**
- the balloon, the helium, zip ties, a second person

**Do**
1. Inflate slowly. Stop every 20 seconds and hold the balloon in front of the marks, at eye height, looking straight on: it is done when its sides just reach both marks.
2. Twist the neck three times, fold the twisted part over, and lock the fold with a zip tie pulled tight.
3. Thread a second zip tie through the KEYRING, then around the neck right next to the first one, and pull it tight: the ring is now captured on the neck. Cut both tails.

**You should see**
- A balloon spanning the two marks, a locked neck, a keyring hanging from it. It rises on its own when let go (catch the ring).

**If not**
- It does not rise, or barely: not enough helium in it (a party blend lifts less). Inflate toward 122 cm and set `D=1.22`.
- The neck leaks (hissing): another zip tie, tighter, further up the neck.

#### Step 29. Clip the gondola on

**You need**
- the balloon, the gondola, the second person holding the balloon by the neck

**Do**
1. Clip the carabiner into the keyring.
2. Let go gently while holding the plate level; steady it; let go completely.
3. Look from the front and the side: the plate hangs level, the props are at least 5 cm from the balloon's skin, the strings touch nothing.

**You should see**
- The balloon lifts the gondola (it rises slowly or hangs in the air). Level plate, clear props.

**If not**
- It sinks straight down: the gondola is heavier than the lift. Inflate more (up to 122 cm) or cut weight.
- A prop tip is closer than 5 cm to the skin: lengthen all three bridle strings by the same amount (re-tie at the carabiner), then re-level as in step 23.
- It hangs nose-down or nose-up: shorten the string on the low side.

#### Step 30. Trim it one gram heavy

**You need**
- coins, blu-tack, tape, a room with the doors closed and no fan

**Do**
1. Hold the balloon still at chest height and let go, motors off. Count seconds until the gondola touches the floor.
2. Rises or floats without sinking: tape a dime (1.75 g) at the plate's centre dot and try again. Repeat, with dimes for coarse steps and pinches of blu-tack for fine ones.
3. Sinks in under 5 seconds: take weight off.
4. Target: touches down 5 to 10 seconds after release, from chest height.

**You should see**
- Released at chest height it drifts down and touches the floor after 5-10 seconds. It feels too heavy; it is not: that is about one gram, and the vertical motor lifts twelve.

**If not**
- It behaves differently on each try: air is moving. Close doors, turn the HVAC off in that room if you can, wait a minute.
- The balloon shrinks over the day (helium leaks): trim again before the demo.

#### Step 31. Measure the hang and finish the config

**You need**
- the tape measure, a helper to steady the balloon
- `laptop/config.py` and `calib/MEASUREMENTS.md` open

**Do**
1. Stand the balloon so the gondola rests on the floor and the balloon is upright (a helper steadies the top).
2. Measure floor to the widest point of the balloon (the equator): E. Measure floor to the motor bar: M. If there is an ultrasonic, floor to its face: U.
3. `ARM_BELOW` = E minus M. `TOF_BELOW` = E minus U. Measure the balloon's diameter D against the wall marks (or tape across the widest point, or circumference divided by 3.14).
4. In `laptop/config.py` `PHYS = dict(...)`: set `D=`, `ARM_BELOW=`, `TOF_BELOW=` (metres). `M_GONDOLA` and `MOTOR_SPACING` are already there from step 26.
5. Log every number with today's date in `calib/MEASUREMENTS.md`.
6. Run `python tools/scenarios.py --seeds 3` and `python tools/control_test.py`.

**You should see**
- `ALL PASS` from the scenarios (about a minute) and `PASS` from the control test.

**If not**
- A scenario fails with the new numbers: paste the table to the software side; a gain may need tuning for a heavier gondola. Hover-only flying (step 39) is still allowed; following waits for the fix.

### Phase D. The room and the first flight

#### Step 32. Set up the room

**You need**
- a space at least 3 x 4 m with a ceiling of 2.5 m or more, no vent or open door blowing across it
- the mat board, a table, the laptop

**Do**
1. Put the table at one edge of the space. The laptop on it, lid open, webcam facing the space, 0.8-1.0 m above the floor.
2. Lay the mat board flat on the floor 2-3 m in front of the laptop, tags up, straight, the way it was surveyed.
3. Look at the space through the webcam (Windows Camera app, then close it): the whole mat is in the lower half of the picture and there is room above for a balloon 3-5 m away up to 2.3 m high. Tilt the lid until both are true.
4. Close the Camera app. Nothing else may hold the webcam.

**You should see**
- Mat in the lower half of the frame, headroom in the upper half, no vents.

**If not**
- Only a low table: the balloon's top gets cut off. Use a higher table or a stack of boxes; the mat can move closer.

#### Step 33. Start the room camera

**You need**
- window 1 in the repo

**Do**
1. Run `python -m laptop.vision.mono --auto-calib --show`.
2. Hands off the laptop and nobody in front of the mat for 3 seconds.

**You should see**
- `[auto-calib] camera laptop at world X=... Y=... Z=...  4 tags  reproj 0.xx px  spread x.x cm`, with Z within 10 cm of the table height.
- A window titled `cam laptop` shows the picture. A status line prints once a second: `[mono] 15 Hz ... tag drift px=1.2`.

**If not**
- `mat seen in only N frames`: the board is not fully in view, too far, or too dark. Move it closer or add light and run again.
- `camera position jumped`: the lid was moving. Hold still and let it retry (it tries three times).
- Z is wildly off the table height: the mat is not flat, or the board moved since its survey. Flatten it; if a page moved, re-survey (README section 2).

#### Step 34. Check the camera sees the balloon and a person

**You need**
- the balloon, one person

**Do**
1. Hold the balloon by its gondola in the middle of the space, about 1.5 m high, and stand still.
2. Look at the `cam laptop` window and the status line.

**You should see**
- A blue box around the balloon and `balloon=[x, y, z]` in the status line with z about 1.5 (metres). A green box on the person and `person=[x, y, z]`.
- Walk a step to the right: `x` or `y` changes by about a metre per big step.

**If not**
- No blue box: the detector does not recognise this balloon in this light. Do the 10-minute site fine-tune (README section 2, `label_site.py` then `train_balloon.py --stage site`) and restart mono.
- The box jumps between the balloon and a lamp or a head: the same fine-tune fixes it.
- `person` never appears: stand fully in the picture, feet visible; the software needs the feet to know where you are.

#### Step 35. Bridge and pilot

**You need**
- windows 2 and 3 in the repo, mono still running in window 1

**Do**
1. Window 2: `python -m laptop.control.ble_gondola` (the box switched on).
2. Window 3: `python -m laptop.control.pilot --no-voice`.
3. Let the balloon float free in the middle of the space, at least 1 m from every wall, gondola still. Nobody touches it for 3 seconds.

**You should see**
- The pilot's status line shows `safe DISARMED ... | (x, y, z) psi=n/a z:cam` with real numbers in the brackets, not `none`.
- Window 2 shows `BLE`, `imu 20 Hz`.

**If not**
- `none` in the brackets: mono does not see the balloon (step 34) or is not running.
- `omni` errors or the pilot waits for a key: you started it without `--no-voice`; use `--no-voice` for the first flight.

#### Step 36. First flight: hover

**You need**
- a person on the battery switch (rule 2), everyone else 2 m back

**Do**
1. Press SPACE in the pilot window.
2. Watch the status line: `learning heading: probe axis 0`, then `commit`, `brake`, `next push`. The balloon makes a few short pushes for 10-30 seconds.
3. Then the note becomes `HOVER`. Watch it for one full minute.
4. To stop at any time: SPACE (motors off, the balloon just floats), or the switch.

**You should see**
- During the minute the balloon stays within about 0.3 m of its spot, with one small wobble every 40 seconds (the heading twitch). Height stays near 1.7 m.

**If not**
- The laptop says `I need more room to learn which way I'm facing`: it is too close to a wall. SPACE to disarm, move it to the middle, arm again.
- `BALLOON LOST -> disarm`: the camera lost it for 1.5 s. Better light, or the balloon left the picture. Move it back in view and arm again.
- It drifts steadily in one direction: a draught. Close doors; it is not a bug.
- It spins up instead of holding still: the yaw sign is wrong. SPACE immediately. Redo step 5 (`GYRO_SIGN`) and step 21.
- It climbs to the ceiling or sinks to the floor: the vertical sign is wrong (step 21, V) or the trim is far off (step 30).

#### Step 37. Follow me

**You need**
- the same three windows, the balloon hovering

**Do**
1. In the pilot window press `t`, type `follow_me`, press Enter.
2. Stand 2 m from the balloon, fully in the camera's view, and stay still for 10 seconds.
3. Walk slowly, one step every two seconds, in a circle around the space, never closer than 1 m to the balloon.
4. Press `t`, type `hover`, Enter, to stop following.

**You should see**
- The note shows `FOLLOW d=1.5` or close. The balloon turns to face you, keeps about 1.5 m from you, and comes along as you walk.

**If not**
- It follows the wrong person: it takes the largest person in the picture. Others step out of view.
- It overshoots and swings back: expected for the first minutes while the heading settles; if it does not calm down in a minute, paste the pilot's log to the software side.

#### Step 38. Voice

**You need**
- the OMNI key in the environment (`YIBU_API_KEY`), or Ollama running for the local voice

**Do**
1. Press ESC in the pilot. Restart it as `python -m laptop.control.pilot` (cloud voice) or `python -m laptop.control.pilot --voice local` (offline, `v` for push-to-talk).
2. SPACE to arm. Say: `Blimpy, follow me.` Then: `Blimpy, stop.`

**You should see**
- It answers out loud, the note changes to `FOLLOW`, then to `HOVER` on `stop`.

**If not**
- It answers but does not move: the command safety net should catch `follow me`; if the note never changes, paste the `[omni]` lines to the software side.
- Nothing heard: `python -m laptop.voice.omni --meter` shows whether the mic opens when you talk; AirPods need `--mic AirPods --spk Speakers`.

---

38 steps. Every number measured along the way lives in `laptop/config.py` (PHYS, BLE) and `calib/MEASUREMENTS.md`; the design behind the steps is `docs/ROBOT_BUILD.md`.
