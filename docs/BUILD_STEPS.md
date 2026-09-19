# Blimpy: build it, one step at a time

Build of the Blimpy robot for two software people while the hardware team rebuilds the motor box.

**Where we are, 2026-09-19 late evening.** The laptop side passes its tests, the box connects as `BalloonRobot`, the IMU
streams 49.9 lines a second, the gyro sign is right, and motors E and F run. Motors C and D did not: their driver (a
DRV8833 whose STBY pin was never wired) is being replaced by a second TB6612 driven by a second ESP32 over I2C
(`firmware/i2c_motor_slave/`). That rebuild is the hardware team's job and the only thing on the critical path.

**So this guide has three phases, and two of them run at the same time.** Phase H is the hardware team's box work: you
do not do it, but you check its handoff test. Phase S is everything the two of you can do meanwhile with no motors:
the balloon, the frame, the room camera, the demo rehearsal. Phase J joins the two: the box goes onto the frame, the
frame onto the balloon, first flight. Phase J needs the Phase H handoff test passed and Phase S finished. Rough
clock: H two to four hours for the hardware team, S three to four hours for two people, J two to three hours.

Each step has **You need**, **Do**, **You should see** and **If not**. Inside a phase, do the steps in order and do not
start a step while the previous step's **You should see** has not happened. Every command runs in PowerShell from the
repo root: `cd C:\Users\raymo\Projects\blimpy`. On Raymond's laptop plain `python` has every package (the Windows Store
Python; there is no venv to activate). On any other machine make a venv first: README section 0.

The design behind these steps (what the software assumes and why) is in `docs/ROBOT_BUILD.md`.

## Rules while building

1. Whenever a motor runs on the bench, the gondola is taped or clamped to the table and hands, hair and cables are clear of the props.
2. Whenever the balloon flies, one person stands next to the battery switch and does nothing else.
3. Never re-plug a motor or driver wire while the battery is on. Switch off, change, switch on.
4. Hot glue burns. Let each joint set for 30 seconds before you let go.
5. A latex balloon pops on rings, watches, rough tables and hot lights. Handle it like an egg. Tether it (string to a chair) whenever it has no gondola on.
6. Write every measured number into `calib/MEASUREMENTS.md` the moment you measure it, with the date.
7. Two teams, two tables. Nobody touches the box while the hardware team has it open. When you need it (a weight, an outline), ask for ten seconds between their steps.

## What to have on the table

**From the hardware team (Phase H)**
- the finished box: two ESP32-C3 boards, two TB6612 drivers, IMU, four motors with leads at least 25 cm long, the ESP32 battery, the motor battery, the switch
- four props: two clockwise, two counter-clockwise (the 8520 kit has both; they are marked A/B or CW/CCW). If they are not on the motors yet, the software side sorts and balances them in Phase S

**Frame**
- a stiff 30 cm stick: a chopstick, a bamboo skewer pair taped together, or a carbon rod
- foam board or stiff corrugated cardboard, about 25 x 15 cm (the chassis plate)
- a wine cork, or a foam block about 4 cm tall (the standoff for the vertical motor)
- soft foam mounting tape (double-sided), masking tape, 6 small zip ties
- hot glue gun with glue, scissors or a craft knife, a marker

**Hang**
- kite string or strong sewing thread, about 3 m (1 m for the bridle, 2 m as the tether)
- one keyring (2-3 cm) and one mini carabiner (keychain clip)
- two or three zip ties for the balloon neck

**Trim and measure**
- coins: dimes (1.75 g each), nickels (3.95 g), quarters (4.40 g); a pinch of blu-tack
- a small zip-lock bag or paper cup to hang coins from the keyring (the lift test)
- a kitchen scale, a tape measure (3 m or longer), a ruler

**Balloon**
- the 48-inch white latex balloon (plus a spare)
- helium: about 25 cubic feet for 1.10 m, so two disposable party tanks, or one cylinder. Keep the tank in the room until Phase J: a top-up may be needed

**Laptop**
- this repo, plain `python` with the packages installed (README section 0), Bluetooth on, the mat board (the four AprilTags on one rigid board)

## The steps

### Phase H. The motor box (hardware team; runs at the same time as Phase S)

#### Step H1. Give the hardware team the folder and the parts list

**You need**
- `git pull` done
- the hardware team, five minutes of their time

**Do**
1. Send them `firmware/i2c_motor_slave/`: `i2c_motor_slave.ino` is the whole sketch for the second ESP32; `README.md` has the wiring table, the patch for the main sketch, the bench test and a fault table. Print the README or open it on their screen.
2. Agree the parts: one more ESP32-C3 SuperMini (the same board as the main one), one more TB6612 board, a dozen jumper wires. The DRV8833 comes out.
3. Ask for the CURRENT main sketch (the 20 ms telemetry copy) as a file, so it can be committed under `firmware/` next to the slave sketch.
4. Agree the handoff test with them: steps H5 and H6 below, nothing less. Phase J does not start before it passes.

**You should see**
- They have the folder open and the parts on their table. You have the current main sketch as a file.

**If not**
- No second ESP32-C3 within reach: the README's one-board fallback (both TB6612s in four-wire mode on the main board, 3 pins spare) has the same handoff test. Tell them to do that instead; nothing else in this guide changes.
- They want to keep the DRV8833 as well: no. Its STBY was floating and its 100 percent command did nothing on the old code; two identical TB6612s is one recipe instead of two.

#### Step H2. Wire the second TB6612 and the second ESP32 (hardware team)

**You need**
- the README wiring table
- the box switched off

**Do**
1. Remove the DRV8833 and its five wires from the main board (GPIO 5, 6, 7, 9 and 8 are free from now on).
2. Second TB6612: PWMA, PWMB, STBY and VCC all to the 3.3 V rail (the ESP32's 3V3 pin, not the motor battery); VM to the motor battery plus; GND to the common ground rail; motor C on AO1 / AO2, motor D on BO1 / BO2.
3. Second ESP32: GPIO 3 and 4 to AIN1 and AIN2; GPIO 5 and 6 to BIN1 and BIN2.
4. Five wires between the boards: SDA (GPIO 0 to GPIO 0, the row the IMU's SDA is on), SCL (GPIO 1 to GPIO 1), GND to the ground rail, 5V to the same row the main board's 5V pin is fed from. Keep SDA and SCL under 20 cm.
5. The FIRST TB6612 (motors E and F): jumper its STBY to the 3.3 V rail. It was floating on 2026-09-19.

**You should see**
- Second driver: STBY, PWMA, PWMB and VCC each have a wire to 3.3 V. First driver: STBY and VCC do (its PWMA and PWMB still come from GPIO 4 and 10). One ground rail with both boards, both drivers, the IMU and both batteries' minus on it. Five wires between the boards.

**If not**
- Unsure where the main board's 5V comes from: follow the wire on its 5V pin back to the battery or rail; the second board's 5V goes to the same row. Never to the motor battery if that is more than one cell.
- The second TB6612 has no VCC pin (some boards, like the old DRV8833, take logic power from VM): then just STBY, PWMA, PWMB to 3.3 V.

#### Step H3. Flash the second ESP32 with the slave sketch (hardware team)

**You need**
- Arduino IDE with the ESP32 core they already use for the main board, a USB cable
- `firmware/i2c_motor_slave/i2c_motor_slave.ino`

**Do**
1. Open the sketch. Board setting: the same ESP32-C3 entry they use for the main sketch. Upload.
2. Open the Serial Monitor at 115200 and press the board's reset button.

**You should see**
- `[slave] i2c motor slave 0x10 ready: C on 3/4, D on 5/6, timeout 500 ms`.

**If not**
- Compile error mentioning `analogWriteFrequency` or `Wire.begin`: their core is older than 3.0. The sketch has an `#if` for the frequency call; paste the error line to the software side.
- Upload fails to connect: hold BOOT while plugging the USB in, then upload; reset afterwards.
- Nothing on the Serial Monitor: wrong port (the C3 appears as a USB serial device), or the monitor was opened before the reset. Reset again.

#### Step H4. Patch the main sketch (hardware team)

**You need**
- the README section `Patch for the main BalloonRobot sketch`
- the current main sketch

**Do**
1. Delete the DRV8833 code: the CIN1 / CIN2 / DIN1 / DIN2 / DRV_STBY pins and every function that uses them, including their lines in `setup()`.
2. Paste the patch: `slaveC` / `slaveD` / `slaveDirty`, the new `motorC` and `motorD` (they only set the values), `serviceSlave()` called from `loop()` next to the MPU6050 read, and the two zero lines in `allOff()`.
3. Add the 500 ms command timeout: stamp `lastCmdMs = millis()` where a command is handled; in `loop()`, if any motor is on and `millis() - lastCmdMs > 500`, call `allOff()`.
4. Check `TELEMETRY_INTERVAL_MS` is 20. Add `analogWriteFrequency(EPWM, 20000)` and the same for FPWM before the first `analogWrite`, so E and F stop whining.
5. Upload. Serial Monitor at 115200.

**You should see**
- The main board boots and advertises `BalloonRobot`; its serial never prints `[i2c] slave 0x10 NOT ANSWERING`.

**If not**
- `NOT ANSWERING (err 2)`: the slave is not powered, the grounds are not joined, or SDA and SCL are swapped. `(err 5)`: a wire is off. The README's fault table has the rest.
- The MPU6050 stops reading after the patch: `serviceSlave()` is being called from the Bluetooth callback instead of `loop()`. Move it.

#### Step H5. Handoff test, part 1: every letter, both ways, full power

**You need**
- the box taped to the table, props off or every finger clear
- the laptop, Bluetooth on
- masking tape and a marker for the motor labels

**Do**
1. Run `python -m laptop.control.ble_gondola --probe`.
2. Run `python -m laptop.control.ble_gondola --motor C 30 --secs 5`. Stick a tape label `C` on the motor that spins. Repeat with `--motor D 30 --secs 5`, `--motor E 30 --secs 5`, `--motor F 30 --secs 5`.
3. Run the same four with `-30`. Watch the shaft or feel the air: each must spin the other way.
4. Run `--motor C 100 --secs 3` and `--motor D 100 --secs 3`.

**You should see**
- The probe ends `[probe] ... per second ...: OK`.
- Each letter spins exactly one motor and every motor has a letter. Each reverses. 100 is clearly harder than 30 (on the old driver it did nothing).

**If not**
- A letter spins nothing: the README fault table, top to bottom (driver power, STBY, motor lead, the slave's serial says `C 30 D 0` or not).
- C or D runs but stutters every few seconds: the 200 ms resend is missing from `loop()`, or `loop()` blocks longer than 500 ms somewhere.
- The IMU line stops while a motor runs: grounds joined somewhere other than the ground rail, or the motor battery sags; a 470-1000 uF capacitor across the motor battery.

#### Step H6. Handoff test, part 2: the two timeouts

**You need**
- the same setup
- two PowerShell windows

**Do**
1. Run `python -m laptop.control.ble_gondola --motor C 30 --secs 10`. While C runs, pull the SDA wire out of the second board. Count. Plug it back.
2. Window 1: `python -m laptop.control.ble_gondola`. Window 2: `python -m laptop.control.teleop`. Hands off for 3 seconds, SPACE, `w`. Motors run.
3. Close window 1 (the BRIDGE, not teleop) with the window's X button. Count.

**You should see**
- C stops within about half a second of the SDA wire coming out (the slave's own timeout).
- Every motor stops within about a second of the bridge window closing (the main board's timeout).

**If not**
- C keeps running with SDA out: the second board is not running the repo sketch (its `TIMEOUT_MS` is 500). Re-flash step H3.
- Motors keep running after the bridge dies: the main-board timeout from H4 is not in. Until it is, rule 2 is law and Phase J step J9 will fail; fix it now, it is ten lines.

#### Step H7. Hand the box over

**You need**
- the box, four labelled motors, props, both batteries
- `calib/MEASUREMENTS.md` open

**Do**
1. For each letter find the lowest `--motor X NN --secs 3` that starts the motor from rest: try 10, then 15, then 20. Write the four numbers in `calib/MEASUREMENTS.md` under `## The box`.
2. Charge both batteries now; Phase J needs them full.
3. Check every motor lead is at least 25 cm from the breadboard edge (L and R sit 12.5 cm out on a bar behind the board, plus routing). Extend short ones: a jumper wire twisted on and taped, or soldered.
4. Commit the main sketch into `firmware/` (a folder next to `i2c_motor_slave/`), so both boards' code is in the repo.
5. Carry the box, the props and the labels to the frame table. Tell the software side: `box done`.

**You should see**
- Four starting percents in the log, batteries charging, leads long enough, the box on the frame table. Phase J can start.

**If not**
- A starting percent above 10: tell the software side; `FOLLOW DUTY_MIN` in `laptop/config.py` goes up to that number.
- Leads too short: step J6 extends them, at the cost of ten minutes there.

### Phase S. Everything that needs no motors (the two of you, now)

#### Step S1. Lay out the table and split the work

**You need**
- everything in the materials list except the box
- two people

**Do**
1. `git pull`. Clear a table away from the hardware team's. Glue gun on to heat.
2. Cut the strings: three of 25 cm (the bridle) and one of 2 m (the tether).
3. Split: one of you takes the balloon (S2 to S4), the other the frame (S5 to S8). Then S9 together, then the room (S10 to S15) together.
4. Put `calib/MEASUREMENTS.md` and `laptop/config.py` open on the laptop; every number goes in the moment it exists.

**You should see**
- Everything within reach, glue gun hot, four strings cut.

**If not**
- Missing a stick: two bamboo skewers taped together along their whole length are stiff enough.
- Only one of you: do S2 to S4 first (the balloon settles while you build), then S5 to S8.

#### Step S2. Mark the target size on the wall

**You need**
- masking tape, the tape measure, a bare wall

**Do**
1. Stick two small pieces of tape on the wall at about eye height, exactly 110 cm apart. These are the size gauge. The lift test in S4 decides whether the balloon needs to grow to 122 cm later.
2. Clear the floor under them: no rough edges, nothing sharp, no ceiling light directly above.

**You should see**
- Two marks 110 cm apart.

**If not**
- No wall: two chairs 110 cm apart, back to back, work as the gauge.

#### Step S3. Inflate, tie, keyring on the neck, tether

**You need**
- the balloon, the helium, zip ties, the keyring, the 2 m string, a second person

**Do**
1. Inflate slowly. Stop every 20 seconds and hold the balloon in front of the marks, at eye height, looking straight on: it is done when its sides just reach both marks.
2. Twist the neck three times, fold the twisted part over, and lock the fold with a zip tie pulled tight.
3. Thread a second zip tie through the KEYRING, then around the neck right next to the first one, and pull it tight: the ring is captured on the neck. Cut both tails.
4. Tie the 2 m tether to the keyring and the other end to a chair. Let go.

**You should see**
- A balloon spanning the two marks, a locked neck, a keyring hanging from it. It rises to the end of the tether and stays there.

**If not**
- It does not rise, or barely: not enough helium in it, or a party blend with air in it. Move the marks to 122 cm apart and inflate to that.
- The neck leaks (hissing): another zip tie, tighter, further up the neck.

#### Step S4. Measure the balloon: diameter and lift

**You need**
- the tape measure, the coin bag, coins, the kitchen scale
- `laptop/config.py` and `calib/MEASUREMENTS.md` open

**Do**
1. Diameter: hold the tape round the widest part (the equator), read the circumference, divide by 3.14. That is D. Write it in `calib/MEASUREMENTS.md` and set `D=` in `PHYS` inside `laptop/config.py` (1.10 m = `1.10`).
2. Lift: hook the bag on the keyring (the tether stays on, slack). Add coins until the balloon neither rises nor sinks when you let it go at chest height: it should drift only a few centimetres in ten seconds. Big steps with quarters, then dimes, then blu-tack.
3. Put the bag with its coins on the scale. That number, in grams, is the net lift: everything that will hang from the keyring must weigh LESS than it, and coins in Phase J make up the difference plus one gram.
4. Log the lift: `| 2026-09-19 | balloon net lift at D=1.10 (bag of coins, neutral) | NNN g | weight budget for the gondola | |`.

**You should see**
- D between 1.05 and 1.15 m. Lift between 400 and 650 g. Both in the log, D in the config.

**If not**
- Lift under 350 g: party-blend helium or under-inflated. Cut the neck zip tie, untwist, add helium up to the 122 cm marks, tie again (S3), measure again. Set D to the new measurement.
- The balloon drifts sideways however you trim it: air is moving. Close the doors; a fan or vent makes this test meaningless.

#### Step S5. Make the chassis plate

**You need**
- foam board or stiff cardboard, marker, ruler, knife or scissors
- the breadboard's length and width

**Do**
1. Get the breadboard's size: ask the hardware team for length and width (a full-size breadboard is 16.5 x 5.5 cm, a half-size 8.2 x 5.5 cm), or borrow it for 30 seconds and draw round it. If the parts spill onto a second breadboard next to the first, treat the two as one rectangle.
2. Draw that rectangle on the board, then a second outline 2 cm outside it on every side. Cut on the outer line. That is the plate.
3. Choose one LONG edge as the FRONT. Draw a big arrow on the plate pointing at it and write FRONT.
4. Draw both diagonals; their crossing is the CENTRE. Mark it with a dot. Draw the front-back centre line through the dot.
5. Poke three small holes 1 cm in from the edge: one at the front edge on the centre line, one at each rear corner. These are the bridle holes.
6. Write L on the left rear corner and R on the right rear corner, as seen from above with FRONT pointing away from you. Write S at the middle of the right edge and V next to the centre dot.

**You should see**
- A plate 4 cm longer and 4 cm wider than the breadboard, FRONT arrow, centre dot, centre line, three holes, five labels.

**If not**
- The board is floppy: laminate two layers with glue, or use the lid of a shoebox. A floppy plate turns thrust into wobble and the IMU reads wobble as motion.

#### Step S6. Glue the rear bar under the rear edge

**You need**
- the 30 cm stick, ruler, marker, hot glue, two zip ties

**Do**
1. Mark the stick's centre. Mark 12.5 cm to each side of the centre: the motor spots, 25 cm apart.
2. Turn the plate over. Lay the stick along the REAR edge, its centre mark on the plate's centre line, the stick parallel to the rear edge, sticking out equally on both sides.
3. Hot-glue it along its whole contact with the plate. Hold 30 seconds.
4. Poke two holes through the plate either side of the stick, near each end of the plate, and zip-tie the stick to the plate.
5. Write `MOTOR_SPACING=0.25` in `PHYS` in `laptop/config.py` (it is 0.25 already; check) and log the 25 cm.

**You should see**
- The bar is straight, centred, parallel to the rear edge, and the two motor marks are 25 cm apart and equally far from the centre line.

**If not**
- It sits crooked: reheat the glue with the gun tip, straighten, hold again.

#### Step S7. Standoff, bridle strings and clip

**You need**
- the cork, the three 25 cm strings, the mini carabiner, hot glue

**Do**
1. Turn the plate over. Hot-glue the cork standing on the centre dot (the wide end on the plate). Hold 30 seconds. The V motor goes on its free end in Phase J.
2. Tie one string through the front hole, one through the rear-left hole, one through the rear-right hole. Double knots.
3. Gather the three free ends and tie them to the carabiner so that the plate hangs 10-12 cm below the carabiner. Do NOT cut the spare string: the final levelling happens in J11 with the box on.
4. Hang the empty plate from a finger through the carabiner: roughly level both ways is enough for now.
5. Check the rear strings will run in FRONT of where the L and R props spin (behind the bar) with 3 cm to spare.

**You should see**
- Cork standing on the centre, three strings to one clip, plate hanging roughly level 10-12 cm below the clip.

**If not**
- No cork: a stack of foam tape 3-4 cm tall, or a short piece of thick marker pen. The V prop needs 2 cm of free air above and below it.

#### Step S8. Sort, label and balance the props

**You need**
- the four props (if they are not already on the motors), a sewing needle or pin, a nail file

**Do**
1. Sort them: two are clockwise, two counter-clockwise; the hub is marked A/B or CW/CCW. Write the kind on a piece of tape under each prop.
2. Balance each: push the pin through the hub hole, hold the pin level and let the prop settle. If one blade always drops, file that blade's tip lightly and try again until it stays in any position.
3. Lay them out on the plate where they will go: L gets a CW prop, R a CCW prop (their twist cancels), S and V the other two.

**You should see**
- Four props, kinds known, each staying where you leave it on the pin, laid out on the plate.

**If not**
- The props are already on the motors at the hardware table: skip; balance in Phase J only if a motor buzzes.
- No time: skip the balancing. Unbalanced props buzz and shake the IMU; the software copes, the sound suffers.

#### Step S9. Weigh what exists and settle the balloon size

**You need**
- the kitchen scale
- the box for ten seconds (rule 7)
- the lift number from S4

**Do**
1. Weigh the frame: plate with bar, cork, strings, clip and props. Log it.
2. Between two of the hardware team's steps, put the box on the scale for ten seconds: both boards, both drivers, IMU, all four motors, both batteries, the switch. Log it.
3. Provisional gondola weight = frame + box + 15 g (tape, glue, coins). Set `M_GONDOLA=` in `PHYS` to that in kilograms (320 g = `0.32`) and log it as provisional.
4. Compare with the lift from S4. The gondola must come in at least 20 g UNDER the lift number.

**You should see**
- Two weights in the log, a provisional `M_GONDOLA` in the config, and the gondola under the lift budget.

**If not**
- Gondola over the lift budget: top the balloon up now, while the helium is out. Cut the neck zip tie, untwist, add helium up to 122 cm marks, tie again, re-measure D and lift (S4). Set D to the measured value.
- Still over budget at 1.22 m: cut weight. The usual culprits are a full-size breadboard (85 g), a big motor battery, four AA cells. Tell the hardware team the number.

#### Step S10. Set up the room

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

#### Step S11. Start the room camera

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

#### Step S12. Check the camera sees the real balloon and a person

**You need**
- the tethered balloon with its coin bag, one person

**Do**
1. Walk the balloon into the middle of the space on its tether; with the coin bag on it hangs at whatever height you leave it. Put it at about 1.5 m and step back, in view, feet visible, and stand still.
2. Look at the `cam laptop` window and the status line.
3. Walk a slow circle round the balloon, one step every two seconds.

**You should see**
- A blue box around the balloon and `balloon=[x, y, z]` in the status line with z about 1.5 (metres). A green box on you and `person=[x, y, z]`; `x` or `y` changes by about a metre per big step.
- The blue box stays on the balloon while you walk past it and while it turns on the string.

**If not**
- No blue box, or the box jumps to a lamp or a head: this is the moment for the 10-minute site fine-tune, and it is the best-value ten minutes of Phase S. README section 2: `python tools/dataset/label_site.py --source 0`, then `python -m laptop.vision.train_balloon --stage site`, then restart mono.
- `person` never appears: stand fully in the picture, feet visible; the software needs the feet to know where you are.
- The balloon's z is far from its real height: D in the config (S4) is wrong, or mono was started before D was changed. Restart mono.

#### Step S13. Rehearse the demo with a real person and a virtual balloon

**You need**
- mono NOT running (the rehearsal starts its own), the OMNI key in the environment (`YIBU_API_KEY`) or Ollama for the local voice
- the mat and the camera as in S10

**Do**
1. Run `python tools/rehearse.py --person real`. It starts the room camera on its own port, the simulator with a virtual balloon, and the pilot. SPACE arms.
2. Stand in view. Say `Blimpy, follow me.` Walk a slow circle. Say `Blimpy, stop.` Then `turn left`, `what do you see`, `set a timer for one minute`.
3. Run through the whole demo script once, timed. Write down every line that was misheard or ignored.
4. ESC to end. This run is also the fallback demo if the real balloon is not flying tomorrow.

**You should see**
- The plot shows the virtual balloon turning toward you and following you round the room. Blimpy answers out loud; the note goes `FOLLOW` on `follow me` and `HOVER` on `stop`.

**If not**
- Nothing heard: `python -m laptop.voice.omni --meter` shows whether the mic opens when you talk; AirPods need `--mic AirPods --spk Speakers` after `--`.
- No key or no network: `python tools/rehearse.py --person real -- --voice local` (offline, `v` for push-to-talk).
- It follows the wrong person: it takes the largest person in the picture. Others step out of view.

#### Step S14. Run the simulator suite with the real balloon's numbers

**You need**
- `laptop/config.py` with D and the provisional `M_GONDOLA` from S4 and S9

**Do**
1. Run `python tools/scenarios.py --seeds 3` (about a minute).
2. Run `python tools/control_test.py`.
3. Commit: `git add laptop/config.py calib/MEASUREMENTS.md` then `git commit -m "balloon measured: D, lift, provisional gondola weight"` and `git push`.

**You should see**
- `ALL PASS` from the scenarios and `PASS` from the control test. The push goes through.

**If not**
- A scenario fails with the new numbers: paste the table to the software side (Raymond's chat); a gain may need tuning for a heavier gondola. Hover-only flying (J13) is still allowed; following waits for the fix.
- `git push` rejected: someone else pushed; `git pull --rebase` then push again.

#### Step S15. Everything ready for the join

**You need**
- the frame, the balloon on its tether, the room set up

**Do**
1. Lay out on the frame table: the plate with bar, cork and bridle; the props; foam tape, zip ties, masking tape, the glue gun; the coins; the scale; the tape measure.
2. Two PowerShell windows open in the repo folder. Laptop charger plugged in. Phone away from the laptop (Bluetooth).
3. Write the Phase J log rows now, empty: gondola weight, starting percents, hang heights, trim coins.
4. Wait for `box done` from the hardware team. While waiting: practise the demo script once more, or help them read the fault table.

**You should see**
- Nothing left to fetch once the box arrives.

**If not**
- The hardware team is stuck for more than two hours on the second ESP32: point them at the one-board fallback in `firmware/i2c_motor_slave/README.md`. Same handoff test.

### Phase J. Join: the box onto the frame, the frame onto the balloon, first flight

#### Step J1. Give the four motors their jobs and write the map

**You need**
- the box with its four labelled motors (H5)
- `laptop/config.py` open

**Do**
1. Look at the motor leads. The two with the longest leads become L (rear left) and R (rear right): they sit 12.5 cm out on the bar. The other two become S (sideways) and V (vertical).
2. Add the job to each tape label: for example `C = L`, `D = R`, `E = S`, `F = V`.
3. In `laptop/config.py` find the line `MOTORS={"L": "C", "R": "D", "S": "E", "V": "F"},` inside `BLE = dict(...)`. Change the letters to match your labels. Save.
4. If H7 reported a starting percent above 10 for any motor, set `DUTY_MIN` in `FOLLOW` to the highest of the four (15 percent = `0.15`).

**You should see**
- Each motor carries a label with a letter and a job. The config line matches the labels.

**If not**
- Unsure which lead is longest: it does not matter much; leads get extended in J6.

#### Step J2. Fix the box to the plate, IMU flat

**You need**
- the plate, the box switched off, foam tape

**Do**
1. Peel the breadboard's adhesive backing if it has one; otherwise put four strips of foam tape under it.
2. Press it onto the plate, centred on the centre dot, its long edges parallel to the plate's long edges, the switch reachable from the edge.
3. Look at the IMU. It must be flat, parallel to the plate, chip facing up, and fixed. If it hangs on jumper wires, stick it to the top of the breadboard with a square of foam tape, chip up (it sat 10 degrees off flat on 2026-09-19).
4. Switch on. Run `python -m laptop.control.ble_gondola --probe`; while it prints, turn the whole plate counter-clockwise (seen from above) for two seconds.

**You should see**
- Box centred and square on the plate. IMU flat, chip up, not moving when you tap it. `az` about +1.0 at rest, `gz` positive during the counter-clockwise turn, the verdict `OK`.

**If not**
- `az` about -1.0: the IMU is upside down; turn the IMU board over (hardware team, box off).
- `gz` negative during the counter-clockwise turn: set `GYRO_SIGN=-1` in `BLE` in `laptop/config.py`.
- A wire came out while moving the IMU: box off, hardware team, then repeat this step.

#### Step J3. Mount L and R on the bar ends

**You need**
- the motors labelled L and R, foam tape, hot glue

**Do**
1. Wrap one turn of foam tape around the can of the L motor.
2. Turn the plate right side up. Place the L motor on TOP of the left end of the bar, its can centred on the left mark, its SHAFT POINTING BACKWARD (away from FRONT), its axis parallel to the plate's centre line.
3. Hot-glue the taped can to the bar. Hold 30 seconds.
4. Same for the R motor at the right mark: shaft backward, axis parallel to the centre line.
5. Sight along the plate from behind: both shafts point exactly backward and are parallel; both motors sit at the same height.

**You should see**
- From above, the two shafts are parallel to the FRONT arrow and point away from it, 25 cm apart. The props (J7) will spin behind the bar.

**If not**
- A shaft points inward or outward: reheat and rotate the can. A few degrees of error is a steady sideways push that the software has to fight.

#### Step J4. Mount S on the right edge and V under the cork

**You need**
- the motors labelled S and V, foam tape, hot glue

**Do**
1. S: find the middle of the plate's RIGHT edge (front to back). Wrap the S can in foam tape. Glue it to the right edge at that middle, on top of the plate, shaft pointing OUTWARD (to the right), axis parallel to the plate's left-right direction. From above, the shaft line passes through the centre dot when extended.
2. V: turn the plate over. Wrap the V can in foam tape. Glue it to the free end of the cork with the SHAFT POINTING DOWN, straight, so the axis is vertical when the plate is level.
3. Check the V shaft is plumb: stand the plate on its bar and look from two sides.

**You should see**
- S points straight out to the right, in line with the plate's centre. V hangs 4 cm under the middle, shaft straight down.

**If not**
- S cannot sit at the exact middle because a component is in the way: up to 2 cm forward or back is acceptable. More than that: the left edge instead, shaft pointing left; then `SIGN` for S becomes -1 in J7.

#### Step J5. Fit the two batteries

**You need**
- both batteries, foam tape or velcro
- the box off

**Do**
1. Stick the ESP32 battery UNDER the plate on the LEFT half, flat, with velcro or two strips of foam tape, so it can come off for charging.
2. Stick the motor battery UNDER the plate on the RIGHT half the same way. Neither may reach the cork or the space under the V prop.
3. Route both leads to the breadboard along the plate's underside and tape them every 5 cm. The switch must be reachable from outside without touching a prop.

**You should see**
- Two batteries firm, one each side of the cork, leads taped, switch reachable.

**If not**
- One battery is much heavier than the other and the plate tips when hung: move the heavy one 1-2 cm toward the centre; the bridle takes care of the rest in J11.

#### Step J6. Route every wire, check the underside

**You need**
- masking tape, zip ties

**Do**
1. Run each motor's leads along the bar or the plate to the driver, taped every 5 cm.
2. Extend a lead that does not reach: twist a jumper wire onto it and tape the joint, or have the hardware team solder it.
3. Check with the box off: no wire passes within 3 cm of where a prop will spin (behind L and R, right of S, under V). No wire crosses under the V motor.
4. Turn the gondola over. Under the plate there must be only: the two batteries, the cork with V, and taped leads. Nothing reaches lower than the V prop will (cork height plus 2 cm).
5. Tug every wire gently: nothing moves.

**You should see**
- A tidy gondola: nothing dangles, nothing is near a prop disc, a flat underside.

**If not**
- A lead had to be re-plugged at the breadboard: the letter may have moved. Switch on and repeat `--motor X 30` for that letter.
- Something hangs lower than the V prop: move it to the top side or tape it flat.

#### Step J7. Props on, air the right way, signs in the config

**You need**
- the four sorted props (S8), a strip of tissue paper
- the gondola taped to the table by its plate, `laptop/config.py` open

**Do**
1. Rule for every prop: the marked (lettered) face of the hub points in the direction the motor must PUSH the gondola; air is blown out the unmarked side.
2. L: a CW prop, lettered face toward FRONT. R: a CCW prop, lettered face toward FRONT. S: lettered face toward the plate (pointing LEFT). V: lettered face UP toward the cork. Press each straight down onto its shaft until it sits firm; spin each by hand: free, touching nothing.
3. Switch on. Hold the tissue 10 cm BEHIND the L prop. Run `python -m laptop.control.ble_gondola --motor <L's letter> 30`. Air must blow BACKWARD. Same for R.
4. S: tissue to the right of its prop: air must blow to the RIGHT. V: tissue below: air must blow DOWN.
5. For every motor whose air went the wrong way, find `SIGN={"L": 1, "R": 1, "S": 1, "V": 1},` in `BLE` and change that motor's `1` to `-1`. Save. Re-run the four commands.

**You should see**
- Four props on, each spinning freely. With a positive percent: tissue blown away behind L and R, out to the right of S, down under V.

**If not**
- A motor blows the wrong way even after the sign flip: the prop is on upside down. Turn the prop over and put the sign back to `1`.
- A prop hits the bar, the plate or a wire: move the wire, or slide the prop 1 mm further onto the shaft. Loose: a drop of hot glue on the hub, not the shaft bearing.
- Air is weak at 30 percent: fine, about 4 grams of push. Weak at 50 with a whine: battery low, or the 20 kHz change from H4 is not in.

#### Step J8. Teleop on the bench, tied down

**You need**
- the gondola taped to the table by the plate edges, props clear of the table
- two PowerShell windows

**Do**
1. Window 1: `python -m laptop.control.ble_gondola`. Window 2: `python -m laptop.control.teleop`.
2. Hands off for 3 seconds (the bridge learns the gyro's resting offset). SPACE.
3. `w` once: window 1 shows the L and R letters at `+10` and those two motors turn. `x`. `j` once: only S. `x`. `q` once: only V. `x`.
4. `w` `w` `w` (30 percent): L and R both blow backward. `x`. `s` `s` `s`: both blow forward. `x`.
5. `j` `j` `j`: S blows to the right. `x`. `l` `l` `l`: to the left. `x`. `q` `q` `q`: V blows down. `x`. `e` `e` `e`: up. `x`.
6. `a` `a` `a`: window 1 shows R's letter larger than L's (a turn to the left). `x`. SPACE to disarm. Leave both windows open.

**You should see**
- Every key moves exactly the motors the plan says, in the direction it says. The bridge line stays `BLE ARMED` throughout with `imu` near 20 Hz.

**If not**
- A different motor moves than the label says: the map in J1 is wrong; fix the letters, restart the bridge, repeat.
- `a` makes L larger than R: the map is mirrored; swap the L and R letters in `MOTORS`.
- Window 1 says `MOTORS OFF (arm=1 age=... ble=0)`: the Bluetooth link dropped. Wait; it reconnects within a few seconds. Phones away.
- Window 2 shows no telemetry: something else holds UDP port 5006 (an old python). Close every other PowerShell window and try again.

#### Step J9. Failsafe, twice

**You need**
- the same two windows, the gondola still tied down
- a person at the switch

**Do**
1. Arm with SPACE and press `w` twice. Motors run.
2. Press Ctrl+C in window 2 (teleop). Count: the motors must stop within half a second and window 1 prints `MOTORS OFF (arm=0 ...)`.
3. Restart teleop, arm, `w` twice again.
4. Now kill the BRIDGE: close window 1 with its X button. Watch the motors.
5. Switch the box off with the switch if the motors are still running after two seconds.

**You should see**
- Test 1: motors off within half a second of Ctrl+C.
- Test 2: motors off within about a second on their own (the firmware timeout from H4).

**If not**
- Test 2 kept the motors running: H6 was not really passed. The hardware team adds the 500 ms timeout now; until then rule 2 is law and the balloon does not fly free.

#### Step J10. Weigh the complete gondola

**You need**
- the kitchen scale
- the lift number from S4

**Do**
1. Put the complete gondola (both batteries, props, bridle, clip) on the scale. Read grams.
2. Write it in `calib/MEASUREMENTS.md`: `| date | gondola, complete, before ballast | NNN g | PHYS M_GONDOLA | |`. Set `M_GONDOLA=` in `PHYS` to the weight in kilograms and remove the word provisional from S9's row.
3. Compare with the lift from S4: the gondola must be under it. The difference plus one gram is the ballast J11 will add.

**You should see**
- A weight in grams, in the config and in the log, under the lift number.

**If not**
- Over the lift number: top up the balloon to 122 cm (S4 If not) or cut weight: tape, spare wire, a long lead coiled up, a battery bigger than the flight needs.

#### Step J11. Clip the gondola on, level it, trim it one gram heavy

**You need**
- the balloon (untie the tether when the gondola is on), the gondola, coins, blu-tack
- a room with the doors closed and no fan
- a second person

**Do**
1. Clip the carabiner into the keyring. Untie the tether. Let go gently while holding the plate level; steady it; let go completely.
2. Look from the front and the side: the plate hangs level, every prop tip is at least 5 cm from the balloon's skin, the strings touch nothing. Not level: re-tie the string on the LOW side a little shorter until it is; then cut the spare ends and dab the knots with hot glue.
3. Trim: hold the balloon still at chest height and let go, motors off. Count seconds until the gondola touches the floor.
4. Rises or floats without sinking: tape a dime at the plate's centre dot and try again. Repeat, dimes for coarse steps and blu-tack for fine ones. Sinks in under 5 seconds: take weight off.
5. Target: touches down 5 to 10 seconds after release. Log the coins added.

**You should see**
- Level plate, clear props, and released at chest height it drifts down and touches the floor after 5-10 seconds. It feels too heavy; it is not: that is about one gram, and the vertical motor lifts twelve.

**If not**
- A prop tip closer than 5 cm to the skin: lengthen all three bridle strings by the same amount (re-tie at the carabiner), re-level.
- It behaves differently on each try: air is moving. Close doors, HVAC off in that room, wait a minute.
- The balloon shrinks over the day (helium leaks): trim again before the demo.

#### Step J12. Measure the hang and finish the config

**You need**
- the tape measure, a helper to steady the balloon
- `laptop/config.py` and `calib/MEASUREMENTS.md` open

**Do**
1. Stand the balloon so the gondola rests on the floor and the balloon is upright (a helper steadies the top).
2. Measure floor to the widest point of the balloon (the equator): E. Measure floor to the motor bar: M. `ARM_BELOW` = E minus M.
3. In `laptop/config.py` `PHYS = dict(...)`: set `ARM_BELOW=` (metres). D, `M_GONDOLA` and `MOTOR_SPACING` are already there. Leave `TOF_BELOW` alone (no ultrasonic on this box).
4. Log every number with today's date in `calib/MEASUREMENTS.md`.
5. Run `python tools/scenarios.py --seeds 3` and `python tools/control_test.py`. Commit and push `laptop/config.py` and `calib/MEASUREMENTS.md`.

**You should see**
- `ALL PASS` from the scenarios (about a minute) and `PASS` from the control test.

**If not**
- A scenario fails with the new numbers: paste the table to Raymond's chat; a gain may need tuning. Hover-only flying (J13) is still allowed; following waits for the fix.

#### Step J13. Room camera, bridge, pilot: first flight is a hover

**You need**
- three PowerShell windows in the repo, the room as in S10
- a person on the battery switch (rule 2), everyone else 2 m back

**Do**
1. Window 1: `python -m laptop.vision.mono --auto-calib --show` (3 seconds hands off for the auto-calib).
2. Window 2: `python -m laptop.control.ble_gondola` (the box switched on). Window 3: `python -m laptop.control.pilot --no-voice`.
3. Let the balloon float free in the middle of the space, at least 1 m from every wall, gondola still. Nobody touches it for 3 seconds. The pilot's status line must show real numbers in the brackets, not `none`.
4. Press SPACE in the pilot window. Watch the status line: `learning heading: probe axis 0`, then `commit`, `brake`, `next push`. The balloon makes a few short pushes for 10-30 seconds.
5. Then the note becomes `HOVER`. Watch it for one full minute. To stop at any time: SPACE (motors off, the balloon just floats), or the switch.

**You should see**
- `safe DISARMED ... | (x, y, z) psi=n/a z:cam` before arming, window 2 shows `BLE`, `imu 20 Hz`.
- During the minute the balloon stays within about 0.3 m of its spot, with one small wobble every 40 seconds (the heading twitch). Height stays near 1.7 m.

**If not**
- `none` in the brackets: mono does not see the balloon (S12) or is not running.
- `I need more room to learn which way I'm facing`: too close to a wall. SPACE, move it to the middle, arm again.
- `BALLOON LOST -> disarm`: the camera lost it for 1.5 s. Better light, or it left the picture. Bring it back in view and arm again.
- It spins up instead of holding still: the yaw sign is wrong. SPACE immediately. Redo J2 (`GYRO_SIGN`) and J7.
- It climbs to the ceiling or sinks to the floor: the V sign is wrong (J7) or the trim is far off (J11). It drifts steadily one way: a draught, not a bug; close doors.

#### Step J14. Follow me, then voice

**You need**
- the same three windows, the balloon hovering
- the OMNI key in the environment (`YIBU_API_KEY`), or Ollama running for the local voice

**Do**
1. In the pilot window press `t`, type `follow_me`, press Enter. Stand 2 m from the balloon, fully in the camera's view, still for 10 seconds.
2. Walk slowly, one step every two seconds, in a circle around the space, never closer than 1 m to the balloon. Then `t`, `hover`, Enter.
3. Press ESC in the pilot. Restart it as `python -m laptop.control.pilot` (cloud voice) or `python -m laptop.control.pilot --voice local` (offline, `v` for push-to-talk).
4. SPACE to arm. Say: `Blimpy, follow me.` Walk. Then: `Blimpy, stop.`

**You should see**
- The note shows `FOLLOW d=1.5` or close. The balloon turns to face you, keeps about 1.5 m from you, and comes along as you walk.
- With voice: it answers out loud, the note changes to `FOLLOW`, then to `HOVER` on `stop`.

**If not**
- It follows the wrong person: it takes the largest person in the picture. Others step out of view.
- It overshoots and swings back: expected for the first minutes while the heading settles; if it does not calm down in a minute, paste the pilot's log to Raymond's chat.
- It answers but does not move on `follow me`: paste the `[omni]` lines to Raymond's chat. Nothing heard: `python -m laptop.voice.omni --meter`; AirPods need `--mic AirPods --spk Speakers`.

---

36 steps (7 in H, 15 in S, 14 in J). Every number measured along the way lives in `laptop/config.py` (PHYS, BLE) and `calib/MEASUREMENTS.md`; the design behind the steps is `docs/ROBOT_BUILD.md`; the second ESP32 and the main-board patch are in `firmware/i2c_motor_slave/`.
