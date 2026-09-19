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
| IMU rate | hardware team: "reads it every 10 ms". **Measured 2026-09-19, step 4 probe: 10 lines in 10 s, one per second** (arrivals 0.96 / 1.02 s apart, `A:-0.144,-0.001,1.070;G:-2.21,1.85,-0.34;T:42.7`, az +1.07 = chip up, gz -0.3 deg/s at rest) | the laptop prints every notification the moment it arrives (bleak callback, no timer, no once-a-second read), so one per second is what the box notifies. Too slow: the yaw loop is blind below 5 per second and wants 20+. Asked: one notification per IMU sample at 20-50 per second; a counter `;N:123` (+1 per notify) in the line tells whether the firmware or the radio is the slow part. `ble_gondola --probe` now ends with a `[probe] N lines in 10 s = X per second` verdict; step 4 is ticked when it says OK |
| Ultrasonic | **none**: the box ran out of pins (2026-09-19) | step 18 has nothing to fit; height comes from the room camera (~15 cm, enough for a 1.7 m hover). `PHYS TOF_BELOW` and `BLE ALT_*` stay unused; `pilot --relative` (flying with no room camera) needs an altimeter, so that mode is out for this box |
