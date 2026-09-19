# Measured numbers (the ones a tape measure or ruler produced; do not guess these)

| Date | What | Value | Lives in | Notes |
|---|---|---|---|---|
| 2026-09-19 | printed AprilTag, edge of the outer black square (tag 0, `targets/apriltag36h11_id0_150mm_A4.png`) | **136.7 mm** | `laptop/config.py` `TAG_SIZE_M = 0.1367` | the 150 mm file printed at ~91 % (letter paper, "fit to page"). All four mat pages came from the same print job. Reprint = re-measure. |
| 2026-09-19 | printed checkerboard, one square (`targets/checkerboard_9x6_25mm_A4.png`) | **22.8 mm** | `laptop/config.py` `SQUARE_M = 0.0228` | same 91 % scaling (25 x 0.912 = 22.8, consistent with the tag). |
| 2026-09-19 | laptop webcam intrinsics, 1280x720, 19 checkerboard shots | **RMS 0.22 px**, f = 836 px, centre (643, 375), k1 0.057 k2 -0.115 | `calib/laptop_intrinsics.npz` (committed) | redo only if the webcam or its resolution changes. |
| 2026-09-19 | mat board survey (webcam at 2.1 m, 61 frames) | tags at (0,0) (0.347,0.012) (0.339,0.359) (-0.007,0.349) m, twists <= 3.4 deg, **residual 0.67 px** | `calib/mat.json` (committed) | the board is ~0.35 m square. Re-survey only if a page moves or is re-glued. |

Still to measure (README section 3a table): balloon diameter inflated -> `PHYS["D"]`; ultrasonic sensor below the balloon
centre -> `PHYS["TOF_BELOW"]`; motor plane below the balloon centre -> `PHYS["ARM_BELOW"]`. The mat tag spacing is NOT
measured by hand: `tools/calib/survey_mat.py` fits it from tag 0's size and writes `calib/mat.json`.

## The box (2026-09-19, answers from the hardware team, build step 3)

| Question | Answer | What it means for the build |
|---|---|---|
| On / off | a switch | fine: that is the "person on the switch" control (rule 2) |
| Battery | TWO: a LiPo for the ESP32 (the Bluetooth board) and a separate battery for the motors. Cells / voltage / mAh not given yet: read them off the labels and add them here | both fly: both go under the plate (step 17), both count in the weight (step 26); charge both before the demo |
| Motor drivers | DRV8833 and TB6612 | good: almost no voltage drop, both directions per motor |
| Firmware stops the motors by itself after 500 ms without a command? | **No** | the laptop bridge is the only failsafe. Step 25 test 2 WILL keep the motors running: a person stays on the switch whenever motors run, and the hardware team is asked to add the timeout (ROBOT_BUILD.md section 9, item 1) |
| IMU rate | one line every 20 ms (50 per second) | more than the 20 the software needs; expect ~500 lines in a 10 s probe (step 4) |
| Ultrasonic | yes, sent over BLE with the IMU data | fit it (step 18). In the step 4 probe the parsed dict must show an `alt` (or `range` / `dist` / `sonar` / `us`) key; if their key is called something else, add it to `ALT_KEYS` in `config.BLE`, and set `ALT_UNITS` to what they print (cm / mm / m) |
