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
