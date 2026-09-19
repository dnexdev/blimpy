# Measured numbers (the ones a tape measure or ruler produced; do not guess these)

| Date | What | Value | Lives in | Notes |
|---|---|---|---|---|
| 2026-09-19 | printed AprilTag, edge of the outer black square (tag 0, `targets/apriltag36h11_id0_150mm_A4.png`) | **136.7 mm** | `laptop/config.py` `TAG_SIZE_M = 0.1367` | the 150 mm file printed at ~91 % (letter paper, "fit to page"). All four mat pages came from the same print job. Reprint = re-measure. |
| 2026-09-19 | printed checkerboard, one square (`targets/checkerboard_9x6_25mm_A4.png`) | **22.8 mm** | `laptop/config.py` `SQUARE_M = 0.0228` | same 91 % scaling (25 x 0.912 = 22.8, consistent with the tag). |

Still to measure (README section 3a table): balloon diameter inflated -> `PHYS["D"]`; ultrasonic sensor below the balloon
centre -> `PHYS["TOF_BELOW"]`; motor plane below the balloon centre -> `PHYS["ARM_BELOW"]`. The mat tag spacing is NOT
measured by hand: `tools/calib/survey_mat.py` fits it from tag 0's size and writes `calib/mat.json`.
