# Blimpy build guide: where to look (this page is a pointer)

This page used to be a long "canonical" build guide written on 2026-09-19. It was overtaken the same day and it
contradicted what the team actually built (it put the motor bar at the REAR with shafts pointing backward, and assumed
one battery). Do not build from an old copy of it. What is current:

1. **The live build-steps page** (the artifact the team works from): phases H / S / J, the "Read this first" section on
   the props (closed-nose props fit ONE way: point each prop's nose where that motor must push; the bar edge is the
   FRONT; L and R on the props = turning direction, one of each on the two forward motors), and step J4 (the vertical
   motor strapped to the SIDE of the cap, nose down).
2. **`docs/BUILD_STEPS.md`** and **`docs/ROBOT_BUILD.md`**: the same steps and the design behind them in the repo. The
   artifact is ahead of them on motor placement.
3. **`calib/MEASUREMENTS.md`** (anything measured, dated) and **`laptop/config.py`** (`PHYS`, `BLE`: the numbers the
   software uses) win over every guide. `PHYS D = 1.00` m was measured on the FIRST balloon: a new inflation is a new
   diameter: measure it against the wall marks (step S4) and set `PHYS D` before flying, because the room camera turns
   the balloon's apparent size into distance.

Three points from the old page that are written nowhere else:
- **Motor current stays off the breadboard.** An 8520 motor draws about 1 A at the 50 % cap and 2 A flat out; breadboard
  contacts and jumper wires are good for about 1 A. Battery -> switch -> driver power pins -> motors in soldered or
  screw-terminal wire, a 470-1000 uF capacitor across the motor supply AT the drivers, one star ground between the two
  batteries. Only signal wires through the breadboard.
- **Expect the motor plane 0.75-0.80 m below the balloon's centre** with the ring + carabiner + 11 cm bridle (radius +
  neck + ring + clip + bridle), not the 0.55 m placeholder in `PHYS ARM_BELOW`. Measure it (step J12).
- **Helium leaks.** The first balloon lost about 26 g of lift while it was being worked on, and the vertical motor has
  about 12 g of push in its strong direction, less in the other. Re-trim with coins right before EACH flight session;
  close the neck with a clip so it can be topped up; nothing sticks to the latex (peeling tape off popped balloon one).
