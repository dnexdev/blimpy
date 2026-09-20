# archive/ — the simulator (frozen 2026-09-20)

Everything here flew the control stack against a simulated balloon while there was no robot. The real balloon flies
now (README section 3), so this moved out of the way. It still runs from the repo root, it is not maintained, and
nothing in `laptop/` imports it except `ble_gondola --fake` (the simulated robot behind the bridge), lazily.

```
sim/world.py, sim/plot.py   balloon physics + every sensor model: vision noise / latency / dropouts, gyro bias, HVAC gusts,
                            lift drift, motor spread, the shared-supply sag (2026-09-20), the eye on the balloon, the ultrasonic;
                            live top-down plot
fake_esp32.py               the simulated board on the UDP protocol (mixer on the board); --sim publishes the world on 5007
tools/scenarios.py          26 closed-loop scenarios on the REAL controller code, ~3 s, scored against truth. GREEN at archive time
                            (all 26; 5 seeds on the follow / hover / rotate / go-to set), with 5 Hz telemetry and the supply sag on
tools/sim_test.py           real-time regression over UDP (failsafe timing + a walking follow); --ble runs it through the bridge
tools/ble_test.py           the bridge on the simulated robot (older-firmware path: MOTORS lines); the onboard-mixer path is tools/bridge_test.py in the live tree
tools/behaviors_test.py     the state machine against fake_esp32 --sim
tools/fpv_test.py           the eye on the balloon: geometry, sim eye vs truth, relative mode
tools/omni_sim_test.py      the cloud voice chain against the simulated robot (spoken wav commands, ~8 cloud calls)
tools/rehearse.py           the whole demo with a virtual balloon: --person real tracks YOU with the room camera. The fallback demo.
```

Run from the repo root:
```powershell
python archive/tools/scenarios.py                 # all scenarios; --seeds 5 worst case; --plot go_to; --sag 0 = separate supplies
python -m archive.fake_esp32 --sim --plot         # then teleop / follow_me / pilot as usual (they see a board on 127.0.0.1)
python -m laptop.control.ble_gondola --fake --sim # the same robot behind the BLE bridge
python archive/tools/rehearse.py --person real    # virtual balloon, real room camera, real voice
```

## What the simulator models (and what it taught)
`sim/world.py` is force-based, built from the parts list (constants at the top of `class Balloon`). **Realism is on
by default** (`REAL`): 120 ms vision latency, 2-4 cm position noise, person dropouts and false detections, 2 % packet
loss, gyro bias after boot calibration, HVAC gusts, lift drifting with room temperature, mismatched motors, the
sideways motor a few cm off-centre, the motors sharing one supply (each unit of total duty takes 15 % of the volts),
walls and obstacles (`venues/default.json`). `--ideal` turns all of it off.
- **Effective mass ~0.9 kg** at D = 1.15 m, not 0.3: a sphere drags half its displaced air along ("added mass").
- **Thrust ~ duty²**: 8520 + 75 mm prop ~50 gf flat out, ~12 gf at the 0.5 cap; motors below ~6 % duty do nothing;
  a fixed prop in **reverse gives ~60 %**. Two motors -> top speed ~0.95 m/s, but 0 to 0.3 m/s takes ~3 s.
- **Quadratic drag**: from 0.3 m/s with motors off it still coasts >2 m in 10 s. Drag does NOT stop it; thrust does.
- **No keel**: heading and velocity are independent. That is why there are **four motors, all reversible**:
  L + R (rear, forward axis: drive, brake, yaw), **S sideways through the centre**, **V vertical** (lift changes by
  grams as the room warms; ballast alone hits the ceiling within a minute). See PROTOCOL.md section 6.
- **Gondola tilt**: horizontal thrust swings the gondola ~4 deg, adding ~1 gf of lift at full push.
- **Yaw**: 25 cm motor spacing, ~0.025 kg m² inertia, ~no damping -> mixer yaw-rate PI loop (P 1.0, I 1.0/s).
- **Shared supply** (2026-09-20): four motors at 0.4 together take ~24 % of the volts and ~42 % of the thrust; the
  braking assumptions had to come down (`A_BRAKE` 0.08 -> 0.055) and the total duty is capped at 1.2.
- **Command chatter** (2026-09-20): 3 cm/s of velocity noise through K_V and a sqrt linearisation slammed the thrust
  commands between +0.4 and -0.4 every tick; the scenarios now score `cmd chatter` (|dvf|+|dvs| per tick, must be < 0.08).

The vehicle numbers (`D`, `T_MAX`, `REV_EFF`, `ARM_BELOW`, `TOF_BELOW`, `M_GONDOLA`, `MOTOR_SPACING`) live in
`laptop/config.py` PHYS, shared by the simulator and the estimator.

Scenarios: failsafe, follow (ideal / real noise / walking / demo route with a pillar), hover in gusts, rotate
90/-180/360, go to the judges and home past a pillar, come here, altitude commands, lift drift, wrong initial heading,
4x gyro drift, long hover then follow, wander, 5 Hz telemetry (follow and rotate), lossy links, dance, ToF with bad
vision z, no ToF, board-side failsafe, and the eye on the balloon (with and without the room camera). The suite keeps
the ToF OFF except in the ToF scenarios so seeded runs stay bit-identical; `fake_esp32 --sim` has it ON.
