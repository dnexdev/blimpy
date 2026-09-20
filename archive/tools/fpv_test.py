"""Offline test of the eye on the balloon (no camera, no YOLO, ~1 s): the pixel -> bearing / range geometry in
laptop/vision/fpv.py, the simulator's eye model against ground truth, the estimator's relative mode and direct heading
fix, and the sign of the FOLLOW-on-the-eye law. The closed-loop behaviour is in archive/tools/scenarios.py (names with "eye").

  python archive/tools/fpv_test.py
  python archive/tools/scenarios.py eye
"""
import math, os, pathlib, sys
ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop import config
from laptop.control.behaviors import Behaviors
from laptop.control.estimator import StateEstimator
from laptop.control.protocol import wrap
from archive.sim.world import IDEAL, World
from laptop.vision.fpv import observe, pick_person

results = {}
def check(name, ok, detail=""):
    results[name] = bool(ok); print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


# 1. geometry
W, H = 640, 480
f = (W / 2) / math.tan(math.radians(62) / 2)
o = observe((300, 100, 340, 300), W, H, hfov_deg=62, person_h=1.65)
check("centred person -> bearing 0, range from height", abs(o["bearing"]) < 1e-6 and abs(o["range"] - 1.65 * f / 200) < 1e-3, str(o))
o = observe((60, 100, 100, 300), W, H, hfov_deg=62)
check("person on the LEFT -> positive bearing (turn left)", o["bearing"] > 0 and abs(o["bearing"] - math.atan((320 - 80) / f)) < 1e-3, f"{math.degrees(o['bearing']):.1f} deg")
o = observe((540, 100, 580, 300), W, H, hfov_deg=62)
check("person on the RIGHT -> negative bearing", o["bearing"] < 0)
o = observe((300, 0, 340, 479), W, H)
check("box cut by the frame -> range None (close)", o["range"] is None and o["box"][1] == 0.0)
o = observe((300, 60, 340, 220), W, H)
check("person above the axis -> positive elevation", o["elev"] > 0)
p = pick_person([{"box": (10, 10, 60, 120)}, {"box": (200, 50, 330, 400)}, {"box": (500, 20, 560, 200)}], W, H)
check("pick_person = the largest box", p["box"] == (200, 50, 330, 400))
from laptop.vision.fpv import count_facing
big = {"box": (W / 2 - 80, 0, W / 2 + 80, H)}; side = {"box": (5, 100, 60, 300)}; big2 = {"box": (W / 2 - 60, 20, W / 2 + 100, H)}
check("count_facing: one person near and centred = 1, a bystander at the edge does not count, a group in front = 2",
      count_facing([big], W, H) == 1 and count_facing([big, side], W, H) == 1 and count_facing([big, big2], W, H) == 2 and count_facing([], W, H) == 0)

# 2. sim eye vs truth
w = World(dict(IDEAL, fpv=True, tof=True), person="static", psi0=0.3, person_start=(1.5, 0.0))
w.command({"vf": 0, "vs": 0, "yr": 0, "vz": 0, "arm": 0})
for _ in range(10): w.advance(0.05)
fpv = [s["fpv"] for s in w.poll_state() if s.get("fpv")]
b = w.b; p = w.person.p
truth_b = wrap(math.atan2(p[1] - b.y, p[0] - b.x) - b.psi); truth_r = math.dist((b.x, b.y, b.z), p)
check("sim eye: bearing and range match truth (IDEAL)", fpv and abs(fpv[-1]["bearing"] - truth_b) < 0.01 and abs(fpv[-1]["range"] - truth_r) < 0.01,
      f"{fpv[-1] if fpv else None} vs bearing {truth_b:.3f} range {truth_r:.3f}")
w2 = World(dict(IDEAL, fpv=True), person="static", psi0=2.5, person_start=(1.5, 0.0))
for _ in range(10): w2.advance(0.05)
check("sim eye: person behind -> no observation", all(s.get("fpv") is None for s in w2.poll_state()))
w3 = World(dict(IDEAL, fpv=True, vision=False), person="static", psi0=0.0)
for _ in range(10): w3.advance(0.05)
msgs = w3.poll_state()
check("vision=False -> no balloon/person fixes, eye still there", msgs and all(m["balloon"] is None and m["person"] is None for m in msgs) and any(m.get("fpv") for m in msgs))

# 3. estimator: relative mode and direct heading fix
est = StateEstimator()
check("seed_relative refuses without an altimeter reading", est.seed_relative() is False and est.p is None)
est.update_telem(yaw=0.5, yr=0.0, alt=1.0, t_ms=1000, now=10.0)
check("seed_relative from the altimeter -> (0, 0, alt + TOF_BELOW), rel", est.seed_relative() and est.rel and abs(est.p[2] - (1.0 + config.PHYS["TOF_BELOW"])) < 1e-9 and est.p[:2] == [0.0, 0.0])
est.update_balloon([1.0, 2.0, 1.6], 1100)
check("a real fix ends relative mode", not est.rel and est.p[:2] == [1.0, 2.0])
est.observe_motion(0, 0, 0, now=10.1)
d = est.fix_heading(1.2, k=1.0)
check("fix_heading sets psi and confirms the heading", abs(wrap(est.psi - 1.2)) < 1e-9 and est.head_ok and est.head_confident, f"psi {est.psi:.3f} step {d:.3f}")

# 4. FOLLOW on the eye: signs
t = [0.0]
beh = Behaviors(lambda s: None, now=lambda: t[0])
beh.on_fpv({"bearing": 0.3, "elev": 0.0, "range": 2.6}); t[0] += 0.1; beh.on_fpv({"bearing": 0.3, "elev": 0.0, "range": 2.6})
vf, yr, note = beh._follow_eye(t[0])
check("person left and far -> turn left (yr > 0) and go forward (vf > 0)", yr > 0 and vf > 0, f"vf {vf:.2f} yr {yr:.2f} {note}")
t[0] += 2.0; beh.on_fpv({"bearing": -0.2, "elev": 0.0, "range": 0.8}); t[0] += 0.1; beh.on_fpv({"bearing": -0.2, "elev": 0.0, "range": 0.8})
vf, yr, note = beh._follow_eye(t[0])
check("person right and too close -> turn right (yr < 0) and back off (vf < 0)", yr < 0 and vf < 0, f"vf {vf:.2f} yr {yr:.2f}")
t[0] += 2.0; beh.on_fpv({"bearing": 0.0, "elev": 0.0, "range": None})
vf, yr, note = beh._follow_eye(t[0])
check("box cut (very close) -> back off", vf < 0 and abs(yr) < 1e-9, f"vf {vf:.2f} {note}")
t[0] += 2.0; beh.on_fpv({"bearing": 0.0, "elev": 0.0, "range": 1.5})
vf, yr, note = beh._follow_eye(t[0])
check("at the follow distance -> no forward command", abs(vf) < 1e-9 and abs(yr) < 1e-9)
t[0] += 2.0; beh.on_fpv({"bearing": 1.0, "elev": 0.0, "range": 3.0})
vf, yr, note = beh._follow_eye(t[0])
check("facing away (> 34 deg) -> turn, do not drive forward", yr > 0 and vf <= 0)
t[0] += 5.0
check("eye lost after LOST_MS", not beh.fpv_ok(t[0]))

n_fail = sum(not v for v in results.values())
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
sys.exit(1 if n_fail else 0)
