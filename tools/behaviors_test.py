"""Headless test of the behavior state machine against the simulator (no voice, no keyboard).

  python tools/behaviors_test.py

Feeds a scripted sequence of intents and checks the balloon does what each one means:
nudge -> follow -> rotate 90 -> timer fires -> go_to home -> dance -> hover.
"""
import math, os, pathlib, subprocess, sys, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop import config
from laptop.control.protocol import CMD_PORT, STATE_PORT, TELEM_PORT, UdpJson, make_cmd, now_ms, wrap
from laptop.control.estimator import StateEstimator
from laptop.control.behaviors import Behaviors

G = config.FOLLOW
LOCAL = ("127.0.0.1", CMD_PORT)
spoken = []
proc = subprocess.Popen([sys.executable, "-m", "laptop.control.fake_esp32", "--sim", "--person", "static", "--psi0", "1.0"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.0)
state_in, tel_in, cmd_out = UdpJson(STATE_PORT), UdpJson(TELEM_PORT), UdpJson()
est = StateEstimator(alpha=G["POS_ALPHA"])
beh = Behaviors(lambda s: (spoken.append(s), print(f"\n   [say] {s}")))
person = None
t0 = time.monotonic(); nudge_end = t0 + G["NUDGE_S"]
script = [  # (time after start, intent)
    (2.5, {"intent": "follow_me"}),
    (22.0, {"intent": "rotate", "degrees": 90}),
    (22.0, {"intent": "timer", "minutes": 0.05}),
    (34.0, {"intent": "go_to", "target": "home"}),
    (64.0, {"intent": "mood", "mood": "dance"}),
    (72.0, {"intent": "hover"}),
]
results = {}
samples = {}
try:
    while time.monotonic() - t0 < 76:
        t, now = time.monotonic(), now_ms(); el = t - t0
        r = state_in.recv_latest(only_from=LOCAL[0])
        if r:
            s = r[0]
            if s.get("balloon"): est.update_balloon(s["balloon"], s.get("t", now))
            if s.get("person"): person = s["person"]; beh.on_person(person)
        r = tel_in.recv_latest(only_from=LOCAL[0])
        if r: est.update_telem(r[0]["yaw"], r[0].get("yr", 0.0))
        if beh.home is None and est.p is not None and el > 1.0: beh.on_armed(est); print(f"home = {beh.home}")
        while script and el >= script[0][0]:
            it = script.pop(0)[1]; print(f"\n[{el:4.1f}s] intent {it} -> {beh.handle(it, est)!r} mode={beh.mode}")
            if it["intent"] == "rotate": samples["yaw_before_rot"] = est.yaw_gyro
        vf = vs = yr = vz = 0.0; note = ""
        if est.p is not None:
            if t < nudge_end: vf = G["NUDGE_VF"]
            else: vf, vs, yr, vz, note = beh.step(est)
        est.observe_motion(vf, vs)
        cmd_out.send(make_cmd(vf, yr, vz, True, vs), LOCAL)
        # checkpoints
        if 21.0 < el < 21.5 and person and est.psi is not None:
            d = math.hypot(person[0] - est.p[0], person[1] - est.p[1]); results["follow_dist"] = d
        if 33.5 < el < 34.0: results["rot_delta_deg"] = math.degrees(abs(wrap(est.yaw_gyro - samples.get("yaw_before_rot", 0))))
        if 50.0 < el < 64.0: results["home_dist"] = min(results.get("home_dist", 9.0), math.hypot(beh.home[0] - est.p[0], beh.home[1] - est.p[1]))
        if 66.0 < el < 69.0: results["dance_mode"] = beh.mode
        if 73.0 < el < 76.0: results["final_mode"] = beh.mode
        if est.p is not None and int(el * 2) != int((el - 1 / G["HZ"]) * 2) and int(el * 2) % 4 == 0:
            print(f"[{el:4.1f}s] {note:26s} vf={vf:+.2f} vs={vs:+.2f} yr={yr:+.2f} vz={vz:+.2f} pos=({est.p[0]:+.2f},{est.p[1]:+.2f},{est.p[2]:.2f}) "
                  f"v={est.speed:.2f} psi={math.degrees(est.psi) if est.psi is not None else 0:+4.0f}")
        time.sleep(max(0, 1 / G["HZ"] - (time.monotonic() - t)))
finally:
    for _ in range(3): cmd_out.send(make_cmd(0, 0, 0, False), LOCAL); time.sleep(0.02)
    time.sleep(0.3); proc.terminate()

print("\n\nresults:", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in results.items()})
print("spoken:", spoken)
checks = {
    "followed to ~1.5 m": abs(results.get("follow_dist", 9) - G["D_FOLLOW"]) < 0.4,
    "rotated ~90 deg": 60 < results.get("rot_delta_deg", 0) < 130,
    "timer spoke": any("Time's up" in s for s in spoken),
    "went home (<0.7 m)": results.get("home_dist", 9) < 0.7,
    "danced": results.get("dance_mode") == "DANCE",
    "ended in HOVER": results.get("final_mode") == "HOVER",
}
# nudge: "move forward a bit" is a short step the way Blimpy faces, then a hold there (seen live: the model sent go_to target "forward")
class _E: p = [1.0, 2.0, 1.5]; psi = math.pi / 2; rel = False; head_confident = True
_b = Behaviors(lambda s: None); _b.handle({"intent": "nudge", "move": "forward", "metres": 0.5}, _E())
checks["nudge forward = 0.5 m the way it faces, then hold"] = _b.mode == "HOVER" and abs(_b.hold_xy[0] - 1.0) < 1e-6 and abs(_b.hold_xy[1] - 2.5) < 1e-6
_b.handle({"intent": "go_to", "target": "forward"}, _E())
checks["go_to with a direction as target is a nudge, an unknown place is refused"] = abs(_b.hold_xy[1] - 2.5) < 1e-6 and "don't know that place" in _b.handle({"intent": "go_to", "target": "kitchen"}, _E())
for k, v in checks.items(): print(f"  {'OK  ' if v else 'FAIL'} {k}")
print("PASS" if all(checks.values()) else "FAIL")
sys.exit(0 if all(checks.values()) else 1)
