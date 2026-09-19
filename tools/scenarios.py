"""Headless scenario suite: runs the REAL controller code (estimator + behaviors, exactly what pilot.py runs)
against the simulated world (laptop/sim/world.py) at ~100x real time and scores it against ground truth.
Run it after touching anything in laptop/control or the gains in laptop/config.py.

  python tools/scenarios.py                 # all scenarios: table + PASS/FAIL
  python tools/scenarios.py rotate go_to    # only scenarios whose name contains one of these
  python tools/scenarios.py --plot          # also save sim_out/<name>.png (trajectory, height, heading, distance)
  python tools/scenarios.py --seeds 3       # repeat each scenario with 3 random seeds; the table shows the worst
  python tools/scenarios.py --tof           # ToF altimeter on in every scenario (default: only the ToF scenarios; see world.py REAL)
"""
import argparse, math, os, pathlib, random, statistics, sys
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop import config
from laptop.control.protocol import make_cmd, wrap
from laptop.control.estimator import StateEstimator
from laptop.control.behaviors import Behaviors
from laptop.control.follow_me import nudge_ok
from laptop.control.link import TelemWatchdog
from laptop.sim.world import IDEAL, REAL, World

G = config.FOLLOW
PILLAR = (1.4, -0.2, 0.2)          # a pillar / tripod between the start and the judges (2 m of room north of it)


class Run:
    """One closed-loop run. Mirrors pilot.py's main loop tick for tick (arm -> nudge -> behaviors.step)."""

    def __init__(self, seconds, person="static", realism=REAL, psi0=1.0, wind=(0.0, 0.0), seed=0, obstacles=None,
                 start=(0.0, 0.0, 1.5), script=(), nudge=True, arm_at=0.5, stop_cmds_at=None, person_speed=0.5, person_start=None,
                 cmd_gap=None, **over):
        self.seconds, self.arm_at, self.stop_cmds_at, self.nudge = seconds, arm_at, stop_cmds_at, nudge
        self.cmd_gap = cmd_gap                    # (t0, t1): commands not delivered in between = a WiFi hiccup
        self.wd_hold = False                      # after a watchdog disarm the 'operator' does not re-arm (real life: SPACE again)
        self.w = World(dict(realism, **over), person, psi0, wind, seed, 0.0, obstacles=obstacles, start=start,
                       person_speed=person_speed, person_start=person_start)
        self.est = StateEstimator(alpha=G["POS_ALPHA"], beta=G["VEL_BETA"])
        self.spoken, self.events, self.log = [], [], []
        self.t = 0.0
        self.beh = Behaviors(lambda s: self.spoken.append((round(self.t, 1), s)), now=lambda: self.t,
                             obstacles=self.w.obstacles, rng=random.Random(seed))
        self.script = sorted(script, key=lambda e: e[0])
        self.lost = 0
        self.wd, self.wd_reasons = TelemWatchdog(), []   # same watchdog as follow_me / pilot, fed with sim time

    def run(self):
        w, est, beh = self.w, self.est, self.beh
        dt = 1.0 / G["HZ"]
        armed, nudge_end, t_balloon = False, 0.0, -9.0
        while self.t < self.seconds:
            t = self.t
            for s in w.poll_state():
                if s["balloon"]: est.update_balloon(s["balloon"], s["t"]); t_balloon = t
                if s["person"]: beh.on_person(s["person"], s["t"])
            for m in w.poll_telem():
                est.update_telem(m["yaw"], m["yr"], m.get("alt"), m.get("t"), now=t); self.wd.telem(m, t, armed)
            while self.script and self.script[0][0] <= t:
                it = self.script.pop(0)[1]
                self.events.append((round(t, 1), it.get("intent"), beh.handle(it, est), beh.mode))
            if not armed and t >= self.arm_at and est.p is not None and not self.wd_hold:
                armed = True; self.wd.arm(t); beh.on_armed(est); nudge_end = t + (G["NUDGE_S"] if self.nudge else 0.0)
            vf = vs = yr = vz = 0.0; note = "safe"
            if armed:
                reason = self.wd.check(armed, t)
                if reason:
                    armed, note = False, reason; self.wd_reasons.append((round(t, 2), reason)); self.wd_hold = True
                elif est.p is None or t - t_balloon > G["BALLOON_LOST_MS"] / 1000:
                    armed, note = False, "BALLOON LOST"; self.lost += 1
                else:
                    vf, vs, yr, vz, note = beh.step(est)
            est.observe_motion(vf, vs, vz, t)
            if (self.stop_cmds_at is None or t < self.stop_cmds_at) and not (self.cmd_gap and self.cmd_gap[0] <= t < self.cmd_gap[1]):
                w.command(make_cmd(vf, yr, vz, armed, vs))
            w.advance(dt)
            self.t += dt
            tr = w.truth
            p = tr["person"]
            self.log.append(dict(
                t=self.t, x=tr["x"], y=tr["y"], z=tr["z"], psi=tr["psi"], v=math.hypot(tr["vx"], tr["vy"]), armed=tr["armed"],
                dist=math.hypot(p[0] - tr["x"], p[1] - tr["y"]), px=p[0], py=p[1],
                bear_err=abs(math.degrees(wrap(math.atan2(p[1] - tr["y"], p[0] - tr["x"]) - tr["psi"]))),
                off_err=abs(math.degrees(wrap(est.offset - tr["offset"]))) if est.yaw_gyro is not None else 180.0,
                bias_err=abs(est.bias_hat - w.r["gyro_bias"]),
                zerr=abs(tr["z"] - (G["Z_HOLD"] + beh.z_offset)), z_est=est.p[2] if est.p else float("nan"), z_src=est.z_src,
                mode=beh.mode, note=note, vf=vf, vs=vs, yr=yr, vz=vz, cmd_armed=armed,
                effort=sum(abs(m) for m in tr["motors"]),
                v_est_err=math.hypot(est.v[0] - tr["vx"], est.v[1] - tr["vy"]) if est.p else 0.0,
            ))
        return self

    # --------------------------------------------------------------- helpers for checks
    def after(self, t0, t1=None):
        return [r for r in self.log if r["t"] >= t0 and (t1 is None or r["t"] < t1)]

    def stat(self, rows, key, f=statistics.mean):
        return f([r[key] for r in rows]) if rows else float("nan")


# ---------------------------------------------------------------------------------------------------- scenarios
# Each returns (run, checks) with checks = [(label, value, ok)].
def follow(realism, person, seconds, seed, psi0=1.0, t_settle=25, d_tol=0.3, head_tol=15, z_tol=0.12, max_d=2.6, **over):
    r = Run(seconds, person, realism, psi0=psi0, seed=seed, script=[(1.0, {"intent": "follow_me"})], **over).run()
    rows = r.after(t_settle)
    still = [x for x in rows if x["dist"] > 0]                    # all rows (person may be walking)
    return r, [
        ("dist mean (m)", r.stat(rows, "dist"), abs(r.stat(rows, "dist") - G["D_FOLLOW"]) < d_tol),
        ("dist max (m)", r.stat(rows, "dist", max), r.stat(rows, "dist", max) < max_d),
        ("dist min (m)", r.stat(rows, "dist", min), r.stat(rows, "dist", min) > 0.85),   # the sim person stops at 0.9
        ("facing err mean (deg)", r.stat(rows, "bear_err"), r.stat(rows, "bear_err") < head_tol),
        ("heading est err mean (deg)", r.stat(rows, "off_err"), r.stat(rows, "off_err") < 12),
        ("z err rms (m)", math.sqrt(r.stat(rows, "zerr", lambda v: statistics.mean(x * x for x in v))), r.stat(rows, "zerr", max) < z_tol + 0.1 and math.sqrt(r.stat(rows, "zerr", lambda v: statistics.mean(x * x for x in v))) < z_tol),
        ("vel est err mean (m/s)", r.stat(rows, "v_est_err"), r.stat(rows, "v_est_err") < 0.05),
        ("collisions", r.w.collisions, r.w.collisions == 0),
        ("lost/disarms", r.lost, r.lost == 0),
    ]


def sc_follow_static_ideal(seed):
    return follow(IDEAL, "static", 45, seed, t_settle=20, d_tol=0.2, head_tol=8, z_tol=0.08)


def sc_follow_static_real(seed):
    return follow(REAL, "static", 60, seed, t_settle=25, d_tol=0.3, head_tol=15)


def sc_follow_walk(seed):
    return follow(REAL, "walk", 90, seed, t_settle=25, d_tol=0.35, head_tol=15)


def sc_follow_route(seed):
    """Person walks a route at 0.5 m/s with stops, pillar in the room. Judged when the person is standing still."""
    r = Run(150, "route", REAL, psi0=1.0, seed=seed, obstacles=[PILLAR] + config.OBSTACLES,
            script=[(1.0, {"intent": "follow_me"})]).run()
    rows = r.after(25)
    # 'standing' = person has not moved for 3 s (the follower needs a few seconds to close in)
    standing = []
    for i in range(len(r.log)):
        if r.log[i]["t"] < 25: continue
        j = i - int(3.0 * G["HZ"])
        if j >= 0 and math.hypot(r.log[i]["px"] - r.log[j]["px"], r.log[i]["py"] - r.log[j]["py"]) < 0.02:
            standing.append(r.log[i])
    return r, [
        ("dist mean, person standing (m)", r.stat(standing, "dist"), abs(r.stat(standing, "dist") - G["D_FOLLOW"]) < 0.5),
        ("dist mean, overall (m)", r.stat(rows, "dist"), r.stat(rows, "dist") < 2.3),
        ("dist min (m)", r.stat(rows, "dist", min), r.stat(rows, "dist", min) > 0.85),   # the sim person sidesteps at 0.9
        ("facing err mean (deg)", r.stat(rows, "bear_err"), r.stat(rows, "bear_err") < 25),
        ("heading est err mean (deg)", r.stat(rows, "off_err"), r.stat(rows, "off_err") < 15),
        ("z err rms (m)", math.sqrt(r.stat(rows, "zerr", lambda v: statistics.mean(x * x for x in v))), r.stat(rows, "zerr", max) < 0.25),
        ("collisions", r.w.collisions, r.w.collisions == 0),
        ("min clearance (m)", r.w.min_clearance, r.w.min_clearance > -0.01),
        ("lost/disarms", r.lost, r.lost == 0),
    ]


def sc_hover_gusts(seed):
    r = Run(120, "static", REAL, psi0=1.0, seed=seed, gust_rms=0.10, script=[(1.0, {"intent": "hover"})]).run()
    rows = r.after(30)                                                # heading acquisition can take until ~20 s
    x0, y0 = r.beh.hold_xy if r.beh.hold_xy else (rows[0]["x"], rows[0]["y"])   # the spot the hover is holding
    drift = [math.hypot(x["x"] - x0, x["y"] - y0) for x in rows]
    return r, [
        ("drift max from hold point (m)", max(drift), max(drift) < 0.85),   # includes the ~0.5 m heading twitch every 40 s
        ("drift mean (m)", statistics.mean(drift), statistics.mean(drift) < 0.3),
        ("z err max (m)", r.stat(rows, "zerr", max), r.stat(rows, "zerr", max) < 0.15),
        ("heading est err, last 30 s (deg)", r.stat(r.after(90), "off_err"), r.stat(r.after(90), "off_err") < 15),
        ("effort mean (sum |duty|)", r.stat(rows, "effort"), True),
        ("lost/disarms", r.lost, r.lost == 0),
    ]


def sc_rotate(seed):
    script = [(1.0, {"intent": "hover"}), (14.0, {"intent": "rotate", "degrees": 90}),
              (32.0, {"intent": "rotate", "degrees": -180}), (54.0, {"intent": "rotate", "degrees": 360})]
    r = Run(82, "static", REAL, psi0=1.0, seed=seed, script=script).run()
    checks = []
    for t0, deg, t1 in ((14.0, 90, 32.0), (32.0, -180, 54.0), (54.0, 360, 82.0)):
        a = [x for x in r.log if x["t"] >= t0]; b = [x for x in r.log if x["t"] >= t1 - 0.2]
        psi0, psi1 = a[0]["psi"], (b[0] if b else r.log[-1])["psi"]
        # unwrap the true rotation between t0 and t1
        done = next((x["t"] - t0 for x in a if x["t"] > t0 + 1 and x["mode"] != "ROTATE"), None)
        t_meas = t0 + done + 1.5 if done is not None else t1        # true rotation from the command until 1.5 s after "done"
        tot, prev = 0.0, psi0
        for x in a:
            if x["t"] >= min(t1, t_meas): break
            tot += wrap(x["psi"] - prev); prev = x["psi"]
        drift = max(math.hypot(x["x"] - a[0]["x"], x["y"] - a[0]["y"]) for x in a if x["t"] < min(t1, t_meas))
        checks += [(f"rotate {deg:+d}: turned (deg)", math.degrees(tot), abs(math.degrees(tot) - deg) < 15),
                   (f"rotate {deg:+d}: took (s)", done if done is not None else 99, done is not None and done < 6 + abs(deg) / 30),
                   (f"rotate {deg:+d}: drift (m)", drift, drift < 0.6)]
    checks.append(("collisions", r.w.collisions, r.w.collisions == 0))
    return r, checks


def sc_go_to(seed):
    """Start at the origin, judges spot 2.8 m away with a pillar in the straight line, then go home."""
    script = [(1.0, {"intent": "hover"}), (6.0, {"intent": "go_to", "target": "judges"}), (45.0, {"intent": "go_to", "target": "home"})]
    r = Run(85, "static", REAL, psi0=1.0, seed=seed, obstacles=[PILLAR] + config.OBSTACLES, script=script, person_start=(0.0, -1.6)).run()
    jx, jy = config.JUDGES_XY
    t_arr = next((x["t"] for x in r.log if 6 < x["t"] < 45 and x["mode"] == "HOVER" and x["t"] > 7), None)
    d_j = min(math.hypot(x["x"] - jx, x["y"] - jy) for x in r.log if 30 < x["t"] < 45)
    hx, hy = r.beh.home
    d_h = min(math.hypot(x["x"] - hx, x["y"] - hy) for x in r.log if 70 < x["t"] < 85)
    return r, [
        ("reached judges within (m)", d_j, d_j < 0.5),
        ("arrived at (s)", (t_arr - 6) if t_arr else 99, t_arr is not None and t_arr - 6 < 40),
        ("said I'm here", sum(1 for _, s in r.spoken if "here" in s), any("here" in s for _, s in r.spoken)),
        ("back home within (m)", d_h, d_h < 0.5),
        ("collisions", r.w.collisions, r.w.collisions == 0),
        ("min clearance (m)", r.w.min_clearance, r.w.min_clearance > -0.01),
    ]


def sc_come_here(seed):
    r = Run(50, "static", REAL, psi0=1.0, seed=seed, script=[(1.0, {"intent": "hover"}), (5.0, {"intent": "go_to", "target": "me"})]).run()
    rows = r.after(35)
    return r, [
        ("final dist to person (m)", r.stat(rows, "dist"), abs(r.stat(rows, "dist") - G["STANDOFF_ME"]) < 0.4),
        ("closest approach (m)", r.w.min_person_dist, r.w.min_person_dist > 0.75),
        ("ended in HOVER", r.log[-1]["mode"], r.log[-1]["mode"] == "HOVER"),
        ("said I'm here", sum(1 for _, s in r.spoken if "here" in s), any("here" in s for _, s in r.spoken)),
    ]


def sc_lift_drift(seed):
    r = Run(200, "static", REAL, psi0=1.0, seed=seed, lift_drift_n=0.015, lift_drift_period=120.0,
            script=[(1.0, {"intent": "hover"})]).run()
    rows = r.after(25)
    return r, [
        ("z err max after 25 s (m)", r.stat(rows, "zerr", max), r.stat(rows, "zerr", max) < 0.2),
        ("z err mean (m)", r.stat(rows, "zerr"), r.stat(rows, "zerr") < 0.08),
        ("lost/disarms", r.lost, r.lost == 0),
    ]


def sc_altitude_cmds(seed):
    script = [(1.0, {"intent": "hover"}), (10.0, {"intent": "altitude", "direction": "up"}), (35.0, {"intent": "altitude", "direction": "down"}),
              (36.0, {"intent": "altitude", "direction": "down"})]
    r = Run(70, "static", REAL, psi0=1.0, seed=seed, script=script).run()
    z_up = r.stat(r.after(28, 35), "z"); z_dn = r.stat(r.after(60, 70), "z")
    return r, [
        ("z after 'up' (m)", z_up, abs(z_up - (G["Z_HOLD"] + 0.3)) < 0.12),
        ("z after 2x 'down' (m)", z_dn, abs(z_dn - (G["Z_HOLD"] - 0.3)) < 0.12),
    ]


def sc_gyro_drift(seed):
    """Badly calibrated gyro (0.015 rad/s = 52 deg/min) while following a walking person for 4 minutes."""
    r, checks = follow(REAL, "walk", 240, seed, t_settle=120, d_tol=0.35, head_tol=20, gyro_bias=0.015)
    rows = r.after(120)
    checks = [c if c[0] != "collisions" else ("collisions (<=1: 4x-realistic gyro drift stress test)", c[1], c[1] <= 1) for c in checks]
    checks += [("bias est err (rad/s)", r.stat(rows, "bias_err"), r.stat(rows, "bias_err") < 0.006)]
    return r, checks


def sc_long_hover_then_follow(seed):
    """Hover 150 s doing nothing (gyro drifting), then 'follow me': must not run away."""
    r = Run(200, "static", REAL, psi0=1.0, seed=seed, script=[(1.0, {"intent": "hover"}), (150.0, {"intent": "follow_me"})]).run()
    early = r.after(150, 165); late = r.after(180)
    return r, [
        ("heading est err at 150 s (deg)", r.log[int(149.5 * G["HZ"])]["off_err"], r.log[int(149.5 * G["HZ"])]["off_err"] < 30),
        ("dist max in first 15 s of follow (m)", r.stat(early, "dist", max), r.stat(early, "dist", max) < 3.0),
        ("dist mean after 30 s (m)", r.stat(late, "dist"), abs(r.stat(late, "dist") - G["D_FOLLOW"]) < 0.3),
        ("facing err mean (deg)", r.stat(late, "bear_err"), r.stat(late, "bear_err") < 15),
    ]


def sc_wrong_initial_heading(seed):
    """Gyro yaw starts at 0 while the balloon really points 170 deg away; then follow."""
    r, checks = follow(REAL, "static", 60, seed, psi0=2.97, t_settle=30, d_tol=0.3, head_tol=15, start=(0.8, 0.0, 1.5))
    first_ok = next((x["t"] for x in r.log if x["off_err"] < 15), None)
    checks.append(("heading learned by (s)", first_ok if first_ok is not None else 99, first_ok is not None and first_ok < 35))
    return r, checks


def sc_wander(seed):
    r = Run(150, "static", REAL, psi0=1.0, seed=seed, obstacles=[PILLAR, (-0.8, 1.2, 0.3)] + config.OBSTACLES,
            script=[(1.0, {"intent": "wander"})]).run()
    rows = r.after(10)
    (x0, y0), (x1, y1) = config.ARENA
    travelled = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(rows, rows[1:]))
    return r, [
        ("collisions", r.w.collisions, r.w.collisions == 0),
        ("collision log", str(r.w.collision_log[:4]), True),
        ("min clearance (m)", r.w.min_clearance, r.w.min_clearance > -0.01),
        ("closest to person (m)", r.w.min_person_dist, r.w.min_person_dist > 0.9),
        ("distance travelled (m)", travelled, travelled > 6.0),
        ("z err max (m)", r.stat(rows, "zerr", max), r.stat(rows, "zerr", max) < 0.25),
    ]


def sc_lossy_links(seed):
    r, checks = follow(REAL, "static", 70, seed, t_settle=30, d_tol=0.35, head_tol=15,
                       cmd_loss=0.15, telem_loss=0.15, person_drop_p=0.05, person_drop_s=(0.3, 1.5), vision_latency=0.2)
    checks.append(("unwanted disarms", r.w.disarm_events, r.w.disarm_events == 0))
    checks.append(("watchdog disarms", len(r.wd_reasons), not r.wd_reasons))
    return r, checks


def sc_failsafe(seed):
    r = Run(14, "static", IDEAL, psi0=1.0, seed=seed, script=[(1.0, {"intent": "follow_me"})], stop_cmds_at=10.0).run()
    t_off = next((x["t"] for x in r.log if x["t"] > 10 and not x["armed"]), None)
    return r, [("motors off after last command (s)", (t_off - 10) if t_off else 99, t_off is not None and 0.45 < t_off - 10 < 0.6),
               ("was flying before", r.log[int(9.5 * G["HZ"])]["armed"], bool(r.log[int(9.5 * G["HZ"])]["armed"]))]


def sc_dance_then_back(seed):
    script = [(1.0, {"intent": "follow_me"}), (30.0, {"intent": "mood", "mood": "dance"})]
    r = Run(60, "static", REAL, psi0=1.0, seed=seed, script=script).run()
    modes = {x["mode"] for x in r.after(31, 36)}
    return r, [("danced", "DANCE" in modes, "DANCE" in modes),
               ("back to FOLLOW", r.log[-1]["mode"], r.log[-1]["mode"] == "FOLLOW"),
               ("dist at end (m)", r.stat(r.after(55), "dist"), abs(r.stat(r.after(55), "dist") - G["D_FOLLOW"]) < 0.5),
               ("collisions", r.w.collisions, r.w.collisions == 0)]


def sc_tof_noisy_vision(seed):
    """Vision z bad (25 cm noise, short dropouts) but the ToF present: height within 5 cm rms, V motor not thrashing.
    The same run without the ToF is printed for comparison (today's vision-only height was 0.10-0.19 m rms here)."""
    bad = dict(z_noise=0.25, balloon_drop_p=0.01, balloon_drop_s=(0.2, 0.6))
    r, checks = follow(REAL, "static", 60, seed, t_settle=25, z_tol=0.05, tof=True, **bad)
    r0 = Run(60, "static", REAL, psi0=1.0, seed=seed, script=[(1.0, {"intent": "follow_me"})], tof=False, **bad).run()
    rows = r.after(25)
    thrash = lambda v: statistics.mean(abs(x) for x in v)
    rms0 = math.sqrt(r0.stat(r0.after(25), "zerr", lambda v: statistics.mean(x * x for x in v)))
    tof_share = sum(x["z_src"] == "tof" for x in rows) / len(rows)
    return r, checks + [
        ("z err max (m)", r.stat(rows, "zerr", max), r.stat(rows, "zerr", max) < 0.12),
        ("V motor |vz| mean", r.stat(rows, "vz", thrash), r.stat(rows, "vz", thrash) < 0.16),
        ("ticks with the ToF owning z", tof_share, tof_share > 0.9),
        ("ToF frames valid (%)", 100 * r.w.tof_valid / max(1, r.w.tof_total), r.w.tof_valid > 0.9 * r.w.tof_total),
        ("same run without ToF: z err rms (m) (info)", rms0, True)]


def sc_no_tof(seed):
    """HAS_TOF 0 (alt = -1 in every frame): exactly the vision-only height hold, and the watchdog stays quiet."""
    r, checks = follow(REAL, "static", 60, seed, t_settle=25, tof=False)
    return r, checks + [("ticks with the ToF owning z", sum(x["z_src"] == "tof" for x in r.log), all(x["z_src"] == "cam" for x in r.log)),
                        ("watchdog disarms", len(r.wd_reasons), not r.wd_reasons)]


def sc_board_failsafe(seed):
    """Our commands do not reach the board for 0.8 s while we think we fly: it trips its failsafe at 0.5 s and says so
    (armed 0, age >= 500). The laptop must disarm for that reason and not spin the motors up again when the link is back."""
    r = Run(20, "static", IDEAL, psi0=1.0, seed=seed, script=[(1.0, {"intent": "hover"})], cmd_gap=(10.0, 10.8), tof=True).run()
    t_wd = next((x["t"] for x in r.log if x["t"] > 10 and not x["cmd_armed"]), None)
    return r, [("laptop disarmed after the gap began (s)", (t_wd - 10) if t_wd else 99, t_wd is not None and 0.45 <= t_wd - 10 < 0.9),
               ("reason", r.wd_reasons[-1][1] if r.wd_reasons else "-", any("failsafe" in s for _, s in r.wd_reasons)),
               ("motors off once the link is back", sum(bool(x["armed"]) for x in r.after(11.0)), not any(x["armed"] for x in r.after(11.0))),
               ("no watchdog disarm before the gap", sum(tt < 10 for tt, _ in r.wd_reasons), all(tt >= 10 for tt, _ in r.wd_reasons)),
               ("was flying before", r.log[int(9.5 * G["HZ"])]["armed"], bool(r.log[int(9.5 * G["HZ"])]["armed"]))]


SCENARIOS = {f.__name__[3:]: f for f in (
    sc_failsafe, sc_follow_static_ideal, sc_follow_static_real, sc_follow_walk, sc_follow_route, sc_hover_gusts, sc_rotate,
    sc_go_to, sc_come_here, sc_altitude_cmds, sc_lift_drift, sc_wrong_initial_heading, sc_gyro_drift,
    sc_long_hover_then_follow, sc_wander, sc_lossy_links, sc_dance_then_back, sc_tof_noisy_vision, sc_no_tof, sc_board_failsafe)}


# ---------------------------------------------------------------------------------------------------- plotting
def save_plot(name, r):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    L = r.log
    t = [x["t"] for x in L]
    fig, axs = plt.subplots(2, 2, figsize=(14, 9))
    ax = axs[0][0]
    (x0, y0), (x1, y1) = config.ARENA
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, lw=2))
    for cx, cy, cr in r.w.obstacles: ax.add_patch(Circle((cx, cy), cr, color="0.6"))
    sc = ax.scatter([x["x"] for x in L], [x["y"] for x in L], c=t, s=4, cmap="viridis")
    ax.plot([x["px"] for x in L], [x["py"] for x in L], "r-", lw=1, alpha=0.6, label="person")
    ax.plot(L[-1]["x"], L[-1]["y"], "bo", ms=10); ax.plot(L[-1]["px"], L[-1]["py"], "r^", ms=10)
    ax.add_patch(Circle((L[-1]["x"], L[-1]["y"]), config.R_BALLOON, fill=False, color="b"))
    if config.JUDGES_XY: ax.plot(*config.JUDGES_XY, "k*", ms=10)
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_title(f"{name}: top view (colour = time)"); ax.legend(loc="lower right")
    plt.colorbar(sc, ax=ax, label="t (s)")
    ax = axs[0][1]
    ax.plot(t, [x["z"] for x in L], label="z"); ax.axhline(G["Z_HOLD"], color="g", ls="--")
    ax.plot(t, [x["z_est"] for x in L], alpha=0.5, label="z est")
    ax.plot(t, [x["vz"] for x in L], alpha=0.5, label="vz cmd"); ax.set_title("height"); ax.grid(alpha=0.3); ax.legend()
    ax = axs[1][0]
    ax.plot(t, [x["off_err"] for x in L], label="heading estimate error (deg)")
    ax.plot(t, [x["bear_err"] for x in L], alpha=0.5, label="not facing person (deg)")
    ax.set_ylim(0, 180); ax.grid(alpha=0.3); ax.legend(); ax.set_title("heading")
    ax = axs[1][1]
    ax.plot(t, [x["dist"] for x in L], label="dist to person (m)"); ax.axhline(G["D_FOLLOW"], color="g", ls="--")
    ax.plot(t, [x["v"] for x in L], label="speed (m/s)")
    ax.plot(t, [x["vf"] for x in L], alpha=0.4, label="vf"); ax.plot(t, [x["vs"] for x in L], alpha=0.4, label="vs")
    ax.plot(t, [x["yr"] for x in L], alpha=0.4, label="yr")
    for te, it, _, _ in r.events: ax.axvline(te, color="k", alpha=0.3); ax.text(te, ax.get_ylim()[1] * 0.95, it, rotation=90, fontsize=7, va="top")
    ax.grid(alpha=0.3); ax.legend(fontsize=8); ax.set_title("distance / commands")
    fig.tight_layout()
    os.makedirs("sim_out", exist_ok=True)
    fig.savefig(f"sim_out/{name}.png", dpi=90); plt.close(fig)


# ---------------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*", help="substring filter")
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--tof", action="store_true", help="ToF altimeter on in every scenario (default: only in the ToF scenarios)")
    ap.add_argument("-v", action="store_true", help="print spoken lines and events")
    args = ap.parse_args()
    if args.tof: REAL["tof"] = IDEAL["tof"] = True
    import time
    all_ok = True
    t_start = time.time()
    for name, fn in SCENARIOS.items():
        if args.names and not any(n in name for n in args.names):
            continue
        worst = None
        for seed in range(args.seeds):
            r, checks = fn(seed)
            if worst is None:
                worst = [list(c) for c in checks]
            else:                                                     # keep the worst outcome per check
                for i, c in enumerate(checks):
                    if not c[2] and worst[i][2]: worst[i] = list(c)
            if args.plot and (seed == 0 or not all(c[2] for c in checks)):
                save_plot(name if seed == 0 else f"{name}_seed{seed}", r)
            if args.v:
                print(f"    seed {seed} events: {r.events}\n    spoken: {r.spoken}")
        ok = all(c[2] for c in worst)
        all_ok &= ok
        print(f"{'PASS' if ok else 'FAIL'}  {name}")
        for label, val, good in worst:
            v = f"{val:8.2f}" if isinstance(val, float) else f"{val!s:>8}"
            print(f"        {'ok  ' if good else 'FAIL'} {label:38s} {v}")
    print(f"\n{'ALL PASS' if all_ok else 'SOME FAILED'}   ({time.time() - t_start:.0f} s)")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
