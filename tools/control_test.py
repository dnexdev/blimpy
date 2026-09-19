"""Host-side control checks, no sockets, ~1 s. Python twin of firmware/test/test_mixer (pio test -e native).

  python tools/control_test.py

(1) mix: golden sequence and the yaw-rate integrator (wind-up, I_YR_MAX clamp, persistence, reset) of laptop/control/protocol.py::mix,
    same numbers as the firmware test.
(2) est: the ToF altitude path of laptop/control/estimator.py: ownership of z, the jump gate (hand under the lens), re-admission,
    hand-back to vision, plausibility limits, first fix.
(3) wd:  laptop/control/link.py::TelemWatchdog: silence, board-side failsafe, consecutive armed:0 frames, warning rate limit.
Run it after touching laptop/control/protocol.py, estimator.py, link.py or firmware/include/mixer.h.
"""
import os, pathlib, sys
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop.control import estimator
from laptop.control.estimator import StateEstimator
from laptop.control.link import TelemWatchdog
from laptop.control.protocol import I_YR_MAX, K_YR, KI_YR, MIX_DT, mix

EPS = 1e-6


def checker(tag):
    results = []

    def check(name, cond, extra=""):
        results.append(bool(cond)); print(f"[{tag}] {name}{'  ' + extra if extra else ''}  {'OK' if cond else 'FAIL'}")
    return results, check


def near(a, b, eps=EPS):
    return all(abs(x - y) < eps for x, y in zip(a, b))


# ---------- (1) ----------
def test_mix():
    results, check = checker("mix")
    cur = (0.0, 0.0, 0.0, 0.0); sp = (0.2, -0.1, 0.1, 0.05); got = {}
    for n in range(1, 13):
        cur = mix(sp, 0.0, cur, None)
        if n in (1, 4, 12): got[n] = cur
    check("golden sequence ticks 1 / 4 / 12 (firmware test_mix_golden_sequence)",
          near(got[1], (0.05, 0.05, -0.05, 0.05)) and near(got[4], (0.1, 0.2, -0.1, 0.05)) and near(got[12], (0.1, 0.3, -0.1, 0.05)), str(got))
    sp = (0.0, 0.0, 0.02, 0.0); state = {"yawI": 0.0}; cur = (0.0, 0.0, 0.0, 0.0); ok = True
    for n in range(1, 11):
        cur = mix(sp, 0.0, cur, state)
        want = K_YR * 0.02 + KI_YR * 0.02 * MIX_DT * n
        ok &= abs(cur[1] - want) < 1e-9 and abs(cur[0] + want) < 1e-9
    check("integrator: R = 0.02 + 0.0004 n for 10 ticks, L = -R", ok, f"tick 10 = {cur[1]:.4f}")
    for _ in range(390):
        cur = mix(sp, 0.0, cur, state)
    check("400 ticks: yawI clamped at I_YR_MAX, outputs (-0.17, 0.17)", abs(state["yawI"] - I_YR_MAX) < EPS and near(cur, (-0.17, 0.17, 0.0, 0.0)))
    for _ in range(100):
        cur = mix(sp, 0.0, cur, state)
    check("stays clamped", near(cur, (-0.17, 0.17, 0.0, 0.0)))
    check("wound-up integrator persists into a fresh output array (slew-limited to 0.05)", abs(mix(sp, 0.0, (0, 0, 0, 0), state)[1] - 0.05) < EPS)
    state["yawI"] = 0.0
    check("state['yawI'] = 0 (what World / the firmware do on disarm) -> 0.0204 on the next tick", abs(mix(sp, 0.0, (0, 0, 0, 0), state)[1] - 0.0204) < 1e-9)
    c = (0, 0, 0, 0)
    for _ in range(50):
        c = mix(sp, 0.0, c, None)
    check("state=None -> P only, 0.02 forever", abs(c[1] - 0.02) < EPS)
    state = {"yawI": 0.0}; c = (0, 0, 0, 0)
    for _ in range(400):
        c = mix((0.0, 0.0, -0.02, 0.0), 0.0, c, state)
    check("negative error clamps at -I_YR_MAX: (0.17, -0.17)", near(c, (0.17, -0.17, 0.0, 0.0)))
    return all(results)


# ---------- (2) ----------
def test_estimator_tof():
    results, check = checker("est")
    est = StateEstimator(); t = [0.0]
    good = 1.70 - estimator.TOF_BELOW

    def tick(alt, vis_z=1.70):
        t[0] += 1 / 15
        est.update_telem(0.0, 0.0, alt, int(t[0] * 1000), now=t[0])
        est.update_balloon([0.0, 0.0, vis_z], int(t[0] * 1000))
        est.observe_motion(0.0, 0.0, 0.0, t[0])
    for _ in range(10): tick(-1)
    check("no ToF (alt -1): vision owns z", not est.alt_ok and est.z_src == "cam" and abs(est.p[2] - 1.70) < 1e-9)
    for _ in range(10): tick(good)
    check("good ToF: alt_ok, z_src tof", est.alt_ok and est.z_src == "tof")
    for i in range(30): tick(good, 1.70 + (0.3 if i % 2 else -0.3))
    check(f"vision z +-0.3 ignored while the ToF is fresh: z = {est.p[2]:.3f}", abs(est.p[2] - 1.70) < 0.02 and est.z_src == "tof")
    r0 = est.alt_rejects
    for _ in range(40): tick(0.30)
    check(f"hand under the lens (alt 0.30) for 40 frames: rejected ({est.alt_rejects - r0}), ToF dropped, z stays {est.p[2]:.3f}",
          est.alt_rejects - r0 == 40 and not est.alt_ok and abs(est.p[2] - 1.70) < 0.02)
    for _ in range(10): tick(good)
    check("good readings again -> re-admitted", est.alt_ok)
    for _ in range(15): tick(-1, 1.50)
    check(f"ToF gone, vision says 1.50 -> vision owns z again ({est.p[2]:.3f})", not est.alt_ok and est.p[2] < 1.62)
    for _ in range(15): tick(good)
    for _ in range(6): tick(2.5)
    check("alt 2.5 m (beyond range) not accepted -> alt_ok drops", not est.alt_ok)
    for _ in range(15): tick(good)
    for _ in range(6): tick(0.02)
    check("alt 0.02 m (covered) not accepted -> alt_ok drops", not est.alt_ok)
    e2 = StateEstimator()
    e2.update_telem(0.0, 0.0, good, 0, now=0.0)
    check("ToF before any vision fix: p stays None", e2.p is None and e2.alt_ok)
    e2.update_balloon([1.0, 2.0, 1.9], 10)
    check("first vision fix takes the fresh, agreeing ToF z (1.70 not 1.90)", abs(e2.p[2] - 1.70) < 1e-9 and e2.p[0] == 1.0)
    e3 = StateEstimator(); e3.update_telem(0.1, 0.0)
    check("update_telem(yaw, yr) old signature still fine", e3.yaw_gyro == 0.1 and not e3.alt_ok)
    return all(results)


# ---------- (3) ----------
def test_watchdog():
    results, check = checker("wd")
    wd = TelemWatchdog(lost_ms=1000, board_off_n=6, age_warn_ms=300)
    fr = lambda armed, age: {"armed": armed, "age": age}
    check("never heard, not armed -> None", wd.check(False, 0.0) is None)
    wd.arm(10.0)
    check("armed, board never seen: None at +0.5 s, reason at +1.5 s", wd.check(True, 10.5) is None and "silent" in (wd.check(True, 11.5) or ""))
    wd = TelemWatchdog(lost_ms=1000, board_off_n=6, age_warn_ms=300); wd.arm(0.0)
    for i in range(3): wd.telem(fr(0, 40), 0.05 * i, True)
    check("3 frames armed:0 right after arming -> ok", wd.check(True, 0.2) is None)
    for i in range(3): wd.telem(fr(0, 40), 0.2 + 0.05 * i, True)
    check("6 consecutive armed:0 frames -> disarm", "motors off" in (wd.check(True, 0.4) or ""))
    wd = TelemWatchdog(lost_ms=1000, board_off_n=6, age_warn_ms=300); wd.arm(0.0)
    for i in range(20): wd.telem(fr(1, 40), 0.05 * i, True)
    check("healthy frames -> None", wd.check(True, 1.0) is None)
    check("silent 0.9 s -> None, 1.1 s -> silent", wd.check(True, 1.85) is None and "silent" in (wd.check(True, 2.05) or ""))
    wd = TelemWatchdog(lost_ms=1000, board_off_n=6, age_warn_ms=300); wd.arm(0.0)
    wd.telem(fr(1, 40), 0.0, True); wd.telem(fr(0, 620), 0.05, True); wd.telem(fr(1, 30), 0.1, True)
    check("one frame armed:0 age 620 between healthy ones -> failsafe reason", "failsafe" in (wd.check(True, 0.15) or ""))
    wd = TelemWatchdog(lost_ms=1000, board_off_n=6, age_warn_ms=300); wd.arm(0.0)
    w1 = wd.telem(fr(1, 350), 0.0, True); w2 = wd.telem(fr(1, 350), 1.0, True); w3 = wd.telem(fr(1, 350), 6.0, True)
    check("age 350 -> warning once per 5 s", w1 is not None and w2 is None and w3 is not None)
    check("not armed: frames with armed:0 never disarm", wd.telem(fr(0, 900), 7.0, False) is None or True and wd.check(False, 7.0) is None)
    return all(results)


if __name__ == "__main__":
    r = [test_mix(), test_estimator_tof(), test_watchdog()]
    print("PASS" if all(r) else "FAIL")
    sys.exit(0 if all(r) else 1)
