"""Headless regression test of the control stack over the real UDP protocol (no hardware, no keyboard).

  python archive/tools/sim_test.py          # over fake_esp32 (UDP board)
  python archive/tools/sim_test.py --ble    # over the BLE bridge + simulated robot (laptop/control/ble_gondola.py --fake --sim)

(1) Failsafe: arm + drive for 1.5 s, stop sending, measure time until telemetry reports armed=0 (must be ~500 ms).
(2) Follow-me vs a walking person for 45 s: heading nudge, then hold distance/heading. Reports tracking stats.
Uses fake_esp32 --sim as a subprocess. Run it after touching protocol.py, estimator.py, follow_me.py or the gains.
"""
import math, os, pathlib, statistics, subprocess, sys, time
ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop import config
from laptop.control.protocol import CMD_PORT, STATE_PORT, TELEM_PORT, UdpJson, make_cmd, now_ms, wrap
from laptop.control.estimator import StateEstimator
from laptop.control.behaviors import Behaviors

G = config.FOLLOW
LOCAL = ("127.0.0.1", CMD_PORT)


BLE = "--ble" in sys.argv          # run the same checks through the BLE bridge on its simulated robot (ble_gondola --fake --sim)


def start_fake(*extra):
    mod = ["laptop.control.ble_gondola", "--fake", "--sim", "--no-http"] if BLE else ["archive.fake_esp32", "--sim"]
    p = subprocess.Popen([sys.executable, "-m", *mod, *extra], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5 if BLE else 1.0)
    return p


def test_failsafe():
    proc = start_fake("--person", "static")
    tel, cmd = UdpJson(TELEM_PORT), UdpJson()
    try:
        t0, last = time.monotonic(), None
        while time.monotonic() - t0 < 1.5:
            cmd.send(make_cmd(0.3, 0, 0, True), LOCAL); time.sleep(0.05)
            r = tel.recv_latest(only_from=LOCAL[0])
            if r: last = r[0]
        if not (last and last["armed"] == 1 and last["mL"] > 0.2):
            print(f"[failsafe] FAIL: never armed/spun: {last}"); return False
        t_stop, t_off = time.monotonic(), None
        while time.monotonic() - t_stop < 2.0:
            r = tel.recv_latest(only_from=LOCAL[0])
            if r and r[0]["armed"] == 0:
                t_off = time.monotonic() - t_stop; break
            time.sleep(0.01)
        if t_off is None:
            print("[failsafe] FAIL: never disarmed"); return False
        ok = 0.45 < t_off < 0.75
        print(f"[failsafe] motors L={last['mL']:.2f} -> disarmed {t_off * 1000:.0f} ms after last command  {'OK' if ok else 'FAIL'}")
        return ok
    finally:
        proc.terminate(); tel.sock.close(); cmd.sock.close(); time.sleep(0.5)


def test_real_person():
    """--person real: the sim publishes person=None until fixes arrive on 5017, tracks them, drops them 1.5 s after the last."""
    import json, socket
    p = subprocess.Popen([sys.executable, "-m", "laptop.control.ble_gondola", "--fake", "--sim", "--person", "real", "--no-http"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    time.sleep(2.5)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", 5007)); rx.settimeout(0.05)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def sample(secs):
        got, t0 = [], time.time()
        while time.time() - t0 < secs:
            try:
                d, _ = rx.recvfrom(2048); got.append(json.loads(d).get("person"))
            except socket.timeout:
                pass
        return got
    try:
        before = sample(1.0)
        for _ in range(20):
            tx.sendto(json.dumps({"t": int(time.time() * 1000), "balloon": None, "person": [1.5, -0.4, 1.1], "person_id": 1,
                                  "src": "mono", "lost": None}).encode(), ("127.0.0.1", 5017))
            time.sleep(0.05)
        during = sample(1.0); after = sample(2.5)
    finally:
        p.terminate(); rx.close(); tx.close()
    ok = (bool(before) and all(x is None for x in before) and any(x and abs(x[0] - 1.5) < 0.2 for x in during)
          and bool(after[-5:]) and all(x is None for x in after[-5:]))
    print(f"[real person] none before {sum(x is None for x in before)}/{len(before)}, tracked {sum(bool(x) for x in during)}/{len(during)}, "
          f"lost after {sum(x is None for x in after[-5:])}/5  {'OK' if ok else 'FAIL'}")
    return ok


def test_follow_walk(seconds=45):
    proc = start_fake("--person", "walk", "--psi0", "-2.0")
    state_in, tel_in, cmd_out = UdpJson(STATE_PORT), UdpJson(TELEM_PORT), UdpJson()
    est = StateEstimator(alpha=G["POS_ALPHA"], beta=G["VEL_BETA"])
    beh = Behaviors(lambda s: print(f"\n   [say] {s}"))
    beh.handle({"intent": "follow_me"}, est)
    person, armed, said = None, False, False
    t0 = time.monotonic()
    dists, heads = [], []
    try:
        while time.monotonic() - t0 < seconds:
            t, now = time.monotonic(), now_ms()
            for s, _ in state_in.recv_all(only_from=LOCAL[0]):      # every row (the BLE sim also sends eye rows)
                if s.get("balloon"): est.update_balloon(s["balloon"], s.get("t", now))
                if s.get("person"): person = s["person"]; beh.on_person(person, s.get("t"))
                if s.get("fpv"): beh.on_fpv(s["fpv"], s.get("t"))
            r = tel_in.recv_latest(only_from=LOCAL[0])
            if r: est.update_telem(r[0]["yaw"], r[0].get("yr", 0.0))
            vf = vs = yr = vz = 0.0
            if est.p is not None:
                if not armed: armed = True; beh.on_armed(est)
                vf, vs, yr, vz, note = beh.step(est)
                if est.head_confident and not said:
                    said = True
                    print(f"[walk] heading acquired after {t - t0:.1f} s: offset={math.degrees(est.offset):+.0f} deg (truth {math.degrees(-2.0):+.0f})")
                if t - t0 > 25 and person and est.psi is not None:
                    dx, dy = person[0] - est.p[0], person[1] - est.p[1]
                    dists.append(math.hypot(dx, dy)); heads.append(abs(math.degrees(wrap(math.atan2(dy, dx) - est.psi))))
            est.observe_motion(vf, vs, vz)
            cmd_out.send(make_cmd(vf, yr, vz, True, vs), LOCAL)
            time.sleep(max(0, 1 / G["HZ"] - (time.monotonic() - t)))
    finally:
        for _ in range(3):
            cmd_out.send(make_cmd(0, 0, 0, False), LOCAL); time.sleep(0.02)
        time.sleep(0.3); proc.terminate()
        for u in (state_in, tel_in, cmd_out): u.sock.close()
    if not dists:
        print("[walk] FAIL: no follow data"); return False
    ok = 1.0 < statistics.mean(dists) < 2.0 and max(dists) < 3.0 and statistics.mean(heads) < 25 and abs(est.p[2] - G["Z_HOLD"]) < 0.25
    print(f"[walk] t>25s: dist mean={statistics.mean(dists):.2f} min={min(dists):.2f} max={max(dists):.2f} m (target {G['D_FOLLOW']}) | "
          f"heading err mean={statistics.mean(heads):.0f} max={max(heads):.0f} deg | z={est.p[2]:.2f} (target {G['Z_HOLD']})  {'OK' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    ok1 = test_failsafe()
    ok2 = test_follow_walk()
    ok3 = test_real_person()
    print("PASS" if ok1 and ok2 and ok3 else "FAIL")
    sys.exit(0 if ok1 and ok2 else 1)
