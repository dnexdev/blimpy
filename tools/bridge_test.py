"""Offline test of the BLE bridge against the ONBOARD-MIXER firmware (firmware/esp32_master.ino, 2026-09-20), no hardware,
no simulator: a stub transport plays the box. ~3 s.

  python tools/bridge_test.py

Checks: the box's telemetry line parses (yaw, gzc, st, age, mc..mf, bias); MAP / CMD text; the map file overrides
config.BLE; on connect the bridge sends MAP; it sends CMD setpoints (not MOTORS) once it has seen an onboard line, only
while armed and fresh; telemetry says armed only once the box reports st 1; a silent box stops the setpoints; STOP is the
disarmed heartbeat; an older firmware (no st in the line) still gets MOTORS lines from the laptop mixer.
"""
import json, os, pathlib, sys, tempfile, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop import config
from laptop.control import imu_store
from laptop.control.ble_gondola import Bridge, Transport, cmd_line, from_pct, map_line, probe_verdict, to_pct
from laptop.control.protocol import CMD_PORT, TELEM_PORT, UdpJson, make_cmd

B = config.BLE
FAILS = []


def check(label, ok, detail=""):
    print(f"  {'ok  ' if ok else 'FAIL'} {label}" + (f"   ({detail})" if detail else ""))
    if not ok: FAILS.append(label)


class StubBox(Transport):
    """The box: records what the bridge writes, lets the test push telemetry lines."""
    def __init__(self):
        self.sent = []; self.connected = True; self.connects = 1; self.on_line = None
    def send(self, text):
        if not self.connected: return False
        self.sent.append(text); return True
    def line(self, s): self.on_line(s.encode())
    def since(self, n): return self.sent[n:]


ONBOARD = "A:-0.16,-0.01,1.08;G:-2.1,2.0,-0.4;yaw:{yaw:.1f};gzc:{gzc:.2f};st:{st};age:{age};mc:{mc};md:{md};me:{me};mf:{mf};bias:-0.36"
LEGACY = "A:-0.161,-0.009,1.077;G:-2.09,2.02,-0.35;T:44.5"


def main():
    print("[bridge_test] parsing")
    d = imu_store.parse(ONBOARD.format(yaw=-123.4, gzc=-12.3, st=1, age=120, mc=-50, md=50, me=0, mf=7))
    check("onboard line: A/G vectors", d is not None and abs(d["az"] - 1.08) < 1e-6 and abs(d["gz"] + 0.4) < 1e-6, str(d)[:80])
    check("onboard line: yaw / gzc / st / age / motors / bias", d is not None and d["yaw"] == -123.4 and d["gzc"] == -12.3 and d["st"] == 1
          and d["age"] == 120 and (d["mc"], d["md"], d["me"], d["mf"]) == (-50, 50, 0, 7) and d["bias"] == -0.36)
    check("onboard line: yaw_rad wrapped, gz_rad", d is not None and abs(d["yaw_rad"] - (-123.4 * 3.141592653589793 / 180)) < 1e-6 and "gz_rad" in d)
    e = imu_store.parse("IMU_ERROR;st:2;age:900;mc:0;md:0;me:0;mf:0")
    check("IMU_ERROR line still carries the state", e is not None and e["st"] == 2 and "gz" not in e, str(e))
    l = imu_store.parse(LEGACY)
    check("legacy line: no st", l is not None and "st" not in l and "gz_rad" in l and l.get("temp") == 44.5)

    print("[bridge_test] text")
    old = dict(MOTORS=dict(B["MOTORS"]), SIGN=dict(B["SIGN"]), GYRO_SIGN=B.get("GYRO_SIGN", 1))
    B["MOTORS"], B["SIGN"], B["GYRO_SIGN"] = {"L": "D", "R": "C", "S": "F", "V": "E"}, {"L": 1, "R": -1, "S": -1, "V": 1}, -1
    check("map_line", map_line() == "MAP LD+ RC- SF- VE+ G-", map_line())
    check("cmd_line rounds to percent and clamps", cmd_line((0.2, -0.05, 1.7, 0.004)) == "CMD 20 -5 100 0", cmd_line((0.2, -0.05, 1.7, 0.004)))
    check("to_pct / from_pct round trip through the map", from_pct(to_pct((0.2, -0.3, 0.1, 0.4))) == (0.2, -0.3, 0.1, 0.4), str(to_pct((0.2, -0.3, 0.1, 0.4))))
    B["MOTORS"], B["SIGN"], B["GYRO_SIGN"] = old["MOTORS"], old["SIGN"], old["GYRO_SIGN"]

    print("[bridge_test] calib/motor_map.json overrides config.BLE")
    b = dict(MOTORS={"L": "C", "R": "D", "S": "E", "V": "F"}, SIGN={"L": 1, "R": 1, "S": 1, "V": 1}, GYRO_SIGN=1)
    saved = config.MOTOR_MAP_FILE
    with tempfile.TemporaryDirectory() as td:
        p = pathlib.Path(td) / "m.json"
        p.write_text(json.dumps({"measured": "2026-09-20", "MOTORS": {"L": "f", "R": "e", "S": "d", "V": "c"}, "SIGN": {"L": -1, "R": 1, "S": 1, "V": -1}, "GYRO_SIGN": -1}))
        config.MOTOR_MAP_FILE = os.path.relpath(p, ROOT); config._load_motor_map(b)
        check("file read, letters upper-cased, gyro sign taken", b["MOTORS"] == {"L": "F", "R": "E", "S": "D", "V": "C"} and b["SIGN"]["V"] == -1 and b["GYRO_SIGN"] == -1, str(b))
        config.MOTOR_MAP_FILE = "calib/does_not_exist.json"; b2 = dict(b); config._load_motor_map(b2)
        check("no file: placeholders flagged NOT MEASURED", "NOT MEASURED" in b2["MAP_SOURCE"], b2["MAP_SOURCE"])
        p.write_text(json.dumps({"MOTORS": {"L": "C", "R": "C", "S": "E", "V": "F"}, "SIGN": {"L": 1, "R": 1, "S": 1, "V": 1}}))
        config.MOTOR_MAP_FILE = os.path.relpath(p, ROOT)
        try: config._load_motor_map(dict(b)); check("two roles on one letter is refused", False)
        except SystemExit as ex: check("two roles on one letter is refused", "one each" in str(ex))
    config.MOTOR_MAP_FILE = saved

    print("[bridge_test] probe verdict")
    got = [(i * 0.2, imu_store.parse(ONBOARD.format(yaw=1, gzc=0, st=0, age=-1, mc=0, md=0, me=0, mf=0))) for i in range(50)]
    v = probe_verdict(got, 10.0)
    check("5 lines/s from the onboard mixer is OK", "OK, onboard mixer" in v, v[:90])
    got = [(i * 0.2, imu_store.parse(LEGACY)) for i in range(50)]
    check("5 lines/s from an older build is flagged SLOW", "SLOW" in probe_verdict(got, 10.0) and "onboard" not in probe_verdict(got, 10.0))

    print("[bridge_test] the bridge against a stub box (onboard mixer)")
    # identity map for the stub sections (calib/motor_map.json is the real box's; the expectations below are in letters)
    real_map = dict(MOTORS=dict(B["MOTORS"]), SIGN=dict(B["SIGN"]), GYRO_SIGN=B.get("GYRO_SIGN", 1))
    B["MOTORS"], B["SIGN"], B["GYRO_SIGN"] = {"L": "C", "R": "D", "S": "E", "V": "F"}, {"L": 1, "R": 1, "S": 1, "V": 1}, 1
    imu_store.reset()
    box = StubBox(); logs = []
    br = Bridge(box, http_port=None, log=logs.append)
    cmd, tel = UdpJson(), UdpJson(TELEM_PORT)
    t = 100.0
    def ticks(n, dt=0.02):
        nonlocal t
        for _ in range(n): t += dt; br.tick(t)
    ticks(3)
    check("MAP sent on connect, before anything else", box.sent[:1] == [map_line()], str(box.sent[:2]))
    check("firmware unknown yet: no motor line", not [s for s in box.sent if s.startswith(("CMD", "MOTORS"))])
    n0 = len(box.sent)
    cmd.send(make_cmd(0.2, 0.1, 0.0, True, -0.1), ("127.0.0.1", CMD_PORT)); time.sleep(0.05); ticks(3)
    check("armed by the pilot but firmware still unknown: nothing sent, armed 0", not box.since(n0) and br.telemetry(t)["armed"] == 0, str(box.since(n0)))
    box.line(ONBOARD.format(yaw=45.0, gzc=2.5, st=0, age=-1, mc=0, md=0, me=0, mf=0))
    br.t_imu = t                                    # the store stamps wall time; this test's clock is synthetic
    ticks(2)
    check("onboard line seen -> logged", any("ONBOARD MIXER" in s for s in logs), str(logs[-1:])[:100])
    check("yaw from the box (deg -> rad), gzc as yaw rate", abs(br.yaw - 45 * 3.141592653589793 / 180) < 1e-6 and abs(br.gz - 2.5 * 3.141592653589793 / 180) < 1e-6, f"{br.yaw:.3f} {br.gz:.3f}")
    lines = box.since(n0)
    check("CMD setpoints sent (percent: vf vs yr vz), no MOTORS", lines and lines[-1] == "CMD 20 -10 10 0" and not [s for s in lines if s.startswith("MOTORS")], str(lines[-3:]))
    check("bridge armed, but telemetry says armed 0 until the box reports st 1", br.armed and br.telemetry(t)["armed"] == 0 and br.telemetry(t)["st"] == 0)
    box.line(ONBOARD.format(yaw=45.1, gzc=2.4, st=1, age=30, mc=20, md=20, me=-10, mf=10)); br.t_imu = t
    ticks(1)
    tl = br.telemetry(t)
    check("box says st 1 -> armed 1, duties mirrored from the box", tl["armed"] == 1 and tl["st"] == 1 and (tl["mL"], tl["mR"], tl["mS"], tl["mV"]) == (0.2, 0.2, -0.1, 0.1), str(tl))
    n1 = len(box.sent)
    for _ in range(10):                                            # keep the pilot's command fresh, box talking every 200 ms
        cmd.send(make_cmd(0.2, 0.1, 0.0, True, -0.1), ("127.0.0.1", CMD_PORT)); time.sleep(0.01)
        ticks(10); box.line(ONBOARD.format(yaw=45.1, gzc=2.4, st=1, age=30, mc=20, md=20, me=-10, mf=10)); br.t_imu = t
    n_cmd = sum(1 for s in box.since(n1) if s.startswith("CMD"))
    check("unchanged setpoint repeats at ~CMD_HZ (heartbeat for the box's 500 ms timeout)", 16 <= n_cmd <= 22, f"{n_cmd} CMD in 2 s")
    cmd.send(make_cmd(0.3, 0.1, 0.0, True, -0.1), ("127.0.0.1", CMD_PORT)); time.sleep(0.01); n2 = len(box.sent); ticks(4)
    check("a changed setpoint goes out within 80 ms", any(s == "CMD 30 -10 10 0" for s in box.since(n2)), str(box.since(n2)))
    n3 = len(box.sent)
    for _ in range(9): cmd.send(make_cmd(0.3, 0.1, 0.0, True, -0.1), ("127.0.0.1", CMD_PORT)); time.sleep(0.01); ticks(10)   # 1.8 s, box silent
    check("box silent 1.5 s -> STOP, armed 0, logged", box.sent[-1] == "STOP" and br.telemetry(t)["armed"] == 0 and any("box silent" in s for s in logs),
          str(box.since(n3)[-3:]))
    box.line(ONBOARD.format(yaw=45.1, gzc=2.4, st=1, age=30, mc=20, md=20, me=-10, mf=10)); br.t_imu = t
    cmd.send(make_cmd(0.3, 0.1, 0.0, True, -0.1), ("127.0.0.1", CMD_PORT)); time.sleep(0.01); ticks(3)
    check("box back -> setpoints resume", box.sent[-1].startswith("CMD"), box.sent[-1])
    cmd.send(make_cmd(0, 0, 0, False), ("127.0.0.1", CMD_PORT)); time.sleep(0.01); n4 = len(box.sent); ticks(60)
    stops = [s for s in box.since(n4) if s == "STOP"]
    check("disarmed: STOP once, then once a second", 1 <= len(stops) <= 3 and not [s for s in box.since(n4) if s.startswith("CMD")], str(box.since(n4)))
    box.connected = False; box.connects += 1; box.connected = True; box.sent.clear(); ticks(2)
    check("reconnect: MAP again, firmware unknown again", box.sent[:1] == [map_line()] and br.onboard is None, str(box.sent[:2]))
    br._unsub(); br.cmd_in.sock.close()               # the next bridge binds the same port

    print("[bridge_test] the bridge against a stub box (older firmware: laptop mixer)")
    imu_store.reset()
    box = StubBox(); logs = []
    br = Bridge(box, http_port=None, log=logs.append)
    t = 200.0; ticks(2)
    for _ in range(3): box.line(LEGACY); br.t_imu = t
    cmd.send(make_cmd(0.2, 0.0, 0.0, True), ("127.0.0.1", CMD_PORT)); time.sleep(0.05); ticks(15)
    check("legacy line -> logged as laptop mixer", any("laptop" in s for s in logs), str(logs[-1:])[:100])
    lines = [s for s in box.sent if s.startswith(("MOTORS", "CMD"))]
    check("MOTORS lines, no CMD", lines and all(s.startswith("MOTORS") for s in lines), str(lines[-2:]))
    check("duties slewed by the laptop mixer", 0 < br.cur[0] <= 0.21 and br.telemetry(t)["armed"] == 1, str(br.cur))
    br._unsub(); br.cmd_in.sock.close()
    for s in (cmd.sock, tel.sock): s.close()
    B["MOTORS"], B["SIGN"], B["GYRO_SIGN"] = real_map["MOTORS"], real_map["SIGN"], real_map["GYRO_SIGN"]

    print(f"\n[bridge_test] {'ALL PASS' if not FAILS else 'FAILED: ' + ', '.join(FAILS)}")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
