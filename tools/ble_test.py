"""Offline test of the BLE bridge (laptop/control/ble_gondola.py) on the simulated robot. No Bluetooth, no hardware, ~12 s.

  python tools/ble_test.py

Checks: IMU line parsing in every layout; the udp command -> mixer -> "MOTORS c d e f" path (values, signs, the 50 % cap,
the 20 Hz rate); telemetry back on 5006 with the heading from the IMU and the correct sign; failsafe STOP within 500 ms
of the last command and on arm 0; imu_store latest/history and the localhost HTTP endpoints; recovery after a link drop.
"""
import json, os, pathlib, sys, threading, time, urllib.request
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop import config
from laptop.control import imu_store
from laptop.control.ble_gondola import Bridge, SimTransport, to_pct
from laptop.control.protocol import CMD_PORT, TELEM_PORT, UdpJson, make_cmd
from laptop.sim.world import IDEAL, World

HTTP = 5018
LOCAL = ("127.0.0.1", CMD_PORT)
results = {}


def check(name, ok, detail=""):
    results[name] = bool(ok); print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def wait_for(cond, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond(): return True
        time.sleep(0.01)
    return cond()


# 1. parser
p = imu_store.parse
check("parse key=value", p("IMU: yaw=90 pitch=-1.5 roll=0.2 gz=-57.2958") and abs(p("yaw=90 gz=-57.2958")["gz_rad"] + 1.0) < 1e-3
      and abs(p("yaw=90 gz=-57.2958")["yaw_rad"] - 1.5708) < 1e-3)
check("parse bare 6 numbers", p("0.01,-0.02,0.98,1.2,-0.4,3.1") == {"ax": 0.01, "ay": -0.02, "az": 0.98, "gx": 1.2, "gy": -0.4, "gz": 3.1, "gz_rad": 3.1 * 3.141592653589793 / 180})
check("parse json + rad units", abs(p('{"gz": 2.0, "yaw": 1.0}', units="rad")["gz_rad"] - 2.0) < 1e-9)
check("parse custom fields", p("1 2 3 4", fields=("a", "b", "gz", "d"))["gz_rad"] > 0)
check("parse garbage -> None", p("hello") is None and p("") is None)

# 2. bridge on the simulated robot
imu_store.reset()
world = World(dict(IDEAL), person="static", psi0=0.0, epoch=time.monotonic())
tr = SimTransport(world, publish_state=False, imu_units=config.BLE["GYRO_UNITS"])
br = Bridge(tr, http_port=HTTP, log=lambda s: None)
threading.Thread(target=br.run, daemon=True).start()
cmd, tel = UdpJson(), UdpJson(TELEM_PORT)
check("sim robot connected", wait_for(lambda: tr.connected, 3))

def drive(vf=0.0, vs=0.0, yr=0.0, vz=0.0, arm=True, seconds=0.6):
    frames = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        cmd.send(make_cmd(vf, yr, vz, arm, vs), LOCAL)
        for m, _ in tel.recv_all(only_from="127.0.0.1"): frames.append(m)
        time.sleep(0.05)
    return frames

fr = drive(vf=0.3)
m = tr.last_motors or {}
check("vf 0.3 -> C,D = 30 (L,R forward), E,F = 0", m.get("C") == 30 and m.get("D") == 30 and m.get("E") == 0 and m.get("F") == 0, str(m))
check("MOTORS line format", (tr.last_cmd or "").startswith("MOTORS ") and len(tr.last_cmd.split()) == 5, repr(tr.last_cmd))
check("telemetry ~20 Hz", 8 <= len(fr) <= 16, f"{len(fr)} frames in 0.6 s")
last = fr[-1] if fr else {}
check("telemetry armed, mL 0.3, small age", last.get("armed") == 1 and abs(last.get("mL", 0) - 0.3) < 0.02 and 0 <= last.get("age", 99) < 120, str(last))
n0 = tr.n_cmds; drive(vf=0.3, seconds=1.0)
check("BLE writes rate-limited (heartbeat ~4/s when steady)", 2 <= tr.n_cmds - n0 <= 8, f"{tr.n_cmds - n0} lines in 1 s")

fr = drive(vs=0.2, vz=-0.2)
m = tr.last_motors
check("vs/vz map to E/F with sign", m["E"] == 20 and m["F"] == -20 and m["C"] == 0, str(m))

yaw0 = (drive(yr=0.0, seconds=0.3) or [{}])[-1].get("yaw", 0.0)
fr = drive(yr=0.6, seconds=1.2)
m = tr.last_motors; yaw1 = fr[-1]["yaw"]
check("yr + -> D > C (CCW torque) and yaw increases", m["D"] > m["C"] and yaw1 > yaw0 + 0.1, f"C={m['C']} D={m['D']} yaw {yaw0:.2f}->{yaw1:.2f}")
check("telemetry yr matches the sim gyro", abs(fr[-1]["yr"] - world.gz_meas) < 0.3, f"{fr[-1]['yr']:.2f} vs {world.gz_meas:.2f}")

drive(vf=1.0, seconds=1.5)                      # the spin from the yr step is still being braked: one side may sit below the cap
m = tr.last_motors
check("cap: vf 1.0 -> 50 % (never above)", max(m["C"], m["D"]) == 50 and m["C"] <= 50 and m["D"] <= 50 and m["F"] == 0, str(m))

# 3. failsafe
drive(vf=0.3, seconds=0.4); t_stop = time.monotonic(); off = None
while time.monotonic() - t_stop < 1.5:
    for m, _ in tel.recv_all(only_from="127.0.0.1"):
        if m["armed"] == 0 and off is None: off = time.monotonic() - t_stop; last = m
    if off is not None: break
    time.sleep(0.01)
check("failsafe: STOP + armed 0 within 0.45-0.75 s", off is not None and 0.45 < off < 0.75 and tr.last_cmd == "STOP" and last["age"] >= 500,
      f"{None if off is None else round(off * 1000)} ms, last line {tr.last_cmd!r}, age {last.get('age')}")
drive(vf=0.3, seconds=0.4); drive(vf=0.3, arm=False, seconds=0.2)
check("arm 0 -> STOP", tr.last_cmd == "STOP" and world.motor_override == (0.0, 0.0, 0.0, 0.0))

# 4. store + http
l = imu_store.latest(); h = imu_store.history(n=50); st = imu_store.stats()
check("imu_store latest/history/stats", l and "gz_rad" in l and "yaw_rad" in l and len(h) == 50 and st["hz"] > 15, f"hz {st['hz']} n {st['n']}")
j = json.load(urllib.request.urlopen(f"http://127.0.0.1:{HTTP}/imu", timeout=2))
s = json.load(urllib.request.urlopen(f"http://127.0.0.1:{HTTP}/status", timeout=2))
hh = json.load(urllib.request.urlopen(f"http://127.0.0.1:{HTTP}/imu/history?n=10", timeout=2))
check("http /imu /status /imu/history", "gz_rad" in j and s["connected"] is True and len(hh) == 10 and "motors" in s)

# 5. link drop and recovery
n = tr.connects; tr.drop()
check("link drop -> motors off, then reconnect", wait_for(lambda: not tr.connected, 1) and wait_for(lambda: tr.connects == n + 1, 3))
fr = drive(vf=0.3, seconds=0.6)
m = tr.last_motors
check("drives again after reconnect", 25 <= m["C"] <= 35 and 25 <= m["D"] <= 35 and fr[-1]["armed"] == 1 and fr[-1]["ble"] == 1, str(m))

br.stop.set(); time.sleep(0.4)
check("exit sends STOP", tr.last_cmd == "STOP")
n_fail = sum(not v for v in results.values())
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
sys.exit(1 if n_fail else 0)
