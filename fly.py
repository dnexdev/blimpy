"""One command to fly the demo. Windows, Mac or Linux (nex's fly.sh is the same idea for a Mac shell).

  python fly.py                 camera + Bluetooth bridge + the pilot with the voice. Hold the balloon where you want it:
                                it arms by itself 8 s after the camera sees it. Then: "Blimpy, follow me" / "come here" / "stop"
  python fly.py hover           the height-only test (laptop/control/hover.py) instead of the pilot
  python fly.py --no-voice      anything else goes to the pilot (or to hover after 'hover'): --voice local, --mic AirPods ...
  python fly.py sim             no robot, no camera: the archived simulator behind the bridge, the pilot without voice (plumbing test)
  python fly.py status          what is running
  python fly.py stop            stop a bridge / camera left running (the bridge gets STOP + a clean disconnect, never a kill)

The bridge and the camera start as children of this window and stay up between flights: when the pilot quits (ESC),
Enter flies again without a new Bluetooth scan; q (or Ctrl+C anywhere) stops everything cleanly.
Their output: data/logs/fly_bridge.log, data/logs/fly_mono.log.  The pilot records to data/positioning/<ts>_pilot_fly/.
"""
import json, os, pathlib, signal, subprocess, sys, time, urllib.request
ROOT = pathlib.Path(__file__).resolve().parent
os.chdir(ROOT); sys.path.insert(0, str(ROOT))
from laptop import config                                     # loads .env (the voice key) for the children too
from laptop.control.protocol import ROOM_HTTP_PORT

BRIDGE_URL = f"http://127.0.0.1:{config.BLE['HTTP_PORT']}"
MONO_URL = f"http://127.0.0.1:{ROOM_HTTP_PORT}/frame.jpg"
LOGS = ROOT / "data" / "logs"; PIDS = LOGS / "fly.json"
# The bridge flags are nex's (fly.sh), for the laptop-mixer firmware: no gyro loop running L/R on its own, the lift motor may go flat
# out, height keeps its duty when the shared-supply budget binds. The onboard-mixer firmware mixes on the box and ignores them.
BRIDGE_ARGS = ["--motor-cap", "1", "--lift-first", "--total-cap", "1.6", "--no-yaw-hold"]
MONO_ARGS = ["--auto-calib", "--show"]
PILOT_ARGS = ["--omni-cam", "room", "--auto-arm", "8", "--log", "fly"]
HOVER_ARGS = ["--auto-arm", "8", "--log", "fly"]
SIM_BRIDGE_ARGS = ["--fake", "--sim", "--person", "walk"]
SIM_PILOT_ARGS = ["--omni-cam", "none", "--auto-arm", "8", "--no-voice"]


def get(url, timeout=1.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, r.headers, r.read()
    except Exception:
        return None


def bridge_status():
    r = get(BRIDGE_URL + "/status")
    if r is None: return None
    try: return json.loads(r[2])
    except ValueError: return None


def camera():
    """None = no camera answering; else the picture's metadata (balloon_box says whether it sees the balloon)."""
    r = get(MONO_URL, 2.0)
    if r is None: return None
    try: return json.loads(r[1].get("X-Blimpy-Meta") or "{}")
    except ValueError: return {}


def start(name, mod, args):
    LOGS.mkdir(parents=True, exist_ok=True)
    path = LOGS / f"fly_{name}.log"
    p = subprocess.Popen([sys.executable, "-u", "-m", mod, *args], stdout=open(path, "w"), stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    print(f"[fly] started the {name}: python -m {mod} {' '.join(args)}   (output: {path.relative_to(ROOT)})")
    return p


def pids_save(d):
    LOGS.mkdir(parents=True, exist_ok=True); PIDS.write_text(json.dumps(d))


def pids_load():
    try: return json.loads(PIDS.read_text())
    except Exception: return {}


def stop_all(children, quiet=False):
    """The bridge first, over HTTP: STOP x3, disconnect, wait for the box to see it go (leaving mid-disconnect keeps Windows holding
    the link and the box off the air for the next run). The camera only holds the webcam: terminate is fine."""
    st = bridge_status()
    if st is not None:
        get(BRIDGE_URL + "/quit")
        if not quiet: print("[fly] bridge: STOP sent, disconnecting", end="", flush=True)
        t0 = time.time()
        while time.time() - t0 < 12 and bridge_status() is not None:
            if not quiet: print(".", end="", flush=True)
            time.sleep(0.5)
        if not quiet: print()
    b = children.get("bridge")
    if b is not None:
        try: b.wait(timeout=8)
        except subprocess.TimeoutExpired: b.kill(); print("[fly] bridge did not exit on its own: killed (cut the motor power if anything still spins)")
    m = children.get("mono")
    if m is not None and m.poll() is None:
        m.terminate()
        try: m.wait(timeout=5)
        except subprocess.TimeoutExpired: m.kill()
    if PIDS.exists(): PIDS.unlink()
    if not quiet: print("[fly] stopped")


def wait_for(label, probe, seconds, child=None, logname=None):
    print(f"[fly] waiting for the {label}", end="", flush=True)
    t0 = time.time()
    while time.time() - t0 < seconds:
        v = probe()
        if v: print(); return v
        if child is not None and child.poll() is not None:
            print(f"\n[fly] the {label} process exited (code {child.returncode}): see data/logs/{logname}"); return None
        print(".", end="", flush=True); time.sleep(1)
    print(); return None


def main():
    argv = sys.argv[1:]
    mode = "pilot"
    if argv and argv[0] in ("hover", "stop", "status", "sim"): mode = argv.pop(0)
    if argv and argv[0] in ("-h", "--help"): print(__doc__); return 0
    if mode == "status":
        st = bridge_status(); cam = camera()
        print("[fly] bridge: " + ("not running" if st is None else
              f"running, {'CONNECTED' if st.get('connected') else 'not connected yet'}, firmware {st.get('firmware')}, {'ARMED' if st.get('armed') else 'off'}, map {str(st.get('map', ''))[4:]}"))
        print("[fly] camera: " + ("not running" if cam is None else "running" + (", sees the balloon" if cam.get("balloon_box") else ", balloon not in the picture")))
        return 0
    if mode == "stop":
        d = pids_load(); children = {}
        if camera() is not None and d.get("mono"):
            try: os.kill(int(d["mono"]), signal.SIGTERM)
            except OSError: pass
        stop_all(children)
        return 0

    sim = mode == "sim"
    children = {}
    try:
        st = bridge_status()
        if st is None:
            children["bridge"] = start("bridge", "laptop.control.ble_gondola", SIM_BRIDGE_ARGS if sim else BRIDGE_ARGS)
        else:
            print(f"[fly] a bridge is already running ({'connected' if st.get('connected') else 'not connected yet'}): using it")
        if not sim:
            if camera() is None: children["mono"] = start("mono", "laptop.vision.mono", MONO_ARGS)
            else: print("[fly] the camera is already running: using it")
        pids_save({k: p.pid for k, p in children.items()})

        st = wait_for("box", lambda: (lambda s: s if s and s.get("connected") else None)(bridge_status()), 60, children.get("bridge"), "fly_bridge.log")
        if st is None:
            print("[fly] the box has not connected: is it powered, held still, not connected to another laptop? See data/logs/fly_bridge.log")
            stop_all(children); return 1
        print(f"[fly] box connected: {st.get('firmware')}, motor map {str(st.get('map', ''))[4:]} from {st.get('map_source')}")
        if "NOT MEASURED" in str(st.get("map_source")) and not sim:
            print("[fly] WARNING: the motor map is the untested placeholder. Do not fly on it: python tools/motor_map.py")
        if not sim:
            meta = wait_for("camera", camera, 40, children.get("mono"), "fly_mono.log")
            if meta is None:
                print("[fly] no picture from the camera: is the webcam free (close other camera apps), is the mat in view? See data/logs/fly_mono.log")
                stop_all(children); return 1
            print("[fly] camera up" + (", it sees the balloon" if meta.get("balloon_box") else ": the balloon is NOT in the picture yet (it arms once it is)"))

        while True:
            if mode == "hover": mod, args = "laptop.control.hover", HOVER_ARGS + argv
            elif sim: mod, args = "laptop.control.pilot", SIM_PILOT_ARGS + argv
            else: mod, args = "laptop.control.pilot", PILOT_ARGS + argv
            print(f"[fly] python -m {mod} {' '.join(args)}   (ESC quits it; the bridge and the camera stay up)\n")
            subprocess.call([sys.executable, "-m", mod, *args])
            try:
                a = input("\n[fly] Enter = fly again   h = hover test   p = pilot   q = quit (stops the bridge and the camera) > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                a = "q"
            if a == "q": break
            if a == "h": mode = "hover"
            elif a == "p": mode = "pilot" if not sim else "sim"
    except KeyboardInterrupt:
        print("\n[fly] Ctrl+C: stopping everything")
    stop_all(children)
    return 0


if __name__ == "__main__":
    sys.exit(main())
