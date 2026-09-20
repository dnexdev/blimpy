"""Spin the lift motor through the RUNNING bridge (no second Bluetooth connection) and print what the bridge sent.

  python -m laptop.control.ble_gondola --no-yaw-hold      # already running, connected
  python tools/lift_test.py                               # vz 0.3 for 2 s
  python tools/lift_test.py --vz 0.5 --secs 3             # the most the mixer allows (its per-motor cap)
  python tools/lift_test.py --vz -0.3                     # reverse

Quit hover / teleop / pilot first: one commander at a time.
"""
import argparse, json, os, sys, time, urllib.request
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from laptop import config
from laptop.control.protocol import CMD_PORT, UdpJson, make_cmd

STATUS = f"http://127.0.0.1:{config.BLE['HTTP_PORT']}/status"


def status():
    try:
        return json.load(urllib.request.urlopen(STATUS, timeout=1))
    except Exception as e:
        return {"error": str(e)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vz", type=float, default=0.3)
    ap.add_argument("--secs", type=float, default=2.0)
    args = ap.parse_args()
    s = status()
    if not s.get("connected"):
        sys.exit(f"[lift] the bridge is not connected to the box: {s}")
    out, t0, t_print = UdpJson(), time.monotonic(), 0.0
    try:
        while (t := time.monotonic() - t0) < args.secs:
            out.send(make_cmd(0, 0, args.vz, True), ("127.0.0.1", CMD_PORT)); time.sleep(0.05)
            if t >= t_print:
                t_print = t + 0.5; s = status()
                print(f"[lift] {t:3.1f} s  armed={s.get('armed')}  motors={s.get('motors')}  sent: {s.get('last_line')}")
    finally:
        for _ in range(5):
            out.send(make_cmd(0, 0, 0, False), ("127.0.0.1", CMD_PORT)); time.sleep(0.02)
    time.sleep(0.5); s = status()
    print(f"[lift] after: armed={s.get('armed')}  motors={s.get('motors')}  sent: {s.get('last_line')}")


if __name__ == "__main__":
    main()
