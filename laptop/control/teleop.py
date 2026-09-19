"""Keyboard teleop: sends 20 Hz commands, prints telemetry.

  python -m laptop.control.teleop                    # to fake_esp32 on this machine
  python -m laptop.control.teleop --esp 192.168.137.50

Keys:  w/s  forward/back (vf +-0.1)   a/d  yaw CCW/CW (yr +-0.1)   q/e  up/down (vz +-0.1)   j/l  strafe left/right (vs +-0.1)
       x    zero all setpoints        SPACE arm / disarm             k    KILL (disarm + zero)
       ESC  quit (sends disarm first)
Setpoints are sticky: press w twice for vf = 0.2 and it stays there until you change it.
"""
import argparse, time
from .protocol import CMD_PORT, TELEM_PORT, UdpJson, clamp, make_cmd, resolve
from .keys import ESC, KeyPoller


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--esp", default="127.0.0.1", help="ESP32 (or fake_esp32) IP")
    ap.add_argument("--hz", type=float, default=20)
    args = ap.parse_args()
    args.esp = resolve(args.esp)

    cmd, tel, keys = UdpJson(), UdpJson(TELEM_PORT), KeyPoller()
    vf = vs = yr = vz = 0.0
    arm, last_tel, step = False, None, 0.1
    print(__doc__)
    try:
        while True:
            t = time.monotonic()
            while (k := keys.poll()) is not None:
                if k == "w": vf += step
                elif k == "s": vf -= step
                elif k == "a": yr += step
                elif k == "d": yr -= step
                elif k == "q": vz += step
                elif k == "e": vz -= step
                elif k == "j": vs += step
                elif k == "l": vs -= step
                elif k == "x": vf = vs = yr = vz = 0.0
                elif k == " ": arm = not arm
                elif k == "k":
                    arm = False
                    vf = vs = yr = vz = 0.0
                elif k == ESC:
                    raise KeyboardInterrupt
                vf, vs, yr, vz = (round(clamp(v, -1, 1), 2) for v in (vf, vs, yr, vz))
            cmd.send(make_cmd(vf, yr, vz, arm, vs), (args.esp, CMD_PORT))
            r = tel.recv_latest(only_from=args.esp)
            if r:
                last_tel = r[0]
            ts = ""
            if last_tel:
                g = last_tel.get
                ts = (f"| armed={g('armed')} age={g('age')} yaw={g('yaw', 0):+.2f} gz={g('yr', 0):+.2f} "
                      f"alt={g('alt')} m=({g('mL', 0):+.2f},{g('mR', 0):+.2f},{g('mS', 0):+.2f},{g('mV', 0):+.2f})")
            print(f"[teleop] {'ARM ' if arm else 'safe'} vf={vf:+.2f} vs={vs:+.2f} yr={yr:+.2f} vz={vz:+.2f} {ts}     ",
                  end="\r", flush=True)
            time.sleep(max(0.0, 1 / args.hz - (time.monotonic() - t)))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            cmd.send(make_cmd(0, 0, 0, False), (args.esp, CMD_PORT)); time.sleep(0.02)
        keys.close()
        print("\n[teleop] disarmed, bye")


if __name__ == "__main__":
    main()
