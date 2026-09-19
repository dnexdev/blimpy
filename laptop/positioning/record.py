"""Standalone recorder for VISION-ONLY runs: localize / mono / fake_esp32 --sim publishing on 5007, no controller.

  python -m laptop.positioning.record                          # state from 5007 until Ctrl+C
  python -m laptop.positioning.record --name walk1 --seconds 60
  python -m laptop.positioning.record --source udp:5008 --telemetry

DO NOT run this next to follow_me / pilot: UdpJson binds with SO_REUSEADDR and this process would steal their
datagrams (they would see "BALLOON LOST"). When a controller runs, use ITS --log flag: it records the same state
stream plus the telemetry and the commands. --telemetry binds 5006 and steals from teleop/follow_me the same way;
use it only with a bare ESP32 / fake_esp32 and nothing else listening.
Output: data/positioning/<ts>_rec[_NAME]/ (laptop/positioning/session.py).
"""
import argparse, time
from ..control.protocol import TELEM_PORT, UdpJson
from .session import SessionLog
from .sources import make_source


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default=None, help="suffix for the session directory")
    ap.add_argument("--source", default="udp", help="udp | udp:<port> (see laptop/positioning/sources.py)")
    ap.add_argument("--telemetry", action="store_true", help="also record telemetry from 5006 (see caveat above)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long; 0 = until Ctrl+C")
    args = ap.parse_args()
    src = make_source(args.source).start()
    tel = UdpJson(TELEM_PORT) if args.telemetry else None
    log = SessionLog(name=args.name, tag="rec", meta={"source": args.source, "telemetry": args.telemetry})
    print(f"[record] {args.source} -> {log.path}   (Ctrl+C to stop)")
    t0 = t_print = time.monotonic()
    n = {"state": 0, "telem": 0}
    try:
        while not args.seconds or time.monotonic() - t0 < args.seconds:
            for msg in src.poll():
                log.state(msg); n["state"] += 1
            if tel:
                for msg, _ in tel.recv_all():
                    log.telem(msg); n["telem"] += 1
            if time.monotonic() - t_print > 1.0:
                t_print = time.monotonic()
                print(f"[record] {time.monotonic() - t0:6.0f} s  state={n['state']} telem={n['telem']}   ", end="\r", flush=True)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        src.stop(); log.close()
        print(f"\n[record] saved {log.n} rows to {log.path}")


if __name__ == "__main__":
    main()
