"""Go / no-go for the camera rig before a flight (README section 3, Build Blueprint section 7).

  python tools/vision_check.py                 # offline: calib files real? cameras placed sanely? tag size consistent? models? venue?
  python tools/vision_check.py --live          # + open both cameras (config.SOURCES or --a/--b) for 3 s: fps, skew, tag id 0, drift
  python tools/vision_check.py --live --person # + YOLO must see a person in both cameras (stand in view)
  python tools/vision_check.py --names A --live --a 1   # single camera (mono.py): no pair / skew checks
  python tools/vision_check.py --live --lag-b 180       # B is a WiFi phone ~180 ms behind: judge skew after that offset

One line per check: OK / WARN / FAIL. Exit 0 = no FAIL (WARNs are allowed), 1 = fix something first.
Logic lives in laptop/vision/preflight.py (tested by tools/vision_test.py on synthetic files and fake streams).
"""
import argparse, os, pathlib, sys
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop import config
from laptop.vision import preflight


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=config.CALIB_DIR); ap.add_argument("--names", nargs="+", default=[config.CALIB_NAMES["A"], config.CALIB_NAMES["B"]], help="calibration names for A and B (config.CALIB_NAMES), or just one for mono")
    ap.add_argument("--tag-size", type=float, default=config.TAG_SIZE_M); ap.add_argument("--targets", default="targets")
    ap.add_argument("--live", action="store_true", help="open the cameras and check fps / skew / tag / drift")
    ap.add_argument("--a", default=config.SOURCES["A"]); ap.add_argument("--b", default=config.SOURCES["B"])
    ap.add_argument("--seconds", type=float, default=3.0); ap.add_argument("--person", action="store_true")
    ap.add_argument("--drift-px", type=float, default=8.0); ap.add_argument("--skew-ms", type=float, default=100)
    ap.add_argument("--min-fps", type=float, default=10)
    ap.add_argument("--lag-b", type=int, default=0, help="ms; transport latency of camera B (same as localize --lag-b)")
    ap.add_argument("--rot-a", type=int, default=config.ROTATE["A"]); ap.add_argument("--rot-b", type=int, default=config.ROTATE["B"])
    args = ap.parse_args()
    checks = preflight.offline_checks(args.calib, tuple(args.names), args.tag_size, args.targets)
    print(preflight.format_checks(checks))
    if args.live:
        from laptop.vision.calib_io import Camera
        from laptop.vision.streams import Stream
        cams = {n: Camera.load(n, args.calib) for n in args.names}
        streams = {}
        try:
            for n, spec, lag, rot in zip(args.names, (args.a, args.b), (0, args.lag_b), (args.rot_a, args.rot_b)):
                streams[n] = Stream(spec, n, lag_ms=lag, rotate=rot).wait_first(timeout=20)
            live = preflight.live_checks(streams, cams, args.seconds, args.tag_size, args.drift_px, args.skew_ms, args.min_fps, args.person)
            print(preflight.format_checks(live)); checks += live
        finally:
            for s in streams.values(): s.stop()
    n_fail = sum(s == "FAIL" for s, _, _ in checks); n_warn = sum(s == "WARN" for s, _, _ in checks)
    print(f"{'NO-GO' if n_fail else 'GO'}: {n_fail} FAIL, {n_warn} WARN, {len(checks) - n_fail - n_warn} OK")
    sys.exit(preflight.worst(checks))


if __name__ == "__main__":
    main()
