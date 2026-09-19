"""Put a place or an obstacle into the venue file, from where something IS (vision) or from a tape measure.

  python -m laptop.positioning.capture_place judges --xy 2.5 0                    # tape measure, no vision needed
  python -m laptop.positioning.capture_place home --from balloon --seconds 5      # park the balloon there; localize/mono running
  python -m laptop.positioning.capture_place pillar --obstacle --r 0.2 --from person   # stand where the pillar is
  python -m laptop.positioning.capture_place judges --xy 3.4 0 --dry-run          # validate only, do not save

Vision path: listens on --source (default udp 5007) for --seconds, takes the MEDIAN of the fixes of --from
(person | balloon), prints count and spread. Same port-stealing rule as record.py: no controller may be running.
Writes venues/default.json (or --venue) after validate(); refuses to save on errors unless --force.
Places are what behaviors.py's go-to knows by name ("judges" is read by config.JUDGES_XY); obstacles feed avoid().
"""
import argparse, math, statistics, sys, time
from .. import config
from . import venue as venue_mod
from .sources import make_source


def collect(source_spec, obj, seconds):
    """-> list of (x, y, z) fixes of `obj` seen on the source within `seconds`."""
    pts = []
    with make_source(source_spec) as src:
        t0 = time.monotonic()
        print(f"[capture] listening on {source_spec} for {seconds:.0f} s, taking the {obj} ...")
        while time.monotonic() - t0 < seconds:
            for msg in src.poll():
                if msg.get(obj) is not None:
                    pts.append(tuple(float(v) for v in msg[obj]))
            time.sleep(0.005)
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="place name (e.g. judges, home, charger) or obstacle name with --obstacle")
    ap.add_argument("--xy", nargs=2, type=float, metavar=("X", "Y"), help="use these world coordinates instead of vision")
    ap.add_argument("--from", dest="obj", choices=["person", "balloon"], default="person", help="which fix to take (vision path)")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--source", default="udp", help="udp | udp:<port> | replay:<session>")
    ap.add_argument("--obstacle", action="store_true", help="store as a cylinder obstacle instead of a place")
    ap.add_argument("--r", type=float, default=0.45, help="obstacle radius, m (with --obstacle)")
    ap.add_argument("--venue", default=str(config.VENUE_FILE), help="venue json to update")
    ap.add_argument("--dry-run", action="store_true", help="show and validate, do not write")
    ap.add_argument("--force", action="store_true", help="save even when validate() reports errors")
    args = ap.parse_args()

    v = venue_mod.load(args.venue)
    if args.xy:
        x, y = args.xy
        print(f"[capture] {args.name}: manual ({x:.2f}, {y:.2f})")
    else:
        pts = collect(args.source, args.obj, args.seconds)
        if len(pts) < 5:
            sys.exit(f"[capture] only {len(pts)} fixes of the {args.obj} seen; is vision running and is the {args.obj} in view?")
        x, y = statistics.median(p[0] for p in pts), statistics.median(p[1] for p in pts)
        spread = max(math.hypot(p[0] - x, p[1] - y) for p in pts)
        print(f"[capture] {args.name}: {len(pts)} fixes, median ({x:.2f}, {y:.2f}), max spread {spread * 100:.0f} cm")
        if spread > 0.5:
            print("[capture] WARNING: spread > 50 cm; hold still / check for a second person in view")
    if args.obstacle:
        v.obstacles = [o for o in v.obstacles if o.name != args.name] + [venue_mod.Obstacle(x, y, args.r, args.name)]
    else:
        v.places[args.name] = (x, y)
    errors, warnings = venue_mod.validate(v, r_balloon=config.R_BALLOON)
    for w in warnings: print(f"[capture] WARNING {w}")
    for e in errors: print(f"[capture] ERROR   {e}")
    if errors and not args.force:
        sys.exit(f"[capture] not saved ({len(errors)} error(s); --force to save anyway)")
    if args.dry_run:
        print(f"[capture] dry run: would write {args.venue}"); return
    venue_mod.save(v, args.venue)
    print(f"[capture] wrote {args.venue}")


if __name__ == "__main__":
    main()
