"""Survey the floor mat ONCE after taping it: where is each page really, how twisted, how big. Any camera that sees
every tag will do. Saves calib/mat.json; mono / localize / extrinsics / vision_check use it from then on.

  python tools/calib/survey_mat.py --source 0 --name iphone
  python tools/calib/survey_mat.py --source 0 --name iphone --ref 0 --seconds 2

Pages can be taped by eye: the fit (laptop/vision/floor.py survey) recovers their positions relative to the
reference page (default tag 0 = world origin) from the image itself; the printed size of tag 0 (config.TAG_SIZE_M,
MEASURED) sets the scale. Move a page -> survey again. Prints the layout, the residual and the camera position.
"""
import argparse, statistics, time
import cv2, numpy as np
import _bootstrap  # noqa: F401
from laptop import config
from laptop.vision import floor
from laptop.vision.calib_io import Camera, make_tag_detector
from laptop.vision.streams import Stream


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", required=True); ap.add_argument("--name", required=True, help="intrinsics to use (calib/<name>_intrinsics.npz)")
    ap.add_argument("--tag-size", type=float, default=config.TAG_SIZE_M, help="m, printed size of the REFERENCE tag")
    ap.add_argument("--ref", type=int, default=0, help="reference tag id = world origin")
    ap.add_argument("--ids", type=int, nargs="+", default=sorted(config.MAT), help="tag ids that make up the mat")
    ap.add_argument("--seconds", type=float, default=2.0); ap.add_argument("--calib", default=config.CALIB_DIR)
    ap.add_argument("--max-reproj", type=float, default=1.5, help="px; refuse to save a worse fit")
    ap.add_argument("--rot", type=int, default=0, help="degrees clockwise to rotate the stream first (config.ROTATE)")
    ap.add_argument("--fit-f", action="store_true", help="also fit this camera's focal length from the mat (needs >= 3 non-reference pages of ONE print) and save it as its intrinsics")
    ap.add_argument("--intrinsics-only", action="store_true", help="with --fit-f: save the intrinsics, leave calib/mat.json alone (second camera)")
    ap.add_argument("--anchor", choices=("ref", "others"), default="ref",
                    help="which page(s) are exactly --tag-size: the reference (default) or the median of the OTHER pages (use when tag 0 is the odd print)")
    args = ap.parse_args()

    det = make_tag_detector()
    s = Stream(args.source, args.name, rotate=args.rot).wait_first(timeout=20)
    h, w = s.latest()[0].shape[:2]
    try:
        cam = Camera.load(args.name, args.calib, need_extrinsics=False)
        if tuple(cam.size) != (w, h):
            print(f"[survey] intrinsics for {args.name} are {cam.size[0]}x{cam.size[1]} but the stream is {w}x{h} (rotation?): using a nominal pinhole instead")
            cam = Camera.nominal(args.name, (w, h))
    except FileNotFoundError:
        cam = Camera.nominal(args.name, (w, h), None if args.fit_f else args.calib)
        print(f"[survey] WARN: no intrinsics for {args.name}; nominal pinhole for {w}x{h}" + ("" if args.fit_f else " saved (chessboard it when there is time)"))
    if args.fit_f:
        t0 = time.monotonic(); fits_f = []
        while time.monotonic() - t0 < args.seconds and len(fits_f) < 6:
            frame, _ = s.latest()
            ids, corners = floor.detect(det, frame, {i: (0.0, 0.0) for i in args.ids})
            if args.ref in ids and len(ids) >= 3:
                fits_f.append(floor.fit_f(cam, ids, corners, args.tag_size, args.ref))
            time.sleep(0.15)
        if not fits_f:
            print("[fit-f] need the reference page plus at least two more in view"); s.stop(); raise SystemExit(1)
        fpx = statistics.median(r[0] for r in fits_f); spread = statistics.median(r[1] for r in fits_f)
        cam = Camera.nominal(args.name, (w, h), args.calib, f_px=fpx)
        print(f"[fit-f] {args.name}: f = {fpx:.0f} px ({fpx / max(w, h):.2f} x long side) from {len(fits_f)} frames, page-size spread {spread * 100:.1f} % "
              f"-> saved {args.calib}/{args.name}_intrinsics.npz (nominal was {0.85 * max(w, h):.0f})")
        if args.intrinsics_only:
            s.stop(); return
    want = {i: (0.0, 0.0) for i in args.ids}
    fits, seen_sets, last = [], [], None
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        frame, t = s.latest()
        if t == last:
            time.sleep(0.01); continue
        last = t
        ids, corners = floor.detect(det, frame, want)
        seen_sets.append(tuple(ids))
        if args.ref not in ids or len(ids) < 2:
            continue
        layout, rvec, tvec, err = floor.survey(cam, ids, corners, args.tag_size, args.ref)
        fits.append((layout, rvec, tvec, err))
    s.stop()
    if not fits:
        print(f"no frame had the reference tag {args.ref} plus at least one more mat tag (seen: {set(seen_sets)}). Aim the camera at the mat.")
        raise SystemExit(1)
    ids_all = sorted(set(i for f in fits for i in f[0]))
    layout = {}
    for i in ids_all:
        rows = [f[0][i] for f in fits if i in f[0]]
        layout[i] = tuple(float(statistics.median(r[k] for r in rows)) for k in range(4))
    err = statistics.median(f[3] for f in fits)
    if args.anchor == "others" and len(layout) > 1:
        k = args.tag_size / statistics.median(e[3] for i, e in layout.items() if i != args.ref)
        layout = {i: (e[0] * k, e[1] * k, e[2], e[3] * k) for i, e in layout.items()}
        print(f"[survey] scale anchored on the other pages (= {args.tag_size * 1000:.0f} mm): everything x{k:.3f}; the reference page measures {layout[args.ref][3] * 1000:.0f} mm")
    missing = [i for i in args.ids if i not in layout]
    print(f"[survey] {len(fits)} frames, tags {ids_all}, residual {err:.2f} px" + (f", NOT seen: {missing}" if missing else ""))
    print(floor.describe_layout(layout, args.tag_size))
    R, _ = cv2.Rodrigues(fits[-1][1]); pos = (-R.T @ fits[-1][2].reshape(3, 1)).ravel()
    if args.anchor == "others" and len(layout) > 1:
        pos = pos * k
    print(f"  camera {args.name} at X={pos[0]:+.2f} Y={pos[1]:+.2f} Z={pos[2]:+.2f} m during the survey")
    sizes = [e[3] for i, e in layout.items() if i != args.ref]
    if sizes and args.anchor == "ref" and abs(statistics.median(sizes) - args.tag_size) > 0.01:
        print(f"  NOTE: other pages measure {min(sizes) * 1000:.0f}-{max(sizes) * 1000:.0f} mm vs the reference {args.tag_size * 1000:.0f} mm: printed at a different "
              f"scale? Sizes are saved per page, but the WORLD SCALE follows the reference: if the others are the trusted print, re-run with --anchor others")
    if err > args.max_reproj:
        print(f"  residual {err:.2f} px > {args.max_reproj}: pages not flat, intrinsics wrong, or a tag mis-detected. Not saved.")
        raise SystemExit(1)
    p = floor.save_layout(args.calib, layout, args.tag_size, ref=args.ref, reproj=err, surveyed_by=args.name, frames=len(fits))
    print("saved", p, "- every camera's auto-calib / extrinsics uses it from now on")


if __name__ == "__main__":
    main()
