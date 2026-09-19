"""Extrinsic calibration: where is this camera? Automatic from the floor mat (laptop/vision/floor.py), 2 s once the
tags are in view. Re-run whenever a camera moves, or let mono/localize --auto-calib do it for you.

  python tools/calib/extrinsics.py --source 0 --name A             # --tag-size = config.TAG_SIZE_M (MEASURE the print)
  python tools/calib/extrinsics.py --source 0 --name A --spacing 0.98   # mat tags taped 0.98 m apart (MEASURE)

Mat = tags 0-3 flat on the floor at the corners of a square (config.MAT), all printed the same way up. Origin = centre
of tag 0, +X = printed left->right, +Y = printed bottom->top, +Z up. One tag alone still works but leaves the pitch
uncertain (~3 deg): fine at 1.8 m camera height, 40 cm range error from a table-top camera.
Saves calib/<name>_extrinsics.npz (R, t, tag_size, rms, n_tags) and prints the camera position: Z should match a tape.
"""
import argparse, time
import cv2, numpy as np
import _bootstrap  # noqa: F401
from laptop import config
from laptop.vision import floor
from laptop.vision.calib_io import Camera, make_tag_detector
from laptop.vision.streams import Stream


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", required=True); ap.add_argument("--name", required=True)
    ap.add_argument("--tag-size", type=float, default=config.TAG_SIZE_M)
    ap.add_argument("--spacing", type=float, nargs="+", default=None, help="m between mat tag centres (one value or +X +Y); default: calib/mat.json from survey_mat.py, else config.MAT")
    ap.add_argument("--seconds", type=float, default=2.0, help="sampling window")
    ap.add_argument("--calib", default=config.CALIB_DIR)
    ap.add_argument("--max-spread", type=float, default=0.03, help="m; camera position spread over the samples before we refuse")
    ap.add_argument("--max-reproj", type=float, default=3.0, help="px; samples worse than this are dropped")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--rot", type=int, default=0, help="degrees clockwise to rotate the stream first (config.ROTATE)")
    args = ap.parse_args()

    layout = floor.get_layout(args.spacing, args.calib)
    s = Stream(args.source, args.name, rotate=args.rot).wait_first(timeout=20)
    try:
        cam = Camera.load(args.name, args.calib, need_extrinsics=False)
    except FileNotFoundError:
        h, w = s.latest()[0].shape[:2]
        cam = Camera.nominal(args.name, (w, h), args.calib)
        print(f"[extrinsics] WARN: no intrinsics for {args.name}; wrote a nominal pinhole for {w}x{h} (chessboard it when there is time)")
    det = make_tag_detector()
    print(f"[extrinsics {args.name}] looking for mat tags {sorted(layout)} of {args.tag_size} m; layout from {floor.layout_source(args.spacing, args.calib)}")
    steady, t_last_fail = 0, 0.0
    try:
        while True:
            frame, _ = s.latest()
            ids, corners = floor.detect(det, frame, layout)
            steady = steady + 1 if ids else 0
            if steady >= 5 and time.monotonic() - t_last_fail > 1.0:
                cam2, info = floor.auto_extrinsics(cam, s, args.tag_size, layout, args.seconds, max_spread=args.max_spread,
                                                   max_reproj=args.max_reproj, calib_dir=args.calib, det=det)
                if cam2 is not None:
                    print(floor.describe(cam2, info)); print(f"saved {args.calib}/{args.name}_extrinsics.npz")
                    break
                print("  " + info["why"]); t_last_fail = time.monotonic(); steady = 0
            if args.no_show:
                time.sleep(0.03); continue
            for c in corners:
                cv2.aruco.drawDetectedMarkers(frame, [c.reshape(1, 4, 2)])
            msg = f"{len(ids)} mat tag(s) {ids}: sampling..." if ids else "no mat tag seen"
            cv2.putText(frame, f"{args.name}: {msg}   [q]=quit", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow(f"extrinsics {args.name}", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("aborted"); break
    finally:
        s.stop(); cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
