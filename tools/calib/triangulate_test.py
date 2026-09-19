"""Validate the whole chain (2 cams + intrinsics + extrinsics -> 3D) before trusting it.

  python tools/calib/triangulate_test.py --a 0 --b 1

TAG check (automatic): when both cameras see the floor tag its 4 corners are triangulated and compared
  with the truth: edges == tag size (within ~1 cm), z == 0, corners at (+-S/2, +-S/2).
CLICK check: left-click the same physical point in window A, then in window B -> prints XYZ.
  Tape-measure test: hold a bright object at a known spot (e.g. 1.00 m along +X from the tag, 1.50 m high).
'q' quits.
"""
import argparse
import cv2, numpy as np
import _bootstrap  # noqa: F401
from laptop import config
from laptop.vision.calib_io import Camera, make_tag_detector, tag_object_points
from laptop.vision.streams import Stream
from laptop.vision.triangulate import triangulate


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", default=config.SOURCES["A"]); ap.add_argument("--b", default=config.SOURCES["B"])
    ap.add_argument("--tag-size", type=float, default=config.TAG_SIZE_M); ap.add_argument("--calib", default=config.CALIB_DIR)
    args = ap.parse_args()

    cams = {"A": Camera.load("A", args.calib), "B": Camera.load("B", args.calib)}
    for n, c in cams.items():
        print(f"[cam {n}] at {np.round(c.position(), 2)} m")
    streams = {"A": Stream(args.a, "A").wait_first(), "B": Stream(args.b, "B").wait_first()}
    det = make_tag_detector()
    clicks = {"A": None, "B": None}

    def on_click(n):
        def cb(ev, x, y, *_):
            if ev == cv2.EVENT_LBUTTONDOWN:
                clicks[n] = (x, y); print(f"  click {n}: {clicks[n]}")
        return cb
    for n in ("A", "B"):
        cv2.namedWindow(f"tri {n}"); cv2.setMouseCallback(f"tri {n}", on_click(n))

    S = args.tag_size
    truth = tag_object_points(S)
    frame_n = 0
    while True:
        frames = {n: streams[n].latest()[0] for n in ("A", "B")}
        seen = {}
        for n, f in frames.items():
            corners, ids, _ = det.detectMarkers(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
            if ids is not None and 0 in ids.ravel():
                k = list(ids.ravel()).index(0)
                seen[n] = corners[k][0]
                cv2.aruco.drawDetectedMarkers(f, [corners[k]])
        frame_n += 1
        if len(seen) == 2 and frame_n % 15 == 0:
            pts = [triangulate(cams["A"], cams["B"], seen["A"][i], seen["B"][i])[0] for i in range(4)]
            edges = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
            errs = [np.linalg.norm(pts[i] - truth[i]) for i in range(4)]
            print(f"[tag] edges {np.round(edges, 3)} m (truth {S})  z {np.round([p[2] for p in pts], 3)}  "
                  f"corner err {np.round(errs, 3)} m -> {'GOOD' if max(errs) < 0.02 else 'CHECK CALIBRATION'}")
        if clicks["A"] and clicks["B"]:
            X, err = triangulate(cams["A"], cams["B"], clicks["A"], clicks["B"])
            print(f"[click] world X={X[0]:+.3f} Y={X[1]:+.3f} Z={X[2]:+.3f} m   reproj err {err:.1f} px")
            clicks["A"] = clicks["B"] = None
        for n, f in frames.items():
            if clicks[n]:
                cv2.circle(f, clicks[n], 8, (0, 0, 255), 2)
            cv2.putText(f, f"{n}: click a point here {'(then B)' if n == 'A' else '(after A)'}  [q]=quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow(f"tri {n}", f)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    for s in streams.values(): s.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
