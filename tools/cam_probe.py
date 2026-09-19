"""Which webcam index is which? Opens indices 0..N, grabs a frame from each, prints size, shows them tiled.

  python tools/cam_probe.py               # window with one tile per working index (q closes)
  python tools/cam_probe.py --no-show     # print only
  python tools/cam_probe.py --max 5

macOS: an iPhone with Continuity Camera ON takes index 0 while it is nearby, pushing the FaceTime cam to 1;
the order can flip when the phone leaves. Run this before intrinsics/extrinsics and use the index you SEE.
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2, numpy as np
from laptop.vision.streams import open_capture


def probe(i, warm=8):
    """(frame, fps estimate) from index i, or (None, 0) if it cannot be opened. warm frames let auto-exposure settle."""
    try:
        cap = open_capture(str(i))
    except RuntimeError:
        return None, 0.0
    frame, t0, n = None, time.monotonic(), 0
    for _ in range(warm):
        ok, f = cap.read()
        if ok:
            frame, n = f, n + 1
    cap.release()
    return frame, n / max(1e-6, time.monotonic() - t0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max", type=int, default=3, help="highest index to try")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()
    tiles = []
    for i in range(args.max + 1):
        frame, fps = probe(i)
        if frame is None:
            print(f"index {i}: cannot open"); continue
        h, w = frame.shape[:2]
        print(f"index {i}: {w}x{h}  ~{fps:.0f} fps")
        tile = cv2.resize(frame, (480, int(480 * h / w)))
        cv2.putText(tile, f"index {i}  {w}x{h}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        tiles.append(tile)
    if not tiles:
        print("no camera opened: is another app (Zoom, FaceTime, DroidCam client) holding them?"); sys.exit(1)
    if args.no_show:
        return
    hmax = max(t.shape[0] for t in tiles)
    row = np.hstack([cv2.copyMakeBorder(t, 0, hmax - t.shape[0], 0, 0, cv2.BORDER_CONSTANT) for t in tiles])
    print("q closes the window")
    cv2.imshow("cam_probe: which index is which?", row)
    while cv2.waitKey(50) & 0xFF not in (ord("q"), 27):
        if cv2.getWindowProperty("cam_probe: which index is which?", cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
