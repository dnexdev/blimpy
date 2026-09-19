"""Record both cameras to disk (offline dev data + YOLO dataset material).

  python tools/calib/record_clips.py --a 0 --b 1 --out data/clip1 --seconds 90
Writes <out>/A.mp4, <out>/B.mp4 and <out>/timestamps.csv (frame index, ms for A and B).
Replay later by using the .mp4 paths as sources for any script.
"""
import argparse, os, time
import cv2
import _bootstrap  # noqa: F401
from laptop import config
from laptop.vision.streams import Stream


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", default=config.SOURCES["A"]); ap.add_argument("--b", default=config.SOURCES["B"])
    ap.add_argument("--out", required=True); ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--fps", type=float, default=15)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    streams = {"A": Stream(args.a, "A").wait_first(), "B": Stream(args.b, "B").wait_first()}
    writers = {}
    for n, s in streams.items():
        h, w = s.latest()[0].shape[:2]
        writers[n] = cv2.VideoWriter(os.path.join(args.out, f"{n}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
    csv = open(os.path.join(args.out, "timestamps.csv"), "w"); csv.write("i,tA_ms,tB_ms\n")
    t0, i = time.monotonic(), 0
    print(f"recording {args.seconds:.0f}s to {args.out} ... (q to stop early)")
    while time.monotonic() - t0 < args.seconds:
        t = time.monotonic()
        fA, tA = streams["A"].latest(); fB, tB = streams["B"].latest()
        writers["A"].write(fA); writers["B"].write(fB); csv.write(f"{i},{tA},{tB}\n"); i += 1
        cv2.imshow("rec A", cv2.resize(fA, None, fx=0.4, fy=0.4)); cv2.imshow("rec B", cv2.resize(fB, None, fx=0.4, fy=0.4))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        time.sleep(max(0.0, 1 / args.fps - (time.monotonic() - t)))
    for w in writers.values(): w.release()
    for s in streams.values(): s.stop()
    csv.close(); cv2.destroyAllWindows()
    print(f"saved {i} frames per camera")


if __name__ == "__main__":
    main()
