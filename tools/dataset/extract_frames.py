"""Dump every Nth frame of recorded clips to JPEGs for labelling.
  python tools/dataset/extract_frames.py data/clip1 --every 15 --out datasets/balloon/images/train
"""
import argparse, glob, os
import cv2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("clip_dir"); ap.add_argument("--every", type=int, default=15); ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    n = 0
    for path in sorted(glob.glob(os.path.join(args.clip_dir, "*.mp4"))):
        cap, i = cv2.VideoCapture(path), 0
        tag = os.path.basename(os.path.normpath(args.clip_dir)) + "_" + os.path.splitext(os.path.basename(path))[0]
        while True:
            ok, f = cap.read()
            if not ok:
                break
            if i % args.every == 0:
                cv2.imwrite(os.path.join(args.out, f"{tag}_{i:06d}.jpg"), f); n += 1
            i += 1
    print(f"wrote {n} frames to {args.out}")


if __name__ == "__main__":
    main()
