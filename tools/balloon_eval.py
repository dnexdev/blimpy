"""Score a balloon detector the way the demo uses it: ONE big balloon per frame, nothing else round allowed.

  python tools/balloon_eval.py                                    # models/balloon.pt on datasets/balloon val
  python tools/balloon_eval.py --weights models/balloon_web_m.pt --sheet sheet.jpg
  python tools/balloon_eval.py --images data/site/images --labels data/site/labels    # your own frames

Reports (much more relevant than crowded-scene mAP):
  big-balloon recall   GT boxes wider than --big (12 %) of the image found with IoU > 0.5 at --conf
  negative FP rate     images with no balloon that still produced a detection
  top-1 hit rate       frames where the LARGEST box over --conf IS a big balloon (detect.py picks the largest, not the most confident)
--sheet writes a contact sheet (magenta = prediction + conf, green = ground truth) to eyeball the failures.
"""
import argparse, glob, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2, numpy as np


def iou(a, b):
    ix, iy = max(0, min(a[2], b[2]) - max(a[0], b[0])), max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    return inter / max((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter, 1e-6)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", default="models/balloon.pt"); ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--images", default="datasets/balloon/images/val"); ap.add_argument("--labels", default="datasets/balloon/labels/val")
    ap.add_argument("--big", type=float, default=0.12); ap.add_argument("--sheet", default=None)
    ap.add_argument("--device", default=None)
    args = ap.parse_args()
    from ultralytics import YOLO
    m = YOLO(args.weights)
    big = found = top1_n = top1_ok = n_neg = fp_neg = 0; tiles = []
    for p in sorted(glob.glob(os.path.join(args.images, "*"))):
        im = cv2.imread(p)
        if im is None: continue
        H, W = im.shape[:2]
        lab = os.path.join(args.labels, os.path.splitext(os.path.basename(p))[0] + ".txt")
        gts = []
        if os.path.exists(lab):
            for l in open(lab):
                if not l.strip(): continue
                _, cx, cy, w, h = map(float, l.split())
                gts.append(((cx - w / 2) * W, (cy - h / 2) * H, (cx + w / 2) * W, (cy + h / 2) * H, w))
        r = m.predict(im, conf=args.conf, verbose=False, device=args.device, quantize=16)[0]
        dets = r.boxes.xyxy.cpu().numpy() if r.boxes is not None and len(r.boxes) else np.zeros((0, 4))
        confs = r.boxes.conf.cpu().numpy() if r.boxes is not None and len(r.boxes) else np.zeros(0)
        bigs = [g for g in gts if g[4] >= args.big]
        if not gts:
            n_neg += 1; fp_neg += int(len(dets) > 0)
        for g in bigs:
            big += 1; found += any(iou(d, g) > 0.5 for d in dets)
        if bigs:
            top1_n += 1
            if len(dets):
                areas = (dets[:, 2] - dets[:, 0]) * (dets[:, 3] - dets[:, 1])      # same rule as detect.py:125-128
                top = dets[int(areas.argmax())]; top1_ok += any(iou(top, g) > 0.5 for g in bigs)
        wrong = (not gts and len(dets)) or (bigs and not all(any(iou(d, g) > 0.5 for d in dets) for g in bigs))
        if args.sheet and len(tiles) < 24 and wrong:
            vis = im.copy()
            for d, c in zip(dets, confs):
                cv2.rectangle(vis, (int(d[0]), int(d[1])), (int(d[2]), int(d[3])), (255, 0, 255), 3)
                cv2.putText(vis, f"{c:.2f}", (int(d[0]), max(20, int(d[1]) - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
            for g in gts:
                cv2.rectangle(vis, (int(g[0]), int(g[1])), (int(g[2]), int(g[3])), (0, 255, 0), 2)
            s = 320 / max(H, W); vis = cv2.resize(vis, (int(W * s), int(H * s)))
            c = np.zeros((320, 320, 3), np.uint8); c[:vis.shape[0], :vis.shape[1]] = vis; tiles.append(c)
    print(f"[{args.weights}] conf {args.conf}")
    print(f"  big-balloon recall : {found}/{big} = {found / max(big, 1):.2f}   (GT wider than {args.big:.0%} of the image)")
    print(f"  top-1 hit rate     : {top1_ok}/{top1_n} = {top1_ok / max(top1_n, 1):.2f}   (largest box over --conf is a big balloon)")
    print(f"  negative FP rate   : {fp_neg}/{n_neg} = {fp_neg / max(n_neg, 1):.2f}   (balloon-free images with a detection)")
    if args.sheet and tiles:
        while len(tiles) % 4: tiles.append(np.zeros((320, 320, 3), np.uint8))
        rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
        cv2.imwrite(args.sheet, np.vstack(rows)); print(f"  failures sheet     : {args.sheet}")


if __name__ == "__main__":
    main()
