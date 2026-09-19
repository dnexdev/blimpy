"""On-site balloon labelling in ~10 minutes: camera (or clip) -> model pre-draws the box -> you accept / fix / reject.

  python tools/dataset/label_site.py --source 0                  # live camera (also DroidCam URL, udp://@:5000)
  python tools/dataset/label_site.py --source data/clip1/A.mp4   # recorded clip
  python tools/dataset/label_site.py --source data/frames/       # folder of jpg/png

Writes data/site/images/*.jpg + data/site/labels/*.txt (YOLO, class 0 = balloon). Then:
  python -m laptop.vision.train_balloon --stage site             # ~3 min on the 5080 -> models/balloon.pt

Label the ENVELOPE only (the round part), not the gondola, fins or string: mono.py turns the box width into range,
so the box must be the sphere. Get ~50 frames: near/far, every corner of the frame, against the window, against the
wall, partly hidden behind a person, and ~10 frames with NO balloon (press n) so it learns your room's round things.

Keys:  SPACE / ENTER  accept the drawn box        n  no balloon in this frame (saved as a negative)
       mouse drag     draw/redraw the box          s  skip frame (nothing saved)       q  quit
       a              auto: accept every frame the model is >= --auto-conf sure about, 1 frame / s (live cameras)
"""
import argparse, glob, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import cv2

OUT = "data/site"


def frames_from(source):
    """Yield (frame, tag). Folder -> each image; .mp4 -> every --every-th frame; else a live Stream."""
    if os.path.isdir(source):
        for p in sorted(glob.glob(os.path.join(source, "*.jpg")) + glob.glob(os.path.join(source, "*.png"))):
            yield cv2.imread(p), os.path.splitext(os.path.basename(p))[0]
        return
    if source.lower().endswith((".mp4", ".mkv", ".avi", ".mov")):
        cap, i = cv2.VideoCapture(source), 0
        tag = os.path.splitext(os.path.basename(source))[0]
        while True:
            ok, f = cap.read()
            if not ok: return
            if i % ARGS.every == 0: yield f, f"{tag}_{i:06d}"
            i += 1
    from laptop.vision.streams import Stream
    s = Stream(source, "cam").wait_first(); last = None
    try:
        while True:
            f, t = s.latest()
            if t == last: time.sleep(0.01); continue
            last = t; yield f.copy(), f"cam{source.replace(':', '').replace('/', '')[:12]}_{int(time.time() * 10) % 10 ** 8:08d}"
    finally:
        s.stop()


class BoxEditor:
    def __init__(self):
        self.box, self.drag = None, None

    def on_mouse(self, ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN: self.drag = (x, y); self.box = (x, y, x, y)
        elif ev == cv2.EVENT_MOUSEMOVE and self.drag: self.box = (*self.drag, x, y)
        elif ev == cv2.EVENT_LBUTTONUP and self.drag:
            x0, y0 = self.drag; self.drag = None
            self.box = (min(x0, x), min(y0, y), max(x0, x), max(y0, y)) if abs(x - x0) > 4 and abs(y - y0) > 4 else None


def save(frame, tag, box):
    os.makedirs(f"{OUT}/images", exist_ok=True); os.makedirs(f"{OUT}/labels", exist_ok=True)
    cv2.imwrite(f"{OUT}/images/{tag}.jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    H, W = frame.shape[:2]
    with open(f"{OUT}/labels/{tag}.txt", "w") as f:
        if box:
            x1, y1, x2, y2 = box
            f.write(f"0 {(x1 + x2) / 2 / W:.6f} {(y1 + y2) / 2 / H:.6f} {(x2 - x1) / W:.6f} {(y2 - y1) / H:.6f}\n")


def main():
    global ARGS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", required=True); ap.add_argument("--every", type=int, default=10, help="clip: keep every Nth frame")
    ap.add_argument("--weights", default=None, help="model that pre-draws boxes (default: models/balloon.pt, else models/balloon_web.pt)")
    ap.add_argument("--conf", type=float, default=0.25); ap.add_argument("--auto-conf", type=float, default=0.7)
    ap.add_argument("--device", default=None)
    ARGS = args = ap.parse_args()
    weights = args.weights or next((w for w in ("models/balloon.pt", "models/balloon_web.pt") if os.path.exists(w)), None)
    det = None
    if weights:
        from ultralytics import YOLO
        det = YOLO(weights); print(f"[label] pre-labelling with {weights}")
    else:
        print("[label] no balloon model yet: draw every box by hand (or run train_balloon.py --stage web first)")
    ed = BoxEditor(); win = "label balloon  (SPACE accept, drag = redraw, n none, s skip, a auto, q quit)"
    cv2.namedWindow(win); cv2.setMouseCallback(win, ed.on_mouse)
    n_pos = n_neg = 0; auto = False; t_auto = 0.0
    for frame, tag in frames_from(args.source):
        pred = None
        if det is not None:
            r = det.predict(frame, conf=args.conf, verbose=False, device=args.device, quantize=16)[0]
            if r.boxes is not None and len(r.boxes):
                i = int(r.boxes.conf.argmax()); pred = (tuple(int(v) for v in r.boxes.xyxy[i].tolist()), float(r.boxes.conf[i]))
        ed.box = pred[0] if pred else None
        if auto:
            if pred and pred[1] >= args.auto_conf and time.time() - t_auto >= 1.0:
                save(frame, tag, ed.box); n_pos += 1; t_auto = time.time()
            vis = frame.copy()
            if ed.box: cv2.rectangle(vis, ed.box[:2], ed.box[2:], (0, 255, 0), 2)
            cv2.putText(vis, f"AUTO  saved {n_pos} (+{n_neg} neg)  [a] stop", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow(win, vis)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("a"): auto = False
            elif k == ord("q"): break
            continue
        while True:
            vis = frame.copy()
            if ed.box:
                cv2.rectangle(vis, ed.box[:2], ed.box[2:], (0, 255, 0), 2)
                if pred and ed.box == pred[0]: cv2.putText(vis, f"model {pred[1]:.2f}", (ed.box[0], max(15, ed.box[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(vis, f"saved {n_pos} balloon + {n_neg} empty   {tag}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow(win, vis)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord(" "), 13) and ed.box: save(frame, tag, ed.box); n_pos += 1; break
            if k == ord("n"): save(frame, tag, None); n_neg += 1; break
            if k == ord("s"): break
            if k == ord("a"): auto = True; t_auto = 0.0; break
            if k == ord("q"): cv2.destroyAllWindows(); print(f"[label] {n_pos} balloon frames, {n_neg} empty -> {OUT}"); return
    cv2.destroyAllWindows()
    print(f"[label] {n_pos} balloon frames, {n_neg} empty -> {OUT}")


if __name__ == "__main__":
    main()
