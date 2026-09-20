"""Detectors. Person = YOLO. Balloon, in order of preference:
  1. fine-tuned YOLO weights (models/balloon.pt, from train_balloon.py) when the file exists
  2. colour blob (ColorBalloonDetector) when a balloon colour is configured: HSV threshold -> largest round blob.
     No training data needed; works for a single plain-colour balloon in a normal room.
  3. COCO 'sports ball' class as a weak stand-in.

One PersonTracker PER CAMERA: ultralytics track(persist=True) keeps state inside the model object.
"""
import os
import cv2, numpy as np

# HSV ranges (OpenCV: H 0-179, S 0-255, V 0-255). Red wraps around, hence two ranges.
BALLOON_HSV = {
    "white":  [((0, 0, 170), (179, 60, 255))],
    "red":    [((0, 120, 80), (8, 255, 255)), ((170, 120, 80), (179, 255, 255))],
    "orange": [((8, 120, 100), (22, 255, 255))],
    "yellow": [((22, 100, 120), (38, 255, 255))],
    "green":  [((40, 80, 60), (85, 255, 255))],
    "blue":   [((95, 100, 60), (130, 255, 255))],
    "pink":   [((140, 60, 120), (175, 255, 255))],
}


class ColorBalloonDetector:
    """Largest roundish blob of the balloon colour. Rejects blobs that touch the image border (cut off, or a wall),
    thin/elongated ones (paper, cables) and tiny ones. Returns {box, conf, pt, diam_px} or None."""

    def __init__(self, color="white", min_px=15, min_circularity=0.75, min_fill=0.75, max_aspect=1.5):
        self.ranges = BALLOON_HSV[color]
        self.min_px, self.min_circ, self.min_fill, self.max_aspect = min_px, min_circularity, min_fill, max_aspect
        self.mode = f"colour blob ({color})"

    def mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m = None
        for lo, hi in self.ranges:
            part = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            m = part if m is None else cv2.bitwise_or(m, part)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)

    def detect(self, frame):
        h, w = frame.shape[:2]
        contours, _ = cv2.findContours(self.mask(frame), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_px ** 2:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
                continue                                    # touches the border
            if max(bw, bh) / max(1, min(bw, bh)) > self.max_aspect:
                continue
            per = cv2.arcLength(c, True)
            circ = 4 * np.pi * area / (per * per) if per > 0 else 0        # 1.0 circle, 0.785 square, less for rectangles
            (_, _), r = cv2.minEnclosingCircle(c)
            fill = area / (np.pi * r * r)                                   # 1.0 circle, 0.64 square
            if circ < self.min_circ or fill < self.min_fill:
                continue
            if best is None or area > best[0]:
                best = (area, x, y, bw, bh, circ)
        if best is None:
            return None
        area, x, y, bw, bh, circ = best
        return {"box": (float(x), float(y), float(x + bw), float(y + bh)), "conf": float(min(1.0, circ)),
                "pt": (x + bw / 2, y + bh / 2), "diam_px": float(max(bw, bh))}


def _use_half(device):
    """fp16 halves YOLO time on the GPU; CPU runs must stay fp32."""
    if device in ("cpu",): return False
    try:
        import torch; return torch.cuda.is_available()
    except Exception: return False


class PersonTracker:
    def __init__(self, weights="yolo11n.pt", conf=0.4, device=None):
        from ultralytics import YOLO   # lazy: slow import, and ColorBalloonDetector must work without it
        self.model, self.conf, self.device, self.half = YOLO(weights), conf, device, _use_half(device)

    EDGE_PX = 4   # a box touching the frame edge within this is cut off there

    def detect(self, frame):
        """List of {id, box, conf, pt, feet, head, cut, area}, largest box first.
        pt = (u,v) 35 % down the box (chest-ish); feet = bottom-centre; head = top-centre; cut = {"top", "bottom"}:
        that edge of the box is the frame edge, so the body continues out of view and that point is NOT a real
        body point. localize.py triangulates feet or heads (real physical points from any viewpoint) when both
        cameras have them uncut, and falls back to pt."""
        r = self.model.track(frame, persist=True, classes=[0], conf=self.conf, verbose=False, device=self.device, quantize=16 if self.half else None)[0]
        out = []
        if r.boxes is None or len(r.boxes) == 0:
            return out
        h, w = frame.shape[:2]
        ids = r.boxes.id.int().tolist() if r.boxes.id is not None else [-1] * len(r.boxes)
        for (x1, y1, x2, y2), c, i in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist(), ids):
            cx = (x1 + x2) / 2
            out.append({"id": i, "box": (x1, y1, x2, y2), "conf": c, "pt": (cx, y1 + 0.35 * (y2 - y1)),
                        "feet": (cx, y2), "head": (cx, y1), "cut": {"top": y1 <= self.EDGE_PX, "bottom": y2 >= h - self.EDGE_PX,
                                                                        "left": x1 <= self.EDGE_PX, "right": x2 >= w - self.EDGE_PX},
                        "area": (x2 - x1) * (y2 - y1)})
        out.sort(key=lambda d: -d["area"])
        return out


class BalloonDetector:
    """Best balloon in the frame as {box, conf, pt, diam_px} or None. The box is the ENVELOPE (the sphere): mono.py
    turns its width into range, so train_balloon.py / label_site.py label the envelope only, not gondola or fins."""

    def __init__(self, weights="models/balloon.pt", conf=0.3, device=None, fallback="yolo11n.pt", color=None, imgsz=640):
        self.conf, self.device, self.blob, self.imgsz, self.half = conf, device, None, imgsz, _use_half(device)
        if not os.path.exists(weights) and os.path.exists("models/balloon_web.pt"):
            weights = "models/balloon_web.pt"        # the committed web-pretrained model; balloon.pt = site fine-tune
        if os.path.exists(weights) or not color:
            from ultralytics import YOLO
        if os.path.exists(weights):
            self.model, self.classes, self.mode = YOLO(weights), None, f"fine-tuned ({weights})"
        elif color:
            self.blob = ColorBalloonDetector(color); self.mode = self.blob.mode
        else:
            print(f"[balloon] {weights} not found and no balloon colour set -> COCO 'sports ball' stand-in. "
                  f"Set config.BALLOON_COLOR or train with: python -m laptop.vision.train_balloon")
            self.model, self.classes, self.mode = YOLO(fallback), [32], "coco sports-ball stand-in"

    def detect(self, frame):
        """Best balloon as {box, conf, pt} or None."""
        if self.blob is not None:
            return self.blob.detect(frame)
        r = self.model.predict(frame, classes=self.classes, conf=self.conf, verbose=False, device=self.device,
                               quantize=16 if self.half else None, imgsz=self.imgsz)[0]
        if r.boxes is None or len(r.boxes) == 0:
            return None
        # Our balloon (1.1 m) is the biggest round thing in the room: among boxes over the threshold take the LARGEST,
        # not the most confident. A coffee cup at 0.85 must not beat the balloon at 0.6. (Cluster scenes are not our case.)
        xyxy = r.boxes.xyxy.tolist(); confs = r.boxes.conf.tolist()
        i = max(range(len(xyxy)), key=lambda k: ((xyxy[k][2] - xyxy[k][0]) * (xyxy[k][3] - xyxy[k][1]), confs[k]))
        x1, y1, x2, y2 = xyxy[i]
        return {"box": (x1, y1, x2, y2), "conf": float(confs[i]), "pt": ((x1 + x2) / 2, (y1 + y2) / 2),
                "diam_px": float(max(x2 - x1, y2 - y1))}
