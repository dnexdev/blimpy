"""The eye on the balloon: the gondola camera (ESP32-CAM style, MJPEG over the hotspot) -> YOLO person -> where the person
is RELATIVE to the balloon: bearing (rad, + = to the left = the yaw the balloon needs), elevation, and range from the
person's height in the frame. No calibration, no heading estimate, no world frame: FOLLOW steers on this directly.

  python -m laptop.vision.fpv --source http://192.168.137.40:81/stream --show     # publishes on udp 5007 ("fpv" field)
  python -m laptop.vision.fpv --source 0 --show                                    # any camera, to try the geometry

In the pilot the same class runs in-process (`pilot --fpv SPEC`): the eye feeds behaviors.on_fpv() and is also what
Blimpy sees in conversation (OMNI), because an ESP32-CAM serves ONE stream client at a time.

Geometry (pinhole, HFOV from config.FPV): f = (W/2) / tan(HFOV/2) px.
  bearing = -atan((cx - W/2) / f)          person left of centre -> positive (counter-clockwise yaw brings it to centre)
  elev    = -atan((cy - H/2) / f)          above the optical axis -> positive
  range   = f * PERSON_H / box_h           only when the box is not cut by the top/bottom edge (else None = "close")
"""
import argparse, math, threading, time
from .. import config
from ..control.protocol import STATE_PORT, UdpJson, now_ms

F = config.FPV


def observe(box, w, h, hfov_deg=None, person_h=None, conf=1.0, edge_px=3):
    """Pure geometry: YOLO box (x1, y1, x2, y2) in a w x h frame -> observation dict. Tests drive this with fake boxes."""
    hfov = math.radians(F["HFOV_DEG"] if hfov_deg is None else hfov_deg)
    person_h = F["PERSON_H"] if person_h is None else person_h
    x1, y1, x2, y2 = box
    f = (w / 2.0) / math.tan(hfov / 2.0)
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    bearing = -math.atan((cx - w / 2.0) / f)
    elev = -math.atan((cy - h / 2.0) / f)
    cut = y1 <= edge_px or y2 >= h - edge_px
    bh = max(1.0, y2 - y1)
    rng = None if cut else person_h * f / bh
    return {"bearing": round(bearing, 4), "elev": round(elev, 4), "range": None if rng is None else round(rng, 3),
            "conf": round(float(conf), 3), "box": [round(x1 / w, 4), round(y1 / h, 4), round(x2 / w, 4), round(y2 / h, 4)]}


def pick_person(people, w, h):
    """The person to follow: the largest box (nearest), ties broken toward the centre."""
    if not people: return None
    def key(p):
        x1, y1, x2, y2 = p["box"]
        return (x2 - x1) * (y2 - y1) - 0.05 * w * h * abs((x1 + x2) / 2 - w / 2) / w
    return max(people, key=key)


def count_facing(people, w, h, bearing_max=None, range_max=None):
    """How many of the detected people are near and centred (the same test the pilot uses for "said to its face").
    One = somebody is talking to Blimpy's face; several = a group in view, nobody in particular."""
    O = config.OMNI
    bearing_max = O["PRESENCE_BEARING_RAD"] if bearing_max is None else bearing_max
    range_max = O["PRESENCE_RANGE_M"] if range_max is None else range_max
    n = 0
    for p in people:
        o = observe(p["box"], w, h)
        n += abs(o["bearing"]) < bearing_max and (o["range"] is None or o["range"] < range_max)
    return n


class FpvEye(threading.Thread):
    """Stream + detector on a thread at ~HZ; latest() -> (obs or None, frame ms) and frame() -> the newest frame."""

    def __init__(self, spec, lag_ms=None, hz=None, weights=None, device=None, publish=False, label="eye"):
        super().__init__(daemon=True, name="fpv-eye")
        from .streams import Stream
        from .detect import PersonTracker
        self.stream = Stream(spec, label, lag_ms=F["LAG_MS"] if lag_ms is None else lag_ms).wait_first(10)
        self.det = PersonTracker(weights or F["WEIGHTS"] or config.YOLO_PERSON, device=device)
        self.hz = hz or F["HZ"]
        self.out = UdpJson() if publish else None
        self.obs, self.t_obs, self.n, self.n_seen = None, None, 0, 0
        self.last_people = []
        self.alive = True
        self.start()

    def run(self):
        period = 1.0 / self.hz
        while self.alive:
            t0 = time.monotonic()
            frame, t_ms = self.stream.latest()
            if frame is not None and t_ms != self.t_obs:
                h, w = frame.shape[:2]
                try: people = self.det.detect(frame)
                except Exception as e:
                    people = []; print(f"[eye] detect: {e}")
                self.last_people = people
                p = pick_person(people, w, h)
                self.obs = observe(p["box"], w, h, conf=p.get("conf", 1.0)) if p else None
                if self.obs: self.obs["people"] = len(people); self.obs["facing"] = count_facing(people, w, h)
                self.t_obs = t_ms; self.n += 1; self.n_seen += 1 if p else 0
                if self.out is not None:
                    self.out.send({"t": t_ms, "balloon": None, "person": None, "fpv": self.obs, "src": "fpv"}, ("127.0.0.1", STATE_PORT))
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def latest(self):
        return self.obs, self.t_obs

    def frame(self):
        return self.stream.latest()[0]

    def stop(self):
        self.alive = False
        self.stream.stop()


if __name__ == "__main__":
    import cv2
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default=F["SOURCE"] or "0"); ap.add_argument("--show", action="store_true")
    ap.add_argument("--lag-ms", type=int, default=None); ap.add_argument("--seconds", type=float, default=1e9)
    a = ap.parse_args()
    eye = FpvEye(a.source, lag_ms=a.lag_ms, publish=True)
    print(f"[eye] {a.source} -> udp {STATE_PORT} (fpv). HFOV {F['HFOV_DEG']} deg, person {F['PERSON_H']} m. Ctrl+C to quit")
    t0 = time.monotonic(); last = None
    try:
        while time.monotonic() - t0 < a.seconds:
            obs, t = eye.latest()
            if t != last:
                last = t
                if obs: print(f"[eye] bearing {math.degrees(obs['bearing']):+5.1f} deg  elev {math.degrees(obs['elev']):+5.1f}  "
                              f"range {obs['range'] if obs['range'] is None else '%.2f m' % obs['range']}  conf {obs['conf']:.2f}   ", end="\r")
                else: print("[eye] no person" + " " * 60, end="\r")
            if a.show:
                fr = eye.frame()
                if fr is not None:
                    fr = fr.copy(); h, w = fr.shape[:2]
                    for p in eye.last_people:
                        x1, y1, x2, y2 = map(int, p["box"]); cv2.rectangle(fr, (x1, y1), (x2, y2), (0, 200, 0), 2)
                    if obs:
                        from .streams import label
                        label(fr, f"{math.degrees(obs['bearing']):+.0f} deg  {obs['range'] and '%.1f m' % obs['range'] or 'close'}",
                              (10, 30), 0.8, (0, 255, 255))
                    cv2.line(fr, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
                    cv2.imshow("eye", fr)
                    if cv2.waitKey(1) & 0xFF == ord("q"): break
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    eye.stop(); print(f"\n[eye] {eye.n} frames, person seen in {eye.n_seen}")
