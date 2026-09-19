"""Camera sources -> always-fresh latest frame (background thread, stale frames dropped).

Source spec strings (also in laptop/config.py SOURCES):
  "0", "1"                              local webcam index. macOS: an iPhone with Continuity Camera ON steals
                                        index 0 (and its camera, killing DroidCam). Turn Continuity Camera off
                                        on the phone (Settings > General > AirPlay & Continuity) so 0 = FaceTime.
  "http://172.20.10.3:4747/video"       DroidCam (Android or iOS) on a phone on the same hotspot. Find the IP with
                                        tools/find_phone.py. 404 on /video -> try /mjpegfeed?1280x720. Set the
                                        resolution IN THE APP before intrinsics.py: the calibration is tied to it.
                                        WiFi adds ~100-250 ms; pass that as lag_ms (localize --lag-b) so A/B skew is
                                        measured on the same clock.
  "udp://@:5000"                        rpicam-vid H.264 over UDP from Pi A  (Pi B -> :5001)
  "data/clip1/A.mp4"                    recorded file (loops forever)
"""
import os, threading, time
import cv2
from ..control.protocol import now_ms


def label(frame, text, org, scale=0.7, color=(255, 255, 255), thickness=2, pad=4):
    """putText that stays readable in any light: the text sits on a dark translucent box. org = bottom-left, like cv2."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (w, h), base = cv2.getTextSize(text, font, scale, thickness)
    x, y = int(org[0]), int(org[1])
    x1, y1, x2, y2 = max(x - pad, 0), max(y - h - pad, 0), min(x + w + pad, frame.shape[1]), min(y + base + pad, frame.shape[0])
    if x2 > x1 and y2 > y1:
        roi = frame[y1:y2, x1:x2]
        roi[:] = (roi * 0.25).astype(roi.dtype)          # darken the box, keep a hint of the picture
    cv2.putText(frame, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)

LOW_LATENCY = "fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0|fifo_size;1000000|overrun_nonfatal;1"
VIDEO_EXT = (".mp4", ".mkv", ".avi", ".mov")
RECONNECT_AFTER = 100   # consecutive failed reads (~1 s) before reopening a live source


def open_capture(spec):
    spec = str(spec)
    if spec.isdigit():
        cap = cv2.VideoCapture(int(spec), cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    else:
        if spec.startswith(("udp://", "rtsp://", "http://", "https://")):   # http = MJPEG phones: without this ffmpeg buffers ~0.5 s
            os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", LOW_LATENCY)
        cap = cv2.VideoCapture(spec, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"could not open video source {spec!r}")
    return cap


class Stream(threading.Thread):
    ROT = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}

    def __init__(self, spec, label="cam", lag_ms=0, rotate=0):
        """lag_ms: transport latency of this source; subtracted from every frame timestamp (frames are stamped on
        ARRIVAL, so a WiFi phone looks that much newer than it is).
        rotate: 0 / 90 / 180 / 270 degrees CLOCKWISE applied to every frame (a phone held in portrait streams
        sideways; YOLO wants people upright). Calibrate at the rotated size."""
        super().__init__(daemon=True)
        self.spec, self.label, self.lag_ms = str(spec), label, int(lag_ms)
        self.rot = self.ROT[int(rotate) % 360]
        self.cap = open_capture(spec)
        self.lock = threading.Lock()
        self.frame, self.t_ms = None, None
        self.fps, self._n, self._t0 = 0.0, 0, time.monotonic()
        self.alive = True
        self.start()

    def run(self):
        fails = 0
        while self.alive:
            ok, f = self.cap.read()
            if not ok:
                if self.spec.lower().endswith(VIDEO_EXT):
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                fails += 1
                if fails >= RECONNECT_AFTER:          # phone slept / WiFi dropped: reopen instead of spinning forever
                    print(f"[{self.label}] no frames from {self.spec!r}, reconnecting...")
                    self.cap.release()
                    try:
                        self.cap = open_capture(self.spec)
                    except RuntimeError:
                        time.sleep(1.0)
                    fails = 0
                else:
                    time.sleep(0.01)
                continue
            fails = 0
            t = now_ms() - self.lag_ms
            if self.rot is not None:
                f = cv2.rotate(f, self.rot)
            with self.lock:
                self.frame, self.t_ms = f, t
            self._n += 1
            if self._n % 30 == 0:
                now = time.monotonic(); self.fps = 30 / max(1e-6, now - self._t0); self._t0 = now

    def latest(self):
        """(frame, monotonic ms per protocol.now_ms()) of the newest frame, or (None, None)."""
        with self.lock:
            return self.frame, self.t_ms

    def wait_first(self, timeout=10):
        t0 = time.monotonic()
        while self.latest()[0] is None:
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(f"{self.label}: no frames from {self.spec!r} in {timeout}s")
            time.sleep(0.05)
        return self

    def stop(self):
        self.alive = False
        self.join(timeout=1)
        self.cap.release()
