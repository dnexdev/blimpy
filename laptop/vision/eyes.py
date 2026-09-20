"""The room camera's picture, shared: ONE process owns the webcam (mono.py), everybody else asks it over localhost.

Why: Windows gives a webcam to one process, and Blimpy's eyes in conversation ARE the room camera on a build with no
camera on the balloon. mono keeps the newest frame as one JPEG (encoded once per processed frame, never per request)
together with what it knows about that exact frame (people, boxes, the camera matrix), so picture and labels cannot
drift apart. A UDP datagram cannot carry a JPEG on macOS (9216 bytes) and swapping a file fails on Windows while a
reader has it open, hence a tiny stdlib HTTP server bound to 127.0.0.1 (no firewall prompt).

  GET /frame.jpg[?target=P2]   image/jpeg + header X-Blimpy-Meta: {"t","run","lost","who","people","balloon_box","P","cam","size","nominal"}
                               204 when there is no frame yet. `target` tells mono who "the person" of the single-person
                               message should be; it is re-sent with every poll, so a restarted mono needs no handshake,
                               and it lapses TARGET_HOLD_S after the last poll (the pilot went away: back to the default).
  GET /scene.json              the meta alone

  server = EyesServer(port).start();  server.publish(jpg_bytes, meta);  server.target  -> "P2" | None
  eyes = RoomEyes().start();  frame, meta = eyes.latest();  eyes.target = "P2"
"""
import json, threading, time, urllib.parse, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ..control.protocol import ROOM_HTTP_PORT, now_ms

_DIRECT = urllib.request.build_opener(urllib.request.ProxyHandler({}))      # 127.0.0.1 must never go through a system / env http proxy

TARGET_HOLD_S = 3.0


class EyesServer:
    def __init__(self, port=ROOM_HTTP_PORT):
        self.port, self._lock = port, threading.Lock()
        self._jpg, self._meta, self._target, self._t_target = None, {}, None, 0.0
        self.httpd = None

    def publish(self, jpg, meta):
        with self._lock: self._jpg, self._meta = jpg, meta

    @property
    def target(self):
        with self._lock:
            return self._target if time.monotonic() - self._t_target < TARGET_HOLD_S else None

    def start(self):
        srv = self
        class H(BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def do_GET(self):
                u = urllib.parse.urlparse(self.path); q = urllib.parse.parse_qs(u.query)
                with srv._lock:
                    if "target" in q:
                        t = q["target"][0]; srv._target, srv._t_target = (t if t and t != "none" else None), time.monotonic()
                    jpg, meta = srv._jpg, srv._meta
                body = json.dumps(meta, separators=(",", ":")).encode("ascii", "replace")
                if u.path == "/scene.json":
                    self.send_response(200); self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
                if jpg is None:
                    self.send_response(204); self.end_headers(); return
                self.send_response(200); self.send_header("Content-Type", "image/jpeg")
                self.send_header("X-Blimpy-Meta", body.decode("ascii")); self.send_header("Content-Length", str(len(jpg)))
                self.end_headers(); self.wfile.write(jpg)
        try:
            self.httpd = ThreadingHTTPServer(("127.0.0.1", self.port), H); self.httpd.daemon_threads = True
        except OSError as e:
            print(f"[eyes] cannot serve the picture on 127.0.0.1:{self.port} ({e}): Blimpy's eyes (pilot --omni-cam room) will see nothing"); return self
        threading.Thread(target=self.httpd.serve_forever, daemon=True, name="eyes-http").start()
        return self

    def stop(self):
        if self.httpd is not None: self.httpd.shutdown(); self.httpd.server_close()


class RoomEyes(threading.Thread):
    """The pilot's side: polls the newest frame + meta at `hz`, never blocks the caller. latest() -> (BGR frame, meta) or
    (None, None) when mono has not answered for STALE_S (then Blimpy is blind, and says so, rather than seeing the past)."""
    STALE_S = 3.0

    def __init__(self, port=ROOM_HTTP_PORT, hz=3.0):
        super().__init__(daemon=True, name="room-eyes")
        self.url, self.period = f"http://127.0.0.1:{port}/frame.jpg", 1.0 / hz
        self.target = None
        self._lock, self._frame, self._meta, self._t_ok = threading.Lock(), None, None, 0.0
        self.alive, self.errors = True, 0

    def poll(self):
        import cv2, numpy as np
        url = self.url + "?target=" + urllib.parse.quote(self.target or "none")
        with _DIRECT.open(url, timeout=0.5) as r:
            if r.status != 200: return False
            meta = json.loads(r.headers.get("X-Blimpy-Meta") or "{}"); img = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
        if img is None: return False
        with self._lock: self._frame, self._meta, self._t_ok = img, meta, time.monotonic()
        return True

    def run(self):
        while self.alive:
            t0 = time.monotonic()
            try: self.poll(); self.errors = 0
            except Exception: self.errors += 1
            time.sleep(max(0.02, self.period - (time.monotonic() - t0)))

    def latest(self):
        with self._lock:
            if self._frame is None or time.monotonic() - self._t_ok > self.STALE_S: return None, None
            t = (self._meta or {}).get("t")                  # mono's frame clock = protocol.now_ms(), the same clock in every process
            if t is not None and now_ms() - t > 1000 * self.STALE_S: return None, None      # a live mono serving a frozen camera is not the present
            return self._frame, self._meta

    def wait_first(self, timeout=30.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.latest()[0] is not None: return True
            time.sleep(0.2)
        return False

    def stop(self): self.alive = False
