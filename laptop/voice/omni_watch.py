"""Focus watcher: every few seconds, show Qwen3.5-Omni (HTTP chat completions) the latest camera frame and ask what the
user is doing -> {"present", "working", "activity", "phone"}. behaviors.py turns "not working" into the focus-guard nag.

  python -m laptop.voice.omni_watch            # laptop webcam, prints a report every 6 s
  python -m laptop.voice.omni_watch --image me.jpg

Cheap (qwen3.5-omni-flash, one small JPEG + ~60 output tokens per look) and independent of the realtime session, so the
nag still works if the websocket is down. Env: OMNI_API_KEY, OMNI_HTTP (default https://yibuapi.com/v1),
OMNI_WATCH_MODEL (default qwen3.5-omni-flash).
"""
import json, os, re, threading, time
import requests

HTTP = os.environ.get("OMNI_HTTP", "https://yibuapi.com/v1")
WATCH_MODEL = os.environ.get("OMNI_WATCH_MODEL", "qwen3.5-omni-flash")
API_KEY = os.environ.get("OMNI_API_KEY") or os.environ.get("YIBU_API_KEY")

PROMPT = """You are the eyes of a study-buddy robot. Look at the frame and answer with ONE JSON object only:
{"present": true/false (is a person in the frame), "working": true/false (are they working: reading, writing, typing,
looking at a screen or notes), "phone": true/false (holding or looking at a phone), "activity": "<= 8 words describing
what the person is doing"}. No other text."""


def ask(frame_b64, model=None, key=None, prompt=PROMPT, purpose="focus_watch", timeout=20):
    """One look. Returns (report dict or None, error string or None). Logs usage to the ledger."""
    from . import usage_log
    key = key or API_KEY; model = model or WATCH_MODEL
    body = {"model": model, "temperature": 0.1, "max_tokens": 120,
            "messages": [{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame_b64}"}},
                {"type": "text", "text": prompt}]}]}
    t0 = time.monotonic(); usage = None; err = None; rep = None
    try:
        r = requests.post(f"{HTTP}/chat/completions", json=body, timeout=timeout,
                          headers={"Authorization": f"Bearer {key}", "Content-Type": "application/json"})
        d = r.json(); usage = d.get("usage")
        if r.status_code != 200: err = f"http {r.status_code}: {str(d.get('error', d))[:200]}"
        else:
            txt = d["choices"][0]["message"].get("content") or ""
            m = re.search(r"\{.*\}", txt, re.S)
            rep = json.loads(m.group(0)) if m else None
            if rep is None: err = f"no json in {txt[:80]!r}"
    except Exception as e:
        err = f"{e.__class__.__name__}: {e}"
    usage_log.record(model, key, purpose, "/v1/chat/completions", "http", err is None,
                     (time.monotonic() - t0) * 1000, usage, err)
    return rep, err


class FocusWatcher(threading.Thread):
    """Calls on_report(report) every `interval` s while enabled (report has present/working/phone/activity/t)."""

    def __init__(self, frame_fn, on_report, interval=6.0, model=None, key=None):
        super().__init__(daemon=True, name="omni-watch")
        self.frame_fn, self.on_report, self.interval = frame_fn, on_report, interval
        self.model, self.key = model, key
        self.enabled = False; self.stopping = False; self.last = None; self.errors = 0
        self.start()

    def run(self):
        from .omni import encode_jpeg
        while not self.stopping:
            if not self.enabled:
                time.sleep(0.2); continue
            t0 = time.monotonic()
            frame = self.frame_fn()
            if frame is not None:
                rep, err = ask(encode_jpeg(frame), self.model, self.key)
                if rep is not None:
                    rep["t"] = time.monotonic(); self.last = rep; self.errors = 0
                    try: self.on_report(rep)
                    except Exception as e: print(f"[watch] on_report: {e}")
                else:
                    self.errors += 1
                    if self.errors in (1, 5, 20): print(f"[watch] {err}")
            time.sleep(max(0.5, self.interval - (time.monotonic() - t0)))

    def stop(self): self.stopping = True


if __name__ == "__main__":
    import argparse, cv2
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", default="0"); ap.add_argument("--image", default=None)
    ap.add_argument("--interval", type=float, default=6.0); ap.add_argument("--seconds", type=float, default=60)
    a = ap.parse_args()
    from .omni import encode_jpeg
    if a.image:
        rep, err = ask(encode_jpeg(cv2.imread(a.image))); print(rep or err)
    else:
        from ..vision.streams import Stream
        cam = Stream(a.cam, "eyes").wait_first()
        w = FocusWatcher(lambda: cam.latest()[0], lambda r: print(f"[watch] {r}"), a.interval); w.enabled = True
        try: time.sleep(a.seconds)
        except KeyboardInterrupt: pass
        w.stop(); cam.stop()
