"""Keeps what a voice session heard and decided, so the next idea can be tried on it WITHOUT the sponsored key:

  data/voice_sessions/<stamp>_<purpose>/
      mic.wav       everything the microphone picked up, 16 kHz mono (also what the gate kept on the laptop)
      turns.jsonl   one row per spoken turn: where it is in mic.wav, the transcript, the context the addressee module saw
                    (addressee.Ctx), the cue scores, what the judge said (and the hash of the prompt it was asked), the
                    verdict, Blimpy's reply, and "label": null until a human says who the turn was for
      frames/       up to two pictures per turn: what Blimpy's camera saw while it was said (what the judge is shown)
      meta.json     model, room mode, addressee parameters

`python tools/addressee_backtest.py` replays the rows through the current code. data/ is gitignored: recordings of a
room full of people stay on this laptop."""
import dataclasses, datetime, hashlib, json, pathlib, threading, wave

ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_DIR = ROOT / "data" / "voice_sessions"


def prompt_hash(prompt): return hashlib.sha1(prompt.encode("utf-8")).hexdigest()[:12]


class SessionRecorder:
    def __init__(self, purpose="omni", root=None, rate=16000, meta=None):
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = pathlib.Path(root or DEFAULT_DIR) / f"{stamp}_{purpose}"; self.dir.mkdir(parents=True, exist_ok=True)
        self.rate = rate; self._lock = threading.Lock(); self._n = 0; self.turns = 0
        self._wav = wave.open(str(self.dir / "mic.wav"), "wb"); self._wav.setnchannels(1); self._wav.setsampwidth(2); self._wav.setframerate(rate)
        (self.dir / "meta.json").write_text(json.dumps(dict(meta or {}, started=datetime.datetime.now().astimezone().isoformat(), purpose=purpose), indent=1, default=str))

    def audio(self, pcm):
        with self._lock:
            if self._wav is not None: self._wav.writeframesraw(pcm); self._n += len(pcm) // 2

    def now(self):
        """Seconds into mic.wav."""
        return self._n / self.rate

    def turn(self, ctx, row):
        """ctx: addressee.Ctx; row: the rest (t0, t1, verdict, why, score, parts, judge_p, judge_why, judge_hash, reply)."""
        self._write(dict(row, who="user", ctx=dataclasses.asdict(ctx), label=None)); self.turns += 1

    def frames(self, jpgs_b64):
        """Keep the pictures of a turn (what the judge saw, or would have seen) -> their paths relative to the session."""
        import base64
        out = []
        for i, j in enumerate(jpgs_b64):
            (self.dir / "frames").mkdir(exist_ok=True)
            rel = f"frames/{self.turns:03d}_{i}.jpg"; (self.dir / rel).write_bytes(base64.b64decode(j)); out.append(rel)
        return out

    def said(self, text):
        """Blimpy's own words (context when reading a session back; the backtest skips these rows)."""
        self._write({"who": "blimpy", "t": round(self.now(), 2), "text": text})

    def _write(self, rec):
        with self._lock:
            with open(self.dir / "turns.jsonl", "a", encoding="utf-8") as f: f.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")

    def close(self):
        with self._lock:
            if self._wav is not None: self._wav.close(); self._wav = None
