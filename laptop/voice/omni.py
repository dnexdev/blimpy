"""Blimpy's cloud ears, eyes and mouth: one Qwen3.5-Omni REALTIME session (Huawei OMNI Live track, via the yibuapi relay).

    mic (16 kHz pcm) ---+                                  +--> audio out (24 kHz pcm) -> speakers
    camera frame 1 fps -+--> websocket /v1/realtime -------+--> set_intent(...) tool call -> on_intent() -> behaviors
    say("...") events --+                                  +--> transcripts / usage -> data/omni_usage.jsonl

  python -m laptop.voice.omni                 # talk to it: mic + laptop webcam, prints tool calls
  python -m laptop.voice.omni --no-cam        # audio only
  python -m laptop.voice.omni --text "what do you see"   # inject a text turn (no mic)

Env:  OMNI_API_KEY (or YIBU_API_KEY)   the sponsored key
      OMNI_URL    default wss://yibuapi.com/v1/realtime      OMNI_MODEL  default qwen3.5-omni-plus-realtime
The event vocabulary is the OpenAI Realtime one that Alibaba's Qwen-Omni-Realtime speaks (input_audio_buffer.append,
input_image_buffer.append, response.audio.delta, function calls). Everything provider-specific is in SESSION below.
"""
import base64, json, os, queue, sys, threading, time, uuid
from collections import deque

MODEL = os.environ.get("OMNI_MODEL", "qwen3.5-omni-plus-realtime")
URL = os.environ.get("OMNI_URL", "wss://yibuapi.com/v1/realtime")
API_KEY = os.environ.get("OMNI_API_KEY") or os.environ.get("YIBU_API_KEY")
IN_RATE, OUT_RATE = 16000, 24000          # Qwen-Omni-Realtime: pcm16 mono in at 16 kHz, out at 24 kHz
MIC_BLOCK = 640                           # samples per mic packet = 40 ms
JPEG_MAX_SIDE, JPEG_QUALITY, JPEG_MAX_B = 640, 70, 250_000   # provider limit: 256 KB base64 per image, 1 fps recommended

# Same intents as laptop/voice/intent.py (behaviors.py consumes exactly these). The model speaks its own confirmation,
# so there is no "reply" field: the tool result we return tells it what actually happened.
INTENT_TOOL = {
    "type": "function", "name": "set_intent",
    "description": "Give the balloon robot a command. Call this for ANY instruction about moving, following, "
                   "timers, pomodoro, focus guard or mood. Do not call it for questions or small talk.",
    "parameters": {
        "type": "object",
        "properties": {
            "intent": {"type": "string", "enum": ["follow_me", "hover", "wander", "go_to", "rotate", "altitude", "timer",
                                                  "pomodoro", "focus_guard", "mood"],
                       "description": "follow_me: follow the speaker. hover: stop and hold. wander: drift around. "
                                      "go_to: fly to a target. rotate: turn by degrees. altitude: up or down a step. "
                                      "timer: minutes. pomodoro: work/break cycle. focus_guard: watch the user work. "
                                      "mood: dance/happy/sleepy."},
            "target": {"type": "string", "enum": ["judges", "home", "me"], "description": "go_to only"},
            "degrees": {"type": "number", "description": "rotate only: + = counter-clockwise/left, - = clockwise/right; turn around = 180"},
            "direction": {"type": "string", "enum": ["up", "down"], "description": "altitude only"},
            "minutes": {"type": "number", "description": "timer only (ninety seconds = 1.5)"},
            "work_min": {"type": "number"}, "break_min": {"type": "number"},
            "enabled": {"type": "boolean", "description": "focus_guard only"},
            "mood": {"type": "string", "enum": ["happy", "dance", "sleepy"]},
        },
        "required": ["intent"],
    },
}

INSTRUCTIONS = """You are Blimpy, a small friendly helium-balloon robot that floats around a room, follows people and helps them
focus. You hear the user through a microphone and SEE through a camera (a new frame about once a second): use what you
see when it matters ("what am I holding", "is my posture okay", "what's on the whiteboard").
Rules:
- Any instruction about moving, following, stopping, turning, height, timers, pomodoro, focus guard or mood: call
  set_intent ONCE, then confirm in at most 10 words ("On it, right behind you.").
- Questions and small talk: answer briefly (max 2 sentences), in character, warm, a little playful. No emojis.
- Messages starting with [EVENT] come from your own sensors and timers, not from the user: say them to the user in
  your own words, briefly. Never call a tool for an [EVENT].
- If speech is clearly not addressed to you (people talking to each other, noise), say nothing useful: reply with a
  single short "Mm-hm." at most.
- Never invent robot abilities you were not given. You cannot pick things up or leave the room."""

# session.update payload. VERIFY on the first live test: Alibaba documents input_audio_format "pcm" (16 kHz) and
# output "pcm" (24 kHz); OpenAI-style relays may want "pcm16". OMNI_AUDIO_FMT overrides without a code change.
SESSION = {
    "modalities": ["text", "audio"],
    "voice": os.environ.get("OMNI_VOICE", "Cherry"),
    "instructions": INSTRUCTIONS,
    "input_audio_format": os.environ.get("OMNI_AUDIO_FMT", "pcm"),
    "output_audio_format": os.environ.get("OMNI_AUDIO_FMT", "pcm"),
    "turn_detection": {"type": "semantic_vad", "threshold": 0.5, "silence_duration_ms": 600},
    "tools": [INTENT_TOOL],
    "tool_choice": "auto",
}


def encode_jpeg(frame):
    """BGR frame -> base64 JPEG under the provider's size cap (resized, then quality stepped down)."""
    import cv2
    h, w = frame.shape[:2]
    s = JPEG_MAX_SIDE / max(h, w)
    if s < 1: frame = cv2.resize(frame, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    q = JPEG_QUALITY
    while True:
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, q])
        b = base64.b64encode(buf.tobytes()).decode()
        if len(b) <= JPEG_MAX_B or q <= 30: return b
        q -= 15


class Speaker:
    """24 kHz mono int16 playback with an interruptible queue (flush() = barge-in)."""

    def __init__(self, device=None, rate=OUT_RATE):
        import sounddevice as sd
        self.q = queue.Queue(); self.gen = 0; self.playing = False; self.t_last = 0.0
        self.stream = sd.RawOutputStream(samplerate=rate, channels=1, dtype="int16", device=device, blocksize=0)
        self.stream.start()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            gen, chunk = self.q.get()
            if chunk is None: break
            if gen != self.gen: continue                   # flushed while queued
            self.playing = True
            for i in range(0, len(chunk), 2400):            # 50 ms pieces so a flush stops within ~50 ms
                if gen != self.gen: break
                self.stream.write(chunk[i:i + 2400])
            self.t_last = time.monotonic()
            self.playing = not self.q.empty()

    def write(self, pcm): self.q.put((self.gen, pcm))

    def flush(self):
        self.gen += 1; self.playing = False
        while not self.q.empty():
            try: self.q.get_nowait()
            except queue.Empty: break

    def busy(self, tail_s=0.3):
        return self.playing or (time.monotonic() - self.t_last) < tail_s

    def close(self):
        self.flush(); self.q.put((self.gen, None))
        try: self.stream.stop(); self.stream.close()
        except Exception: pass


class OmniLive:
    """One realtime session. Threads: websocket (run_forever), mic (sounddevice callback), frames (1 fps), speaker.
    on_intent(dict) -> str  is called on the websocket thread when the model calls set_intent; return what happened
    (that string is the tool result the model reads before it speaks). Keep it under ~1 s."""

    def __init__(self, on_intent, frame_fn=None, api_key=None, model=None, url=None, session=None, fps=1.0,
                 mic=True, speaker=True, mic_device=None, spk_device=None, half_duplex=True, purpose="demo",
                 usage_log=True, debug=False, on_text=None):
        self.on_intent, self.frame_fn, self.fps = on_intent, frame_fn, fps
        self.key = api_key or API_KEY
        self.model, self.url = model or MODEL, url or URL
        self.session = dict(SESSION, **(session or {}))
        self.use_mic, self.use_spk = mic, speaker
        self.mic_device, self.spk_device = mic_device, spk_device
        self.half_duplex = half_duplex      # laptop speakers + laptop mic: drop mic packets while Blimpy talks (no AEC)
        self.purpose, self.debug, self.on_text = purpose, debug, on_text
        self.usage_log, self.usage_path = bool(usage_log), (usage_log if isinstance(usage_log, str) else None)   # True = data/omni_usage.jsonl
        self.ws = None; self.spk = None; self.mic_stream = None
        self.ready = threading.Event(); self.stopping = False; self.muted = False
        self.connected = False; self.last_error = None; self.n_reconnect = 0
        self.stats = {"audio_out": 0, "img_out": 0, "audio_in": 0, "calls": 0, "responses": 0, "events": 0}
        self.events = deque(maxlen=200)     # last raw server events (debug / tests)
        self._done_calls = set(); self._t_resp = 0.0
        self.transcript = []                # (who, text) for the demo log

    # ------------------------------------------------------------------ lifecycle
    def start(self, timeout=10.0):
        if not self.key: raise RuntimeError("OMNI_API_KEY not set")
        if self.use_spk: self.spk = Speaker(self.spk_device) if self.use_spk is True else self.use_spk   # or any object with write/flush/busy/close
        threading.Thread(target=self._ws_loop, daemon=True, name="omni-ws").start()
        if not self.ready.wait(timeout):
            raise RuntimeError(f"omni: no session after {timeout}s ({self.last_error or 'no reply'})")
        if self.use_mic: self._start_mic()
        if self.frame_fn: threading.Thread(target=self._frame_loop, daemon=True, name="omni-frames").start()
        return self

    @property
    def ok(self): return self.connected and self.ready.is_set()

    def stop(self):
        self.stopping = True
        try:
            if self.mic_stream: self.mic_stream.stop(); self.mic_stream.close()
        except Exception: pass
        try:
            if self.ws: self.ws.close()
        except Exception: pass
        if self.spk: self.spk.close()

    # ------------------------------------------------------------------ websocket
    def _ws_loop(self):
        import websocket
        while not self.stopping:
            self.ws = websocket.WebSocketApp(
                f"{self.url}?model={self.model}", header=[f"Authorization: Bearer {self.key}", "OpenAI-Beta: realtime=v1"],
                on_open=self._on_open, on_message=self._on_message, on_error=self._on_error, on_close=self._on_close)
            self.ws.run_forever(ping_interval=20, ping_timeout=10)
            self.connected = False; self.ready.clear()
            if self.stopping: break
            self.n_reconnect += 1
            wait = min(10, 1.5 * self.n_reconnect)
            print(f"[omni] connection lost ({self.last_error}); reconnecting in {wait:.0f}s")
            time.sleep(wait)

    def _on_open(self, ws):
        self.connected = True; self.last_error = None
        self.send({"type": "session.update", "session": self.session})

    def _on_error(self, ws, err): self.last_error = f"{err.__class__.__name__}: {err}"

    def _on_close(self, ws, code, msg):
        self.connected = False
        if code or msg: self.last_error = f"closed {code} {msg}"

    def send(self, ev):
        ws = self.ws
        if ws is None or not self.connected: return False
        try:
            ws.send(json.dumps(ev)); return True
        except Exception as e:
            self.last_error = f"send: {e}"; return False

    def _on_message(self, ws, raw):
        try: ev = json.loads(raw)
        except ValueError: return
        t = ev.get("type", ""); self.stats["events"] += 1
        if self.debug or not t.endswith((".delta", ".append")): self.events.append(ev)
        if t in ("session.created", "session.updated"):
            if t == "session.updated" or not self.ready.is_set():
                self.ready.set(); self.n_reconnect = 0
        elif t == "response.audio.delta":
            if self.spk: self.spk.write(base64.b64decode(ev.get("delta", "")))
        elif t == "input_audio_buffer.speech_started":
            if self.spk: self.spk.flush()                 # barge-in: user talks over Blimpy
        elif t in ("response.function_call_arguments.done",):
            self._tool_call(ev.get("call_id"), ev.get("name"), ev.get("arguments"))
        elif t == "response.output_item.done":
            item = ev.get("item") or {}
            if item.get("type") == "function_call":
                self._tool_call(item.get("call_id"), item.get("name"), item.get("arguments"))
        elif t == "response.audio_transcript.done" or t == "response.text.done":
            txt = ev.get("transcript") or ev.get("text") or ""
            if txt: self.transcript.append(("blimpy", txt)); self._log("blimpy", txt)
        elif t == "conversation.item.input_audio_transcription.completed":
            txt = ev.get("transcript") or ""
            if txt: self.transcript.append(("you", txt)); self._log("you", txt)
        elif t == "response.created":
            self._t_resp = time.monotonic()
        elif t == "response.done":
            self.stats["responses"] += 1
            r = ev.get("response") or {}
            if self.usage_log:
                from . import usage_log
                usage_log.record(self.model, self.key, self.purpose, "/v1/realtime", "websocket",
                                 r.get("status", "completed") != "failed", (time.monotonic() - self._t_resp) * 1000,
                                 r.get("usage"), r.get("status_details") if r.get("status") == "failed" else None,
                                 path=self.usage_path)
        elif t == "error":
            e = ev.get("error") or ev
            self.last_error = f"server: {e.get('message') or e}"
            print(f"[omni] error: {e}")

    def _log(self, who, text):
        if self.on_text: self.on_text(who, text)
        else: print(f"\n[omni] {who}: {text}")

    def _tool_call(self, call_id, name, args):
        if not call_id or call_id in self._done_calls: return
        self._done_calls.add(call_id); self.stats["calls"] += 1
        try: d = json.loads(args) if isinstance(args, str) else dict(args or {})
        except ValueError: d = {}
        if name != "set_intent":
            out = f"unknown tool {name}"
        else:
            d = {k: v for k, v in d.items() if v is not None}
            try: out = self.on_intent(d) or "done"
            except Exception as e: out = f"failed: {e}"
        self.send({"type": "conversation.item.create",
                   "item": {"type": "function_call_output", "call_id": call_id, "output": json.dumps({"result": out})}})
        self.send({"type": "response.create"})

    # ------------------------------------------------------------------ inputs
    def feed_audio(self, pcm16_bytes):
        """Push 16 kHz mono int16 PCM (tests, or a custom capture)."""
        if self.muted or not self.ok: return
        if self.half_duplex and self.spk and self.spk.busy(): return
        if self.send({"type": "input_audio_buffer.append", "audio": base64.b64encode(pcm16_bytes).decode()}):
            self.stats["audio_in"] += len(pcm16_bytes)

    def _start_mic(self):
        import sounddevice as sd
        self.mic_stream = sd.RawInputStream(samplerate=IN_RATE, channels=1, dtype="int16", blocksize=MIC_BLOCK,
                                            device=self.mic_device, callback=lambda d, f, t, s: self.feed_audio(bytes(d)))
        self.mic_stream.start()

    def send_frame(self, frame):
        if frame is None or not self.ok: return False
        if self.send({"type": "input_image_buffer.append", "image": encode_jpeg(frame)}):
            self.stats["img_out"] += 1; return True
        return False

    def _frame_loop(self):
        period = 1.0 / max(0.1, self.fps)
        while not self.stopping:
            t0 = time.monotonic()
            try: self.send_frame(self.frame_fn())
            except Exception as e: self.last_error = f"frame: {e}"
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def say(self, text):
        """Proactive speech (timers, focus nags, arrival): the model says it in its own words. Falls through to False
        when the session is down so the caller can use local TTS."""
        if not text or not self.ok: return False
        ok = self.send({"type": "conversation.item.create",
                        "item": {"type": "message", "role": "user",
                                 "content": [{"type": "input_text", "text": f"[EVENT] Tell the user, in your own words: {text}"}]}})
        return ok and self.send({"type": "response.create"})

    def text(self, text):
        """A typed user turn (debug / the 't' key)."""
        if not self.ok: return False
        self.send({"type": "conversation.item.create",
                   "item": {"type": "message", "role": "user", "content": [{"type": "input_text", "text": text}]}})
        return self.send({"type": "response.create"})


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", default="0", help="camera spec for the eyes (laptop/vision/streams.py), or 'none'")
    ap.add_argument("--no-cam", action="store_true"); ap.add_argument("--text", default=None)
    ap.add_argument("--seconds", type=float, default=120); ap.add_argument("--debug", action="store_true")
    ap.add_argument("--full-duplex", action="store_true", help="headphones: keep the mic open while Blimpy talks (barge-in)")
    a = ap.parse_args()
    cam = None
    if not a.no_cam and a.cam != "none":
        from ..vision.streams import Stream
        cam = Stream(a.cam, "eyes").wait_first()
    om = OmniLive(on_intent=lambda d: (print(f"\n[omni] TOOL set_intent {d}"), "ok, doing that")[1],
                  frame_fn=(lambda: cam.latest()[0]) if cam else None, mic=a.text is None, debug=a.debug,
                  half_duplex=not a.full_duplex, purpose="omni_cli")
    print(f"[omni] connecting to {om.url} model {om.model} key {om.key[-4:] if om.key else None}")
    om.start()
    print("[omni] session up. talk to Blimpy (Ctrl+C to quit)")
    if a.text: om.text(a.text)
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.seconds:
            time.sleep(1); s = om.stats
            print(f"[omni] audio in {s['audio_in'] / 32000:5.1f}s  frames {s['img_out']:3d}  calls {s['calls']}  responses {s['responses']}  "
                  f"{'UP ' if om.ok else 'DOWN'} {om.last_error or ''}", end="\r")
    except KeyboardInterrupt:
        pass
    om.stop()
    if cam: cam.stop()
    print("\n[omni] transcript:"); [print(f"  {w}: {t}") for w, t in om.transcript]
