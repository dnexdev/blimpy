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
Verified live on the relay with the sponsored key: "pcm" in/out, semantic_vad, voice "Tina", set_intent arrives as
response.function_call_arguments.done AND response.output_item.done (same call_id), input transcription comes back as
conversation.item.input_audio_transcription.completed, usage is in response.done (null when a reply was cut off by
barge-in), and an image is refused until audio has been appended in the session.

WHAT THE RELAY BILLS (read back from its per-call log, GET /api/log/token, 2026-09-19): the audio the CLIENT appends, at
100 tokens per second of 16 kHz pcm (15.94 s of wav -> 1594 audio_input tokens) x audio ratio 8; text items the client
creates x 1; output text x 6; output audio x 8 x 3.75 counted from the bytes it forwards (8.96 s of 24 kHz reply ->
139 tokens, ~15.6 per second); all x model ratio 2 x group ratio 2, in quota of which ~250 000 make one dashboard
"dollar" (the 200 limit). The growing prompt (context re-reads, 20k prompt_tokens a session) and the images are NOT
billed. So an open mic costs ~0.77 units per MINUTE whether anyone talks or not (a 74 s pilot session with the mic open
cost 0.92 units) and Blimpy's own speech ~0.45 units per minute: MicGate below streams only while someone is talking
(verified live: 6 s of room noise sent nothing, a 5.5 s question was billed as 5.9 s), and the instructions keep replies short.
"""
import base64, json, math, os, queue, re, sys, threading, time, uuid
from collections import deque

MODEL = os.environ.get("OMNI_MODEL", "qwen3.5-omni-plus-realtime")
URL = os.environ.get("OMNI_URL", "wss://yibuapi.com/v1/realtime")
API_KEY = os.environ.get("OMNI_API_KEY") or os.environ.get("YIBU_API_KEY")
IN_RATE, OUT_RATE = 16000, 24000          # Qwen-Omni-Realtime: pcm16 mono in at 16 kHz, out at 24 kHz
MIC_BLOCK = 640                           # samples per mic packet = 40 ms
JPEG_MAX_SIDE, JPEG_QUALITY, JPEG_MAX_B = 640, 70, 250_000   # provider limit: 256 KB base64 per image, 1 fps recommended

# The relay's tariff (see the header). relay_units() turns what this client sent / received into dashboard units.
RELAY = dict(audio_in_tok_per_s=100, audio_out_tok_per_s=48000 / 3072, audio_in=8, text_in=1, text_out=6, audio_out=30,
             model=2, group=2, unit_quota=250_000)
GATE = dict(open_db=12.0, min_dbfs=-50.0, preroll_ms=320, hangover_ms=1000)   # config.OMNI GATE_* override these in the pilot
# Who is that for? The server transcribes every turn and addressed() below decides from the name, the state of the
# conversation, what was said and who is in front of the eye. A turn that is not for Blimpy makes no sound and runs no
# command: the reply and the tool calls of a turn are HELD until its transcript has been judged (the transcript comes on
# its own clock, often after the first reply audio). config.OMNI NAME_* / ADDRESS* override these in the pilot.
NAME_GATE = dict(words=("blimpy", "blimpie", "blippi", "limpie", "blimp", "limpy", "blimpey"),
                 followup_s=8.0, followup_loud_s=5.0,   # how long after Blimpy's last words a follow-up needs no name (quiet / loud room)
                 policy="smart",                        # smart: the rules in addressed() | name: the name, a stop word or press-to-talk only | open: everything
                 mode="auto",                           # auto: loud when the mic gate's noise floor is above loud_floor_db | quiet | loud
                 loud_floor_db=-45.0, gate_db_loud=None,   # gate_db_loud: GATE_DB used while the room is loud (None = unchanged)
                 verdict_timeout_s=1.5)                 # no transcript this long after the first held reply audio: judged without the text

# directed(): does the sentence read as something said TO a robot? An imperative from Blimpy's vocabulary at the start
# (after fillers), a request ("can you ..."), or a question to "you". "yeah recording started" is none of these.
FILLERS = ("ok", "okay", "now", "and", "then", "so", "please", "hey", "alright", "right", "um", "uh", "well", "also", "actually", "just", "yeah", "yes")
REQUESTS = (("can", "you"), ("could", "you"), ("would", "you"), ("will", "you"), ("i", "want", "you"), ("i", "need", "you"), ("let's",), ("lets",))
IMPERATIVES = {   # first word -> the words allowed next (None = anything; a bare imperative always counts)
    "follow": None, "stay": None, "spin": None, "rotate": None, "dance": None, "wander": None, "roam": None, "explore": None,
    "hover": None, "land": None, "fly": None, "float": None, "rise": None, "climb": None, "descend": None, "higher": None, "lower": None,
    "go": ("to", "home", "up", "down", "higher", "lower", "left", "right", "back", "forward", "over", "there", "around"),
    "come": ("here", "over", "back", "to", "with", "down", "up", "closer"),
    "turn": ("left", "right", "around", "to", "by", "clockwise", "counter", "counterclockwise", "a"),
    "look": ("at", "here", "over"), "watch": ("me", "this"), "keep": ("me", "an", "watching"), "tell": ("me", "us"), "show": ("me", "us"),
    "set": ("timer", "pomodoro"), "start": ("timer", "pomodoro", "focus"), "cancel": ("timer", "pomodoro", "focus"),   # within the next 4 words
}
QUESTIONS = (r"\b(what|who|where|how|why) (do|can|are|did|would|will|were) you\b", r"\b(do|did|can|could|are|were|will|would|have) you\b",
             r"\bwhat('s| is| are) (this|that|these|those|in my|on my)\b", r"\bwhat am i\b", r"\bhow (much time|long)\b",
             r"\byour (name|battery|job)\b", r"\bwho are you\b")


def is_addressed(text, words=NAME_GATE["words"]):
    low = (text or "").lower()
    return any(w in low for w in words)


def directed(text):
    low = (text or "").lower()
    w = re.findall(r"[a-z']+", low)
    while w and w[0] in FILLERS: w = w[1:]
    if not w: return False
    if any(tuple(w[:len(r)]) == r for r in REQUESTS): return True
    if w[0] in ("up", "down", "left", "right") and len(w) <= 3: return True
    if w[0] in IMPERATIVES:
        nxt = IMPERATIVES[w[0]]
        if nxt is None or len(w) == 1: return True
        return any(x in nxt for x in (w[1:5] if w[0] in ("set", "start", "cancel") else w[1:2]))
    return any(re.search(q, low) for q in QUESTIONS)


def addressed(text, engaged=False, asked=False, presence=False, ptt=False, loud=False, policy="smart", words=NAME_GATE["words"]):
    """Is this turn for Blimpy? -> (bool, why). text None = the transcript never came (judged on the rest).
      1. the name anywhere in the sentence, a stop word, a press-to-talk turn            ("turn left, Blimpy": no "hey" needed)
      2. engaged (the person started within the follow-up window of Blimpy's last words to an addressed turn)
         AND (the sentence is directed() OR Blimpy had just asked a question)             (follow-ups, answers)
      3. quiet room only: directed() AND someone is near and centred in Blimpy's eye      (a command to its face, cold)
    Anything else is people talking to each other."""
    if policy == "open": return True, "open"
    if ptt: return True, "press-to-talk"
    if text is not None and is_addressed(text, words): return True, "name"
    if text is not None and is_stop(text): return True, "stop word"
    if policy == "name": return False, "no name"
    d = text is not None and directed(text)
    if engaged and d: return True, "follow-up"
    if engaged and asked: return True, "answer to Blimpy's question"
    if not loud and d and presence: return True, "said to its face"
    if engaged: return False, "in conversation, but not said to Blimpy"
    return False, ("no name" + (", loud room" if loud and d else ""))


def pick_device(spec, kind):
    """Resolve a device index or a name fragment ("AirPods", "Speakers") to a sounddevice index, or None for the default.
    Windows lists the same speaker under five drivers: prefer MME (it resamples to our 16 kHz / 24 kHz; WASAPI and
    WDM-KS refuse rates the device does not run natively), so "Speakers" is not ambiguous."""
    if spec is None or spec == "": return None
    if isinstance(spec, int) or str(spec).isdigit(): return int(spec)
    import sounddevice as sd
    want = str(spec).lower(); apis = sd.query_hostapis()
    hits = [(i, d) for i, d in enumerate(sd.query_devices()) if want in d["name"].lower() and d[f"max_{kind}_channels"] > 0]
    if not hits: raise ValueError(f"no {kind} device matching {spec!r} (python -m sounddevice lists them)")
    rank = {"MME": 0, "Windows DirectSound": 1, "Windows WASAPI": 2}
    return min(hits, key=lambda h: rank.get(apis[h[1]["hostapi"]]["name"], 3))[0]


def open_mic(callback, device=None, rate=IN_RATE, block=MIC_BLOCK):
    """Start a 16 kHz mono int16 capture that calls callback(pcm_bytes) per 40 ms. If the device refuses 16 kHz (Bluetooth
    headsets under WDM-KS), capture at its own rate and resample down here."""
    import sounddevice as sd
    try:
        s = sd.RawInputStream(samplerate=rate, channels=1, dtype="int16", blocksize=block, device=device,
                              callback=lambda d, f, t, st: callback(bytes(d)))
        s.start(); return s
    except Exception as e:
        native = int(sd.query_devices(device, "input")["default_samplerate"])
        if native == rate: raise
        import numpy as np
        step = native / rate

        def cb(d, f, t, st):
            a = np.frombuffer(bytes(d), np.int16).astype(np.float32)
            idx = np.arange(0, len(a) - 1, step)
            callback(np.interp(idx, np.arange(len(a)), a).astype(np.int16).tobytes())
        s = sd.RawInputStream(samplerate=native, channels=1, dtype="int16", blocksize=int(block * step), device=device, callback=cb)
        s.start(); print(f"[omni] mic runs at {native} Hz, resampled to {rate} ({e.__class__.__name__} at {rate})"); return s


def relay_units(audio_in_s=0.0, audio_out_s=0.0, text_in=0, text_out=0):
    """Dashboard units for what a client sent (16 kHz audio seconds, text tokens) and got back (24 kHz audio seconds, text)."""
    q = (audio_in_s * RELAY["audio_in_tok_per_s"] * RELAY["audio_in"] + audio_out_s * RELAY["audio_out_tok_per_s"] * RELAY["audio_out"]
         + text_in * RELAY["text_in"] + text_out * RELAY["text_out"]) * RELAY["model"] * RELAY["group"]
    return q / RELAY["unit_quota"]


class MicGate:
    """Stream the mic only while someone is talking. Energy gate on 40 ms packets: a packet louder than the tracked noise
    floor + open_db (and louder than min_dbfs) opens it and releases the pre-roll ring (the first syllable); it closes
    hangover_ms after the last loud packet, longer than the server VAD's silence window so the turn still ends on the
    server. The floor follows the quietest recent packet (instant down, 3 dB/s up), so a hall that gets louder is
    tracked and a close-talk mic still wins. Not a wake word: anything loud enough goes up. process(pcm) -> packets to
    send; level / floor / open are for the meter (python -m laptop.voice.omni --meter)."""

    def __init__(self, open_db=None, min_dbfs=None, preroll_ms=None, hangover_ms=None, rate=IN_RATE, warmup_s=0.5, rise_db_s=3.0,
                 on_ready=None):
        # warmup_s: the gate listens to the room first (nothing goes up); at the end the noise floor is the 20th percentile
        # of what it heard (someone talking during the listen does not poison it) and on_ready(gate) gets the verdict.
        self.open_db = GATE["open_db"] if open_db is None else open_db
        self.min_dbfs = GATE["min_dbfs"] if min_dbfs is None else min_dbfs
        self.preroll_s = (GATE["preroll_ms"] if preroll_ms is None else preroll_ms) / 1000.0
        self.hangover = (GATE["hangover_ms"] if hangover_ms is None else hangover_ms) / 1000.0
        self.rate, self.warmup_s, self.rise = rate, warmup_s, rise_db_s
        self.floor = None; self.level = -100.0; self.open = False; self.t0 = None; self.t_loud = 0.0
        self.pre = deque(); self.pre_s = 0.0
        self.total_s = 0.0; self.sent_s = 0.0; self.opens = 0
        self.on_ready, self.ready, self._levels = on_ready, False, []

    def verdict(self):
        """One line about the room, from the floor measured during warmup."""
        f = self.floor if self.floor is not None else -100.0
        room = ("LOUD room: hold a headset mic close to your mouth, or use hold-to-talk" if f > -35 else
                "noisy room: a close-talk mic is recommended" if f > -45 else "quiet room: the laptop mic is fine")
        return f"room listened to for {self.warmup_s:.0f} s: noise floor {f:.0f} dBFS, the gate opens above {self.threshold:.0f} dBFS ({room})"

    @property
    def threshold(self): return max((self.floor if self.floor is not None else -100.0) + self.open_db, self.min_dbfs)

    def process(self, pcm, now=None):
        import numpy as np
        now = time.monotonic() if now is None else now
        a = np.frombuffer(pcm, np.int16).astype(np.float32)
        secs = len(a) / self.rate; self.total_s += secs
        self.level = db = 20.0 * math.log10(math.sqrt(float(np.mean(a * a))) / 32768.0 + 1e-6)   # dBFS, floor -120
        if self.t0 is None: self.t0 = now
        self.floor = db if self.floor is None else min(db, self.floor + self.rise * secs)
        if not self.ready:
            self._levels.append(db)
            if now - self.t0 >= self.warmup_s:
                self.floor = sorted(self._levels)[len(self._levels) // 5]; self.ready = True; self._levels = []
                if self.on_ready: self.on_ready(self)
        loud = db > self.threshold and self.ready
        if loud: self.t_loud = now
        out = []
        if self.open:
            out.append(pcm)
            if now - self.t_loud > self.hangover: self.open = False
        elif loud:
            self.open = True; self.opens += 1
            out.extend(self.pre); out.append(pcm)
            self.pre.clear(); self.pre_s = 0.0
        else:
            self.pre.append(pcm); self.pre_s += secs
            while self.pre_s > self.preroll_s and len(self.pre) > 1:
                self.pre_s -= len(self.pre.popleft()) / 2 / self.rate
        self.sent_s += sum(len(p) for p in out) / 2 / self.rate
        return out

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

SILENT_TOOL = {
    "type": "function", "name": "stay_silent",
    "description": "Call this, and say NOTHING, when what you heard was not said to you: people talking to each other, "
                   "status updates between teammates, background speech, noise.",
    "parameters": {"type": "object", "properties": {}, "required": []},
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
- If the user interrupts you mid-sentence, what they said is for you: act on it (a command -> set_intent, a question
  -> answer) and NEVER resume or repeat what you were saying. "Stop", "hold", "hover", "halt" always mean set_intent
  hover, also right after an interruption.
- You share the room with people who talk to EACH OTHER most of the time. Speech is for you only when it uses your
  name ("Blimpy", anywhere in the sentence), continues a conversation you are having, or is plainly a command or a
  question to a robot. Remarks between people ("yeah, recording started", "the code is running", "pass me that") are NOT
  for you: call stay_silent and produce no speech at all, not even "okay" or "mm-hm". When in doubt, stay silent.
- Never invent robot abilities you were not given. You cannot pick things up or leave the room."""

# Safety net, independent of the model: a spoken stop acts the moment its transcription arrives (seen live: spoken over
# Blimpy talking, "Blimpy, stop and hover." was transcribed right but answered with "Mm-hm." or a resumed story and no
# tool call; the ASR also wrote "Blippi", so the name is not required). A stop word within the first words, not negated,
# not "stop by" / "a stop sign".
STOP_WORDS = ("stop", "halt", "freeze", "hover")
NEG_WORDS = ("don't", "dont", "do", "not", "never", "no", "didn't", "won't", "a", "the", "what", "what's")


def is_stop(text):
    words = re.findall(r"[a-z']+", (text or "").lower())[:8]
    for i, w in enumerate(words):
        hold = w == "hold" and i + 1 < len(words) and words[i + 1] in ("still", "position", "it", "there", "here")
        if w in STOP_WORDS or hold:
            nxt = words[i + 1] if i + 1 < len(words) else ""
            return not any(p in NEG_WORDS for p in words[:i]) and nxt not in ("by", "sign", "signs", "watch", "light", "lights")
    return False

# session.update payload. Verified live: "pcm" (16 kHz in, 24 kHz out) is what the relay wants; OMNI_AUDIO_FMT still
# overrides it. The server adds input_audio_transcription (qwen3-asr-flash-realtime) itself.
SESSION = {
    "modalities": ["text", "audio"],
    "voice": os.environ.get("OMNI_VOICE", "Tina"),                # verified live: "Cherry" is rejected by the relay, "Tina" is its default
    "instructions": INSTRUCTIONS,
    "input_audio_format": os.environ.get("OMNI_AUDIO_FMT", "pcm"),
    "output_audio_format": os.environ.get("OMNI_AUDIO_FMT", "pcm"),
    "turn_detection": {"type": "semantic_vad", "threshold": 0.5, "silence_duration_ms": 600},
    "tools": [INTENT_TOOL, SILENT_TOOL],
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
                 usage_log=True, debug=False, on_text=None, gate=True, name_gate=True, presence_fn=None):
        self.on_intent, self.frame_fn, self.fps = on_intent, frame_fn, fps
        self.gate = gate if isinstance(gate, MicGate) else (MicGate(**gate) if isinstance(gate, dict) else (MicGate() if gate else None))
        self.name_gate = (dict(NAME_GATE, **name_gate) if isinstance(name_gate, dict) else (dict(NAME_GATE) if name_gate else None))
        # Addressing state. _turn is the user turn being judged: {"t0", "verdict": None|True|False, "audio": [held reply
        # pcm], "calls": [held tool calls], "engaged", "asked", "presence", "ptt"}. presence_fn() -> True when someone is
        # near and centred in Blimpy's eye (the pilot builds it from the FPV observation); None = that rule is off.
        self.presence_fn = presence_fn
        self._turn = None; self._tlock = threading.RLock(); self._ptt_turn = False; self.t_last_reply = 0.0
        self._resp_client = False; self._client_creates = 0      # is the running response one WE asked for (say, tool result)? never held
        self._asked = False; self._loud = False; self.last_verdict = ""; self._resp_played = False
        self._gate_db = self.gate.open_db if self.gate else None
        self.cost = {"audio_in_s": 0.0, "audio_out_s": 0.0, "text_in": 0, "text_out": 0}   # what the relay charges for, this session
        self.key = api_key or API_KEY
        self.model, self.url = model or MODEL, url or URL
        self.session = dict(SESSION, **(session or {}))
        self.use_mic, self.use_spk = mic, speaker
        self.mic_device, self.spk_device = pick_device(mic_device, "input"), pick_device(spk_device, "output")
        self.half_duplex = half_duplex      # laptop speakers + laptop mic: drop mic packets while Blimpy talks (no AEC)
        self.purpose, self.debug, self.on_text = purpose, debug, on_text
        self.usage_log, self.usage_path = bool(usage_log), (usage_log if isinstance(usage_log, str) else None)   # True = data/omni_usage.jsonl
        self.ws = None; self.spk = None; self.mic_stream = None
        self.ready = threading.Event(); self.stopping = False; self.muted = False
        self.connected = False; self.last_error = None; self.n_reconnect = 0
        self.stats = {"audio_out": 0, "img_out": 0, "audio_in": 0, "calls": 0, "responses": 0, "events": 0}
        self.events = deque(maxlen=200)     # last raw server events (debug / tests)
        self._done_calls = set(); self._t_resp = 0.0; self._audio_since_commit = 0; self._sess_ok = False
        self.t_speech_stopped = 0.0; self.t_first_audio = 0.0     # per response: VAD end of the user's turn -> first reply audio
        self._ptt_until = 0.0                                      # press-to-talk window (push_to_talk)
        self._resp_active = False; self._after_done = []           # response.create is refused while a response runs: queue it
        self.transcript = []                # (who, text) for the demo log
        self._t_frame = 0.0

    def units(self):
        """Estimated dashboard units this session has cost so far (the relay's tariff, see RELAY)."""
        return relay_units(**self.cost)

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
            self._sess_ok = False; self._audio_since_commit = 0
            self.ws = websocket.WebSocketApp(
                f"{self.url}?model={self.model}", header=[f"Authorization: Bearer {self.key}", "OpenAI-Beta: realtime=v1"],
                on_open=self._on_open, on_message=self._on_message, on_error=self._on_error, on_close=self._on_close)
            self.ws.run_forever(ping_interval=20, ping_timeout=10)
            self.connected = False; self.ready.clear()
            if not self._sess_ok and self.usage_log:              # a connection that never reached a session is a failed call
                from . import usage_log
                usage_log.record(self.model, self.key, self.purpose, f"{self.url}?model={self.model}", "websocket", False,
                                 0, None, self.last_error or "no session", path=self.usage_path)
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
            self._sess_ok = True
            if t == "session.updated" or not self.ready.is_set():
                self.ready.set(); self.n_reconnect = 0
        elif t == "response.audio.delta":
            if not self.t_first_audio: self.t_first_audio = time.monotonic()
            pcm = base64.b64decode(ev.get("delta", "")); self.stats["audio_out"] += len(pcm); self.cost["audio_out_s"] += len(pcm) / 2 / OUT_RATE
            turn = self._turn
            with self._tlock:
                verdict = True if (self._resp_client or turn is None) else turn["verdict"]
                if verdict is None:                       # not judged yet: hold the reply, make no sound
                    turn["audio"].append(pcm)
                    if len(turn["audio"]) == 1 and self.name_gate:
                        tm = threading.Timer(self.name_gate["verdict_timeout_s"], self._judge, [turn, None]); tm.daemon = True; tm.start()
            if verdict: self._play(pcm)                   # verdict False: the reply to someone else's sentence is dropped
        elif t == "input_audio_buffer.speech_started":
            self._new_turn()                              # judged when its transcript arrives
            if self.spk: self.spk.flush()                 # barge-in: user talks over Blimpy
        elif t == "input_audio_buffer.speech_stopped":
            self.t_speech_stopped = time.monotonic(); self._ptt_until = 0.0      # a press-to-talk turn ends here
        elif t == "input_audio_buffer.committed":
            self._audio_since_commit = 0
        elif t in ("response.function_call_arguments.done",):
            self._tool_call(ev.get("call_id"), ev.get("name"), ev.get("arguments"))
        elif t == "response.output_item.done":
            item = ev.get("item") or {}
            if item.get("type") == "function_call":
                self._tool_call(item.get("call_id"), item.get("name"), item.get("arguments"))
        elif t == "response.audio_transcript.done" or t == "response.text.done":
            txt = ev.get("transcript") or ev.get("text") or ""
            turn = self._turn
            with self._tlock:
                held = turn is not None and not self._resp_client and turn["verdict"] is None
                if held: turn["reply_text"] = txt         # logged if the turn turns out to be for Blimpy
            if txt and self._resp_played and not held: self._said(txt)
        elif t == "conversation.item.input_audio_transcription.completed":
            txt = ev.get("transcript") or ""
            turn = self._turn or self._new_turn()
            if self.debug and self.t_speech_stopped: print(f"\n[omni] transcript {time.monotonic() - self.t_speech_stopped:.2f} s after speech_stopped, "
                                                           f"{len(turn['audio'])} reply packets held")
            if not self._judge(turn, txt):                # someone else talking: no sound, no command
                if txt: self.transcript.append(("ignored", txt)); self._log("ignored", f"{txt}   [{self.last_verdict}]")
                return
            if txt: self.transcript.append(("you", txt)); self._log("you", txt)
            if is_stop(txt):                              # local stop: hover now, whether or not the model calls the tool
                self.stats["local_stops"] = self.stats.get("local_stops", 0) + 1
                try: self.on_intent({"intent": "hover"})
                except Exception as e: self.last_error = f"local stop: {e}"
                if self.spk: self.spk.flush()
                if self._resp_active: self.send({"type": "response.cancel"})   # seen live: it would resume its story otherwise
                self.say("you stopped and are holding position now (one short sentence, no tool call)")
        elif t == "response.created":
            self._t_resp = time.monotonic(); self.t_first_audio = 0.0; self._resp_active = True; self._resp_played = False
            self._resp_client = self._client_creates > 0          # ours (say / tool result), or the server's answer to a spoken turn
            if self._resp_client: self._client_creates -= 1
        elif t == "response.done":
            self.stats["responses"] += 1; self._resp_active = False
            r = ev.get("response") or {}
            if r.get("status") == "completed" and self._resp_played: self.t_last_reply = time.monotonic()
            od = ((r.get("usage") or {}).get("output_tokens_details") or (r.get("usage") or {}).get("output_token_details") or {})
            self.cost["text_out"] += od.get("text_tokens", 0) or 0
            if self.usage_log:
                from . import usage_log
                usage_log.record(self.model, self.key, self.purpose, f"{self.url}?model={self.model}", "websocket",
                                 r.get("status", "completed") != "failed", (time.monotonic() - self._t_resp) * 1000,
                                 r.get("usage"), r.get("status_details") if r.get("status") == "failed" else None,
                                 path=self.usage_path, status_code=101)
            if self._after_done:
                self._after_done.clear(); self._respond()
        elif t == "error":
            e = ev.get("error") or ev
            msg = str(e.get("message", ""))
            if "append image" in msg:                          # a frame raced a commit: harmless, the next one goes
                self.stats["img_refused"] = self.stats.get("img_refused", 0) + 1; self._audio_since_commit = 0
            elif "active response" in msg:                     # our response.create raced a server-started one: retry after it
                self._resp_active = True; self._after_done.append(True); self._client_creates = max(0, self._client_creates - 1)
            else:
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
        turn = self._turn
        if name == "stay_silent":                         # the model itself says: not for me. Its "no" is final for this turn.
            self.stats["silent"] = self.stats.get("silent", 0) + 1
            self.send({"type": "conversation.item.create",
                       "item": {"type": "function_call_output", "call_id": call_id, "output": json.dumps({"result": "ok"})}})
            if turn is not None and not self._resp_client: self._judge(turn, None, force=(False, "the model stayed silent"))
            return
        if turn is not None and not self._resp_client:
            with self._tlock:
                if turn["verdict"] is None: turn["calls"].append((call_id, name, d)); return      # held until the turn is judged
            if turn["verdict"] is False: return self._call_ignored(call_id)
        self._exec_call(call_id, name, d)

    def _call_ignored(self, call_id):
        """The turn was not for Blimpy: answer the call, run nothing, ask for no reply."""
        self.send({"type": "conversation.item.create",
                   "item": {"type": "function_call_output", "call_id": call_id,
                            "output": json.dumps({"result": "ignored: that was not addressed to you. Stay quiet."})}})

    def _exec_call(self, call_id, name, d):
        if name != "set_intent":
            out = f"unknown tool {name}"
        else:
            d = {k: v for k, v in d.items() if v is not None}
            try: out = self.on_intent(d) or "done"
            except Exception as e: out = f"failed: {e}"
        self.send({"type": "conversation.item.create",
                   "item": {"type": "function_call_output", "call_id": call_id, "output": json.dumps({"result": out})}})
        self._respond()

    def _respond(self):
        """response.create now, or right after the running response finishes (the relay refuses a second one)."""
        if self._resp_active: self._after_done.append(True)
        elif self.send({"type": "response.create"}): self._client_creates += 1

    # ------------------------------------------------------------------ who is that for
    @property
    def loud(self):
        """Loud room (the science-fair floor) or quiet room (the judging room)? From the mic gate's noise floor, 3 dB of
        hysteresis, read only while the gate is shut (a voice is not the room). name_gate mode quiet / loud pins it."""
        ng = self.name_gate
        if not ng: return False
        if ng["mode"] in ("quiet", "loud"): return ng["mode"] == "loud"
        g = self.gate
        if g is not None and g.ready and not g.open and g.floor is not None:
            self._loud = g.floor > ng["loud_floor_db"] + (-3.0 if self._loud else 3.0)
        return self._loud

    def cycle_mode(self):
        """auto -> quiet -> loud -> auto (the pilot's l key)."""
        if not self.name_gate: return "off"
        self.name_gate["mode"] = {"auto": "quiet", "quiet": "loud", "loud": "auto"}[self.name_gate["mode"]]
        return self.name_gate["mode"]

    def _new_turn(self):
        now = time.monotonic(); ng = self.name_gate; loud = self.loud
        if self.gate and ng and ng["gate_db_loud"] is not None: self.gate.open_db = ng["gate_db_loud"] if loud else self._gate_db
        try: presence = bool(self.presence_fn()) if self.presence_fn else False
        except Exception: presence = False
        talking = self._resp_active and self._resp_played                  # talking over Blimpy is talking to Blimpy
        engaged = bool(ng) and (talking or (self.t_last_reply > 0 and now - self.t_last_reply < ng["followup_loud_s" if loud else "followup_s"]))
        self._turn = {"t0": now, "verdict": None if ng else True, "audio": [], "calls": [], "engaged": engaged, "asked": self._asked,
                      "presence": presence, "loud": loud, "timed_out": False}
        return self._turn

    def _play(self, pcm):
        self._resp_played = True; self.t_last_reply = time.monotonic()
        if self.spk: self.spk.write(pcm)

    def _said(self, txt):
        self.transcript.append(("blimpy", txt)); self._log("blimpy", txt)
        self._asked = txt.rstrip().endswith("?")          # an answer to Blimpy's own question needs no name

    def _judge(self, turn, text, force=None):
        """Settle a turn once: release what was held (reply audio, tool calls) or drop it. text None = the timeout (no
        transcript yet); a transcript that arrives after a timed-out "no" and carries the name gets a fresh reply."""
        with self._tlock:
            ng = self.name_gate
            if turn["verdict"] is not None:
                if turn["verdict"] is False and turn["timed_out"] and text is not None and turn is self._turn:
                    ok, why = addressed(text, ptt=False, policy="name", words=ng["words"])
                    if ok: turn["verdict"] = True; turn["timed_out"] = False; self.last_verdict = why + " (late transcript)"; self._respond()
                return turn["verdict"]
            ok, why = force or addressed(text, engaged=turn["engaged"], asked=turn["asked"], presence=turn["presence"], ptt=self._ptt_turn,
                                         loud=turn["loud"], policy=ng["policy"], words=ng["words"])
            if text is None and not force: turn["timed_out"] = True; why += " (no transcript yet)"
            if text is not None: self._ptt_turn = False
            turn["verdict"] = ok; self.last_verdict = why
            audio, calls = turn["audio"], turn["calls"]; turn["audio"], turn["calls"] = [], []
            said = turn.pop("reply_text", None)
        if ok:
            for pcm in audio: self._play(pcm)
            if said and audio: self._said(said)
            for c in calls: self._exec_call(*c)
        else:
            self.stats["ignored"] = self.stats.get("ignored", 0) + 1
            for c in calls: self._call_ignored(c[0])
            if self._resp_active and not self._resp_client: self.send({"type": "response.cancel"})
        return ok

    # ------------------------------------------------------------------ inputs
    def push_to_talk(self, max_s=10.0):
        """Press-to-talk: the mic goes up in full for ONE turn, past the gate and past mute, until the server hears you
        stop (speech_stopped) or max_s. Stage mode = mute once, then one press per command."""
        self._ptt_until = time.monotonic() + max_s; self._t_frame = 0.0; self._ptt_turn = True
        if self.gate: self.gate.pre.clear(); self.gate.pre_s = 0.0

    @property
    def ptt(self): return time.monotonic() < self._ptt_until

    def feed_audio(self, pcm16_bytes):
        """Push 16 kHz mono int16 PCM (the mic callback, tests, or a custom capture). With a gate, silence stays here."""
        if not self.ok: return
        if self.half_duplex and self.spk and self.spk.busy(): return
        if self.ptt:
            packets = [pcm16_bytes]
            if self.gate: self.gate.total_s += len(pcm16_bytes) / 2 / IN_RATE; self.gate.sent_s += len(pcm16_bytes) / 2 / IN_RATE
        elif self.muted:
            return
        elif self.gate:
            was = self.gate.open
            packets = self.gate.process(pcm16_bytes)
            if self.gate.open and not was: self._t_frame = 0.0      # a frame with the first words, not a second later
        else:
            packets = [pcm16_bytes]
        for p in packets:
            if self.send({"type": "input_audio_buffer.append", "audio": base64.b64encode(p).decode()}):
                self.stats["audio_in"] += len(p); self._audio_since_commit += 1; self.cost["audio_in_s"] += len(p) / 2 / IN_RATE

    def _start_mic(self):
        import sounddevice as sd
        self.mic_stream = open_mic(self.feed_audio, self.mic_device)

    def send_frame(self, frame):
        # Verified live: the relay refuses an image until audio has been appended to the CURRENT input buffer ("Error
        # append image before append audio"; a commit opens a fresh buffer), so frames wait for the next mic packet.
        # With a gate, frames only go while someone is talking: that is the only moment the model looks at them.
        if frame is None or not self.ok or self._audio_since_commit == 0 or (self.gate and not self.gate.open): return False
        if self.send({"type": "input_image_buffer.append", "image": encode_jpeg(frame)}):
            self.stats["img_out"] += 1; self._t_frame = time.monotonic(); return True
        return False

    def _frame_loop(self):
        period = 1.0 / max(0.1, self.fps)
        while not self.stopping:
            if time.monotonic() - self._t_frame >= period:
                try: self.send_frame(self.frame_fn())
                except Exception as e: self.last_error = f"frame: {e}"
            time.sleep(0.05)

    def say(self, text):
        """Proactive speech (timers, focus nags, arrival): the model says it in its own words. Falls through to False
        when the session is down so the caller can use local TTS."""
        if not text or not self.ok: return False
        ok = self.send({"type": "conversation.item.create",
                        "item": {"type": "message", "role": "user",
                                 "content": [{"type": "input_text", "text": f"[EVENT] Tell the user, in your own words: {text}"}]}})
        if ok: self._respond(); self.cost["text_in"] += 12 + len(text) // 4
        return ok

    def text(self, text):
        """A typed user turn (debug / the 't' key)."""
        if not self.ok: return False
        ok = self.send({"type": "conversation.item.create",
                         "item": {"type": "message", "role": "user", "content": [{"type": "input_text", "text": text}]}})
        if ok: self._respond(); self.cost["text_in"] += len(text) // 4
        return ok

    def mic_status(self):
        """One short string for a status line: what the mic sent vs heard, gate state and level, estimated units."""
        g = self.gate
        if g is None: return f"mic {self.stats['audio_in'] / 32000:.0f}s ~{self.units():.2f}u"
        state = "PTT " if self.ptt else ("MUTED" if self.muted else ("OPEN" if g.open else "shut"))
        ign = f" ignored {self.stats['ignored']}" if self.stats.get("ignored") else ""
        room = "" if not self.name_gate else (" LOUD" if self.loud else " QUIET") + ("" if self.name_gate["mode"] == "auto" else "!")
        return f"mic {g.sent_s:.0f}/{g.total_s:.0f}s {state} {g.level:.0f}dB>{g.threshold:.0f} ~{self.units():.2f}u{room}{ign}"


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", default="0", help="camera spec for the eyes (laptop/vision/streams.py), or 'none'")
    ap.add_argument("--no-cam", action="store_true"); ap.add_argument("--text", default=None)
    ap.add_argument("--seconds", type=float, default=120); ap.add_argument("--debug", action="store_true")
    ap.add_argument("--full-duplex", action="store_true", help="headphones: keep the mic open while Blimpy talks (barge-in)")
    ap.add_argument("--no-gate", action="store_true", help="stream the mic continuously (costs ~0.77 units per minute)")
    ap.add_argument("--meter", action="store_true", help="no cloud: show the mic level, the noise floor and when the gate would open")
    ap.add_argument("--listen", type=float, default=5.0, help="seconds the gate listens to the room before it opens (sets the noise floor)")
    ap.add_argument("--mic", default=None, help="input device: index or name fragment (AirPods, Headset); python -m sounddevice lists them")
    ap.add_argument("--spk", default=None, help="output device for the voice (Speakers)")
    ap.add_argument("--calibrate", action="store_true", help="no cloud: 3 x 6 s (room, you, other people) -> the GATE_DB to put in config.OMNI")
    a = ap.parse_args()
    if a.calibrate:
        import sounddevice as sd, numpy as np
        dev = pick_device(a.mic, "input"); print(f"[calibrate] mic: {sd.query_devices(dev, 'input')['name'][:60]}")
        levels = []
        s = open_mic(lambda pcm: levels.append(20.0 * math.log10(math.sqrt(float(np.mean(np.frombuffer(pcm, np.int16).astype(np.float32) ** 2))) / 32768.0 + 1e-6)), dev)
        def phase(what, secs=6):
            input(f"\n[calibrate] {what}  (press ENTER, then {secs} s)"); levels.clear(); time.sleep(secs)
            return np.percentile(levels, [20, 50, 90])
        room = phase("ROOM: everyone quiet, mic where it will be during the demo")
        me = phase("YOU: talk to Blimpy the way you will on the day (\"Blimpy, follow me. Blimpy, what do you see?\")")
        them = phase("OTHERS: you stay quiet, let the people around you talk")
        s.stop(); s.close()
        floor, my, their = room[0], me[2], them[2]           # floor: 20th percentile of the room; you / them: 90th percentile
        print(f"\n[calibrate] noise floor {floor:.0f} dBFS   you {my:.0f} dBFS   other people {their:.0f} dBFS   (your margin over them: {my - their:.0f} dB)")
        if my - their >= 6:
            db = round((my + their) / 2 - floor)
            print(f"[calibrate] set GATE_DB={db} in config.OMNI: the gate opens halfway between them and you"
                  + ("" if db <= 30 else " (a big number is fine: it is relative to this room)"))
        else:
            print("[calibrate] the mic hears them nearly as loud as you: no threshold separates you. Put the mic closer to your mouth "
                  "(wired earbuds, a headset) or use stage mode (m to mute, p before each command). The name gate still stops them "
                  "commanding Blimpy; what you lose is a little cost while they talk.")
        sys.exit(0)
    if a.meter:
        import sounddevice as sd
        g = MicGate(warmup_s=a.listen, on_ready=lambda g: print(f"\n[meter] {g.verdict()}"))
        dev = pick_device(a.mic, "input"); print(f"[meter] mic: {sd.query_devices(dev, 'input')['name'][:60]}")
        s = open_mic(g.process, dev)
        print("[meter] talk normally, then stay quiet; the gate should be OPEN only while you talk (Ctrl+C to quit)")
        try:
            while True:
                time.sleep(0.1); bar = "#" * max(0, int((g.level + 60) / 2))
                print(f"[meter] {g.level:6.1f} dBFS floor {g.floor if g.floor is not None else -100:6.1f} opens at {g.threshold:6.1f}  "
                      f"{'OPEN ' if g.open else 'shut '} sent {g.sent_s:5.1f}/{g.total_s:5.1f}s {bar:30s}", end="\r", flush=True)
        except KeyboardInterrupt: print()
        s.stop(); s.close(); sys.exit(0)
    cam = None
    if not a.no_cam and a.cam != "none":
        from ..vision.streams import Stream
        cam = Stream(a.cam, "eyes").wait_first()
    om = OmniLive(on_intent=lambda d: (print(f"\n[omni] TOOL set_intent {d}"), "ok, doing that")[1],
                  frame_fn=(lambda: cam.latest()[0]) if cam else None, mic=a.text is None, debug=a.debug,
                  half_duplex=not a.full_duplex, purpose="omni_cli", mic_device=a.mic, spk_device=a.spk,
                  gate=MicGate(warmup_s=a.listen, on_ready=lambda g: print(f"\n[omni] {g.verdict()}")) if not a.no_gate else False)
    print(f"[omni] connecting to {om.url} model {om.model} key {om.key[-4:] if om.key else None}")
    om.start()
    print("[omni] session up. talk to Blimpy (Ctrl+C to quit)")
    if a.text: om.text(a.text)
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.seconds:
            time.sleep(1); s = om.stats
            print(f"[omni] {om.mic_status()}  frames {s['img_out']:3d}  calls {s['calls']}  responses {s['responses']}  "
                  f"{'UP ' if om.ok else 'DOWN'} {om.last_error or ''}      ", end="\r")
    except KeyboardInterrupt:
        pass
    om.stop()
    if cam: cam.stop()
    print(f"\n[omni] this session: {om.cost['audio_in_s']:.1f} s of audio sent, {om.cost['audio_out_s']:.1f} s heard back, about {om.units():.2f} dashboard units")
    print("[omni] transcript:"); [print(f"  {w}: {t}") for w, t in om.transcript]
