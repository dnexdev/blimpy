"""Live check of the OMNI realtime client (laptop/voice/omni.py) against the real relay with the sponsored key.
The README 1d checklist, automated: ~90 s, about ten calls, all logged to the ledger with purpose "live_test".

  set YIBU_API_KEY (or OMNI_API_KEY), then:   python tools/omni_live_test.py [--image me.jpg] [--wav-dir DIR]

No mic or speaker: spoken commands are wav files (16 kHz mono int16). On Windows they are synthesised on the fly with
the built-in voices into data/tts_cache/; elsewhere pass --wav-dir with name.wav, rotate.wav, see.wav, stop.wav.
Checks: session accepted (voice, pcm, semantic VAD, tool); a text turn speaks; "follow me" calls set_intent and the
confirmation is spoken; frames are held back until audio has been appended; a spoken "turn left ninety degrees" is
transcribed and becomes rotate +90; with frames flowing, a spoken "what can you see" describes the image; barge-in
cancels a reply; every response lands in the ledger in the organisers' schema. Prints the latencies that matter.
"""
import argparse, json, os, pathlib, subprocess, sys, time, wave
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
import numpy as np
from laptop.voice import omni, usage_log

PHRASES = {"name": "Hey Blimpy, what is your name?",
           "rotate": "Blimpy, turn left ninety degrees.",
           "see": "Blimpy, what can you see right now? Describe it in one sentence.",
           "stop": "Blimpy, stop and hover."}
results = {}


def check(name, ok, detail=""):
    results[name] = bool(ok); print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def wait_for(cond, timeout=20.0, step=0.05):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond(): return True
        time.sleep(step)
    return cond()


def make_wavs(d):
    """Windows: synthesise the phrases with System.Speech at exactly 16 kHz mono int16."""
    os.makedirs(d, exist_ok=True)
    missing = {k: v for k, v in PHRASES.items() if not os.path.exists(os.path.join(d, k + ".wav"))}
    if not missing: return
    if os.name != "nt": sys.exit(f"missing {list(missing)} in {d}: pass --wav-dir with 16 kHz mono wavs")
    lines = ["Add-Type -AssemblyName System.Speech", "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer",
             "$f = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(16000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)"]
    for k, text in missing.items():
        p = os.path.abspath(os.path.join(d, k + ".wav")).replace("'", "''")
        lines.append(f"$s.SetOutputToWaveFile('{p}', $f); $s.Speak('{text}'); $s.SetOutputToNull()")
    subprocess.run(["powershell", "-NoProfile", "-Command", "; ".join(lines)], check=True)


def wav_pcm(path):
    w = wave.open(path, "rb")
    assert (w.getframerate(), w.getnchannels(), w.getsampwidth()) == (16000, 1, 2), w.getparams()
    return w.readframes(w.getnframes())


class FakeSpeaker:
    def __init__(self): self.chunks = []; self.flushes = 0; self.t_first = 0.0
    def write(self, pcm):
        if not self.chunks: self.t_first = time.monotonic()
        self.chunks.append(pcm)
    def flush(self): self.flushes += 1; self.chunks.clear()
    def busy(self, tail_s=0.3): return False
    def close(self): pass


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--image", default=None, help="frame Blimpy sees (default: one shot from camera 0, else a synthetic scene)")
ap.add_argument("--wav-dir", default="data/tts_cache")
a = ap.parse_args()
if not (os.environ.get("OMNI_API_KEY") or os.environ.get("YIBU_API_KEY")): sys.exit("set YIBU_API_KEY first")
make_wavs(a.wav_dir)
pcm = {k: wav_pcm(os.path.join(a.wav_dir, k + ".wav")) for k in PHRASES}

import cv2
frame = cv2.imread(a.image) if a.image else None
if frame is None and not a.image:
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
    for _ in range(30):
        ok, f = cap.read()
        if ok and f is not None and f.mean() > 5: frame = f; break
        time.sleep(0.1)
    cap.release()
if frame is None:
    frame = np.full((480, 640, 3), 235, np.uint8)
    cv2.rectangle(frame, (60, 300), (580, 470), (40, 90, 160), -1); cv2.circle(frame, (320, 180), 90, (30, 30, 220), -1)
    cv2.putText(frame, "BLIMPY", (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255), 4)
    print("[live] no camera: using a synthetic scene (red circle over a blue table)")

ledger = usage_log.DEFAULT_PATH
n_rows0 = sum(1 for _ in open(ledger, encoding="utf-8")) if os.path.exists(ledger) else 0
intents, said = [], []
spk = FakeSpeaker()
om = omni.OmniLive(on_intent=lambda d: (intents.append(d), "done")[1], frame_fn=lambda: frame, fps=1.0, mic=False,
                   speaker=spk, half_duplex=False, purpose="live_test", on_text=lambda w, t: said.append((w, t)))
print(f"[live] {om.url} model {om.model} key ...{om.key[-4:]}")
t0 = time.monotonic()
try: om.start(timeout=20)
except Exception as e: sys.exit(f"[live] no session: {e}")
print(f"[live] session up in {time.monotonic() - t0:.2f} s")


def feed(name, pace=0.04, tail_s=1.0):
    """Stream a wav as 40 ms packets at real-time pace, then `tail_s` of silence so the VAD closes the turn."""
    data = pcm[name] + bytes(int(16000 * tail_s) * 2)
    for i in range(0, len(data), 1280):
        om.feed_audio(data[i:i + 1280]); time.sleep(pace)


def responses(): return om.stats["responses"]


def ev_types(): return [e.get("type") for e in om.events]


# 1. session (start() returns on session.created; the session.updated echo follows a moment later)
wait_for(lambda: any(e.get("type") == "session.updated" for e in om.events), 5)
sess = next((e.get("session") for e in reversed(om.events) if e.get("type") == "session.updated"), {}) or {}
check("session accepted: voice/pcm/semantic_vad/tool", sess.get("voice") == om.session["voice"] and sess.get("input_audio_format") == "pcm"
      and (sess.get("turn_detection") or {}).get("type") == "semantic_vad" and [t.get("name") for t in sess.get("tools") or []] == ["set_intent"],
      f"voice {sess.get('voice')} fmt {sess.get('input_audio_format')}/{sess.get('output_audio_format')} vad {(sess.get('turn_detection') or {}).get('type')}")

# 2. text turn speaks
n = responses(); spk.chunks.clear(); t = time.monotonic(); om.text("Hi! In one short sentence, who are you?")
check("text turn: audio + transcript", wait_for(lambda: responses() > n, 30) and spk.chunks and any(w == "blimpy" for w, _ in said),
      f"first audio {spk.t_first - t:.2f} s, {sum(map(len, spk.chunks)) / 48000:.1f} s of speech: {said[-1][1][:70] if said else ''!r}")

# 3. tool call from text, then spoken confirmation
n = responses(); spk.chunks.clear(); om.text("Blimpy, follow me.")
check("tool: set_intent follow_me", wait_for(lambda: intents, 30) and intents[-1] == {"intent": "follow_me"}, str(intents))
check("tool result spoken", wait_for(lambda: responses() >= n + 2, 30) and spk.chunks, f"{said[-1][1][:60] if said else ''!r}")

# 4. frames are held until audio has been appended
time.sleep(2.2)
check("no frames before audio", om.stats["img_out"] == 0, f"{om.stats['img_out']} frames sent")

# 5. spoken command -> transcription -> rotate +90
n = responses(); n_int = len(intents); said.clear(); om.t_speech_stopped = 0.0; spk.chunks.clear()
feed("rotate")
ok = wait_for(lambda: len(intents) > n_int, 30)
d = intents[-1] if ok else {}
check("spoken 'turn left ninety degrees' -> rotate +90", ok and d.get("intent") == "rotate" and abs(float(d.get("degrees", 0)) - 90) < 1e-6, str(d))
you = next((t for w, t in said if w == "you"), "")
check("input transcription event", bool(you), repr(you))
wait_for(lambda: responses() >= n + 2 and spk.chunks, 30)
lat = (spk.t_first - om.t_speech_stopped) if (spk.chunks and om.t_speech_stopped) else None
print(f"        end of speech -> tool call -> first confirmation audio: {lat and round(lat, 2)} s")

# 6. frames now flow, and the model uses them
check("frames flow after audio (1 fps)", wait_for(lambda: om.stats["img_out"] >= 2, 5), f"{om.stats['img_out']} frames")
n = responses(); said.clear(); spk.chunks.clear()
feed("see")
wait_for(lambda: responses() > n and any(w == "blimpy" for w, _ in said), 40)
desc = next((t for w, t in said if w == "blimpy"), "")
check("spoken 'what can you see' -> describes the frame", len(desc) > 15, repr(desc[:120]))

# 7. barge-in: talk over a reply (the model's answer to the interruption is its own call; the transcript is printed)
n = responses(); spk.chunks.clear(); f0 = spk.flushes; n_int = len(intents); said.clear()
om.text("Tell me a two-sentence story about a balloon.")
wait_for(lambda: spk.chunks, 30)
feed("stop", tail_s=1.0)
check("barge-in: playback flushed, reply cancelled", wait_for(lambda: spk.flushes > f0, 20)
      and wait_for(lambda: any(((e.get("response") or {}).get("status") == "cancelled") for e in om.events if e.get("type") == "response.done"), 20),
      f"flushes {spk.flushes - f0}")
ok = wait_for(lambda: len(intents) > n_int, 30)
wait_for(lambda: responses() >= n + 3, 20)
check("spoken 'stop and hover' (over Blimpy talking) -> hover", ok and intents[-1].get("intent") == "hover",
      f"intents {intents[n_int:]}; heard {[t for w, t in said if w == 'you']}; said {[t[:50] for w, t in said if w == 'blimpy']}")

# 8. ledger
om.stop(); time.sleep(0.5)
rows = [json.loads(l) for l in open(ledger, encoding="utf-8")][n_rows0:] if os.path.exists(ledger) else []
mine = [r for r in rows if r.get("purpose") == "live_test"]
check("ledger rows in the organisers' schema", len(mine) >= 6 and all(r.get("schema_version") == "yibu_call_audit_v1" and r.get("transport") == "websocket"
      and r.get("key_suffix", "").startswith("...") and len(r.get("key_suffix", "")) == 7 for r in mine),
      f"{len(mine)} rows, tokens in {sum(r.get('input_tokens') or 0 for r in mine)} out {sum(r.get('output_tokens') or 0 for r in mine)}, "
      f"usage missing on {sum(1 for r in mine if not r.get('usage_reported'))} (cancelled replies)")
check("ledger never stores the key", om.key not in open(ledger, encoding="utf-8").read())

n_fail = sum(not v for v in results.values())
print(f"\n{len(results) - n_fail}/{len(results)} checks passed   (transcript: {len(said)} lines, reconnects {om.n_reconnect}, last error {om.last_error})")
print(f"[live] cloud: audio sent {om.cost['audio_in_s']:.1f} s of {om.gate.total_s:.1f} s fed (gate), heard back {om.cost['audio_out_s']:.1f} s, "
      f"about {om.units():.2f} units (python tools/omni_report.py --sessions shows the relay's own bill)")
sys.exit(1 if n_fail else 0)
