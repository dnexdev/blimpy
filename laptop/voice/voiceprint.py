"""Voice fingerprints (speaker embeddings): can one open mic tell WHO is talking? A measuring tool, not a cue (yet).

  python -m laptop.voice.voiceprint                     # every recorded session under data/voice_sessions
  python -m laptop.voice.voiceprint <session dir>

The idea: Blimpy talks with ONE person at a time; a follow-up without the name should count only from the voice that
called it, so somebody cutting in gets no credit. That needs the same person's turns to look alike and other people's
to look different. Measured 2026-09-19 on the two hacker-bay sessions (laptop mic, 2-4 s turns, loud hall), honest
leave-one-out: the same person's named turns 0.75-0.84 against each other; OTHER people's turns median 0.74-0.79 and up
to 0.90. The distributions overlap: on this mic in that noise a fingerprint cannot carry a decision, so addressee.py has
no voice cue. Halves of one turn (a "two voices in one turn" detector) were worse still: 1.5 s halves are too short.
Rerun this after a change of microphone (a table mic, a headset) before building on it: the gap between the two lines
it prints is the whole question.

Backend: resemblyzer (pip install resemblyzer webrtcvad-wheels "setuptools<81"), CPU, ~40 ms per turn once warm. Not in
requirements.txt: nothing at run time depends on it."""
import glob, json, pathlib, sys, wave
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]


class VoicePrint:
    def __init__(self):
        import warnings; warnings.filterwarnings("ignore")
        from resemblyzer import VoiceEncoder
        self.enc = VoiceEncoder("cpu", verbose=False)

    def embed(self, pcm16, rate=16000, min_s=0.8):
        """int16 mono PCM (bytes or array) -> unit vector, or None when too short to mean anything."""
        x = np.frombuffer(pcm16, np.int16) if isinstance(pcm16, (bytes, bytearray)) else np.asarray(pcm16)
        if rate != 16000 or len(x) < rate * min_s: return None
        x = x.astype(np.float32) / 32768.0
        return self.enc.embed_utterance(x / (np.abs(x).max() + 1e-6) * 0.9)


def measure(session, vp, pre_s=1.5):
    d = pathlib.Path(session); rows = [r for r in (json.loads(l) for l in open(d / "turns.jsonl", encoding="utf-8")) if r.get("who") == "user"]
    with wave.open(str(d / "mic.wav")) as w:
        rate = w.getframerate(); E = []
        for r in rows:
            if r.get("t0") is None: E.append(None); continue
            w.setpos(min(w.getnframes(), max(0, int((r["t0"] - pre_s) * rate)))); E.append(vp.embed(w.readframes(int((r["t1"] - r["t0"] + pre_s) * rate)), rate))
    named = [(r, e) for r, e in zip(rows, E) if e is not None and r.get("verdict") and "name" in (r.get("why") or "")]
    if len(named) < 2: print(f"{d.name}: fewer than two named turns, nothing to compare"); return
    same = []
    for i, (_, e) in enumerate(named):                     # each named turn against the mean of the OTHER named turns
        ref = np.mean([x for j, (_, x) in enumerate(named) if j != i], 0); same.append(float(e @ ref / np.linalg.norm(ref)))
    ref = np.mean([e for _, e in named], 0); ref /= np.linalg.norm(ref)
    other = sorted((float(e @ ref) for r, e in zip(rows, E) if e is not None and not r.get("verdict")), reverse=True)
    print(f"{d.name}\n  the person who called Blimpy, turn against turn: {' '.join(f'{s:.2f}' for s in same)}   (lowest {min(same):.2f})")
    if other: print(f"  every turn NOT for Blimpy against that voice:    median {np.median(other):.2f}, highest {' '.join(f'{s:.2f}' for s in other[:5])}\n"
                    f"  usable only if the first line sits clearly ABOVE the second (some ignored turns are the same person talking to teammates)")


if __name__ == "__main__":
    vp = VoicePrint()
    for s in sys.argv[1:] or sorted(glob.glob(str(ROOT / "data" / "voice_sessions" / "*"))): measure(s, vp)
