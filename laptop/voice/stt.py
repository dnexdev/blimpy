"""Push-to-talk speech to text: hold a key, speak, release -> text. Uses faster-whisper on the GPU.

  python -m laptop.voice.stt        # test: press ENTER to start recording, ENTER to stop, prints the text
"""
import queue, sys, time
import numpy as np
import sounddevice as sd

RATE = 16000
_model = None


GPU_SIZE = "large-v3-turbo"   # 0.3 s per sentence on the RTX 5080 (fp16); best in a noisy hall. ~1.6 GB download once.
CPU_SIZE = "base.en"          # fallback when CUDA is off (Lenovo "iGPU only" mode!): ~0.3 s, weaker in noise


def get_model(size=None):
    global _model
    if _model is None:
        from faster_whisper import WhisperModel
        try:
            _model = WhisperModel(size or GPU_SIZE, device="cuda", compute_type="float16")
            print(f"[stt] whisper {size or GPU_SIZE} on GPU")
        except Exception as e:                 # no CUDA build available -> CPU
            print(f"[stt] cuda unavailable ({e.__class__.__name__}); using {size or CPU_SIZE} on CPU int8")
            _model = WhisperModel(size or CPU_SIZE, device="cpu", compute_type="int8")
    return _model


class Recorder:
    """Start/stop microphone capture at 16 kHz mono."""

    def __init__(self, device=None):
        self.q = queue.Queue(); self.device = device; self.stream = None

    def start(self):
        self.q = queue.Queue()
        self.stream = sd.InputStream(samplerate=RATE, channels=1, dtype="float32", device=self.device,
                                     callback=lambda ind, f, t, s: self.q.put(ind.copy()))
        self.stream.start()

    def stop(self):
        self.stream.stop(); self.stream.close(); self.stream = None
        chunks = []
        while not self.q.empty(): chunks.append(self.q.get())
        return np.concatenate(chunks).ravel() if chunks else np.zeros(0, np.float32)


def transcribe(audio):
    if len(audio) < RATE * 0.3:
        return ""
    segs, _ = get_model().transcribe(audio, language="en", beam_size=1, vad_filter=True)
    return " ".join(s.text.strip() for s in segs).strip()


if __name__ == "__main__":
    print("[stt] loading whisper..."); get_model(); print("[stt] ready")
    rec = Recorder()
    while True:
        input("ENTER to start recording (Ctrl+C to quit)... ")
        rec.start(); input("recording. ENTER to stop... ")
        audio = rec.stop(); t0 = time.time()
        print(f"  heard: {transcribe(audio)!r}  ({len(audio) / RATE:.1f}s audio, {time.time() - t0:.1f}s to transcribe)")
