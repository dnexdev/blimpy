"""Text to speech through the laptop speakers (offline Windows voices via pyttsx3). Non-blocking.

  python -m laptop.voice.tts "hello, I am blimpy"
"""
import queue, sys, threading

_q = queue.Queue()


def _worker():
    import pyttsx3
    while True:
        text = _q.get()
        if text is None: break
        try:
            # pyttsx3 2.9x on Windows speaks only the FIRST runAndWait() of an engine (later calls return in
            # ~0.08 s, silent). A fresh engine per sentence costs ~50 ms and speaks every time.
            eng = pyttsx3.init()
            eng.setProperty("rate", 175)
            eng.say(text); eng.runAndWait(); eng.stop()
            del eng
        except Exception as e:                       # a dead audio device must never kill the pilot
            print(f"[tts] {e}")
        finally:
            _q.task_done()                           # without this, wait() (= queue.join) blocks forever


_thread = threading.Thread(target=_worker, daemon=True)
_thread.start()


def say(text):
    """Queue a sentence; returns immediately."""
    if text: _q.put(str(text))


def wait():
    _q.join() if hasattr(_q, "join") else None


if __name__ == "__main__":
    import time
    say(" ".join(sys.argv[1:]) or "Hello, I am Blimpy.")
    time.sleep(4)
