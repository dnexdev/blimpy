"""Offline test of the OMNI realtime client (laptop/voice/omni.py) against tools/omni_mock_server.py. No key needed.

  python tools/omni_test.py

Checks: session.update carries the set_intent tool; frames are held back until the first mic packet (the relay refuses
them before audio), then mic packets and frames reach the server; a tool call reaches
on_intent and its result goes back as function_call_output + response.create; the reply audio reaches the speaker;
say() injects an [EVENT] turn; barge-in flushes playback; half-duplex drops mic packets while Blimpy talks; usage rows
land in the ledger; the client reconnects after the server drops it.
"""
import json, os, pathlib, sys, tempfile, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
import numpy as np
from laptop.voice import omni, usage_log
from tools.omni_mock_server import Mock

PORT = 8765 + os.getpid() % 100
ledger = os.path.join(tempfile.mkdtemp(), "usage.jsonl")


class FakeSpeaker:
    def __init__(self): self.chunks = []; self.flushes = 0; self.force_busy = False
    def write(self, pcm): self.chunks.append(pcm)
    def flush(self): self.flushes += 1; self.chunks.clear()
    def busy(self, tail_s=0.3): return self.force_busy
    def close(self): pass


def wait_for(cond, timeout=5.0, step=0.05):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond(): return True
        time.sleep(step)
    return cond()


def talk(om, packets=40):
    """Feed `packets` x 40 ms of fake mic audio (silence is fine: the mock's VAD counts packets)."""
    for _ in range(packets):
        om.feed_audio(bytes(omni.MIC_BLOCK * 2)); time.sleep(0.005)


results = {}
def check(name, ok, detail=""):
    results[name] = bool(ok); print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


mock = Mock(script=[{"tool": {"intent": "rotate", "degrees": -90, "target": None}},
                    {"text": "long story", "audio_chunks": 40, "chunk_ms": 100}])
th, stop = mock.serve_in_thread(PORT)
intents = []
spk = FakeSpeaker()
frame = np.zeros((720, 1280, 3), np.uint8); frame[100:600, 200:900] = (200, 30, 200)
om = omni.OmniLive(on_intent=lambda d: (intents.append(d), "turning right")[1], frame_fn=lambda: frame,
                   api_key="test-key-1234", url=f"ws://127.0.0.1:{PORT}/v1/realtime", fps=4.0,
                   mic=False, speaker=spk, usage_log=ledger, purpose="omni_test", on_text=lambda w, t: None)
t0 = time.monotonic(); om.start(timeout=5); t_conn = time.monotonic() - t0
print(f"[test] session up in {t_conn:.2f}s")

# 1. session.update content
su = mock.stats["session_updates"][0]
check("session.update has set_intent tool", su.get("tools") and su["tools"][0]["name"] == "set_intent", str(su.get("modalities")))
check("session.update has VAD + audio formats", su.get("turn_detection") and su.get("input_audio_format"), str(su.get("turn_detection")))

# 2. frames wait for audio, then stream at fps
time.sleep(1.0)
check("no frames before the first mic packet", om.stats["img_out"] == 0 and mock.stats["images"] == 0 and mock.stats["images_refused"] == 0,
      f"sent {om.stats['img_out']} refused {mock.stats['images_refused']}")
talk(om, 3)
time.sleep(1.3)
check("frames stream (~4 fps) once audio has been appended", 3 <= mock.stats["images"] <= 8 and mock.stats["images_refused"] == 0,
      f"{mock.stats['images']} images in 1.3 s, refused {mock.stats['images_refused']}")
b64 = omni.encode_jpeg(frame)
check("jpeg under provider cap", len(b64) <= omni.JPEG_MAX_B, f"{len(b64)} chars")

# 3. a spoken command -> tool call -> on_intent -> function_call_output -> reply audio
talk(om, 40)
check("tool call reached on_intent", wait_for(lambda: len(intents) == 1) and intents[0] == {"intent": "rotate", "degrees": -90},
      str(intents))
check("input transcription reaches the transcript", wait_for(lambda: ("you", "mock transcript") in om.transcript, 3))
ok = wait_for(lambda: mock.stats["tool_outputs"])
out = json.loads(mock.stats["tool_outputs"][0]["output"]) if ok else {}
check("function_call_output returned", ok and out.get("result") == "turning right", str(out))
check("response.create after tool output", wait_for(lambda: mock.stats["response_creates"] >= 1))
check("reply audio reached speaker", wait_for(lambda: len(spk.chunks) >= 3, 3), f"{len(spk.chunks)} chunks")
check("no duplicate tool call (done + output_item)", len(intents) == 1 and len(mock.stats["tool_outputs"]) == 1)

# 4. proactive say()
n0 = mock.stats["response_creates"]
check("say() sends [EVENT] + response.create", om.say("Time's up. Your five minute timer is done.") and
      wait_for(lambda: mock.stats["response_creates"] > n0 and any("[EVENT]" in json.dumps(m) for m in mock.stats["messages"])))
wait_for(lambda: om.stats["responses"] >= 2, 3)

# 5. barge-in: long reply, user talks over it -> speaker flushed, reply cancelled
spk.chunks.clear(); f0 = spk.flushes
talk(om, 40)                                          # triggers the second scripted turn (40 chunks = 4 s)
wait_for(lambda: len(spk.chunks) >= 5, 3)
talk(om, 30)                                          # speak over it
check("barge-in flushes playback", wait_for(lambda: spk.flushes > f0, 3), f"flushes {spk.flushes - f0}")

# 6. half duplex: mic dropped while Blimpy is talking
a0 = om.stats["audio_in"]; spk.force_busy = True; talk(om, 5); spk.force_busy = False
check("half-duplex drops mic while speaking", om.stats["audio_in"] == a0)

# 7. ledger
wait_for(lambda: om.stats["responses"] >= 3, 5)
rows = [json.loads(l) for l in open(ledger)] if os.path.exists(ledger) else []
check("usage ledger rows (organisers' schema)", len(rows) >= 3 and rows[0]["key_suffix"] == "...1234" and rows[0]["input_tokens"] == 500
      and rows[0]["schema_version"] == "yibu_call_audit_v1" and rows[0]["ok"] is True and rows[0]["usage_reported"] is True
      and rows[0]["status_code"] == 101 and rows[0]["endpoint"].startswith("ws://"),
      f"{len(rows)} rows, first {rows[0] if rows else None}")
check("cancelled reply logged with usage null (unknown, not zero)",
      any(r["ok"] and not r["usage_reported"] and r["input_tokens"] is None for r in rows), str([(r['ok'], r['usage_reported']) for r in rows]))
check("ledger never stores the key", "test-key-1234" not in open(ledger).read())
check("no frame ever refused (frames wait for audio after each commit)", mock.stats["images_refused"] == 0 and om.stats.get("img_refused", 0) == 0,
      f"mock refused {mock.stats['images_refused']}, client saw {om.stats.get('img_refused', 0)}")

# 8. reconnect after the server drops us
om.send({"type": "mock.close"})
check("reconnects after drop", wait_for(lambda: mock.stats["sessions"] >= 2 and om.ok, 8),
      f"sessions {mock.stats['sessions']} ok={om.ok} err={om.last_error}")

om.stop(); stop()
n_fail = sum(not v for v in results.values())
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
sys.exit(1 if n_fail else 0)
