"""Offline test of the OMNI realtime client (laptop/voice/omni.py) against tools/omni_mock_server.py. No key needed.

  python tools/omni_test.py

Checks: session.update carries the set_intent tool; frames are held back until the first mic packet (the relay refuses
them before audio), then mic packets and frames reach the server; a tool call reaches
on_intent and its result goes back as function_call_output + response.create; the reply audio reaches the speaker;
say() injects an [EVENT] turn; barge-in flushes playback; half-duplex drops mic packets while Blimpy talks; usage rows
land in the ledger; the client reconnects after the server drops it; a spoken "stop" hovers from the transcription alone;
the mic gate keeps silence off the wire (the relay bills every second sent) and frames only go while someone talks.
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
                   api_key="test-key-1234", url=f"ws://127.0.0.1:{PORT}/v1/realtime", fps=4.0, gate=False, name_gate=False,   # gates tested in 10./11.
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

# 9. a spoken "stop" acts locally from the transcription even when the model only says "Mm-hm."
mock.script.append({"heard": "Blimpy, stop.", "text": "Mm-hm.", "audio_chunks": 2})
n_int = len(intents); talk(om, 40)
check("spoken stop -> hover from the transcription (model said only Mm-hm.)",
      wait_for(lambda: len(intents) > n_int, 3) and intents[-1] == {"intent": "hover"} and om.stats.get("local_stops") == 1,
      f"{intents[n_int:]} heard {om.transcript[-2:]} sessions {mock.stats['sessions']} appends {mock.stats['audio_appends']} ok={om.ok} err={om.last_error}")

check("cost estimate follows the relay's tariff", abs(om.units() - omni.relay_units(**om.cost)) < 1e-9 and om.cost["audio_out_s"] > 1
      and om.cost["audio_in_s"] > 3 and abs(omni.relay_units(audio_in_s=60) - 0.768) < 0.001 and abs(omni.relay_units(audio_out_s=60) - 0.45) < 0.001,
      f"{ {k: round(v, 2) for k, v in om.cost.items()} } -> {om.units():.3f} units; a minute of mic = {omni.relay_units(audio_in_s=60):.3f}, "
      f"a minute of Blimpy talking = {omni.relay_units(audio_out_s=60):.3f}")
om.stop()

# 10. the mic gate: silence never leaves the laptop, speech goes up with a pre-roll and a hangover, frames only while open
PK = omni.MIC_BLOCK * 2                                   # one 40 ms packet
quiet = (np.random.default_rng(1).normal(0, 30, omni.MIC_BLOCK)).astype(np.int16).tobytes()    # ~-60 dBFS room
loud = (np.sin(np.arange(omni.MIC_BLOCK) * 0.3) * 8000).astype(np.int16).tobytes()             # ~-13 dBFS voice
g = omni.MicGate(); t = [0.0]
def step(pcm, n):
    out = []
    for _ in range(n): out += g.process(pcm, now=t[0]); t[0] += 0.04
    return len(out)
n_quiet = step(quiet, 75)                                 # 3 s of room
n_talk = step(loud, 25)                                   # 1 s of talking
n_after = step(quiet, 75)                                 # 3 s of room after: the hangover, then shut
check("gate: 3 s of room noise sends nothing", n_quiet == 0 and not g.open, f"{n_quiet} packets, floor {g.floor:.0f} dB, opens at {g.threshold:.0f}")
check("gate: speech goes up with the pre-roll", n_talk == 25 + 8 and g.opens == 1, f"{n_talk} packets for 25 of speech (8 pre-roll)")
check("gate: shuts 1 s after the last loud packet", 24 <= n_after <= 26 and not g.open, f"{n_after} packets after, open={g.open}")
g2 = omni.MicGate(); t2 = [0.0]
def step2(pcm, n):
    out = 0
    for _ in range(n): out += len(g2.process(pcm, now=t2[0])); t2[0] += 0.04
    return out
step2(quiet, 50); n_fan = step2(loud, 400)                # a fan switched on: 16 s of constant -13 dB
check("gate: a constant loud noise becomes the floor within ~13 s and the gate shuts", not g2.open and 300 < n_fan < 380 and g2.floor > -16,
      f"sent {n_fan} of 400 packets, floor now {g2.floor:.0f} dB, opens at {g2.threshold:.0f}")

mock.script.append({"tool": {"intent": "hover"}})
om2 = omni.OmniLive(on_intent=lambda d: (intents.append(d), "holding")[1], frame_fn=lambda: frame,
                    api_key="test-key-1234", url=f"ws://127.0.0.1:{PORT}/v1/realtime", fps=4.0, gate=True,
                    mic=False, speaker=spk, usage_log=ledger, purpose="omni_test", on_text=lambda w, t: None)
om2.start(timeout=5)
a0, i0 = mock.stats["audio_appends"], mock.stats["images"]
for _ in range(50): om2.feed_audio(quiet); time.sleep(0.005)
time.sleep(0.6)
check("gated client: 2 s of silence -> no audio, no frames on the wire", mock.stats["audio_appends"] == a0 and mock.stats["images"] == i0 and om2.cost["audio_in_s"] == 0,
      f"appends +{mock.stats['audio_appends'] - a0}, images +{mock.stats['images'] - i0}")
for _ in range(40): om2.feed_audio(loud); time.sleep(0.04)
check("gated client: speech -> audio and a frame within the first words", mock.stats["audio_appends"] - a0 >= 40 and mock.stats["images"] > i0,
      f"appends +{mock.stats['audio_appends'] - a0}, images +{mock.stats['images'] - i0}, sent {om2.gate.sent_s:.2f} s")
for _ in range(30): om2.feed_audio(quiet); time.sleep(0.04)     # 1.2 s of room after the words
check("gated client: hangover closes the gate, cost counts only what was sent", not om2.gate.open and abs(om2.cost["audio_in_s"] - om2.gate.sent_s) < 1e-6
      and 2.5 < om2.cost["audio_in_s"] < 3.2, f"sent {om2.cost['audio_in_s']:.2f} s of {om2.gate.total_s:.2f} s -> {om2.units():.4f} units; {om2.mic_status()}")
om2.muted = True; a1 = mock.stats["audio_appends"]
for _ in range(5): om2.feed_audio(loud); time.sleep(0.005)
om2.push_to_talk(); time.sleep(0.05)
for _ in range(5): om2.feed_audio(quiet); time.sleep(0.005)
time.sleep(0.3)
check("press-to-talk: muted sends nothing, one press sends everything past mute and gate", mock.stats["audio_appends"] == a1 + 5 and om2.ptt and "PTT" in om2.mic_status(),
      f"appends +{mock.stats['audio_appends'] - a1}, {om2.mic_status()}")
om2._ptt_until = 0.0; om2.muted = False; om2._ptt_turn = False

# 11. the name gate: a turn without "Blimpy" in its transcript is someone else talking -> reply cancelled, command dropped
def speak():
    """Exactly one mock turn: its VAD starts after 25 appends and stops 10 later; 40 appends leaves no room for a second."""
    for _ in range(30): om2.feed_audio(loud); time.sleep(0.02)
    for _ in range(10): om2.feed_audio(quiet); time.sleep(0.02)
    time.sleep(0.3)
mock.script.append({"heard": "hey guys come look at this thing", "tool": {"intent": "wander"}})
om2.t_last_reply = 0.0; n_int = len(intents); n_out = len(mock.stats["tool_outputs"]); ign0 = om2.stats.get("ignored", 0); speak()
wait_for(lambda: len(mock.stats["tool_outputs"]) > n_out, 4)
check("name gate: a stranger's command is dropped (tool answered 'ignored', nothing executed)", len(intents) == n_int and om2.stats.get("ignored") == ign0 + 1
      and any("ignored" in o["output"] for o in mock.stats["tool_outputs"][n_out:]) and ("ignored", "hey guys come look at this thing") in om2.transcript,
      f"intents +{len(intents) - n_int}, ignored {om2.stats.get('ignored')}, {om2.mic_status()}")
mock.script.append({"heard": "Blippi, wander around a bit", "tool": {"intent": "wander"}})
speak()
check("name gate: the name (even misspelt by the recogniser) gets through", wait_for(lambda: len(intents) > n_int, 4) and intents[-1] == {"intent": "wander"},
      f"{intents[n_int:]}")
mock.script.append({"heard": "actually turn left", "tool": {"intent": "rotate", "degrees": 90}})
om2.t_last_reply = time.monotonic(); n_int = len(intents); speak()      # right after Blimpy spoke: a follow-up, no name needed
check("name gate: a follow-up right after Blimpy's reply counts without the name", wait_for(lambda: len(intents) > n_int, 4) and intents[-1].get("intent") == "rotate",
      f"{intents[n_int:]}")
mock.script.append({"heard": "everyone stop", "text": "Mm-hm.", "audio_chunks": 1})
om2.t_last_reply = 0.0; n_int = len(intents); speak()
check("name gate: a stop word always acts", wait_for(lambda: len(intents) > n_int, 4) and intents[-1] == {"intent": "hover"}, f"{intents[n_int:]}")
om2.stop(); stop()
n_fail = sum(not v for v in results.values())
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
sys.exit(1 if n_fail else 0)
