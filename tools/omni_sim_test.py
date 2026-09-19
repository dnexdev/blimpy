"""End-to-end rehearsal WITHOUT the robot, with the REAL cloud voice: spoken commands (wav files) -> Qwen3.5-Omni realtime
-> set_intent tool calls -> behaviors -> commands -> BLE bridge -> simulated gondola (eye on the balloon + ultrasonic)
-> state / telemetry back -> the same loop. Exactly the pilot's chain, minus the keyboard, the mic and the speakers.

  set YIBU_API_KEY, then:   python tools/omni_sim_test.py [--relative]        (~2 min, ~8 cloud calls, purpose sim_rehearsal)

Checks: "Blimpy, follow me" -> FOLLOW and the sim balloon settles about 1.5 m from the person, facing them (the eye);
"turn left ninety degrees" -> ROTATE and the gyro yaw grows by ~90 deg; "stop and hover" -> HOVER with quiet motors;
a six-second timer set by voice is announced by the cloud voice ([EVENT] path). --relative: the sim has no room camera.
"""
import argparse, json, math, os, pathlib, statistics, subprocess, sys, threading, time, wave
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop import config
from laptop.control.behaviors import Behaviors
from laptop.control.estimator import StateEstimator
from laptop.control.link import TelemWatchdog
from laptop.control.protocol import CMD_PORT, STATE_PORT, TELEM_PORT, UdpJson, make_cmd, now_ms, wrap
from laptop.voice import omni

G = config.FOLLOW
LOCAL = ("127.0.0.1", CMD_PORT)
PHRASES = {"follow": "Blimpy, follow me.", "rotate": "Blimpy, turn left ninety degrees.", "stop": "Blimpy, stop and hover.",
           "timer": "Blimpy, set a timer for six seconds."}
results = {}


def check(name, ok, detail=""):
    results[name] = bool(ok); print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def make_wavs(d):
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
    def __init__(self): self.chunks = []
    def write(self, pcm): self.chunks.append(pcm)
    def flush(self): self.chunks.clear()
    def busy(self, tail_s=0.3): return False
    def close(self): pass


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--relative", action="store_true"); ap.add_argument("--wav-dir", default="data/tts_cache")
a = ap.parse_args()
if not (os.environ.get("OMNI_API_KEY") or os.environ.get("YIBU_API_KEY")): sys.exit("set YIBU_API_KEY first")
make_wavs(a.wav_dir)
pcm = {k: wav_pcm(os.path.join(a.wav_dir, k + ".wav")) for k in PHRASES}

# ---- the simulated gondola behind the BLE bridge (eye + ultrasonic on by default)
sim = [sys.executable, "-m", "laptop.control.ble_gondola", "--fake", "--sim", "--no-http", "--person", "static", "--psi0", "0.3"]
if a.relative: sim.append("--no-vision")
proc = subprocess.Popen(sim, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(2.0)

# ---- the pilot's loop, headless
state_in, tel_in, cmd_out = UdpJson(STATE_PORT), UdpJson(TELEM_PORT), UdpJson()
est = StateEstimator(alpha=G["POS_ALPHA"], beta=G["VEL_BETA"])
wd = TelemWatchdog()
spoken, pending, said = [], [], []
beh = Behaviors(lambda s: spoken.append(s))
lock = threading.Lock()


def on_intent(it):                      # websocket thread -> main loop (like pilot.omni_intent)
    done, box = threading.Event(), {}
    with lock: pending.append((it, done, box))
    return (box.get("reply") or "done") if done.wait(1.5) else "queued"


spk = FakeSpeaker()
om = omni.OmniLive(on_intent=on_intent, frame_fn=None, mic=False, speaker=spk, half_duplex=False, purpose="sim_rehearsal",
                   on_text=lambda w, t: said.append((w, t)))
try: om.start(timeout=20)
except Exception as e:
    proc.terminate(); sys.exit(f"[sim] no cloud session: {e}")
print(f"[sim] cloud session up ({om.model}); simulator {'relative (no room camera)' if a.relative else 'eye + room camera'}")

armed, t_balloon, t_eye, last_fpv = False, 0.0, None, None
snap = []                               # (t, mode, dist, yaw, effort, note)
stop_loop = False


def loop():
    global armed, t_balloon, last_fpv
    while not stop_loop:
        t, now = time.monotonic(), now_ms()
        for s, _ in state_in.recv_all(only_from="127.0.0.1"):
            if s.get("balloon"): est.update_balloon(s["balloon"], s.get("t", now)); t_balloon = now
            if s.get("person"): beh.on_person(s["person"], s.get("t"))
            if s.get("fpv"): beh.on_fpv(s["fpv"], s.get("t")); last_fpv = s["fpv"]
        yaw = None
        for m, _ in tel_in.recv_all(only_from="127.0.0.1"):
            est.update_telem(m.get("yaw", 0.0), m.get("yr", 0.0), m.get("alt"), m.get("t"), now=t); yaw = m.get("yaw"); wd.telem(m, t, armed)
            effort = abs(m.get("mL", 0)) + abs(m.get("mR", 0)) + abs(m.get("mS", 0)) + abs(m.get("mV", 0))
        if a.relative and est.p is None: est.seed_relative()
        with lock:
            while pending:
                it, done, box = pending.pop(0)
                box["reply"] = beh.handle(it, est) or "done"; done.set()
                print(f"\n[sim] set_intent {it} -> mode {beh.mode}, {box['reply']!r}")
        if beh.timers or beh.pomo: pass
        for s in spoken[:]:                                  # proactive lines go out through the cloud voice
            spoken.remove(s); om.say(s); print(f"\n[sim] event -> cloud: {s!r}")
        if not armed and est.p is not None: armed = True; wd.arm(t); beh.on_armed(est)
        vf = vs = yr = vz = 0.0; note = "safe"
        if armed:
            reason = wd.check(armed, t)
            lost = (not est.alt_ok) if est.rel else (est.p is None or now - t_balloon > G["BALLOON_LOST_MS"])
            if reason or lost: armed, note = False, reason or "lost"
            else: vf, vs, yr, vz, note = beh.step(est)
        est.observe_motion(vf, vs, vz)
        cmd_out.send(make_cmd(vf, yr, vz, armed, vs), LOCAL)
        d = None
        if last_fpv and last_fpv.get("range"): d = last_fpv["range"]
        elif beh.person is not None and est.p is not None and not est.rel: d = math.hypot(beh.person[0] - est.p[0], beh.person[1] - est.p[1])
        if yaw is not None: snap.append((t, beh.mode, d, yaw, locals().get("effort", 0.0), note))
        time.sleep(max(0.0, 1 / G["HZ"] - (time.monotonic() - t)))


th = threading.Thread(target=loop, daemon=True); th.start()


def feed(name, pace=0.04, tail_s=1.0):
    data = pcm[name] + bytes(int(16000 * tail_s) * 2)
    for i in range(0, len(data), 1280):
        om.feed_audio(data[i:i + 1280]); time.sleep(pace)


def wait_for(cond, timeout, step=0.1):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond(): return True
        time.sleep(step)
    return cond()


def since(t0): return [s for s in snap if s[0] >= t0]


t0 = time.monotonic()
check("armed on the simulated gondola", wait_for(lambda: armed, 10), f"{'relative' if est.rel else 'world'} p={est.p}")

# 1. follow me
feed("follow")
check("'follow me' -> FOLLOW (tool call)", wait_for(lambda: beh.mode == "FOLLOW", 20), f"mode {beh.mode}")
t1 = time.monotonic(); time.sleep(30)
rows = [s for s in since(t1 + 20) if s[2] is not None]
dist = statistics.mean(r[2] for r in rows) if rows else float("nan")
check("balloon settles ~1.5 m from the person", rows and abs(dist - G["D_FOLLOW"]) < 0.4, f"dist mean {dist:.2f} over the last 10 s ({len(rows)} samples)")
check("eye sees the person (facing them)", last_fpv is not None and abs(last_fpv["bearing"]) < 0.3, f"last eye bearing {last_fpv and round(math.degrees(last_fpv['bearing']))} deg")
check("cloud confirmed in words", any(w == "blimpy" for w, _ in said), f"{[t for w, t in said if w == 'blimpy'][-1:]!r}")

# 2. turn left ninety degrees
yaw0 = snap[-1][3]; n_said = len(said)
feed("rotate")
check("'turn left ninety degrees' -> ROTATE", wait_for(lambda: beh.mode == "ROTATE", 20), f"mode {beh.mode}")
wait_for(lambda: beh.mode != "ROTATE", 25)
turned = math.degrees(wrap(snap[-1][3] - yaw0))
check("gyro yaw grew by ~90 deg", 70 < turned < 110, f"{turned:.0f} deg, mode now {beh.mode}")

# 3. stop and hover
feed("stop")
check("'stop and hover' -> HOVER", wait_for(lambda: beh.mode == "HOVER", 20), f"mode {beh.mode}")
time.sleep(6)
eff = [s[4] for s in since(time.monotonic() - 4)]
check("motors quiet in HOVER", eff and statistics.mean(eff) < 0.6, f"mean effort {statistics.mean(eff) if eff else 'nan'}")

# 4. a timer, announced by the cloud voice
n_said = len(said)
feed("timer")
check("'set a timer for six seconds' -> timer", wait_for(lambda: beh.timers, 20), f"timers {[(round(d - beh.now(), 1), m) for d, m in beh.timers]}")
wait_for(lambda: not beh.timers, 15)                      # the timer fires -> behaviors speaks -> om.say([EVENT]) -> the cloud says it
n_fire = len(said)
check("timer announced through the cloud ([EVENT])", wait_for(lambda: any(w == "blimpy" and any(k in t.lower() for k in ("time", "timer", "done", "up")) for w, t in said[n_fire:]), 30),
      f"{[t for w, t in said[n_fire:] if w == 'blimpy']!r}")

stop_loop = True; time.sleep(0.3)
for _ in range(3): cmd_out.send(make_cmd(0, 0, 0, False), LOCAL); time.sleep(0.02)
om.stop(); proc.terminate()
n_fail = sum(not v for v in results.values())
print(f"\n{len(results) - n_fail}/{len(results)} checks passed in {time.monotonic() - t0:.0f} s   (cloud responses {om.stats['responses']}, last error {om.last_error})")
sys.exit(1 if n_fail else 0)
