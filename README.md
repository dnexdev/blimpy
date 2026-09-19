# Blimpy

A near-silent helium-balloon robot that follows you around. The gondola is a Bluetooth motor box (the hardware team's
firmware: per-motor percentages in, IMU lines out); everything clever (mixer, failsafe, vision, control, voice) runs
on the laptop. `laptop/control/ble_gondola.py` is the bridge to the gondola; everything above it speaks
[PROTOCOL.md](PROTOCOL.md). **Read that first.**

```
firmware/            legacy ESP32-C3 WiFi build (PlatformIO): mixer + failsafe + IMU on the board. Not what flies now (README 3).
laptop/config.py     one place for sources, IPs, gains, vehicle physics (PHYS); room geometry is loaded from venues/default.json
laptop/control/      ble_gondola.py (Bluetooth bridge to the gondola: mixer + failsafe + telemetry) · imu_store.py (IMU samples for everyone, http :5008)
                     protocol.py (shared mixer) · fake_esp32.py (stand-in + simulator) · teleop.py · follow_me.py · pilot.py · behaviors.py · estimator.py · link.py (telemetry watchdog)
laptop/vision/       streams.py · calib_io.py · triangulate.py · detect.py (YOLO) · localize.py · mono.py (1 cam) · preflight.py · train_balloon.py
laptop/positioning/  schema.py · session.py (record/load) · evaluate.py · sources.py (udp/replay/sim) · fuse.py (stub) · venue.py · record.py · replay.py · capture_place.py
laptop/sim/          world.py (balloon physics + every sensor model) · plot.py (live top-down view)
laptop/voice/        stt.py (whisper) · intent.py (local LLM -> JSON intents) · tts.py · omni.py (Qwen3.5-Omni realtime: ears/eyes/mouth, OMNI Live track) · omni_watch.py (focus watcher) · usage_log.py
tools/calib/         make_targets.py · intrinsics.py · extrinsics.py · triangulate_test.py · record_clips.py
tools/dataset/       build_balloon_dataset.py · label_site.py · extract_frames.py
tools/               scenarios.py · sim_test.py (--ble) · ble_test.py · behaviors_test.py · control_test.py · vision_test.py · positioning_test.py · intent_test.py
                     webcam_test.py · balloon_eval.py · vision_check.py (go/no-go) · positioning_eval.py (sessions vs truth)
calib/               <name>_intrinsics.npz (once) and <name>_extrinsics.npz (every placement)
venues/              default.json: the room (arena, obstacles, places). Committed; config.py exports it as ARENA / OBSTACLES / JUDGES_XY
data/positioning/    recorded sessions (gitignored): <ts>_<tag>[_name]/session.json + log.jsonl
```

## 0. Setup (laptop, Windows PowerShell)

```powershell
python -m venv .venv; .\.venv\Scripts\Activate.ps1
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128   # RTX 5080 needs cu128+
pip install -r requirements.txt
```
Gondola link (Bluetooth LE): the gondola advertises as `BalloonRobot`. Windows Bluetooth on, then:
```powershell
python -m laptop.control.ble_gondola      # scan, connect, bridge udp 5005/5006 <-> BLE. Leave it running (Ctrl+C = STOP)
python -m laptop.control.teleop           # second terminal: SPACE arm, w/s a/d q/e j/l, k = kill. pilot / follow_me the same way
```
`http://127.0.0.1:5008/imu` (latest IMU sample), `/imu/history?n=200`, `/status` while the bridge runs; the same data in
Python via `laptop.control.imu_store`. Motor letters, signs and the IMU layout are set once on the bench (section 3).
Legacy WiFi firmware (`firmware/`, PlatformIO, `cd firmware && pio run`) still works: pass `--esp wisp-xxxx.local`.

**Network rule:** phones and Pis join the *laptop's hotspot*, never the hackathon WiFi. The gondola is on Bluetooth.

## 1. Track B — protocol works with no hardware (5 minutes)

Terminal 1 and 2 (from the repo root):
```powershell
python -m laptop.control.fake_esp32
python -m laptop.control.teleop
```
Press SPACE to arm, `w` a few times, watch the fake motors ramp; kill teleop with Ctrl+C and watch the
fake disarm within 0.5 s. That is the failsafe working.

The same thing through the Bluetooth bridge on its simulated robot: `python -m laptop.control.ble_gondola --fake --sim`
instead of `fake_esp32` (teleop / follow_me / pilot do not know the difference). `python tools/ble_test.py` (12 s, no
hardware) checks the bridge itself: udp command -> mixer -> `MOTORS` percentages, IMU lines -> telemetry, the 500 ms
STOP, the IMU store and its HTTP endpoints, recovery after a link drop. `python tools/sim_test.py --ble` is the control
regression over the bridge.

Regression test of the whole control stack (failsafe timing + follow-me vs a walking person, ~50 s, no hardware):
```powershell
python tools/sim_test.py
```

### What the simulator models (and what it taught us)
`laptop/sim/world.py` is force-based, built from the parts list (constants at the top of `class Balloon`), and
`fake_esp32.py --sim` runs it in real time behind the real UDP protocol. **Realism is on by default** (`REAL`):
120 ms vision latency, 2-4 cm position noise, person dropouts and false detections, 2 % packet loss, gyro bias
after boot calibration, HVAC gusts, lift drifting with room temperature, mismatched motors, the sideways motor
a few cm off-centre, walls and obstacles (`venues/default.json`, exported by `laptop/config.py` as ARENA / OBSTACLES). `--ideal` turns all of it off.
- **Effective mass ~0.83 kg**, not 0.3: a sphere drags half its displaced air along ("added mass").
- **Thrust ~ duty²**: 8520 + 75 mm prop ~50 gf flat out, ~12 gf at the 0.5 cap; motors below ~6 % duty do nothing;
  a fixed prop in **reverse gives ~60 %**. Two motors -> top speed ~0.95 m/s, but 0 to 0.3 m/s takes ~3 s.
- **Quadratic drag**: from 0.3 m/s with motors off it still coasts >2 m in 10 s. Drag does NOT stop it; thrust does.
- **No keel**: heading and velocity are independent. That is why there are **four motors, all reversible**:
  L + R (rear, forward axis: drive, brake, yaw), **S sideways through the centre**, **V vertical** (lift changes by
  grams as the room warms; ballast alone hits the ceiling within a minute). See PROTOCOL.md section 6.
- **Gondola tilt**: horizontal thrust swings the gondola ~4 deg, adding ~1 gf of lift at full push.
- **Yaw**: 25 cm motor spacing, ~0.025 kg m² inertia, ~no damping -> mixer yaw-rate PI loop (P 1.0, I 1.0/s).
The vehicle numbers (`D`, `T_MAX`, `REV_EFF`, `ARM_BELOW`, `TOF_BELOW`, `M_GONDOLA`, `MOTOR_SPACING`) live in `laptop/config.py` PHYS,
shared by the simulator and the estimator; measure them on the bench (section 3b) and rerun the tests.

### How the control stack copes with all that (laptop/control)
- **Estimator** (`estimator.py`): alpha-beta filter whose prediction uses the commanded thrust, so it is smooth
  without lagging our own moves. **Heading** = gyro yaw + an offset learned from how the balloon *responds* to a
  push (commanded thrust direction vs. measured velocity change), so it works from any manoeuvre and is immune to
  a steady draught; only strong, single-axis pushes from rest are trusted, and the gyro bias is fitted from the drift
  of that offset between pushes and fed forward.
- **Heading acquisition** on first arm (`behaviors.py::_acquire`): short body-axis pushes with a probe/commit/brake
  pattern that never brakes with the (unknown) heading, never coasts, and reverses when the room ahead along the
  measured velocity gets small. A **twitch** (2 s, 0.35 m/s) re-teaches the heading every ~40 s of quiet.
- **Controller** (`follow_me.py`): holonomic velocity tracking with vector saturation (braking keeps its direction),
  reverse thrust compensated, approach speed limited by a braking parabola, the person's walking velocity fed
  forward, a sideways dodge when someone walks straight at it, walls/obstacles avoided by capping the closing speed
  (judged 1.2 s ahead) and sliding around with a committed side; hover is a **position** hold; height is P+I+D
  with the integrator learning the ballast trim.
- **Behaviours**: follow, hover, go-to (with detours when wedged and "as close as I can get"), wander, rotate
  (bias-compensated, PI-rate), dance, timers, pomodoro, focus guard, altitude offsets.

### Scenario suite (run this after touching anything in laptop/control or the gains)
```powershell
python tools/scenarios.py                 # 20 scenarios, ~3 s, headless, 100x real time, scored against truth
python tools/scenarios.py --seeds 5       # repeat with 5 random seeds, worst case shown
python tools/scenarios.py --plot go_to    # sim_out/<name>.png: trajectory, height, heading error, distance
python tools/scenarios.py --tof           # ToF altimeter on in every scenario (see the note below)
```
Scenarios: failsafe, follow (ideal / real noise / walking / demo route with a pillar), hover in gusts, rotate
90/-180/360, go to the judges and home past a pillar, come here, altitude commands, lift drift, wrong initial
heading, 4x gyro drift, long hover then follow, wander, lossy links, dance, ToF with bad vision z, no ToF, board-side
failsafe (WiFi hiccup). All pass across seeds 0-4. Known fragility: `go_to` / `wrong_initial_heading` / `follow_route` are
seed-marginal (main fails some of seeds 5-9, and `--tof` flips `go_to` seed 0 into a failed detour because the V motor's tilt
term leaks a little thrust into xy). The scenario suite therefore keeps the ToF OFF except in the ToF scenarios, so seeded
runs stay bit-identical; `fake_esp32 --sim` has it ON (the real gondola does). Height with the ToF: 2 cm rms vs 15 cm
with 25 cm vision noise, and the V motor stops thrashing.

Offline vision regression (synthetic two-camera rig, AprilTag pose round-trip, localize tick with fake detectors;
no cameras, no YOLO, ~2 s). Run it after touching `laptop/vision/*` or `tools/calib/*`:
```powershell
python tools/vision_test.py
python tools/control_test.py     # mixer twin of the firmware test (yaw-rate integrator), ToF altitude path, telemetry watchdog (~1 s)
```

### Tune follow-me in the simulator
```powershell
python -m laptop.control.fake_esp32 --sim --plot          # add --person route | static | random, --ideal, --gusts 0.1
python -m laptop.control.follow_me
```
SPACE arms. The balloon learns its heading with a few short pushes (gyro yaw starts at an unknown offset),
then turns to face the walking person and holds ~1.5 m from them. Edit gains in `laptop/config.py` FOLLOW.

## 1b. Voice + behaviors (laptop only)

Speech -> `laptop/voice/stt.py` (whisper) -> `laptop/voice/intent.py` (local LLM via Ollama, strict JSON) ->
`laptop/control/behaviors.py` (state machine: follow, hover, wander, go_to, rotate, altitude, timer, pomodoro,
focus guard, dance) -> same vf/yr/vz commands as follow_me. `laptop/control/pilot.py` ties it together.

One-time: install Ollama, then `ollama pull qwen2.5:7b` (pinned in GPU memory by pilot.py; short phrases like
"follow me" / "stop" / "come here" skip the model entirely). **Ollama must be (re)started after the GPU is enabled**
or it silently runs on the CPU (`ollama ps` must show 100% GPU). `python tools/intent_test.py --models qwen2.5:3b qwen2.5:7b`
scores 35 phrases: 7b = 35/35 at 0.8 s median on the 5080, 3b = 31/35 at 0.5 s (use `BLIMPY_LLM=qwen2.5:3b` on CPU).
Whisper runs `large-v3-turbo` on the GPU (0.3 s per sentence) and falls back to `base.en` on CPU. Then:
```powershell
python -m laptop.control.fake_esp32 --sim --plot
python -m laptop.control.pilot            # SPACE arm, v = push-to-talk, t = type a command
python tools/behaviors_test.py            # headless regression of the state machine
```
Places (judges spot, obstacles, optional wander box) are in `venues/default.json`; measure them once the cameras are
calibrated: `python -m laptop.positioning.capture_place judges --xy X Y` (tape measure) or `--from person` (vision). See 1c.

## 1d. OMNI Live (Huawei track): cloud ears, eyes and mouth

Same robot, same behaviors, one more voice mode. `laptop/voice/omni.py` holds ONE Qwen3.5-Omni **realtime** session
(WebSocket on the sponsor's yibuapi relay): the laptop mic streams up at 16 kHz, the desk camera streams up at 1 fps,
the model speaks back at 24 kHz through the laptop speakers, and every command becomes a `set_intent(...)` tool call
that lands in the same `Behaviors.handle` as the local stack. `laptop/voice/omni_watch.py` is the second use of the
model: while the focus guard is on it shows `qwen3.5-omni-flash` a frame every 6 s and asks "what is the user doing";
two looks in a row of "not working" -> the nag, phrased from what it saw ("I can see you're scrolling on a phone").
Positioning, control and safety never touch the cloud: no internet = local whisper/ollama/pyttsx3 take over.

```powershell
$env:YIBU_API_KEY = "sk-..."              # the team key from the organisers' e-mail (OMNI_API_KEY also works). Never commit it.
python tools/omni_test.py                 # offline: 18 checks against tools/omni_mock_server.py (no key, no internet)
python tools/omni_live_test.py            # LIVE: 13 checks on the real relay in ~90 s (spoken commands from wav files, no mic)
python -m laptop.voice.omni               # live: mic + laptop webcam, prints tool calls and the transcript (Ctrl+C)
python -m laptop.voice.omni_watch --image me.jpg    # one look: {"present","working","phone","activity"}
python -m laptop.control.ble_gondola --fake --sim   # (or the real gondola over Bluetooth, section 1b)
python -m laptop.control.pilot            # omni mode is the default once the key is set; --voice local to compare
python -m laptop.voice.usage_log          # token totals per model/purpose from data/omni_usage.jsonl
python tools/omni_report.py               # the two files + e-mail text the organisers want back
```
The key is a per-user environment variable on this laptop (`setx YIBU_API_KEY ...` was run once); every NEW PowerShell
window has it. It expires 2026-09-20 08:00 EDT.

Knobs: `config.OMNI` (CAMERA = what Blimpy sees, FPS, WATCH_S, HALF_DUPLEX), env `OMNI_MODEL` (default
`qwen3.5-omni-plus-realtime`), `OMNI_URL`, `OMNI_VOICE` (default `Tina`), `OMNI_AUDIO_FMT` (default `pcm`).
**HALF_DUPLEX=True** mutes the cloud mic while Blimpy is talking (laptop speakers + laptop mic have no echo cancellation,
the model would hear itself); with headphones set it False and you can interrupt Blimpy mid-sentence. `m` in the pilot
mutes the cloud mic.

**Models** (the key enables qwen3.5-omni-flash / -plus / -plus-realtime, qwen3.8-omni-flash, gemini-3.1-flash-live-preview):
the voice loop uses `qwen3.5-omni-plus-realtime`, the only Qwen realtime model (Gemini Live is a different protocol).
The focus watcher uses `qwen3.5-omni-flash`: measured live on the same frame, flash answered in 1.2-1.4 s with valid JSON
every time, plus was as accurate at 1.4-3.4 s, qwen3.8-omni-flash took 3-7 s and disagreed with itself.

**Windows gives a webcam to one process only**: if `localize.py`/`mono.py` runs on this laptop with source "0",
give the pilot another camera (`--omni-cam 1`, a DroidCam URL, or `none` for ears-only).

**Sponsor reporting** (due 2026-09-20 23:59 EDT, reply to the key e-mail): every cloud call is appended to
`data/omni_usage.jsonl` through the organisers' own `yibu_audit.append_audit_record` (unmodified copy in `tools/yibu/`,
schema `yibu_call_audit_v1`: model, purpose, tokens, latency, key suffix only; never prompts/audio/images). Realtime
sessions log one row per response (its `response.done` usage; null when barge-in cancelled the reply) plus one failed
row per connection that never reached a session; focus-watch HTTP calls one row each. `python tools/omni_report.py`
runs their `summarize_usage.py` and writes `data/omni_report/usage_summary.json` + `usage_by_model_key_purpose.csv`
and prints the e-mail text (team, project link, period, key suffix, how calls were logged and the gaps). Calls made
with their example scripts land in their own `artifacts/yibu_api_calls.jsonl`; merge with `--extra-log`.

**Verified live on the relay** (2026-09-19, `tools/omni_live_test.py`): audio format `pcm` both ways; voice `Tina`
(`Cherry` is rejected and the server drops the socket); `semantic_vad` accepted; the server adds input transcription
(`qwen3-asr-flash-realtime`) and returns it as `conversation.item.input_audio_transcription.completed`; a tool call
arrives as `response.function_call_arguments.done` and `response.output_item.done` with the same call_id (handled once);
an image is refused until audio has been appended in the session, so frames wait for the first mic packet;
`response.create` is refused while a response is running, so tool results and `[EVENT]` turns queue until
`response.done`; a reply cut off by barge-in comes back `cancelled` with usage null. Latency: text turn to first audio
1.1-1.5 s; end of a spoken command to tool call to first confirmation audio 1.5 s.

## 1e. Cameras: one eye ON the balloon, one in the room

Hardware on hand: one Arduino/ESP32 camera, one Pi camera module, one ultrasonic sensor. The plan:

| where | what | gives | code |
|---|---|---|---|
| **on the gondola, looking forward, tilted ~15 deg down** | the Arduino camera (ESP32-CAM style, MJPEG over the hotspot) | Blimpy's own **eye**: what it sees in conversation (OMNI), and the person to follow: **bearing straight from the image** (no heading calibration, no learning push) + range from the person's height in the frame | `laptop/vision/fpv.py`, `config.FPV`, `pilot --fpv URL` |
| **on the gondola, looking down** | the ultrasonic | height above the floor -> altitude hold without any camera. The firmware prints it in the IMU line (`alt=87.3`, cm); the bridge forwards it as telemetry `alt` | `config.BLE ALT_KEYS / ALT_UNITS`, `estimator.update_telem` |
| **in the room, on a tripod / table, ~1.8 m up** | the Pi camera (needs a Pi streaming `rpicam-vid` to `udp://@:5000`), or the laptop webcam, or a phone (DroidCam) | x/y/z of the balloon and the person in the room: hover-in-place, go_to judges, wander, wall avoidance | `laptop/vision/mono.py` (one camera), `localize.py` (two) |

Why not two in the room: the ESP32-CAM is low-res and 250 ms late, poor for triangulation, and the eye is the better
OMNI story. Why not two on the balloon: weight. **No room camera at all?** `pilot --relative`: the eye + ultrasonic fly
FOLLOW, ROTATE and HOVER-still; GO_TO and WANDER are refused ("I can't see the room from up here"). Nothing senses walls
in that mode: the person leads, keep 1 m off the walls.

Offline, all of it runs against the simulated eye (`laptop/sim/world.py`, `fpv=True`):
```powershell
python tools/fpv_test.py                       # geometry, sim eye vs truth, relative mode, sign of the follow law (19 checks)
python tools/scenarios.py eye                  # eye + room camera walk; eye-only static / walk / rotate (no room camera)
python -m laptop.control.ble_gondola --fake --sim      # the simulated robot now has the ultrasonic and the eye (--no-tof / --no-fpv)
python -m laptop.control.pilot --no-voice              # FOLLOW uses the eye for yaw; add --relative to pretend there is no room camera
```

**On the real thing, in this order** (each step is a go/no-go for the next):
1. Ultrasonic: hardware team adds `alt=<cm>` to the IMU line, pointing down. `python -m laptop.control.ble_gondola --probe`
   must show `alt=` changing as you lift the gondola; `curl http://127.0.0.1:5008/status` shows `alt` in metres.
   Set `config.BLE ALT_UNITS` to what they print. Measure sensor-to-balloon-centre -> `config.PHYS TOF_BELOW`.
2. Eye stream: flash the ESP32-CAM CameraWebServer sketch with the laptop hotspot's SSID/password (`wisp` / see
   `firmware/`), 640x480, find its IP (`python tools/find_phone.py`), open `http://<ip>:81/stream` in a browser.
   Then `python -m laptop.vision.fpv --source http://<ip>:81/stream --show`: green box on you, bearing sign flips as
   you step left/right, range roughly right at 1.5 m and 3 m (else adjust `config.FPV HFOV_DEG` / `PERSON_H`).
   Put the URL in `config.FPV SOURCE`.
3. Bench, balloon held by hand, `pilot --no-voice` (+ `--relative` if the room camera is not up yet): say/type
   `follow_me`, stand 3 m away: the L/R motors should push forward and the gondola turn toward you; step left: it
   yaws left (if it yaws right, the stream is mirrored: set `config.FPV K_PSI` negative); walk closer than 1 m: it
   backs off. Then let go.
4. Room camera (when the Pi or a phone is up): section 2 as before. With both, the pilot prints `FOLLOW eye+room`
   and the heading estimate locks within a second of the eye seeing you.

## 1c. Positioning data (record / replay / venue)

`laptop/positioning/` is the scaffold for everything about WHERE things are, independent of which sensor says so.
The sensor set and the fusion maths are not settled; what exists is the structure: a **schema** (one JSONL row per
datagram, stored verbatim), a **session** recorder/loader, pluggable **sources** (udp / replay / sim, plus a placeholder
for whatever comes next), a **fusion stub** (newest-by-priority, TODO the maths), and the **venue** file.
Row format and clock conventions: `laptop/positioning/schema.py`.

```powershell
python -m laptop.control.fake_esp32 --sim --log demo      # records ground TRUTH + what it published / received
python -m laptop.control.follow_me --log demo             # records what the controller consumed + the commands it sent (pilot: same flag)
python -m laptop.positioning.session data/positioning/<session>            # rows per kind, duration, rates
python -m laptop.positioning.replay data/positioning/<session> --speed 3   # re-emit on 5007; run follow_me against it, no sim / ESP32 needed
python -m laptop.positioning.record --name walk1          # vision-only runs: localize / mono on, NO controller running
python -m laptop.positioning.capture_place judges --xy 2.5 0               # venue: a place from the tape measure ...
python -m laptop.positioning.capture_place home --from balloon --seconds 5 # ... or from live vision (again: no controller running)
python tools/positioning_eval.py data/positioning/<session> --plot   # errors vs sim truth, latency, dropouts, jumps, link gaps -> sim_out/*.png
python tools/positioning_eval.py <follow session> --truth-session <fake session>   # two sessions recorded together, joined on wall time
python tools/positioning_eval.py <real session> --expect balloon 1.2 0.4 1.7        # static balloon vs the tape measure
python tools/positioning_test.py                          # offline regression, ~1 s; run after touching laptop/positioning or venues/
```
Sessions land in `data/positioning/<ts>_<tag>[_name]/` (gitignored). `session.load(dir)` returns numpy arrays per kind
(`d["state"]["balloon"]` is (N, 3) with NaN where not seen; `d["cmd"]["vf"]`, `d["truth"]["x"]`, ...). Every row carries
the recorder's clock (`t_ms`) and wall time, so a fake session (truth) and a follow session (commands) recorded together
can be joined afterwards. `session.json` holds the git hash and a snapshot of the gains and the venue.

**Why the controller records and there is no separate tap:** `UdpJson` binds with `SO_REUSEADDR`, so a second socket on
5007 would steal the datagrams from follow_me / pilot (they would see BALLOON LOST). Use `--log` on the process that
already listens; `record.py` and `capture_place.py` are for runs without a controller.

Plugging in a new positioning source (UWB, ToF ranging, a phone, mocap...): subclass `PositioningSource` in
`laptop/positioning/sources.py`, emit PROTOCOL s4 messages from `poll()`, register it, give `fuse.py` its noise figure.

## 2. Track A — cameras (phones now, Pi cams later; the scripts don't care)

Any source works: webcam index, DroidCam URL, `udp://@:5000` from a Pi, or a recorded .mp4.
Put yours in `laptop/config.py` SOURCES or pass `--a/--b`.

Smoke test one camera + YOLO first (green = person, magenta = balloon stand-in; `q` quits):
```powershell
python tools/cam_probe.py                     # which index is which (macOS: Continuity Camera iPhone takes 0, FaceTime becomes 1)
python tools/webcam_test.py --source 0        # --no-show for headless; ~50 ms/frame on CPU, faster once CUDA works; --device mps on a Mac
python tools/find_phone.py --grab             # phone on the hotspot running DroidCam -> prints the URL for SOURCES["B"] + its frame size
```
Phone cameras (DroidCam): set the resolution in the app BEFORE `intrinsics.py`; WiFi adds ~100-250 ms of latency, so
`vision_check.py --live` reports an A/B skew; pass that number as `--lag-b` to `vision_check` / `localize` so the frames
are compared on the same clock. One camera only: `vision_check.py --names A --live --a 0`.

```powershell
python tools/calib/make_targets.py                          # checkerboard + the 4 mat tags; print at 100 %, MEASURE a tag edge -> config.TAG_SIZE_M
python tools/calib/intrinsics.py --source 0 --name A        # ONCE PER DEVICE, EVER (stiff board, focus locked, Center Stage off). Never at the venue.
python tools/calib/intrinsics.py --name B --nominal 1280 720   # no board / no time: nominal pinhole, ~5 % range error; a tape from lens to a tag checks it
# FLOOR MAT: tape tags 0-3 flat on the floor roughly at the corners of a square, pages the same way up (by eye is fine), then:
python tools/calib/survey_mat.py --source 0 --name A     # ONCE per taping: fits each page's position / twist / size -> calib/mat.json (residual < 1 px)
python tools/vision_check.py --live                        # GO / NO-GO: calib files real? cameras placed sanely? tag size? models? venue? fps / skew / mat / drift
python -m laptop.vision.localize --auto-calib --show        # cameras anywhere that see the mat: poses solved in 2 s at start, re-solved if one moves
python -m laptop.vision.mono --a 0 --auto-calib --show      # ONE camera only: feet-on-floor + balloon size (same message on udp 5007)
python tools/calib/extrinsics.py --source 0 --name A        # the same solve as a standalone step (prints the camera position; Z = tape height?)
python tools/calib/triangulate_test.py --a 0 --b 1          # two cameras: tag corners must come back within ~1 cm; then tape-measure test
# balloon: models/balloon.pt if trained, else colour blob (config.BALLOON_COLOR = "white"; --balloon-color red|none ...)
```
Why a mat: one 16 cm tag leaves the camera pitch ~3 deg uncertain even at 0.2 px reprojection error; from a table-top
camera that was a 40 cm range error at 2.5 m. Four tags a metre apart pin it to ~0.2 deg. One tag still
works (flagged "pitch uncertain"); put cameras at ~1.8 m and the error shrinks 4x anyway.
Moved a camera? `--auto-calib` re-solves it when the drift check trips for 3 s. Changed focus/zoom? Redo `intrinsics.py`.

### Numbers to measure on site (and where each one lives)
| Measure | Goes into | Why it matters |
|---|---|---|
| black edge of the printed tag | `config.TAG_SIZE_M` (saved into `calib/<name>_extrinsics.npz`, checked by `vision_check`) | every world position scales with it |
| one checkerboard square | `config.SQUARE_M` | intrinsics scale |
| balloon diameter, inflated | `config.PHYS["D"]` (`R_BALLOON` = D/2) | mono range, simulator mass and drag |
| ToF lens below the balloon centre | `config.PHYS["TOF_BELOW"]` | `alt` + this = height the controller holds |
| motor plane below the balloon centre | `config.PHYS["ARM_BELOW"]` | pendulum tilt in the simulator |
| camera positions | printed by `extrinsics.py`; check Z against a tape | sanity of the pose |
| walls, table, judges spot | `venues/default.json` via `capture_place` | avoidance and go-to |
| cruise height | `config.FOLLOW["Z_HOLD"]` | where it hovers |

### Balloon detector (YOLO, no markers)
`BalloonDetector` (detect.py) uses `models/balloon.pt` (site fine-tune) when it exists, else the committed `models/balloon_web.pt`,
else the colour blob, else COCO "sports ball". It returns the LARGEST box over the threshold (`balloon_eval.py` scores the same way).
The box is the **envelope** (the sphere), never gondola/fins/string: `mono.py` turns box width into range.
Two training stages (`laptop/vision/train_balloon.py`):
- **web** (done once, ~40 min, committed as `models/balloon_web.pt`): yolo11s fine-tuned on
  ~1.7k public photos (Open Images "Balloon": party + hot-air balloons with baskets, strings, every colour; Matterport
  balloon set) + 128 balloon-free photos as negatives. Strong hue jitter so colour does not matter.
  `python tools/dataset/build_balloon_dataset.py --matterport balloon_dataset.zip --openimages --negatives` then
  `python -m laptop.vision.train_balloon --stage web`.
- **site** (on the day, ~10 min total): ~50 frames of THE balloon with its fins/gondola in the demo light:
  ```powershell
  python tools/dataset/label_site.py --source 0        # model pre-draws the box; SPACE accept, drag to fix, n = no balloon
  python tools/dataset/build_balloon_dataset.py --site data/site
  python -m laptop.vision.train_balloon --stage site   # ~3 min, backbone frozen -> models/balloon.pt (+ metrics on your frames)
  python tools/webcam_test.py --source 0               # magenta box on the balloon, none on heads/lamps
  ```
  `python -m laptop.vision.train_balloon --eval` prints mAP on web val and on the held-out site frames separately.

Pi streaming command (later), one per Pi, ports 5000 / 5001:
```
rpicam-vid -t 0 --width 1280 --height 720 --framerate 30 --codec h264 --inline --autofocus-mode manual --lens-position 0.4 --shutter 8000 --gain 4 --awb indoor -o udp://<LAPTOP_IP>:5000
```

## 3. Real gondola — bench checklist (in this order)

1. Bluetooth on, gondola powered. `python -m laptop.control.ble_gondola --probe`: it must connect to `BalloonRobot` and
   print IMU lines with the parsed dict next to each. Set `config.BLE` `IMU_FIELDS` / `GYRO_UNITS` until the dict shows
   `gz_rad` (and `yaw_rad` if the firmware sends a heading). Rotate the gondola CCW by hand: `gz_rad` must be positive
   (set `GYRO_SIGN = -1` if not). No ToF on this build: telemetry `alt` is -1 and height comes from vision.
2. Which letter is which: `python -m laptop.control.ble_gondola --motor C 30` runs one motor at 30 % for 2 s. Repeat for
   D, E, F and fill `config.BLE` `MOTORS` (L and R at the rear pushing forward, S pushing left, V pushing up) and `SIGN`
   (-1 where a positive percent pushes the wrong way).
3. `python -m laptop.control.ble_gondola` in one terminal, `python -m laptop.control.teleop` in another. SPACE arms;
   `w` -> L and R forward, `s` -> reverse, `j` -> S pushes left, `q` -> V pushes up, `a` -> L back / R forward and
   telemetry `yaw` rises. Props balanced, foam tape under motors, throttle stays <= 50 % by construction.
4. Failsafe: Ctrl+C teleop -> the bridge sends `STOP` within 0.5 s and every motor stops. Then kill the BRIDGE while the
   motors run: if they keep spinning, the firmware has no command timeout yet. Ask the hardware team for one (STOP after
   500 ms without a command); until then keep the gondola tethered and a `STOP` ready.
5. Legacy WiFi board instead? Flash `firmware/` (`cd firmware && pio run -t upload`) and pass `--esp wisp-xxxx.local`
   to teleop / follow_me / pilot; the pin table is PROTOCOL.md section 9.
6. Only now attach to the balloon. Trim ballast ~1 gf HEAVY (sinks very slowly with motors off; see PROTOCOL.md section 6).

## 3b. Bench measurements -> `laptop/config.py` PHYS

Now, with the gondola on Bluetooth (bridge running), the motors and a kitchen scale (no balloon needed):
1. **T_MAX, REV_EFF**: one motor on the L channel. Tape the motor upright on a light block on the scale, prop blowing UP
   (air away from the pan, >= 15 cm above it), flight LiPo on VM, tare. `teleop`: SPACE, `w` x3 (vf 0.3) read grams, `w` x4 (0.4),
   `w` x5 (0.5 = the mixer cap). Expect ~4.5 / 8 / 12 g. `T_MAX = 9.81e-3 * g(0.5) / 0.25` N; check g(0.5)/g(0.3) ~ 2.8
   (thrust ~ duty^2; if not, note the exponent). Then `s` x3/4/5: `REV_EFF = g(-0.5) / g(+0.5)`. Repeat for a second motor to
   see the spread (REAL `motor_gain` in world.py).
2. **M_GONDOLA**: the complete flight gondola with battery, props, ToF and wire, on the scale.
3. **MOTOR_SPACING**: ruler, L to R axis.
4. **TOF_BELOW (partial)**: tape from the lens to the gondola's hanging point + `R_BALLOON`; final value once the balloon hangs.
5. **IMU sign**: section 3 step 1.

Later, with the balloon inflated:
6. **D**: tape round the equator / pi. **TOF_BELOW (final)**: balloon hanging still, tape floor->lens (h) and floor->equator
   (z_c): `TOF_BELOW = z_c - h`; telemetry `alt` must read h +- 0.03.
7. **ARM_BELOW**: tape from the motor plane to the equator.
8. **Free lift** (world.py `FREE_LIFT_N`, and the ballast trim): motors off, release at rest, time the second metre of sinking:
   `F = -0.268 * v^2` (0.19 m/s -> -0.010 N ~ -1 gf). Trim with 1 g coins until it sinks that slowly.
9. **RESPONSE_DELAY** (estimator.py): `follow_me --log resp`, arm (the 3 s nudge is a vf step); `positioning_eval.py --plot`
   shows the lag between the `vf` step and the balloon speeding up.

## 4. Still to buy / confirm (not ordered yet)

- Legacy WiFi build only: 3.3 V low-dropout regulator, **>= 500 mA** (ESP32-C3 WiFi bursts ~350 mA; the MCP1700 in the old notes is 250 mA — too small).
  Search amazon.ca for "HT7833", "XC6220 3.3V module", or "ME6211 3.3V LDO module". Feeds the C3's 3V3 pin from the LiPo.
- TP4056 USB-C 1S LiPo charger board (with protection).
- JST-PH 2.0 mm connector kit + 26-28 AWG silicone wire.
- Soft foam mounting tape (motors), Hi-Float optional.
- Confirm: Pi models (Ethernet?), Camera Module 3 variant (standard vs Wide — the two must match), soldering iron access.

## Milestones
- [x] M0a fake_esp32 + teleop round-trip; Bluetooth bridge + simulated robot pass ble_test / sim_test --ble
- [ ] M0b bridge connected to the gondola; motor letters and signs verified; IMU sign; STOP within 0.5 s (section 3)
- [ ] M0c PHYS measured (section 3b)
- [ ] M1 both camera streams in, focus/exposure locked, skew < 100 ms
- [ ] M2 intrinsics + floor-tag extrinsics; tag corners within 1 cm; tape-measure test passes
- [ ] M3 person 3D from YOLO; balloon 3D (COCO stand-in, then fine-tuned)
- [ ] M4 follow-me loop closed on the real balloon; gains tuned
- [ ] M5 ToF altitude hold (done in the simulator and the estimator; needs `HAS_TOF 1` + `TOF_BELOW` measured); LLM intents; wander / go-to-judges / mood
