# Blimpy

A near-silent helium-balloon robot that follows you around. Reflexes live on an ESP32-C3 in the gondola;
everything clever (vision, control, voice) runs on the laptop. The contract between the two is
[PROTOCOL.md](PROTOCOL.md). **Read that first.**

```
firmware/            ESP32-C3 (PlatformIO). Mixer + failsafe + IMU + telemetry.
laptop/config.py     one place for sources, IPs, gains, vehicle physics (PHYS); room geometry is loaded from venues/default.json
laptop/control/      protocol.py (shared mixer) · fake_esp32.py (stand-in + simulator) · teleop.py · follow_me.py · pilot.py · behaviors.py · estimator.py · link.py (telemetry watchdog)
laptop/vision/       streams.py · calib_io.py · triangulate.py · detect.py (YOLO) · localize.py · mono.py (1 cam) · preflight.py · train_balloon.py
laptop/positioning/  schema.py · session.py (record/load) · evaluate.py · sources.py (udp/replay/sim) · fuse.py (stub) · venue.py · record.py · replay.py · capture_place.py
laptop/sim/          world.py (balloon physics + every sensor model) · plot.py (live top-down view)
laptop/voice/        stt.py (whisper) · intent.py (local LLM -> JSON intents) · tts.py · omni.py (Qwen3.5-Omni realtime: ears/eyes/mouth, OMNI Live track) · omni_watch.py (focus watcher) · usage_log.py
tools/calib/         make_targets.py · intrinsics.py · extrinsics.py · triangulate_test.py · record_clips.py
tools/dataset/       build_balloon_dataset.py · label_site.py · extract_frames.py
tools/               scenarios.py · sim_test.py · behaviors_test.py · control_test.py · vision_test.py · positioning_test.py · intent_test.py
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
Firmware: PlatformIO CLI comes with `requirements.txt` (no VS Code needed). Copy
`include/secrets.h.example` to `include/secrets.h`, fill in the laptop hotspot name/password, then:
```
cd firmware && pio run                        # compile (first run downloads the toolchain, ~1 GB)
cd firmware && pio run -t upload -t monitor   # flash + serial monitor, board on USB
cd firmware && pio test -e native             # mixer / failsafe / command-parse tests on the laptop, no board (~2 s)
```
Platform is pinned (`espressif32@7.1.3`, Arduino core 2.0.17) so every laptop builds the same binary.

**Network rule:** everything (laptop, ESP32, phones, Pis) joins the *laptop's hotspot*. Never the hackathon WiFi.

## 1. Track B — protocol works with no hardware (5 minutes)

Terminal 1 and 2 (from the repo root):
```powershell
python -m laptop.control.fake_esp32
python -m laptop.control.teleop
```
Press SPACE to arm, `w` a few times, watch the fake motors ramp; kill teleop with Ctrl+C and watch the
fake disarm within 0.5 s. That is the failsafe working.

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
$env:OMNI_API_KEY = "sk-..."              # from the sponsor's email. Never commit it. (YIBU_API_KEY also works)
python tools/omni_test.py                 # offline: 15 checks against tools/omni_mock_server.py (no key, no internet)
python -m laptop.voice.omni               # live: mic + laptop webcam, prints tool calls and the transcript (Ctrl+C)
python -m laptop.voice.omni_watch --image me.jpg    # one look: {"present","working","phone","activity"}
python -m laptop.control.fake_esp32 --sim --plot
python -m laptop.control.pilot            # omni mode is the default once the key is set; --voice local to compare
python -m laptop.voice.usage_log          # token totals per model/purpose from data/omni_usage.jsonl
```
Knobs: `config.OMNI` (CAMERA = what Blimpy sees, FPS, WATCH_S, HALF_DUPLEX), env `OMNI_MODEL` (default
`qwen3.5-omni-plus-realtime`), `OMNI_URL`, `OMNI_VOICE`, `OMNI_AUDIO_FMT`. **HALF_DUPLEX=True** mutes the cloud mic while
Blimpy is talking (laptop speakers + laptop mic have no echo cancellation, the model would hear itself); with headphones
set it False and you can interrupt Blimpy mid-sentence. `m` in the pilot mutes the cloud mic.

**Windows gives a webcam to one process only**: if `localize.py`/`mono.py` runs on this laptop with source "0",
give the pilot another camera (`--omni-cam 1`, a DroidCam URL, or `none` for ears-only).

**Sponsor reporting**: every cloud call appends one line to `data/omni_usage.jsonl` (model, purpose, tokens, latency,
key suffix only; never prompts/audio/images), the same fields as the organisers' `yibu_audit.py`. Their
`summarize_usage.py --log data/omni_usage.jsonl` makes the two files they want back by the end of the event day (deadline in the key email)
(reply to the key email). Missing token counts stay null: unknown is not zero.

**First live session checklist** (things the mock cannot prove; fix in `omni.py` SESSION / event handlers):
audio in/out format name (`pcm` per Alibaba, `pcm16` on OpenAI-style relays -> `OMNI_AUDIO_FMT`), the tool-call event
(`response.function_call_arguments.done` and/or `response.output_item.done` are both handled), `input_image_buffer.append`
accepted at 1 fps, `semantic_vad` end-of-turn feel (`silence_duration_ms`), voice name, and that `[EVENT]` turns are
spoken without a tool call.

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

1. Flash with `HAS_IMU 0`. Serial shows IP + name. `teleop --esp wisp-xxxx.local`: LED fast blink turns to slow blink
   when commands arrive; SPACE (arm) turns it off.
2. **DRV8833 STBY to GPIO 8** (firmware drives it HIGH when armed; the onboard LED goes OFF when armed).
   One motor on Motor L (AIN1 = GPIO 0, AIN2 = GPIO 4, motor on AO1/AO2): arm, `w` -> spins; `s` -> reverses.
   No spin? Check STBY first, then VM, then GND shared with the ESP32. Full pin table in PROTOCOL.md section 9.
3. `HAS_IMU 1`, reflash, keep still 2 s at boot. Rotate the gondola CCW by hand: telemetry `yaw` must go UP.
   If it goes down, flip the IMU (chip Z must point up) or negate `gz` in `imuStep`.
   Then `HAS_TOF 1`: `teleop` prints `alt`. Hold the gondola over the floor at a tape-measured 0.5 / 1.0 / 1.5 / 1.8 m: within
   3 cm, and note where it turns to -1. The VL53L0X's default mode ranges ~1.2 m; at cruise the lens is ~1.1 m up, so if it
   drops out below 1.5 m switch the firmware to long-range mode before `startContinuous`.
4. All four motors (L/R/S/V) with teleop. Props balanced, foam tape under motors, throttle stays <= 0.5 by construction.
5. Failsafe: Ctrl+C teleop -> motors stop within 0.5 s, LED starts blinking (slow, then fast after 2 s).
6. Only now attach to the balloon. Trim ballast ~1 gf HEAVY (sinks very slowly with motors off; see PROTOCOL.md section 6).

## 3b. Bench measurements -> `laptop/config.py` PHYS

Now, with the ESP32, the motors and a kitchen scale (no balloon needed):
1. **T_MAX, REV_EFF**: one motor on the L channel. Tape the motor upright on a light block on the scale, prop blowing UP
   (air away from the pan, >= 15 cm above it), flight LiPo on VM, tare. `teleop`: SPACE, `w` x3 (vf 0.3) read grams, `w` x4 (0.4),
   `w` x5 (0.5 = the mixer cap). Expect ~4.5 / 8 / 12 g. `T_MAX = 9.81e-3 * g(0.5) / 0.25` N; check g(0.5)/g(0.3) ~ 2.8
   (thrust ~ duty^2; if not, note the exponent). Then `s` x3/4/5: `REV_EFF = g(-0.5) / g(+0.5)`. Repeat for a second motor to
   see the spread (REAL `motor_gain` in world.py).
2. **M_GONDOLA**: the complete flight gondola with battery, props, ToF and wire, on the scale.
3. **MOTOR_SPACING**: ruler, L to R axis.
4. **TOF_BELOW (partial)**: tape from the lens to the gondola's hanging point + `R_BALLOON`; final value once the balloon hangs.
5. **IMU sign, ToF range**: section 3 steps 3.

Later, with the balloon inflated:
6. **D**: tape round the equator / pi. **TOF_BELOW (final)**: balloon hanging still, tape floor->lens (h) and floor->equator
   (z_c): `TOF_BELOW = z_c - h`; telemetry `alt` must read h +- 0.03.
7. **ARM_BELOW**: tape from the motor plane to the equator.
8. **Free lift** (world.py `FREE_LIFT_N`, and the ballast trim): motors off, release at rest, time the second metre of sinking:
   `F = -0.268 * v^2` (0.19 m/s -> -0.010 N ~ -1 gf). Trim with 1 g coins until it sinks that slowly.
9. **RESPONSE_DELAY** (estimator.py): `follow_me --log resp`, arm (the 3 s nudge is a vf step); `positioning_eval.py --plot`
   shows the lag between the `vf` step and the balloon speeding up.

## 4. Still to buy / confirm (not ordered yet)

- 3.3 V low-dropout regulator, **>= 500 mA** (ESP32-C3 WiFi bursts ~350 mA; the MCP1700 in the old notes is 250 mA — too small).
  Search amazon.ca for "HT7833", "XC6220 3.3V module", or "ME6211 3.3V LDO module". Feeds the C3's 3V3 pin from the LiPo.
- TP4056 USB-C 1S LiPo charger board (with protection).
- JST-PH 2.0 mm connector kit + 26-28 AWG silicone wire.
- Soft foam mounting tape (motors), Hi-Float optional.
- Confirm: Pi models (Ethernet?), Camera Module 3 variant (standard vs Wide — the two must match), soldering iron access.

## Milestones
- [x] M0a fake_esp32 + teleop round-trip; firmware flashed; motors spin
- [ ] M0b failsafe verified on the board (motors stop 0.5 s after Ctrl+C); IMU sign; ToF range; PHYS measured (section 3b)
- [ ] M1 both camera streams in, focus/exposure locked, skew < 100 ms
- [ ] M2 intrinsics + floor-tag extrinsics; tag corners within 1 cm; tape-measure test passes
- [ ] M3 person 3D from YOLO; balloon 3D (COCO stand-in, then fine-tuned)
- [ ] M4 follow-me loop closed on the real balloon; gains tuned
- [ ] M5 ToF altitude hold (done in the simulator and the estimator; needs `HAS_TOF 1` + `TOF_BELOW` measured); LLM intents; wander / go-to-judges / mood
