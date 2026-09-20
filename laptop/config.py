"""Project-wide settings. Edit here; scripts take these as defaults and accept CLI overrides.
Room geometry (walls, obstacles, places) is data in venues/default.json, loaded below.
"""
import os
from .positioning import venue as _venue   # stdlib-only, ~1 ms; resolves venues/ from the repo root

# --- Camera sources (anything cv2.VideoCapture understands; see laptop/vision/streams.py) ---
# webcams: "0", "1"    phones (DroidCam): "http://192.168.137.21:4747/video"
# Raspberry Pi (rpicam-vid H.264 over UDP): "udp://@:5000" (A) and "udp://@:5001" (B)
SOURCES = {"A": "0", "B": "http://192.168.137.25:4747/video"}   # A: the laptop webcam (1280x720) = the ROOM camera. B: a phone running DroidCam on the laptop hotspot (tools/find_phone.py), optional second camera
CALIB_NAMES = {"A": os.environ.get("BLIMPY_CALIB_A", "laptop"), "B": "B"}   # which calib/<name>_*.npz each role uses (localize --names); calib/laptop_intrinsics.npz
                                          # is committed (the Windows demo laptop's webcam, calibrated once). Another machine's webcam needs its own file:
                                          # BLIMPY_CALIB_A=macbook in .env (calib/macbook_intrinsics.npz: tools/calib/intrinsics.py --name macbook, or a nominal
                                          # one fitted from the mat), so every tool on that machine uses it without a flag
VISION_DEVICE = os.environ.get("BLIMPY_DEVICE") or None   # ultralytics device for the detectors: None = auto (CUDA when there is one, else CPU). A Mac needs
                                          # BLIMPY_DEVICE=mps in .env: measured 2026-09-19 on a MacBook Air, mono ran 4 Hz on the CPU and 14-19 Hz on mps
ROTATE = {"A": 0, "B": 90}              # degrees clockwise applied to each stream (phone held in portrait -> 90); calibrate at the rotated size

# --- OMNI Live (Huawei track): cloud ears/eyes/mouth via Qwen3.5-Omni on the yibuapi relay (laptop/voice/omni.py) ---
#     Team key from the organisers' e-mail -> env YIBU_API_KEY (or OMNI_API_KEY), never in the repo. Every call is logged
#     to data/omni_usage.jsonl in the organisers' schema; tools/omni_report.py builds the report they want back.
OMNI = dict(
    CAMERA="0",              # what Blimpy SEES in conversation when there is no eye on the balloon (laptop webcam at the desk).
                             # With FPV SOURCE set (or pilot --fpv) Blimpy sees through its own eye; --omni-cam overrides.
                             # Not one of SOURCES above if localize.py runs on this laptop: Windows gives a webcam to ONE process.
    FPS=1.0,                 # frames per second sent to the realtime model (provider recommends 1)
    WATCH_S=6.0,             # focus watcher: seconds between "what is the user doing" looks (laptop/voice/omni_watch.py)
    HALF_DUPLEX=True,        # laptop speakers + laptop mic (no echo cancellation): mute the mic while Blimpy talks.
                             # False with headphones or a conference speaker -> real barge-in.
    # The relay bills every second of audio the laptop sends (0.77 units per minute, talking or not; README 1d). The gate
    # sends the mic only while someone is talking: a packet louder than the room's noise floor + GATE_DB (and louder
    # than GATE_MIN_DBFS) opens it, it shuts GATE_HANGOVER_MS after the last loud packet. Calibrate at the venue with
    # python -m laptop.voice.omni --meter. GATE=False streams everything (the old behaviour).
    GATE=True,
    GATE_DB=12.0,            # dB above the noise floor that counts as someone talking (lower = more sensitive).
                             # python -m laptop.voice.omni --calibrate --mic AirPods  measures room / you / others and says what to put here.
    GATE_MIN_DBFS=-50.0,     # never open below this absolute level (a fan in a silent room)
    GATE_PREROLL_MS=320,     # audio kept from just before the gate opened (the first syllable)
    GATE_HANGOVER_MS=1000,   # audio kept after the last loud packet; must stay > the server VAD's 600 ms of silence
    GATE_LISTEN_S=5.0,       # on startup the gate listens to the room this long (nothing goes up), sets the floor, prints a verdict
    # Who is that for (laptop/voice/addressee.py): independent cues (how close a word sounds to the name, how fresh the
    # conversation with Blimpy is, an answer to its question, someone in front of its eye, the form of the sentence) are
    # weighed against two thresholds that slide with the room's noise; what falls between them goes to a language-model
    # judge. Stop words (safety) and press-to-talk always count. The reply and the tool calls of a turn are held until it
    # is judged, so team chatter makes no sound and runs nothing. False = answer everything.
    NAME_GATE=True,
    NAME_WORDS=("blimpy", "blimpie", "blippi", "limpie", "blimp", "limpy", "blimpey"),
    ADDRESS="smart",         # smart = cues + room + judge | name = the name, a stop word or p only | open = answer everything
    JUDGE="relay",           # settles the unclear turns: relay (sponsored text model, ~1-2 s, logged as addressee_judge) |
                             # ollama (local, no key) | off (the midpoint of the thresholds decides)
    RECORD=True,             # keep each session (mic.wav + turns.jsonl, data/voice_sessions, gitignored) for tools/addressee_backtest.py
    ADDRESSEE=dict(),        # overrides of addressee.DEFAULTS, e.g. dict(engaged_tau_s=(10, 5), weights=dict(presence=0.4), accept=(0.8, 0.85))
    ADDRESS_MODE="auto",     # auto = loud when the mic gate's noise floor is above LOUD_FLOOR_DB | quiet | loud   (pilot key l)
    LOUD_FLOOR_DB=-45.0,
    GATE_DB_LOUD=None,       # GATE_DB while the room is loud; None = unchanged. Take it from `omni --calibrate` on the floor.
    VERDICT_TIMEOUT_S=1.5,   # replies are held until the turn's transcript is judged; this long without one = judged without it
    PRESENCE_BEARING_RAD=0.35, PRESENCE_RANGE_M=3.0,   # "said to its face": someone this centred and this near in the FPV eye
)

# --- Everybody in the room camera's view (laptop/vision/people.py): labels P1, P2 ... that survive occlusion, leaving and
#     coming back, and tracker-id swaps. First values 2026-09-19, from the recorded two-person session; nothing here is
#     tuned to a test. ---
PEOPLE = dict(
    EDGE_FRAC=0.02,        # a box ending within this fraction of the frame edge counts as cut there (its real end is out of view)
    ASSUMED_H_M=1.70,      # m, standing height used ONLY when the feet are out of the picture (position flagged q="head")
    SIG_MAX=0.35,          # Bhattacharyya distance of torso colour histograms below which two sightings may be one person
    AMBIG_MARGIN=0.08,     # a re-attachment that does not beat the next candidate by this is flagged amb (the name on it is dropped)
    GALLERY=4, GALLERY_NEW=0.2,   # signatures remembered per person; a view this far from all of them is a new one (their back)
    SIG_EMA=0.05,          # how fast a signature follows the light (updated only on clean, uncut, unoccluded boxes)
    GATE_M=0.8, V_MAX=2.0, # re-attach only within GATE_M + V_MAX * gap metres of where the person was (when both are known)
    POS_GATE_S=3.0,        # ... and only for gaps shorter than this; after that the position says nothing
    TID_TRUST_S=1.0,       # a tracker id seen this recently is trusted (its motion model vouches for it) ...
    TID_BONUS=0.3,         # ... by this much against the signature: a clear colour contradiction still overrules it (swap)
    MIN_HITS=3,            # frames before a new person is published (a one-frame ghost never gets a label)
    GHOST_S=1.0,           # s after which a box that never reached MIN_HITS is forgotten (it must not rival real people for 2 minutes)
    HEAD_RAY_MIN=0.15,     # feet-cut tier: the head ray must climb or drop at least this much per metre, else no metres (ill-conditioned)
    HEAD_RANGE_MAX_M=6.0,  # ... and a head-tier position further than this from the camera is not believed
    FORGET_S=120.0,        # s out of view before a person (and the name bound to the label) is forgotten; labels are never reused
    PRIMARY_HOLD_S=2.5,    # s the single-person message waits for "the person" to come back before it moves to someone else
    MAX_PUBLISHED=6,       # people per message (udp datagram budget, protocol.UdpJson reads 2048 bytes)
    BEHIND_M=1.5,          # m, "go behind <person>": the goal is this far beyond them on the line from Blimpy. Below ~1.4 the
                           # balloon's own radius + the avoidance margin make the goal unreachable (follow_me.avoid)
    ROOM_POLL_HZ=3.0,      # how often the pilot fetches mono's picture + people (laptop/vision/eyes.py); the model gets 1 frame/s
)

# --- Eye on the balloon (laptop/vision/fpv.py): the Arduino/ESP32 camera on the gondola streams over the hotspot. It is
#     what Blimpy SEES in conversation (OMNI) and how it finds the person to follow: bearing straight from the image, so
#     no heading calibration; range from the person's height in the frame. The room camera (SOURCES A, mono.py) stays
#     for x/y (go_to, wander, hover-in-place); without it set RELATIVE=True (or pilot --relative): eye + ultrasonic only. ---
FPV = dict(
    SOURCE=None,            # e.g. "http://192.168.137.40:81/stream" (ESP32-CAM CameraWebServer sketch). None = no eye
    HFOV_DEG=62.0,          # horizontal field of view of that camera (OV2640 ~62; measure: README 3c)
    PITCH_DEG=15.0,         # camera tilted DOWN this much from the balloon's forward axis (chest-height person at 1.5 m)
    PERSON_H=1.65,          # m, standing person head to feet: range = f * PERSON_H / box height (px); ~5-10 % noisy
    LAG_MS=250,             # WiFi + JPEG latency of the stream (frames are stamped on arrival)
    LOST_MS=1500,           # no person in the eye for this long -> FOLLOW holds still
    HZ=10,                  # detections per second (YOLO on the laptop)
    K_PSI=1.0,              # yaw rate per rad of bearing (the person is seen directly; FOLLOW K_PSI is for the estimated heading)
    SEARCH_YR=0.35,         # no room camera and the eye lost the person: yaw this fast toward where it last saw them
    RELATIVE=False,         # True = no room camera: follow / rotate / hover-still / timers work, go_to / wander are refused
    WEIGHTS=None,           # YOLO weights for the eye (None = YOLO_PERSON below)
)

# --- Network ---
ESP32_IP = "127.0.0.1"            # where the control programs send commands. The gondola is on Bluetooth: run
                                  # `python -m laptop.control.ble_gondola` on this laptop and everything talks to it here.
                                  # (Legacy WiFi firmware boards answer on "blimpy-9910.local" / "blimpy-91c8.local".)

# --- Gondola over Bluetooth LE (laptop/control/ble_gondola.py). Two firmware generations, the bridge tells them apart:
#     2026-09-20 (firmware/esp32_master.ino, "onboard mixer"): the box mixes; the bridge sends `CMD vf vs yr vz` setpoints
#     10 times a second and `MAP ...` on connect; telemetry every 200 ms with the heading integrated on the box.
#     Earlier builds: the bridge mixes at 50 Hz and writes `MOTORS c d e f` (needs telemetry at 20 Hz or more). ---
BLE = dict(
    NAME="BalloonRobot",
    SERVICE_UUID="12345678-1234-1234-1234-123456789000",     # discover by advertised service, even when the device name is missing
    COMMAND_UUID="12345678-1234-1234-1234-123456789001",     # write: "CMD 20 0 0 0" | "MAP LC+ RD+ SE+ VF+ G+" | "C 40" | "MOTORS c d e f" | "STOP"
    TELEMETRY_UUID="12345678-1234-1234-1234-123456789002",   # notify: one text line per sample
    # Which firmware LETTER (C D E F) drives which of OUR motors, and which way. The motors were rewired 2026-09-20, so these
    # defaults are placeholders: run  python tools/motor_map.py  (spins each letter, you say which motor it was and which way
    # the air went) -> calib/motor_map.json, which OVERRIDES the two lines below whenever it exists. L/R: rear left/right,
    # + = pushes the balloon forward (air blows backward). S: + = pushes LEFT. V: + = pushes UP.
    MOTORS={"L": "C", "R": "D", "S": "E", "V": "F"},
    SIGN={"L": 1, "R": 1, "S": 1, "V": 1},
    PCT_MAX=100,          # firmware clamp; the mixer's CAP (0.5) keeps normal flight at +-50
    HZ=20,                # old firmware: MOTORS lines per second at most (BLE write-without-response)
    CMD_HZ=10,            # onboard mixer: CMD setpoint lines per second (the box slews and closes the yaw loop itself)
    IMU_FIELDS=None,      # names for a bare-numbers IMU line, e.g. ("ax","ay","az","gx","gy","gz"); None = by count / key=value
    GYRO_UNITS="deg",     # "deg" (deg/s and degrees, most Arduino IMU libraries) or "rad"
    GYRO_ZERO=True, GYRO_ZERO_S=3.0, GYRO_STILL_DPS=0.6, GYRO_BIAS_MAX_DPS=5.0,   # old firmware: gyro zero learnt on the laptop while disarmed
                          # and still (bridge._zero_gyro). The onboard mixer zeroes its own gyro (telemetry `bias`). Bench 2026-09-19: -0.36 deg/s.
    GYRO_SIGN=1,          # -1 if turning the gondola counter-clockwise (seen from above) gives a NEGATIVE gz. MEASURED 2026-09-19: +1.
                          # Sent to the box in the MAP line (G+ / G-); calib/motor_map.json may override it too.
    IMU_FRESH_MS=450,     # old firmware: yaw-rate feedback in the laptop mixer only with a sample younger than this (200 ms telemetry
                          # still counts). Onboard mixer: the box is considered gone when its telemetry is older than BOX_SILENT_MS.
    BOX_SILENT_MS=1500,   # onboard mixer: no telemetry from the box for this long while flying -> setpoints stop, bridge reports armed 0
    ALT_KEYS=("alt", "range", "dist", "sonar", "us"),   # a downward ultrasonic in the same IMU line, first key found wins.
                          # NONE on the 2026-09-19 box (it ran out of pins): the path stays for when one is fitted
    ALT_UNITS="cm",       # "cm" | "mm" | "m" as the firmware prints it; <= 0 = no echo. Telemetry alt is metres, -1 = none
    ALT_FRESH_MS=300,     # older than this -> alt -1 (the estimator then falls back to the room camera for height)
    HTTP_PORT=5008,       # GET http://127.0.0.1:5008/imu | /imu/history?n=200 | /status
)
MOTOR_MAP_FILE = "calib/motor_map.json"   # written by tools/motor_map.py; overrides BLE MOTORS / SIGN / GYRO_SIGN when present


def _load_motor_map(b):
    """calib/motor_map.json -> config.BLE MOTORS / SIGN (/ GYRO_SIGN). Measured beats typed."""
    import json
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), MOTOR_MAP_FILE)
    try:
        with open(path, encoding="utf-8") as f:
            m = json.load(f)
        b["MOTORS"] = {k: str(v).upper() for k, v in m["MOTORS"].items()}
        b["SIGN"] = {k: int(v) for k, v in m["SIGN"].items()}
        if m.get("GYRO_SIGN") in (1, -1): b["GYRO_SIGN"] = int(m["GYRO_SIGN"])
        b["MAP_SOURCE"] = f"{MOTOR_MAP_FILE} ({m.get('measured', '?')})"
    except FileNotFoundError:
        b["MAP_SOURCE"] = "config.py placeholders: NOT MEASURED, run  python tools/motor_map.py"
    except (KeyError, ValueError, TypeError) as e:
        b["MAP_SOURCE"] = f"{MOTOR_MAP_FILE} unreadable ({e}); config.py placeholders in use"
    if sorted(b["MOTORS"].values()) != ["C", "D", "E", "F"] or set(b["MOTORS"]) != {"L", "R", "S", "V"}:
        raise SystemExit(f"config.BLE MOTORS must map L R S V to C D E F, one each: {b['MOTORS']} ({b['MAP_SOURCE']})")


_load_motor_map(BLE)

# --- Calibration targets ---
CALIB_DIR = "calib"
POSITIONING_DIR = "data/positioning"   # recorded sessions (laptop/positioning/session.py); gitignored
CHECKER_COLS, CHECKER_ROWS = 9, 6  # INNER corners
SQUARE_M = 0.0228                  # MEASURED 2026-09-19 on the printed board (the printer scaled the page to 91 %)
TAG_SIZE_M = 0.1367                # MEASURED 2026-09-19: black-square edge of the printed tags (150 mm file printed at 91 %). MEASURE new prints!
MAT_SPACING_M = (0.35, 0.35)       # floor mat: centre-to-centre metres tag 0->1 (along +X) and 0->3 (along +Y). Board surveyed 2026-09-19: 0.347 x 0.355 m.
#     The mat is the four tags glued to ONE rigid board (README 3a), so it is surveyed ONCE at home (tools/calib/survey_mat.py ->
#     calib/mat.json, committed) and just dropped on the floor in every room. These are the nominal numbers; the survey wins.
MAT = {0: (0.0, 0.0), 1: (MAT_SPACING_M[0], 0.0), 2: (MAT_SPACING_M[0], MAT_SPACING_M[1]), 3: (0.0, MAT_SPACING_M[1])}
#     tag id -> centre (x, y) m on the floor, ALL printed the same way up. World origin = tag 0 (laptop/vision/floor.py).
#     One tag alone works but leaves the camera pitch uncertain (~3 deg = 40 cm at 2.5 m from a low camera).

# --- Detection ---
YOLO_PERSON = "yolo11s.pt"         # COCO model, class 0 = person. On the RTX 5080 (fp16) "s" is as fast as "n" (11 ms/720p frame) and more accurate; use "yolo11n.pt" on CPU.
BALLOON_WEIGHTS = "models/balloon.pt"   # fine-tuned (laptop/vision/train_balloon.py) when it exists, else ...
BALLOON_COLOR = "white"                 # ... colour-blob detector (white/red/orange/yellow/green/blue/pink, None = COCO "sports ball")

# --- The room (world frame, metres from the floor AprilTag). Geometry is DATA: venues/default.json (committed).
#     PLACEHOLDERS until tape-measured on site, or:  python -m laptop.positioning.capture_place <name> --xy X Y  ---
# --- Vehicle physics: ONE place. archive/sim/world.py (Balloon), laptop/control/estimator.py and follow_me.py read it.
#     Parts-list PLACEHOLDERS until measured on the bench (README section 3b). ---
PHYS = dict(
    D=1.15,               # m, envelope diameter. MEASURED 2026-09-20 (re-inflated for the flight): 115 cm. (2026-09-19 it was 1.00.)
    T_MAX=0.050 * 9.81,   # N per motor at duty 1.0 (~50 gf). MEASURE: one motor on a kitchen scale at duty 0.3 / 0.4 / 0.5
    REV_EFF=0.6,          # reverse / forward thrust at the same duty (fixed prop + DRV8833 slow decay). MEASURE: same rig, reverse
    ARM_BELOW=0.55,       # m, motor plane below the balloon centre (pendulum arm). MEASURE: tape, balloon hanging
    TOF_BELOW=0.60,       # m, downward ultrasonic lens below the balloon centre: centre z = alt + TOF_BELOW. UNUSED on the
                          # 2026-09-19 box (no ultrasonic); measure it (README 3b) if one is ever fitted
    M_GONDOLA=0.25,       # kg, flight gondola with battery and props. MEASURE: scale
    MOTOR_SPACING=0.25,   # m between the L and R motor axes. MEASURE: ruler
)
R_BALLOON = PHYS["D"] / 2          # m, envelope radius
VENUE_FILE = os.environ.get("BLIMPY_VENUE") or _venue.DEFAULT_PATH   # BLIMPY_VENUE=venues/other.json switches rooms
VENUE = _venue.load(VENUE_FILE)
ARENA = VENUE.arena_tuple()        # ((min_x, min_y), (max_x, max_y)): the walls. The balloon is kept R_BALLOON + margin inside.
OBSTACLES = VENUE.obstacle_tuples()  # [(x, y, radius)] cylinders the balloon must not touch: tables, pillars, tripods
JUDGES_XY = VENUE.place("judges")  # where to hover for the judges (keep >= 0.5 m clear of the table edge); None if unknown
WANDER_BOX = VENUE.wander_tuple()  # ((min_x, min_y), (max_x, max_y)) or None = arena shrunk by R_BALLOON + 0.35 (behaviors.py)
AVOID = dict(MARGIN=0.6, K_OBS=0.5, Z_MIN=0.9, Z_MAX=2.4)   # start steering away MARGIN m before contact; height limits

# --- Follow-me controller (laptop/control/follow_me.py) ---
FOLLOW = dict(
    D_FOLLOW=1.5,         # m, desired distance to the person (centre to centre; the envelope is 0.55 m radius)
    D_DEADBAND=0.15,      # m, no forward/back inside this band
    Z_HOLD=1.7,           # m, cruise height of the balloon centre
    Z_DEADBAND=0.02,      # m
    K_PSI=0.8,            # yaw gain (normalised yaw-rate per rad of error); yaw only faces the person
    K_P=0.7,              # desired speed toward the target = K_P * distance error (1/s)
    V_DES_MAX=0.5,        # m/s cap on that closing speed
    V_ABS_MAX=0.7,        # m/s cap on the total desired speed (person's speed + closing); ~top speed at the duty cap
    K_V=2.8,              # thrust = K_V * (desired velocity - actual velocity), in thrust fraction per m/s. Drag is quadratic
                          # so the balloon coasts for metres; this term (forward AND sideways) is what stops it.
    K_Z=1.3,              # vertical gain (natural period of the height loop ~13 s: it is a slow, heavy axis). 1.0 until 2026-09-20:
                          # with the shared supply the V motor loses up to a third of its thrust whenever L/R/S run, so a little more gain
    K_VZ=3.0,             # vertical braking gain (more damping than gain: the vertical axis coasts)
    Z_KI=0.15,            # vertical integrator: learns the ballast trim (thrust fraction per m*s). Keep slow (~T/4)
    Z_I_MAX=0.12,         # integrator clamp (thrust fraction); +-12 % of max V thrust ~ +-6 gf of trim error
    YR_CAP=0.5, VF_CAP=0.4, VS_CAP=0.4, VZ_CAP=0.3,
    POS_ALPHA=0.3, VEL_BETA=0.05,    # alpha-beta position/velocity filter on the vision fixes (prediction uses the commands)
    DUTY_MIN=0.1,         # duties below this are sent as 0: below the motor start threshold, and it keeps noise out of the props
    # Smoothness (2026-09-20, after the live log showed vf/vs slamming +-0.4 every frame): the velocity estimate is ~3 cm/s
    # noisy, K_V turns that into thrust, and the sqrt linearisation has infinite gain at zero. So: no thrust for velocity
    # errors under V_DEAD, a linear section below 2 x DUTY_MIN instead of the sqrt, a first-order low-pass on the forward /
    # sideways commands, and every threshold in the law is a ramp (deadbands, the person-speed feed-forward, the dodge).
    V_DEAD=0.04,          # m/s, velocity error below which the velocity loop asks for nothing (the props cannot do 3 cm/s anyway)
    CMD_TAU_S=0.2,        # s, low-pass on vf / vs before the motor-start cut (0 = off); the heading twitch bypasses it (behaviors._smooth).
                          # The box slews too (0.05 duty per 20 ms)
    PV_GATE=(0.05, 0.15), # m/s, the person's walking speed is fed forward from 0 at the first to fully at the second (was a step at 0.1)
    A_BRAKE=0.055,        # m/s^2 the controller assumes it can brake at (reverse thrust is weak): approach speed = sqrt(2*A_BRAKE*d).
                          # Two motors at the 0.4 cap in reverse give ~0.11 m/s^2 with a stiff supply; the motors SHARE one battery
                          # (2026-09-20) and four running together take a third of that away, hence 0.055 (was 0.08)
    A_WALL=0.045,         # same idea for walls / obstacles, more conservative (touching a wall is worse than being late)
    V_LAG_S=1.2,          # s the velocity loop needs to respond; obstacle distances are judged that far ahead
    NUDGE_CLEAR=1.0,      # m of clearance the pilot asks for before the blind heading acquisition (it warns below this)
    SLIDE_V_MAX=0.25,     # m/s max speed when sliding along a wall / around an obstacle
    PERSON_LOST_MS=2500,  # -> hover (vision drops the person for up to ~1 s when they turn or get occluded)
    BALLOON_LOST_MS=1500, # -> disarm
    TELEM_LOST_MS=1000,   # armed and no telemetry for this long -> disarm (the bridge sends 20 frames a second whatever the box does)
    BOARD_OFF_N=12,       # board says armed:0 in this many consecutive frames (20 Hz) while we are armed -> disarm. With the onboard
                          # mixer the bridge only says armed:1 once the BOX reports it is flying, and the box talks every 200 ms:
                          # up to ~6 frames of armed:0 right after arming are normal, 12 (0.6 s) is a box that really did not take it
    AGE_WARN_MS=300,      # the board's 'ms since last command' above this -> print a link warning (its failsafe trips at 500)
    # One horizontal motor set at a time (behaviors.AxisArbiter): REAR = the two motors at the back (forward / back AND turning, also mixed:
    # it is still only those two) or SIDE = the sideways motor.
    # The motors share one weak supply: all of them at once is none of them, and it reads as random motion. The lift motor is not part of this.
    AXIS_MIN_S=1.0,       # s the chosen set keeps running before another may take over
    AXIS_GAP_S=0.3,       # s of nothing between two sets (spin down, the supply recovers, and it reads as: turn, pause, push)
    AXIS_SWITCH_RATIO=1.4,   # another set takes over only when it asks this many times harder than the running one
    AXIS_DEAD=0.15,       # a set asking for less than this share of its cap asks for nothing
    NUDGE_MAX_S=6.0,      # s a "move left / right / forward / back" body push lasts at most (it ends earlier once the camera saw the distance)
    NUDGE_S=3.0, NUDGE_VF=0.3,   # heading-calibration nudge on first arm: fly forward, learn heading from the response
    TWITCH_S=2.5, TWITCH_AFTER_S=40,   # re-learn heading with a short forward push after this long without a manoeuvre (2.0 until
                          # the CMD_TAU_S low-pass: the rounded push must carry the same thrust-time for the learner)
    K_ROT=0.9, ROT_MIN=0.08, ROT_CAP=0.7, ROT_DONE_RAD=0.09,   # rotate: yaw rate = K_ROT * remaining, floor ROT_MIN, cap ROT_CAP; done within 5 deg
    STANDOFF_ME=1.1, STANDOFF_PLACE=0.25, ARRIVE_M=0.2,   # "come here" stops 1.1 m from the person; places 0.25 m; arrived within +ARRIVE_M
    HOLD_DEADBAND=0.08, HOLD_V_MAX=0.3,   # hover = hold POSITION (gusts and the twitch would otherwise walk it away)
    HZ=15,
)

# --- Height hold alone (laptop/control/hover.py): only the V motor runs, so it may use the whole per-motor cap of the box
#     (MIX_CAP 0.5; the shared-supply budget never binds with L/R/S at zero). The balloon LEAKS: the weight the fan has to
#     carry grows all the time, so the trim integrator gets nearly the whole range instead of FOLLOW's +-12 %.
#     u is thrust in units of the cap thrust, so a larger cap is a larger loop gain: the FOLLOW gains are scaled back by
#     the cap ratio and the height loop stays the one that was tuned. ---
_HOVER_CAP = 0.5
_k = FOLLOW["VZ_CAP"] / _HOVER_CAP
HOVER = dict(FOLLOW,
    VZ_CAP=_HOVER_CAP,
    K_Z=FOLLOW["K_Z"] * _k, K_VZ=FOLLOW["K_VZ"] * _k, Z_KI=FOLLOW["Z_KI"] * _k,
    Z_I_MAX=0.85 * _HOVER_CAP,    # thrust fraction the trim may hold; the rest is left for P and D
    ON_DUTY=1.0,                  # hover.py --onoff: the fan is either at this duty or off (needs the bridge's --motor-cap at least this high)
    VZ_WIN_S=0.8,                 # s of camera fixes the height / vertical-speed line is fitted through (hover.py ZTrack)
    Z_GATE_M=0.25,                # a fix this far from the median of that window is a wrong box: left out
    ONOFF=True,                   # the fan is flat out or off (altitude.LiftHold); False = AltHold's P + I + D
    DROP_WIN_S=0.4,               # s, the short look that catches a drop early
    DROP_V=0.06,                  # m/s downward that counts as dropping: the fan goes on at once
    DROP_MARGIN_M=0.05,           # ... unless the balloon is still this far ABOVE its target (coming down on purpose)
    LAT_S=0.4,                    # s from a height change to the fan answering it (camera ~0.13 + Bluetooth + mixer slew + spin-up)
    A_ON0=0.05, A_OFF0=-0.03,     # m/s^2 up with the fan on / down with it off: first guesses only, measured in flight from then on
    A_MIN=0.005, A_MAX=0.3, LEARN_MIN_S=1.0, LEARN_K=0.3,
    ONOFF_MIN_S=0.6,              # hover.py --onoff: shortest time the fan stays on (or off) once switched
    TRIM_SAT_S=5.0,               # s the trim sits at its clamp before hover.py says the fan cannot carry the balloon
)
