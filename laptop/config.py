"""Project-wide settings. Edit here; scripts take these as defaults and accept CLI overrides.
Room geometry (walls, obstacles, places) is data in venues/default.json, loaded below.
"""
import os
from .positioning import venue as _venue   # stdlib-only, ~1 ms; resolves venues/ from the repo root

# --- Camera sources (anything cv2.VideoCapture understands; see laptop/vision/streams.py) ---
# webcams: "0", "1"    phones (DroidCam): "http://192.168.137.21:4747/video"
# Raspberry Pi (rpicam-vid H.264 over UDP): "udp://@:5000" (A) and "udp://@:5001" (B)
SOURCES = {"A": "0", "B": "http://192.168.137.25:4747/video"}   # A: iPhone via Continuity Camera (index 0; FaceTime = 1). B: friend's iPhone, DroidCam, on the laptop hotspot (tools/find_phone.py)
CALIB_NAMES = {"A": "iphone", "B": "B"}   # which calib/<name>_*.npz each role uses (localize --names)
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
    # Name gate: a turn whose transcript has no "Blimpy" (or the recogniser's spellings of it) is NOT for Blimpy: its reply
    # is cancelled and its command dropped. Exceptions: a follow-up within NAME_FOLLOWUP_S of Blimpy's last reply, stop
    # words (safety), and a press-to-talk turn. The audio still goes up (it is what gets transcribed); a crowd cannot
    # command the robot but can still cost a little. False = trust the model's own judgement of who is talking to it.
    NAME_GATE=True,
    NAME_WORDS=("blimpy", "blimpie", "blippi", "limpie", "blimp", "limpy", "blimpey"),
    NAME_FOLLOWUP_S=8.0,
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
                                  # (Legacy WiFi firmware boards answer on "wisp-9910.local" / "wisp-91c8.local".)

# --- Gondola over Bluetooth LE (laptop/control/ble_gondola.py). The firmware takes per-motor PERCENTAGES as text. ---
BLE = dict(
    NAME="BalloonRobot",
    COMMAND_UUID="12345678-1234-1234-1234-123456789001",     # write: "C 40" | "ALL 30" | "MOTORS c d e f" | "STOP"
    TELEMETRY_UUID="12345678-1234-1234-1234-123456789002",   # notify: one IMU text line per sample
    MOTORS={"L": "C", "R": "D", "S": "E", "V": "F"},   # our motor -> firmware letter. VERIFY on the bench: ble_gondola --motor C 30
    SIGN={"L": 1, "R": 1, "S": 1, "V": 1},              # -1 if a motor pushes the wrong way for a positive percent
    PCT_MAX=100,          # firmware clamp; the mixer's CAP (0.5) keeps normal flight at +-50
    HZ=20,                # MOTORS lines per second at most (BLE write-without-response)
    IMU_FIELDS=None,      # names for a bare-numbers IMU line, e.g. ("ax","ay","az","gx","gy","gz"); None = by count / key=value
    GYRO_UNITS="deg",     # "deg" (deg/s and degrees, most Arduino IMU libraries) or "rad"
    GYRO_SIGN=1,          # -1 if turning the gondola counter-clockwise (seen from above) gives a NEGATIVE gz
    IMU_FRESH_MS=200,     # yaw-rate feedback in the mixer only with a sample younger than this
    ALT_KEYS=("alt", "range", "dist", "sonar", "us"),   # the ultrasonic (downward) in the same IMU line, first key found wins
    ALT_UNITS="cm",       # "cm" | "mm" | "m" as the firmware prints it; <= 0 = no echo. Telemetry alt is metres, -1 = none
    ALT_FRESH_MS=300,     # older than this -> alt -1 (the estimator then falls back to the room camera for height)
    HTTP_PORT=5008,       # GET http://127.0.0.1:5008/imu | /imu/history?n=200 | /status
)

# --- Calibration targets ---
CALIB_DIR = "calib"
POSITIONING_DIR = "data/positioning"   # recorded sessions (laptop/positioning/session.py); gitignored
CHECKER_COLS, CHECKER_ROWS = 9, 6  # INNER corners
SQUARE_M = 0.025                   # measure a printed square and correct this if needed
TAG_SIZE_M = 0.15                  # black-square edge of tag 0: all four mat pages are the 150 mm print. MEASURE new prints!
MAT_SPACING_M = 1.0                # floor mat: centre-to-centre distance between tags 0-1 and 0-3 (tape them, MEASURE, put it here)
MAT = {0: (0.0, 0.0), 1: (MAT_SPACING_M, 0.0), 2: (MAT_SPACING_M, MAT_SPACING_M), 3: (0.0, MAT_SPACING_M)}
#     tag id -> centre (x, y) m on the floor, ALL printed the same way up. World origin = tag 0 (laptop/vision/floor.py).
#     One tag alone works but leaves the camera pitch uncertain (~3 deg = 40 cm at 2.5 m from a low camera).

# --- Detection ---
YOLO_PERSON = "yolo11s.pt"         # COCO model, class 0 = person. On the RTX 5080 (fp16) "s" is as fast as "n" (11 ms/720p frame) and more accurate; use "yolo11n.pt" on CPU.
BALLOON_WEIGHTS = "models/balloon.pt"   # fine-tuned (laptop/vision/train_balloon.py) when it exists, else ...
BALLOON_COLOR = "white"                 # ... colour-blob detector (white/red/orange/yellow/green/blue/pink, None = COCO "sports ball")

# --- The room (world frame, metres from the floor AprilTag). Geometry is DATA: venues/default.json (committed).
#     PLACEHOLDERS until tape-measured on site, or:  python -m laptop.positioning.capture_place <name> --xy X Y  ---
# --- Vehicle physics: ONE place. laptop/sim/world.py (Balloon), laptop/control/estimator.py and follow_me.py read it.
#     Parts-list PLACEHOLDERS until measured on the bench (README section 3b). ---
PHYS = dict(
    D=1.1,                # m, envelope diameter (48" latex inflated to ~1.1 m). MEASURE: tape round the equator / pi
    T_MAX=0.050 * 9.81,   # N per motor at duty 1.0 (~50 gf). MEASURE: one motor on a kitchen scale at duty 0.3 / 0.4 / 0.5
    REV_EFF=0.6,          # reverse / forward thrust at the same duty (fixed prop + DRV8833 slow decay). MEASURE: same rig, reverse
    ARM_BELOW=0.55,       # m, motor plane below the balloon centre (pendulum arm). MEASURE: tape, balloon hanging
    TOF_BELOW=0.60,       # m, VL53L0X lens below the balloon centre: centre z = alt + TOF_BELOW. MEASURE (README 3b)
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
    K_Z=1.0,              # vertical gain (natural period of the height loop ~15 s: it is a slow, heavy axis)
    K_VZ=3.0,             # vertical braking gain (more damping than gain: the vertical axis coasts)
    Z_KI=0.1,             # vertical integrator: learns the ballast trim (thrust fraction per m*s). Keep slow (~T/4)
    Z_I_MAX=0.12,         # integrator clamp (thrust fraction); +-12 % of max V thrust ~ +-6 gf of trim error
    YR_CAP=0.5, VF_CAP=0.4, VS_CAP=0.4, VZ_CAP=0.3,
    POS_ALPHA=0.3, VEL_BETA=0.05,    # alpha-beta position/velocity filter on the vision fixes (prediction uses the commands)
    DUTY_MIN=0.1,         # duties below this are sent as 0: below the motor start threshold, and it keeps noise out of the props
    A_BRAKE=0.08,         # m/s^2 the controller assumes it can brake at (reverse thrust is weak): approach speed = sqrt(2*A_BRAKE*d)
    A_WALL=0.06,          # same idea for walls / obstacles, more conservative (touching a wall is worse than being late)
    V_LAG_S=1.2,          # s the velocity loop needs to respond; obstacle distances are judged that far ahead
    NUDGE_CLEAR=1.0,      # m of clearance the pilot asks for before the blind heading acquisition (it warns below this)
    SLIDE_V_MAX=0.25,     # m/s max speed when sliding along a wall / around an obstacle
    PERSON_LOST_MS=2500,  # -> hover (vision drops the person for up to ~1 s when they turn or get occluded)
    BALLOON_LOST_MS=1500, # -> disarm
    TELEM_LOST_MS=1000,   # armed and no telemetry for this long -> disarm (the board / fake_esp32 send at 20 Hz)
    BOARD_OFF_N=6,        # board says armed:0 in this many consecutive frames while we are armed -> disarm (1-3 are normal right after arming)
    AGE_WARN_MS=300,      # the board's 'ms since last command' above this -> print a link warning (its failsafe trips at 500)
    NUDGE_S=3.0, NUDGE_VF=0.3,   # heading-calibration nudge on first arm: fly forward, learn heading from the response
    TWITCH_S=2.0, TWITCH_AFTER_S=40,   # re-learn heading with a short forward push after this long without a manoeuvre
    K_ROT=0.9, ROT_MIN=0.08, ROT_CAP=0.7, ROT_DONE_RAD=0.09,   # rotate: yaw rate = K_ROT * remaining, floor ROT_MIN, cap ROT_CAP; done within 5 deg
    STANDOFF_ME=1.1, STANDOFF_PLACE=0.25, ARRIVE_M=0.2,   # "come here" stops 1.1 m from the person; places 0.25 m; arrived within +ARRIVE_M
    HOLD_DEADBAND=0.08, HOLD_V_MAX=0.3,   # hover = hold POSITION (gusts and the twitch would otherwise walk it away)
    HZ=15,
)
