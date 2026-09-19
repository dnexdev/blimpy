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
#     Key from the sponsor email -> set OMNI_API_KEY in the shell (never in the repo). Usage ledger: data/omni_usage.jsonl
OMNI = dict(
    CAMERA="0",              # what Blimpy SEES in conversation (laptop webcam facing the user at the desk). Not one of SOURCES
                             # above if localize.py is running on this laptop: Windows gives a webcam to ONE process only.
    FPS=1.0,                 # frames per second sent to the realtime model (provider recommends 1)
    WATCH_S=6.0,             # focus watcher: seconds between "what is the user doing" looks (laptop/voice/omni_watch.py)
    HALF_DUPLEX=True,        # laptop speakers + laptop mic (no echo cancellation): mute the mic while Blimpy talks.
                             # False with headphones or a conference speaker -> real barge-in.
)

# --- Network ---
ESP32_IP = "wisp-9910.local"      # the flight board (Build Blueprint). Spare = "wisp-91c8.local". Name = "wisp-" + last 4 hex of the serial; printed at boot. IP changes, name does not.

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
