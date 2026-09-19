"""Pre-flight checks for the camera rig: files, geometry, tag size, models, venue (offline) and fps / skew / tag / drift (live).

  python tools/vision_check.py                 # offline: are the calib files real, are the cameras placed sanely, does the tag size agree
  python tools/vision_check.py --live          # + open config.SOURCES for 3 s: fps, skew, tag id 0 visible in both, tag drift
  python tools/vision_check.py --live --person # + YOLO must see a person in both cameras

Every check returns a list of (status, name, detail) with status "OK" | "WARN" | "FAIL"; nothing here prints or opens
windows, so tools/vision_test.py can run it on synthetic files and fake streams. Mirrors the go/no-go list in README
section 3 and the Build Blueprint section 7. ultralytics is imported only inside live_checks (person / balloon mode).
"""
import importlib.util, re, statistics, time
from pathlib import Path
import numpy as np
from .. import config
from . import floor
from .calib_io import Camera, make_tag_detector

OK, WARN, FAIL = "OK", "WARN", "FAIL"


# ------------------------------------------------------------------ offline
def check_intrinsics(name, calib_dir=config.CALIB_DIR):
    p = Path(calib_dir) / f"{name}_intrinsics.npz"
    tag = f"intrinsics {name}"
    if not p.exists():
        return [(FAIL, tag, f"{p} missing: python tools/calib/intrinsics.py --name {name} --source ...")]
    d = np.load(p)
    K, dist, size = np.asarray(d["K"], float), np.asarray(d["dist"], float), d["size"]
    rms = float(d["rms"]) if "rms" in d else -1.0
    try:
        w, h = int(size[0]), int(size[1]); assert w > 0 and h > 0
    except Exception:
        return [(FAIL, tag, f"size {size!r} is not two positive ints")]
    fx, cx, cy = K[0, 0], K[0, 2], K[1, 2]
    out = []
    if rms < 0 or not np.any(dist):
        out.append((WARN, tag, f"nominal pinhole (intrinsics.py --nominal): fx {fx:.0f}, no distortion -> ~5 % range error. "
                               "Run a real checkerboard calibration when there is time"))
    elif rms > 1.0:
        out.append((FAIL, tag, f"rms {rms:.2f} px > 1: redo intrinsics (lock focus, more views, flat board)"))
    elif rms > 0.6:
        out.append((WARN, tag, f"rms {rms:.2f} px (0.6..1): usable; redo if the tape-measure test disagrees"))
    else:
        out.append((OK, tag, f"rms {rms:.2f} px, {w}x{h}, fx {fx:.0f}"))
    if not (0.3 * w <= cx <= 0.7 * w and 0.3 * h <= cy <= 0.7 * h):
        out.append((WARN, f"{tag} principal point", f"({cx:.0f}, {cy:.0f}) outside the central 30-70 % of {w}x{h}: suspicious fit"))
    return out


def check_extrinsics(name, calib_dir=config.CALIB_DIR, tag_size=config.TAG_SIZE_M):
    tag = f"extrinsics {name}"
    ip, ep = Path(calib_dir) / f"{name}_intrinsics.npz", Path(calib_dir) / f"{name}_extrinsics.npz"
    if not ep.exists():
        return [(FAIL, tag, f"{ep} missing: python tools/calib/extrinsics.py --name {name} --source ...")]
    if not ip.exists():
        return [(FAIL, tag, "cannot check: intrinsics missing")]
    cam = Camera.load(name, calib_dir)
    d = np.load(ep)
    out = []
    pos = cam.position(); z = float(pos[2])
    if z < 0.3 or z > 5.0:
        out.append((FAIL, tag, f"camera at Z={z:.2f} m: below the floor or above any stand. Re-run extrinsics (tag flat, id 0, right size)"))
    elif not 0.5 <= z <= 4.0:
        out.append((WARN, tag, f"camera at Z={z:.2f} m: outside the 0.5-4 m a stand gives; check with a tape"))
    else:
        out.append((OK, tag, f"camera at X={pos[0]:+.2f} Y={pos[1]:+.2f} Z={z:.2f} m"))
    axis = cam.R.T @ np.array([0.0, 0.0, 1.0])                      # optical axis in the world frame
    if axis[2] > -0.15:
        out.append((WARN, f"{tag} aim", f"optical axis z component {axis[2]:+.2f}: not looking down; the floor tag should sit in the lower half of the image"))
    x_cam = cam.R @ np.zeros(3) + cam.t.ravel()
    if x_cam[2] <= 0:
        out.append((FAIL, f"{tag} tag", "the floor tag centre is BEHIND the camera: pose is garbage, re-run extrinsics"))
    else:
        u, v = cam.project((0.0, 0.0, 0.0))
        if not (0 <= u < cam.size[0] and 0 <= v < cam.size[1]):
            out.append((FAIL, f"{tag} tag", f"the floor tag centre projects to ({u:.0f}, {v:.0f}), outside the {cam.size[0]}x{cam.size[1]} image"))
    if "tag_size" in d:
        saved = float(d["tag_size"])
        if abs(saved - tag_size) > 1e-3:
            out.append((FAIL, f"{tag} tag size", f"computed for a {saved * 1000:.0f} mm tag, config.TAG_SIZE_M is {tag_size * 1000:.0f} mm: "
                                                 f"{saved / tag_size:.2f}x scale error in every position. Re-run extrinsics"))
        else:
            out.append((OK, f"{tag} tag size", f"{saved * 1000:.0f} mm, matches config"))
    else:
        out.append((WARN, f"{tag} tag size", "not recorded: re-run extrinsics.py so the size is checked"))
    if "rms" in d and float(d["rms"]) > 3.0:
        out.append((WARN, f"{tag} fit", f"tag reprojection rms {float(d['rms']):.1f} px at calibration time (> 3)"))
    if "n_tags" in d and int(d["n_tags"]) < 2:
        out.append((WARN, f"{tag} mat", "pose from ONE tag: pitch uncertain (~3 deg = 40 cm at 2.5 m from a low camera). Tape mat tags 1-3 and re-run"))
    return out


def check_pair(calib_dir=config.CALIB_DIR, names=("A", "B")):
    cams = []
    for n in names:
        try:
            cams.append(Camera.load(n, calib_dir))
        except FileNotFoundError:
            return []                                                   # reported per camera already
    a, b = cams[0], cams[1]
    if np.allclose(a.R, b.R) and np.allclose(a.t, b.t):
        return [(FAIL, "camera pair", f"{names[0]} and {names[1]} have identical extrinsics: one file was copied. Calibrate both")]
    base = float(np.linalg.norm(a.position() - b.position()))
    if base < 0.1:
        return [(FAIL, "camera pair", f"baseline {base:.2f} m: cameras on top of each other, no depth")]
    if not 0.5 <= base <= 4.0:
        return [(WARN, "camera pair", f"baseline {base:.2f} m (want 1-2 m for depth at 2-4 m range)")]
    return [(OK, "camera pair", f"baseline {base:.2f} m")]


def check_targets(targets_dir="targets", tag_size=config.TAG_SIZE_M, tag_id=0):
    files = sorted(Path(targets_dir).glob(f"apriltag36h11_id{tag_id}_*mm_A4.png"))
    if not files:
        return [(WARN, "printed target", f"no {targets_dir}/apriltag36h11_id{tag_id}_<mm>mm_A4.png: python tools/calib/make_targets.py, print at 100 %, MEASURE")]
    out = []
    missing = [i for i in config.MAT if i != tag_id and not list(Path(targets_dir).glob(f"apriltag36h11_id{i}_*mm_A4.png"))]
    if missing:
        out.append((WARN, "mat targets", f"mat tags {missing} not generated: python tools/calib/make_targets.py (one tag = pitch uncertain)"))
    for f in files:
        m = re.search(r"_(\d+(?:\.\d+)?)mm_", f.name)
        mm = float(m.group(1)) if m else float("nan")
        if abs(mm - tag_size * 1000) > 1:
            out.append((WARN, "printed target", f"{f.name} is {mm:.0f} mm, config.TAG_SIZE_M is {tag_size * 1000:.0f} mm: do not print this one"))
        else:
            out.append((OK, "printed target", f"{f.name} matches config ({mm:.0f} mm); measure the print after printing"))
    return out


def check_models(person=config.YOLO_PERSON, balloon=config.BALLOON_WEIGHTS, color=config.BALLOON_COLOR):
    out = []
    if importlib.util.find_spec("ultralytics") is None:
        out.append((FAIL, "ultralytics", "not installed: pip install -r requirements.txt"))
    try:
        import cv2
        if not hasattr(cv2, "aruco"):
            out.append((FAIL, "opencv aruco", "cv2.aruco missing: need opencv-contrib-python (AprilTag detection)"))
    except ImportError:
        out.append((FAIL, "opencv", "cv2 not importable"))
    if Path(person).exists():
        out.append((OK, "person weights", f"{person} on disk"))
    else:
        out.append((WARN, "person weights", f"{person} not on disk: the first run downloads it. Do that NOW while online, not on hackathon WiFi"))
    if Path(balloon).exists():
        out.append((OK, "balloon weights", f"{balloon}: site fine-tune present"))
    elif Path("models/balloon_web.pt").exists():
        out.append((WARN, "balloon weights", "models/balloon_web.pt (web-pretrained) will be used; do the ~10 min site fine-tune in the demo light (README)"))
    elif color:
        out.append((WARN, "balloon weights", f"no YOLO balloon weights: colour blob ({color}) fallback"))
    else:
        out.append((WARN, "balloon weights", "no YOLO balloon weights and no colour: COCO 'sports ball' fallback (weak)"))
    return out


def check_venue(path=config.VENUE_FILE):
    from ..positioning import venue
    try:
        v = venue.load(path)
    except (FileNotFoundError, ValueError) as e:
        return [(FAIL, "venue", str(e))]
    errors, warnings = venue.validate(v, r_balloon=config.R_BALLOON)
    out = [(FAIL, "venue", e) for e in errors] + [(WARN, "venue", w) for w in warnings]
    if "PLACEHOLDER" in v.notes.upper():
        out.append((WARN, "venue", f"{Path(path).name}: placeholder geometry; tape-measure the room or use capture_place"))
    if not out:
        out.append((OK, "venue", f"{v.name}: arena {v.arena}, {len(v.obstacles)} obstacle(s), places {sorted(v.places)}"))
    return out


def offline_checks(calib_dir=config.CALIB_DIR, names=("A", "B"), tag_size=config.TAG_SIZE_M, targets_dir="targets"):
    out = []
    for n in names:
        out += check_intrinsics(n, calib_dir) + check_extrinsics(n, calib_dir, tag_size)
    if len(names) >= 2:
        out += check_pair(calib_dir, names)
    lay = floor.load_layout(calib_dir)
    if lay:
        xs = [e[0] for e in lay.values()]; ys = [e[1] for e in lay.values()]
        out.append((OK, "mat layout", f"surveyed ({calib_dir}/{floor.LAYOUT_FILE}): {len(lay)} tags over {max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} m"))
    else:
        out.append((WARN, "mat layout", "not surveyed: pages assumed exactly at config.MAT. python tools/calib/survey_mat.py --source ... once the mat is taped"))
    out += check_targets(targets_dir, tag_size) + check_models() + check_venue()
    return out


# ------------------------------------------------------------------ live
def live_checks(streams, cams, seconds=3.0, tag_size=config.TAG_SIZE_M, drift_px=8.0, skew_ms=100, min_fps=10,
                person=False, sleep=time.sleep, clock=time.monotonic):
    """streams: {name: obj with .latest() -> (frame, t_ms)}; cams: {name: Camera}. Samples for `seconds`, then judges
    frame size, fps, inter-frame gaps, A/B skew, tag id 0 visibility and size, tag drift vs the saved extrinsics."""
    det = make_tag_detector()
    names = list(streams)
    seen_t = {n: [] for n in names}; shape = {n: None for n in names}
    tag_seen = {n: 0 for n in names}; tag_w = {n: [] for n in names}; drift = {n: [] for n in names}; n_tags = {n: [] for n in names}
    last = {n: None for n in names}; skews = []
    t0 = clock()
    while clock() - t0 < seconds:
        cur = {}
        for n in names:
            frame, t = streams[n].latest()
            cur[n] = t
            if frame is None or t is None or t == last[n]:
                continue
            last[n] = t; seen_t[n].append(t); shape[n] = frame.shape
            ids, corners = floor.detect(det, frame)
            if ids:
                tag_seen[n] += 1; n_tags[n].append(len(ids))
                tag_w[n].append(float(np.median([np.linalg.norm(c[0] - c[1]) for c in corners])))
                dr = floor.drift_px(cams[n], frame, det, tag_size)
                if dr is not None: drift[n].append(dr)
        if len(names) >= 2 and all(cur[n] is not None for n in names):
            skews.append(abs(cur[names[0]] - cur[names[1]]))
        sleep(0.05)
    el = max(1e-3, clock() - t0)
    out = []
    for n in names:
        cam = cams[n]; ts = seen_t[n]
        if not ts:
            out.append((FAIL, f"camera {n}", "no frames")); continue
        if shape[n][1::-1] != tuple(cam.size):
            out.append((FAIL, f"camera {n} size", f"frames are {shape[n][1]}x{shape[n][0]} but the intrinsics were made at {cam.size[0]}x{cam.size[1]}: "
                                                   "same camera, same resolution setting as when calibrated"))
        fps = len(ts) / el
        out.append((OK if fps >= min_fps else FAIL, f"camera {n} fps", f"{fps:.1f} fps over {el:.1f} s" + ("" if fps >= min_fps else f" (< {min_fps})")))
        gaps = np.diff(ts) if len(ts) > 1 else np.array([0.0])
        if gaps.max() > 500:
            out.append((WARN, f"camera {n} gaps", f"a {gaps.max():.0f} ms hole between frames (WiFi / phone sleeping?)"))
        frac = tag_seen[n] / len(ts)
        if frac < 0.5:
            out.append((FAIL, f"camera {n} tag", f"floor mat seen in {frac * 100:.0f} % of frames: not in view, occluded, or too small"))
        else:
            w, k = statistics.median(tag_w[n]), int(statistics.median(n_tags[n]))
            out.append((OK if w >= 60 else WARN, f"camera {n} tag", f"{k} mat tag(s) seen in {frac * 100:.0f} % of frames, {w:.0f} px wide" + ("" if w >= 60 else " (< 60 px: move closer or bigger tag)")))
            if k < 2:
                out.append((WARN, f"camera {n} mat", f"only {k} mat tag in view: pitch uncertain, tape tags 0-3 where this camera sees them all"))
            if drift[n]:
                dm = statistics.median(drift[n])
                out.append((OK if dm < drift_px else FAIL, f"camera {n} drift", f"tag {dm:.1f} px from where the extrinsics expect it" + ("" if dm < drift_px else f" (> {drift_px}): camera or tag moved, re-run extrinsics")))
    if skews:
        sk = max(skews)
        out.append((OK if sk < skew_ms else FAIL, "A/B skew", f"max {sk:.0f} ms" + ("" if sk < skew_ms else f" (> {skew_ms} ms: frames will be rejected by localize)")))
    if person:
        from .detect import PersonTracker
        pt = PersonTracker(config.YOLO_PERSON)
        for n in names:
            frame, _ = streams[n].latest()
            hit = bool(frame is not None and pt.detect(frame))
            out.append((OK if hit else FAIL, f"camera {n} person", "YOLO sees a person" if hit else "no person detected: stand in view, check lighting / exposure lock"))
    try:
        from .detect import BalloonDetector
        out.append((OK, "balloon detector", BalloonDetector(config.BALLOON_WEIGHTS, color=config.BALLOON_COLOR).mode))
    except Exception as e:                                             # ultralytics missing / weights unreadable
        out.append((WARN, "balloon detector", f"could not build: {e}"))
    return out


def worst(checks):
    return 1 if any(s == FAIL for s, _, _ in checks) else 0


def format_checks(checks):
    return "\n".join(f"[check] {s:4s}  {n:28s} {d}" for s, n, d in checks)
