"""Floor mat: several AprilTags at known spots on the floor -> where a camera is, in 2 s, no human step.

Why a mat and not one tag: a single 16 cm tag 2 m away is ~90 px wide. Its 4 corners fit a pose to 0.2 px but leave
the camera PITCH uncertain by ~3 deg, and from a low camera that is a 40 cm range error at 2.5 m (measured).
Four tags 1 m apart give 16 corners spread over a metre: pitch to ~0.2 deg.

Layout: tag id -> (x, y) or (x, y, yaw_rad) or (x, y, yaw_rad, size_m) on the floor. config.MAT is the nominal
layout (pages a known distance apart, all the same way up). Hand-taped pages are never that neat, so SURVEY the mat
once from any camera that sees every tag (tools/calib/survey_mat.py -> calib/mat.json): the pose and each page's
position, twist and printed size are fitted jointly; get_layout() prefers that file. World frame = tag 0's frame
(PROTOCOL s5): origin at the centre of tag 0, +X printed left->right, +Y bottom->top, +Z up.
A camera that sees only one tag still gets a pose (flagged n_tags=1: pitch uncertain).

  ids, corners = detect(det, frame, layout)            # visible mat tags
  rvec, tvec, err = solve_pose(cam, ids, corners, S, layout)
  cam2, info = auto_extrinsics(cam, stream, S, layout)  # sample 2 s, median, save calib/<name>_extrinsics.npz
  drift_px(cam, frame, det, S, layout)                  # has the camera (or the mat) moved since the saved pose?
"""
import functools, json, math, os, time
import cv2, numpy as np
from .. import config
from .calib_io import Camera, make_tag_detector, save_extrinsics, tag_object_points


def mat_layout(spacing=None, ids=(0, 1, 2, 3)):
    """Rectangular mat: ids[0] at the origin, then counter-clockwise (+X, +X+Y, +Y). spacing = centre-to-centre
    metres, one number (square) or (along +X, along +Y)."""
    if spacing is None:
        spacing = config.MAT_SPACING_M
    sx, sy = (float(spacing), float(spacing)) if np.isscalar(spacing) else (float(spacing[0]), float(spacing[-1]))
    return {ids[0]: (0.0, 0.0), ids[1]: (sx, 0.0), ids[2]: (sx, sy), ids[3]: (0.0, sy)}


def _entry(layout, i, tag_size):
    e = layout[i]
    return float(e[0]), float(e[1]), (float(e[2]) if len(e) > 2 else 0.0), (float(e[3]) if len(e) > 3 else float(tag_size))


def object_points(tag_size, layout, ids):
    """(4N, 3) world corners (TL, TR, BR, BL per tag, ArUco order) of the given tag ids, in that order.
    Each layout entry may carry the page's twist (yaw, rad CCW seen from above) and its own printed size."""
    out = []
    for i in ids:
        x, y, yaw, size = _entry(layout, i, tag_size)
        c, s = math.cos(yaw), math.sin(yaw)
        base = tag_object_points(size)
        rot = base @ np.float32([[c, s, 0], [-s, c, 0], [0, 0, 1]])        # row vectors: p' = p R^T
        out.append(rot + np.float32([x, y, 0.0]))
    return np.vstack(out).astype(np.float32)


LAYOUT_FILE = "mat.json"


def save_layout(calib_dir, layout, tag_size, **meta):
    os.makedirs(calib_dir, exist_ok=True)
    p = os.path.join(calib_dir, LAYOUT_FILE)
    with open(p, "w") as f:
        json.dump({"tag_size": tag_size, "tags": {str(i): [float(v) for v in e] for i, e in layout.items()}, **meta}, f, indent=1)
    return p


def load_layout(calib_dir):
    """Surveyed layout {id: (x, y, yaw, size)} from calib/mat.json, or None."""
    p = os.path.join(calib_dir, LAYOUT_FILE)
    if not os.path.exists(p):
        return None
    with open(p) as f:
        d = json.load(f)
    return {int(i): tuple(e) for i, e in d["tags"].items()}


@functools.lru_cache(maxsize=8)
def _cached_layout(spacing, calib_dir):
    if spacing is not None:
        return mat_layout(spacing)
    return load_layout(calib_dir) or dict(config.MAT)


def get_layout(spacing=None, calib_dir=None):
    """The layout to use: explicit --spacing wins, else the surveyed calib/mat.json, else config.MAT (nominal)."""
    calib_dir = config.CALIB_DIR if calib_dir is None else calib_dir
    key = None if spacing is None else (tuple(spacing) if not np.isscalar(spacing) else (float(spacing),))
    return _cached_layout(key, calib_dir)


def layout_source(spacing=None, calib_dir=None):
    calib_dir = config.CALIB_DIR if calib_dir is None else calib_dir
    if spacing is not None:
        return "--spacing"
    return f"{calib_dir}/{LAYOUT_FILE} (surveyed)" if load_layout(calib_dir) else "config.MAT (nominal, NOT surveyed)"


def detect(det, frame, layout=None):
    """Visible mat tags in frame -> (ids sorted ascending, corners (N, 4, 2) float32). Empty when none."""
    layout = get_layout() if layout is None else layout
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        return [], np.zeros((0, 4, 2), np.float32)
    keep = sorted((int(i), c[0]) for i, c in zip(ids.ravel(), corners) if int(i) in layout)
    if not keep:
        return [], np.zeros((0, 4, 2), np.float32)
    return [i for i, _ in keep], np.float32([c for _, c in keep])


def solve_pose(cam, ids, corners, tag_size, layout=None):
    """Camera pose from the visible mat tags -> (rvec, tvec, mean reprojection px). One tag: IPPE_SQUARE (pitch
    weakly observed); two or more: planar IPPE over every corner, then Levenberg-Marquardt refinement."""
    layout = get_layout() if layout is None else layout
    obj, img = object_points(tag_size, layout, ids), np.float32(corners).reshape(-1, 2)
    if len(ids) == 1:
        ok, rvec, tvec = cv2.solvePnP(obj, img, cam.K, cam.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    else:
        ok, rvec, tvec = cv2.solvePnP(obj, img, cam.K, cam.dist, flags=cv2.SOLVEPNP_IPPE)
        if ok:
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, cam.K, cam.dist, rvec, tvec)
    if not ok:
        return None, None, float("inf")
    proj = cv2.projectPoints(obj, rvec, tvec, cam.K, cam.dist)[0].reshape(-1, 2)
    return rvec, tvec, float(np.linalg.norm(proj - img, axis=1).mean())


def drift_px(cam, frame, det, tag_size, layout=None):
    """Mean px between the mat corners seen now and where the SAVED pose puts them. None = no mat tag in view.
    Big value = the camera (or the mat) moved since extrinsics."""
    layout = get_layout() if layout is None else layout
    ids, corners = detect(det, frame, layout)
    if not ids:
        return None
    rvec, _ = cv2.Rodrigues(cam.R)
    pred = cv2.projectPoints(object_points(tag_size, layout, ids), rvec, cam.t, cam.K, cam.dist)[0].reshape(-1, 2)
    return float(np.linalg.norm(pred - corners.reshape(-1, 2), axis=1).mean())


def pose_from_samples(cam, samples):
    """Element-wise median of (rvec, tvec) samples -> Camera with that pose, plus position spread (m) over the samples."""
    rv = np.median(np.array([s[0].ravel() for s in samples]), axis=0)
    tv = np.median(np.array([s[1].ravel() for s in samples]), axis=0)
    R, _ = cv2.Rodrigues(rv)
    pos = np.array([(-cv2.Rodrigues(s[0])[0].T @ s[1].reshape(3, 1)).ravel() for s in samples])
    # spread = 10th..90th percentile range of the positions (a couple of shaky frames must not veto a still camera)
    spread = np.linalg.norm(np.percentile(pos, 90, axis=0) - np.percentile(pos, 10, axis=0)) if len(pos) > 1 else 0.0
    return Camera(cam.name, cam.K, cam.dist, cam.size, R, tv), float(spread)


def auto_extrinsics(cam, stream, tag_size=None, layout=None, seconds=2.0, min_samples=8, max_spread=0.03,
                    max_reproj=3.0, calib_dir=None, save=True, det=None, sleep=time.sleep, clock=time.monotonic):
    """Watch `stream` (.latest() -> (frame, t_ms)) for `seconds`, solve the mat pose on every new frame, take the median.
    Returns (Camera with the new pose, info) or (None, info) with info["why"] set. info: n_samples, n_tags (median
    tags per sample), reproj (px), spread (m). Saves calib/<name>_extrinsics.npz (tag_size, rms, n_tags) when save."""
    tag_size = config.TAG_SIZE_M if tag_size is None else tag_size
    calib_dir = config.CALIB_DIR if calib_dir is None else calib_dir
    layout = get_layout(None, calib_dir) if layout is None else layout
    det = det or make_tag_detector()
    samples, n_tags, errs, last = [], [], [], None
    t0 = clock()
    while clock() - t0 < seconds:
        frame, t = stream.latest()
        if frame is None or t == last:
            sleep(0.01); continue
        last = t
        ids, corners = detect(det, frame, layout)
        if not ids:
            continue
        rvec, tvec, err = solve_pose(cam, ids, corners, tag_size, layout)
        if rvec is None or err > max_reproj:
            continue
        samples.append((rvec, tvec)); n_tags.append(len(ids)); errs.append(err)
    if n_tags:                                       # someone walking past hides a page: keep only the frames that saw the most tags
        best = max(n_tags)
        keep = [i for i, n in enumerate(n_tags) if n == best]
        if len(keep) >= min_samples:
            samples, n_tags, errs = [samples[i] for i in keep], [n_tags[i] for i in keep], [errs[i] for i in keep]
    info = {"n_samples": len(samples), "n_tags": int(np.median(n_tags)) if n_tags else 0,
            "reproj": float(np.median(errs)) if errs else None, "spread": None, "why": None, "cam": None}
    if len(samples) < min_samples:
        info["why"] = f"mat seen in only {len(samples)} frames in {seconds:.0f} s (need {min_samples}): tags in view? in focus? too small?"
        return None, info
    cam2, spread = pose_from_samples(cam, samples)
    info["spread"] = spread
    info["cam"] = cam2                               # the median pose, usable as a last resort even when the spread is too big
    if spread > max_spread:
        info["why"] = (f"camera position jumped {spread * 100:.1f} cm during sampling: hold the camera still, "
                       f"nobody in front of the mat")
        return None, info
    if save:
        save_extrinsics(calib_dir, cam.name, cam2.R, cam2.t, tag_size=tag_size, rms=info["reproj"], n_tags=info["n_tags"])
    return cam2, info


def describe(cam, info):
    p = cam.position()
    warn = "  (ONE tag only: pitch uncertain, tape the other mat tags)" if info["n_tags"] == 1 else ""
    return (f"camera {cam.name} at world X={p[0]:+.2f} Y={p[1]:+.2f} Z={p[2]:+.2f} m  {info['n_tags']} tags  "
            f"reproj {info['reproj']:.2f} px  spread {info['spread'] * 100:.1f} cm  ({info['n_samples']} samples){warn}")


# ------------------------------------------------------------------ survey
def _floor_hit(K, dist, R, t, uv):
    """World point where the ray through pixel uv meets the floor (z = 0)."""
    n = cv2.undistortPoints(np.float32([[uv]]), K, dist).ravel()
    d = R.T @ np.array([n[0], n[1], 1.0]); o = (-R.T @ np.asarray(t).reshape(3, 1)).ravel()
    return o + (-o[2] / d[2]) * d


def survey(cam, ids, corners, tag_size, ref_id=0, iters=30):
    """Layout of every visible tag in ref_id's frame, from ONE view, jointly with the camera pose.
    Alternates: pose = PnP on the current layout (all corners); layout = corners back-projected onto the floor with
    that pose, each page re-fitted as a square (centre, twist, printed size). Returns (layout, rvec, tvec, reproj px).
    The reference page is by definition at (0, 0, 0, tag_size); every other page's size is measured against it."""
    ids = list(ids)
    if ref_id not in ids:
        raise ValueError(f"reference tag {ref_id} not in view (seen {ids})")
    k = ids.index(ref_id)
    layout = {ref_id: (0.0, 0.0, 0.0, float(tag_size))}
    rvec, tvec, _ = solve_pose(cam, [ref_id], corners[k:k + 1], tag_size, layout)
    err = float("inf")
    for _ in range(iters):
        R, _ = cv2.Rodrigues(rvec)
        for i, c in zip(ids, corners):
            if i == ref_id:
                continue
            pts = np.array([_floor_hit(cam.K, cam.dist, R, tvec, uv) for uv in c])       # TL, TR, BR, BL on the floor
            ex = (pts[1] - pts[0]) + (pts[2] - pts[3]); ey = (pts[0] - pts[3]) + (pts[1] - pts[2])
            v = np.array([ex[0], ex[1]]) + np.array([ey[1], -ey[0]])                       # +x edge and (+y edge rotated -90)
            yaw = math.atan2(v[1], v[0])
            size = 0.25 * (np.linalg.norm(ex) + np.linalg.norm(ey))
            layout[i] = (float(pts[:, 0].mean()), float(pts[:, 1].mean()), float(yaw), float(size))
        r2, t2, e2 = solve_pose(cam, ids, corners, tag_size, layout)
        if r2 is None:
            break
        rvec, tvec, err = r2, t2, e2
        if abs(err - e2) < 1e-4 and err < 0.05:
            break
    return layout, rvec, tvec, err


def fit_f(cam, ids, corners, tag_size, ref_id=0, span=(0.55, 1.35), steps=33):
    """Focal length from the mat alone: the non-reference pages are the same print, so only the right f makes their
    back-projected sizes equal (a wrong f makes far pages come out a different size than near ones). Scans f over
    span x the long side, then refines. Returns (f_px, size spread as a fraction, survey residual px)."""
    long = max(cam.size)

    def spread_at(fpx):
        K = np.array([[fpx, 0, cam.K[0, 2]], [0, fpx, cam.K[1, 2]], [0, 0, 1]], float)
        c = Camera(cam.name, K, cam.dist, cam.size)
        lay, _, _, e = survey(c, ids, corners, tag_size, ref_id)
        sizes = np.array([lay[i][3] for i in ids if i != ref_id])
        return float(sizes.std() / sizes.mean()), e
    grid = np.linspace(span[0] * long, span[1] * long, steps)
    coarse = [(fpx,) + spread_at(fpx) for fpx in grid]
    best = min(coarse, key=lambda r: r[1])
    step = grid[1] - grid[0]
    fine = [(fpx,) + spread_at(fpx) for fpx in np.linspace(best[0] - step, best[0] + step, 9)]
    return min(fine, key=lambda r: r[1])


def describe_layout(layout, tag_size):
    lines = []
    for i in sorted(layout):
        x, y, yaw, size = _entry(layout, i, tag_size)
        lines.append(f"  tag {i}: x={x:+.3f} y={y:+.3f} m  twist {math.degrees(yaw):+.1f} deg  printed {size * 1000:.0f} mm")
    return "\n".join(lines)


# ------------------------------------------------------------------ runtime helpers (mono / localize)
class DriftGuard:
    """Per-camera state of the once-a-second drift check. update(drift_px) -> "ok" | "bad" | "resolve".
    bad     : the saved pose no longer matches the mat -> the caller publishes lost="drift", NOT fixes from a stale pose
    resolve : bad for `bad_checks` checks running -> the caller re-solves the pose (auto-calib) and calls reset()
    None (no mat tag in view: occluded, or the camera never saw it) is neither: the last verdict stands."""

    def __init__(self, drift_px=8.0, bad_checks=2):
        self.drift_px, self.bad_checks, self.n_bad, self.last = float(drift_px), int(bad_checks), 0, None

    def update(self, drift):
        self.last = drift
        if drift is None:
            return "bad" if self.n_bad else "ok"
        if drift <= self.drift_px:
            self.n_bad = 0
            return "ok"
        self.n_bad += 1
        return "resolve" if self.n_bad >= self.bad_checks else "bad"

    @property
    def bad(self):
        return self.n_bad > 0

    def reset(self):
        self.n_bad = 0


def load_camera(name, calib_dir, stream, auto_calib, tag_size=None, layout=None, log=print):
    """Camera with a pose for mono / localize.
    No intrinsics file + auto_calib -> nominal pinhole from the stream's own frame size (saved, WARN).
    auto_calib -> pose from the mat seen by `stream` (saved); mat not found -> the saved extrinsics; none -> error."""
    tag_size = config.TAG_SIZE_M if tag_size is None else tag_size
    ip = os.path.join(calib_dir, f"{name}_intrinsics.npz")
    if auto_calib and not os.path.exists(ip):
        frame, _ = stream.latest()
        h, w = frame.shape[:2]
        cam0 = Camera.nominal(name, (w, h), calib_dir)
        log(f"[auto-calib] WARN: no {ip}; wrote a nominal pinhole for {w}x{h} (fx {cam0.K[0, 0]:.0f}, ~5 % range error). "
            f"Chessboard it when there is time: tools/calib/intrinsics.py --name {name}")
    if auto_calib:
        cam0 = Camera.load(name, calib_dir, need_extrinsics=False)
        info = {}
        for attempt in range(3):                    # the lid shakes for a second after Enter: try again before giving up
            cam, info = auto_extrinsics(cam0, stream, tag_size, layout, calib_dir=calib_dir)
            if cam is not None:
                log("[auto-calib] " + describe(cam, info))
                return cam
            log(f"[auto-calib] {info['why']}" + (" -> trying again" if attempt < 2 else ""))
        if info.get("cam") is not None:              # shaky but seen: the median of 60 frames beats a pose from another day
            cam = info["cam"]
            save_extrinsics(calib_dir, name, cam.R, cam.t, tag_size=tag_size, rms=info["reproj"], n_tags=info["n_tags"])
            log(f"[auto-calib] WARN: using the median pose anyway (spread {info['spread'] * 100:.0f} cm; positions may be off by that much). "
                + describe(cam, info))
            return cam
        ep = os.path.join(calib_dir, f"{name}_extrinsics.npz")
        if os.path.exists(ep):
            log("[auto-calib] -> using the saved extrinsics (from an earlier run: the drift check will say if the camera moved)")
        else:
            raise SystemExit(f"[auto-calib] could not solve the camera pose from the mat and there is no saved {ep}. "
                             "Keep the laptop still, all four tags in view and in focus, then run again.")
    return Camera.load(name, calib_dir)
