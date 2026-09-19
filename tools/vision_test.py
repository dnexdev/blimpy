"""Offline regression test of the vision stack: no cameras, no YOLO (numpy + opencv-contrib only).

  python tools/vision_test.py

(1) triangulate: synthetic two-camera rig, known points, pixel noise, mismatch rejection.
(2) extrinsics round-trip: render the floor AprilTag into synthetic views, recover the camera pose with the same
    code path as tools/calib/extrinsics.py, check position/orientation and the PROTOCOL.md world-frame convention.
(3) localize.step: fake detectors drive one tick at a time; checks the PROTOCOL s4 message, skew/stale gating,
    the reprojection and height gates, and that YOLO is not re-run on frames already processed.
(4) mono (single camera): feet-on-floor person position, size-based balloon depth, mono.step gating.
(5) colour-blob balloon detector on a synthetic room: picks the disc, ignores paper, walls and cables.
(6) preflight (tools/vision_check.py logic) on temp calib files and fake streams: real vs nominal vs bad intrinsics,
    camera geometry, tag-size mismatch, copied files, baseline, printed-target name, live fps / skew / tag / drift.
Run it after touching laptop/vision/* or tools/calib/*.
"""
import json, math, os, pathlib, sys, tempfile
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
import cv2, numpy as np
from laptop.vision.calib_io import TAG_DICT, Camera, make_tag_detector, save_extrinsics, save_intrinsics, tag_object_points
from laptop.vision import floor, preflight
from laptop.vision.localize import step, tag_drift_px
from laptop.vision import mono
from laptop.vision.detect import ColorBalloonDetector
from laptop.vision.triangulate import triangulate

SIZE = (1280, 720)


# ---------- synthetic rig ----------
def unit(v):
    return v / np.linalg.norm(v)


def look_at(pos, target, up=(0, 0, 1)):
    """OpenCV camera (x right, y down, z forward) at pos looking at target. Returns R, t with x_cam = R X + t."""
    z = unit(np.float64(target) - np.float64(pos)); x = unit(np.cross(z, np.float64(up))); y = np.cross(z, x)
    R = np.vstack([x, y, z]); t = -R @ np.float64(pos)
    return R, t


def synth_cam(name, pos, target, f=1000.0, size=SIZE, dist=(-0.12, 0.05, 0.001, -0.001, 0.0)):
    K = [[f, 0, size[0] / 2], [0, f, size[1] / 2], [0, 0, 1]]
    R, t = look_at(pos, target)
    return Camera(name, K, dist, size, R, t)


def observe(cam, X):
    """DISTORTED pixel of world point X: what a real detector reports (exercises Camera.undistort)."""
    rvec, _ = cv2.Rodrigues(cam.R)
    return cv2.projectPoints(np.float64([X]), rvec, cam.t, cam.K, cam.dist)[0].ravel()


def in_view(cam, uv):
    return 0 <= uv[0] < cam.size[0] and 0 <= uv[1] < cam.size[1]


def rig():
    return {"A": synth_cam("A", (-1.0, -1.5, 2.2), (0, 0, 0.8)), "B": synth_cam("B", (1.0, -1.5, 2.2), (0, 0, 0.8))}


# ---------- (1) ----------
def test_triangulate():
    cams = rig()
    ok = True
    for n, pos in (("A", (-1.0, -1.5, 2.2)), ("B", (1.0, -1.5, 2.2))):
        if np.linalg.norm(cams[n].position() - pos) > 1e-9:
            print(f"[triangulate] FAIL: position() of {n} = {cams[n].position()}"); ok = False
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(-1, 1, 400), rng.uniform(-1, 1, 400), rng.uniform(0, 2, 400)])
    obs = [(X, observe(cams["A"], X), observe(cams["B"], X)) for X in pts]
    obs = [o for o in obs if in_view(cams["A"], o[1]) and in_view(cams["B"], o[2])][:200]
    if len(obs) < 100:
        print(f"[triangulate] FAIL: only {len(obs)} points in view"); return False

    e3, ep = [], []
    for X, ua, ub in obs:
        Y, err = triangulate(cams["A"], cams["B"], ua, ub)
        e3.append(np.linalg.norm(Y - X)); ep.append(err)
    clean = max(e3) < 1e-3 and max(ep) < 1e-3
    print(f"[triangulate] noise-free: max 3D err {max(e3) * 1000:.4f} mm, max reproj {max(ep):.5f} px  {'OK' if clean else 'FAIL'}")

    e3, ep = [], []
    for X, ua, ub in obs:
        Y, err = triangulate(cams["A"], cams["B"], ua + rng.normal(0, 0.5, 2), ub + rng.normal(0, 0.5, 2))
        e3.append(np.linalg.norm(Y - X)); ep.append(err)
    noisy = np.mean(e3) < 0.005 and max(e3) < 0.02 and max(ep) < 3
    print(f"[triangulate] 0.5 px noise: mean 3D err {np.mean(e3) * 1000:.1f} mm, max {max(e3) * 1000:.1f} mm, "
          f"max reproj {max(ep):.2f} px  {'OK' if noisy else 'FAIL'}")

    # Mismatched pairs (point i in A, point j in B). With two views the reprojection error only measures the
    # distance to the epipolar line, so a mismatch that happens to lie near it slips through: --max-reproj is a
    # partial guard and the height gate in localize.step complements it.
    errs = [triangulate(cams["A"], cams["B"], obs[i][1], obs[(i + 7) % len(obs)][2])[1] for i in range(0, len(obs), 5)]
    caught = sum(e > 15 for e in errs) / len(errs)
    mism = np.median(errs) > 15 and caught > 0.8
    print(f"[triangulate] mismatched pairs: median reproj err {np.median(errs):.0f} px, {caught * 100:.0f}% over 15 px "
          f"(epipolar-line mismatches slip through by design)  {'OK' if mism else 'FAIL'}")
    return ok and clean and noisy and mism


# ---------- (2) ----------
def render_tag(cam, S, marker_px=200, quiet=60):
    """Warp a rendered 36h11 id-0 marker onto the floor as seen by cam (distortion-free cam only)."""
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(TAG_DICT), 0, marker_px)
    canvas = np.full((marker_px + 2 * quiet, marker_px + 2 * quiet), 255, np.uint8)
    canvas[quiet:quiet + marker_px, quiet:quiet + marker_px] = marker
    q, m = quiet, quiet + marker_px
    src = np.float32([[q, q], [m, q], [m, m], [q, m]])            # TL, TR, BR, BL of the black square in the print
    dst = np.float32([cam.project(p) for p in tag_object_points(S)])
    H = cv2.getPerspectiveTransform(src, dst)
    img = cv2.warpPerspective(canvas, H, cam.size, borderValue=255)
    return img, H


def recover_pose(cam, img, S):
    corners, ids, _ = make_tag_detector().detectMarkers(img)
    if ids is None or 0 not in ids.ravel():
        return None, None
    c = corners[list(ids.ravel()).index(0)]
    ok, rvec, tvec = cv2.solvePnP(tag_object_points(S), c[0], cam.K, cam.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None, None
    R, _ = cv2.Rodrigues(rvec)
    return Camera(cam.name, cam.K, cam.dist, cam.size, R, tvec), c[0]


def test_extrinsics_roundtrip():
    S = 0.18
    truth_pos = {"A": (0.5, -0.9, 1.4), "B": (-0.6, -0.8, 1.5)}
    cams = {n: synth_cam(n, p, (0, 0, 0), dist=(0, 0, 0, 0, 0)) for n, p in truth_pos.items()}
    rec, corners, ok = {}, {}, True
    for n, cam in cams.items():
        img, H = render_tag(cam, S)
        r, c = recover_pose(cam, img, S)
        if r is None:
            print(f"[extrinsics] FAIL: tag id 0 not detected in synthetic view {n}"); return False
        rec[n], corners[n] = r, c
        dpos = np.linalg.norm(r.position() - truth_pos[n])
        cosang = np.clip((np.trace(cam.R.T @ r.R) - 1) / 2, -1, 1); dang = math.degrees(math.acos(cosang))
        # convention checks independent of look_at(): +Z up, +X = printed right, +Y = printed top
        q, m = 60, 260
        right_print = cv2.perspectiveTransform(np.float32([[[m, (q + m) / 2]]]), H).ravel()
        top_print = cv2.perspectiveTransform(np.float32([[[(q + m) / 2, q]]]), H).ravel()
        centre = r.project((0, 0, 0))
        x_world = r.project((S / 2, 0, 0)); y_world = r.project((0, S / 2, 0))
        conv = (r.position()[2] > 0
                and np.linalg.norm(x_world - right_print) < 3 and np.linalg.norm(y_world - top_print) < 3
                and np.linalg.norm(x_world - centre) > 20)
        good = dpos < 0.03 and dang < 2 and conv
        ok &= good
        print(f"[extrinsics {n}] recovered pos err {dpos * 100:.1f} cm, rot err {dang:.2f} deg, "
              f"convention {'ok' if conv else 'WRONG'}  {'OK' if good else 'FAIL'}")
    # drift check used by localize: 0 px when nothing moved, large after the camera shifts
    img, _ = render_tag(cams["A"], S)
    d0 = tag_drift_px(rec["A"], img, make_tag_detector(), tag_object_points(S))
    moved = synth_cam("A", (0.5, -0.9, 1.4), (0.03, 0, 0), dist=(0, 0, 0, 0, 0))   # ~1 degree aim change
    d1 = tag_drift_px(rec["A"], render_tag(moved, S)[0], make_tag_detector(), tag_object_points(S))
    d2 = tag_drift_px(rec["A"], np.full(cams["A"].size[::-1], 255, np.uint8), make_tag_detector(), tag_object_points(S))
    dr = d0 is not None and d0 < 1.5 and d1 is not None and d1 > 8 and d2 is None
    ok &= dr
    print(f"[extrinsics] tag drift: still {d0:.2f} px, camera tilted 1 deg {d1:.0f} px, no tag -> None  {'OK' if dr else 'FAIL'}")
    pts = [triangulate(rec["A"], rec["B"], corners["A"][i], corners["B"][i])[0] for i in range(4)]
    truth = tag_object_points(S)
    cerr = max(np.linalg.norm(pts[i] - truth[i]) for i in range(4))
    eerr = max(abs(np.linalg.norm(pts[i] - pts[(i + 1) % 4]) - S) for i in range(4))
    zerr = max(abs(p[2]) for p in pts)
    tri = cerr < 0.01 and eerr < 0.01 and zerr < 0.01
    ok &= tri
    print(f"[extrinsics] tag corners via recovered cams: corner err {cerr * 100:.2f} cm, edge err {eerr * 100:.2f} cm, "
          f"|z| {zerr * 100:.2f} cm  {'OK' if tri else 'FAIL'}")
    return bool(ok)


def render_mat(cam, S, layout, marker_px=200, quiet=60):
    """All mat tags on the floor as seen by cam (distortion-free cam only). layout entries may carry twist and size."""
    img = np.full(cam.size[::-1], 255, np.uint8)
    q, m = quiet, quiet + marker_px
    src = np.float32([[q, q], [m, q], [m, m], [q, m]])
    for tag_id in layout:
        marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(TAG_DICT), tag_id, marker_px)
        canvas = np.full((marker_px + 2 * quiet, marker_px + 2 * quiet), 255, np.uint8)
        canvas[quiet:quiet + marker_px, quiet:quiet + marker_px] = marker
        dst = np.float32([cam.project(p) for p in floor.object_points(S, layout, [tag_id])])
        H = cv2.getPerspectiveTransform(src, dst)
        img = np.minimum(img, cv2.warpPerspective(canvas, H, cam.size, borderValue=255))
    return img


def ang_err(R_true, R):
    return math.degrees(math.acos(np.clip((np.trace(R_true.T @ R) - 1) / 2, -1, 1)))


def test_floor_mat():
    """(2b) floor mat: 4 tags pin the pitch that one tag leaves loose; drift; auto_extrinsics on a fake stream."""
    S, layout = 0.16, floor.mat_layout(1.0)
    # low table-top camera like tonight: 0.86 m up, mat ~1.5 m away
    cam = synth_cam("A", (-0.4, -1.5, 0.86), (0.5, 0.5, 0.0), f=1088.0, dist=(0, 0, 0, 0, 0))
    img = render_mat(cam, S, layout)
    det = make_tag_detector()
    ids, corners = floor.detect(det, img, layout)
    ok = sorted(ids) == [0, 1, 2, 3]
    print(f"[mat] tags detected in the synthetic view: {ids}  {'OK' if ok else 'FAIL'}")
    rvec, tvec, err = floor.solve_pose(cam, ids, corners, S, layout)
    R, _ = cv2.Rodrigues(rvec); pos = (-R.T @ tvec).ravel()
    dpos, dang = np.linalg.norm(pos - cam.position()), ang_err(cam.R, R)
    good = dpos < 0.02 and dang < 0.3 and err < 1.0
    ok &= good
    print(f"[mat] 4-tag pose: pos err {dpos * 100:.1f} cm, rot err {dang:.2f} deg, reproj {err:.2f} px  {'OK' if good else 'FAIL'}")
    # corner noise: what a 90 px tag really gives. Same noise, one tag vs four.
    rng = np.random.default_rng(0)
    e1, e4 = [], []
    for _ in range(30):
        noisy = corners + rng.normal(0, 0.4, corners.shape).astype(np.float32)
        r1, _, _ = floor.solve_pose(cam, [0], noisy[:1], S, layout)
        r4, _, _ = floor.solve_pose(cam, ids, noisy, S, layout)
        e1.append(ang_err(cam.R, cv2.Rodrigues(r1)[0])); e4.append(ang_err(cam.R, cv2.Rodrigues(r4)[0]))
    m1, m4 = float(np.median(e1)), float(np.median(e4))
    good = m4 < 0.3 and m4 < m1 / 3
    ok &= good
    print(f"[mat] 0.4 px corner noise: rotation error one tag {m1:.2f} deg, four tags {m4:.2f} deg  {'OK' if good else 'FAIL'}")
    # floor-plane range at 1 m beyond the mat origin, from a feet pixel: the number the tape test measures
    R1, _ = cv2.Rodrigues(floor.solve_pose(cam, [0], (corners + rng.normal(0, 0.4, corners.shape).astype(np.float32))[:1], S, layout)[0])
    print(f"[mat] (info) single-tag pitch error this draw: {ang_err(cam.R, R1):.2f} deg")
    # drift: 0 when still, big after a 1 deg tilt, None with no mat
    d0 = floor.drift_px(cam, img, det, S, layout)
    moved = synth_cam("A", (-0.4, -1.5, 0.86), (0.5, 0.5 + 0.06, 0.0), f=1088.0, dist=(0, 0, 0, 0, 0))
    d1 = floor.drift_px(cam, render_mat(moved, S, layout), det, S, layout)
    d2 = floor.drift_px(cam, np.full(cam.size[::-1], 255, np.uint8), det, S, layout)
    good = d0 is not None and d0 < 1.0 and d1 is not None and d1 > 8 and d2 is None
    ok &= good
    print(f"[mat] drift: still {d0:.2f} px, aim moved ~2 deg {d1:.0f} px, no mat -> {d2}  {'OK' if good else 'FAIL'}")
    # auto_extrinsics on a fake stream -> file with n_tags = 4, pose within 2 cm
    clock = [0.0]

    class FakeStream:
        """A new frame every call (33 ms apart on the fake clock), like a 30 fps camera."""
        def latest(self):
            clock[0] += 0.033
            return img, int(clock[0] * 1000)
    with tempfile.TemporaryDirectory() as d:
        cam0 = Camera("A", cam.K, cam.dist, cam.size)
        cam2, info = floor.auto_extrinsics(cam0, FakeStream(), S, layout, seconds=1.0, calib_dir=d,
                                           sleep=lambda s: clock.__setitem__(0, clock[0] + max(s, 0.03)), clock=lambda: clock[0])
        saved = np.load(os.path.join(d, "A_extrinsics.npz")) if cam2 is not None else {}
        good = (cam2 is not None and np.linalg.norm(cam2.position() - cam.position()) < 0.02 and int(saved["n_tags"]) == 4
                and abs(float(saved["tag_size"]) - S) < 1e-6 and info["n_samples"] >= 8)
        ok &= good
        print(f"[mat] auto_extrinsics: {info}  {'OK' if good else 'FAIL'}")
        # too few frames -> refused with a reason
        clock[0] = 0.0
        none, info = floor.auto_extrinsics(cam0, FakeStream(), S, layout, seconds=0.1, calib_dir=d,
                                           sleep=lambda s: clock.__setitem__(0, clock[0] + 0.05), clock=lambda: clock[0])
        good = none is None and "need" in info["why"]
        ok &= good
        print(f"[mat] auto_extrinsics with 2 frames -> refused: {info['why']!r}  {'OK' if good else 'FAIL'}")
    # survey: pages taped by eye (twisted, uneven, one printed smaller) -> recovered from a single view
    truth = {0: (0.0, 0.0, 0.0, S), 1: (0.43, 0.02, math.radians(4.0), S), 2: (0.40, 0.55, math.radians(-3.0), 0.15),
             3: (-0.03, 0.50, math.radians(6.0), S)}
    img2 = render_mat(cam, S, truth)
    ids, corners = floor.detect(det, img2, truth)
    lay, rvec, tvec, err = floor.survey(cam, ids, corners, S, ref_id=0)
    R, _ = cv2.Rodrigues(rvec); pos = (-R.T @ tvec).ravel()
    dxy = max(math.hypot(lay[i][0] - truth[i][0], lay[i][1] - truth[i][1]) for i in (1, 2, 3))
    dyaw = max(abs(math.degrees(lay[i][2] - truth[i][2])) for i in (1, 2, 3))
    dsize = max(abs(lay[i][3] - truth[i][3]) for i in (1, 2, 3))
    good = (sorted(ids) == [0, 1, 2, 3] and dxy < 0.01 and dyaw < 0.5 and dsize < 0.004 and err < 1.0
            and np.linalg.norm(pos - cam.position()) < 0.02 and ang_err(cam.R, R) < 0.3)
    ok &= good
    print(f"[mat] survey of an eyeballed mat: page position err {dxy * 100:.1f} cm, twist err {dyaw:.2f} deg, size err {dsize * 1000:.1f} mm, "
          f"residual {err:.2f} px, camera pos err {np.linalg.norm(pos - cam.position()) * 100:.1f} cm rot {ang_err(cam.R, R):.2f} deg  {'OK' if good else 'FAIL'}")
    # the nominal square layout on that eyeballed mat would be a bad fit; the surveyed one is a good one, from another camera too
    other = synth_cam("B", (1.2, -1.2, 1.3), (0.2, 0.3, 0.0), f=1000.0, dist=(0, 0, 0, 0, 0))
    ids_b, corners_b = floor.detect(det, render_mat(other, S, truth), truth)
    _, _, e_nom = floor.solve_pose(other, ids_b, corners_b, S, floor.mat_layout((0.43, 0.5)))
    rv_b, tv_b, e_sur = floor.solve_pose(other, ids_b, corners_b, S, lay)
    Rb, _ = cv2.Rodrigues(rv_b)
    good = e_sur < 1.0 and e_nom > 3 * e_sur and ang_err(other.R, Rb) < 0.3
    ok &= good
    print(f"[mat] second camera on the surveyed layout: residual {e_sur:.2f} px (nominal square layout {e_nom:.2f} px), rot err {ang_err(other.R, Rb):.2f} deg  {'OK' if good else 'FAIL'}")
    # focal length from the mat alone: pages of one print must come out the same size
    ids_u, corners_u = floor.detect(det, img, layout)            # the uniform mat (every page the same print)
    f_est, spread, res = floor.fit_f(Camera("A", [[800.0, 0, 640], [0, 800.0, 360], [0, 0, 1]], np.zeros(5), cam.size), ids_u, corners_u, S)
    good = abs(f_est - 1088.0) / 1088.0 < 0.03 and spread < 0.01
    ok &= good
    print(f"[mat] fit_f from a wrong 800 px start: f = {f_est:.0f} px (truth 1088), page-size spread {spread * 100:.2f} %  {'OK' if good else 'FAIL'}")
    # drift guard: fixes are published only while the saved pose still matches the mat
    g = floor.DriftGuard(8.0, bad_checks=2)
    seq = [g.update(v) for v in (1.0, None, 12.0, None, 12.0, 0.5, 20.0)]
    good = seq == ["ok", "ok", "bad", "bad", "resolve", "ok", "bad"] and g.bad
    g.reset(); good &= not g.bad
    ok &= good
    print(f"[mat] DriftGuard verdicts {seq}  {'OK' if good else 'FAIL'}")
    # zero-config camera: no intrinsics file -> nominal from the stream size, then the pose from the mat
    with tempfile.TemporaryDirectory() as d:
        floor.save_layout(d, {i: (x, y, 0.0, S) for i, (x, y) in layout.items()}, S)
        clock[0] = 0.0
        logs = []
        cam3 = floor.load_camera("Z", d, FakeStream(), True, S, floor.get_layout(None, d), log=logs.append)
        # (the synthetic cam is f = 0.85 W exactly, so the nominal pinhole IS the truth here)
        good = (os.path.exists(os.path.join(d, "Z_intrinsics.npz")) and os.path.exists(os.path.join(d, "Z_extrinsics.npz"))
                and np.linalg.norm(cam3.position() - cam.position()) < 0.02 and any("nominal" in l for l in logs))
        ok &= good
        print(f"[mat] load_camera with no intrinsics: nominal written, pose err {np.linalg.norm(cam3.position() - cam.position()) * 100:.1f} cm  {'OK' if good else 'FAIL'}")
    return bool(ok)


# ---------- (3) ----------
class FakePerson:
    """detect(frame) -> [] or one person at self.world (set per test case), keyed by camera."""
    def __init__(self, cam, pid=7):
        self.cam, self.pid, self.world, self.calls = cam, pid, None, 0

    def detect(self, frame):
        self.calls += 1
        if self.world is None:
            return []
        uv = observe(self.cam, self.world)
        return [{"id": self.pid, "box": (uv[0] - 50, uv[1] - 100, uv[0] + 50, uv[1] + 200), "conf": 0.9,
                 "pt": (float(uv[0]), float(uv[1])), "area": 30000.0}]


class FakeBalloon:
    """One shared object; which camera is looking is identified by the frame object passed in."""
    def __init__(self, cams, frames_of):
        self.cams, self.frames_of, self.world, self.calls = cams, frames_of, None, 0

    def detect(self, frame):
        self.calls += 1
        if self.world is None:
            return None
        n = self.frames_of(frame)
        uv = observe(self.cams[n], self.world)
        return {"box": (uv[0] - 30, uv[1] - 30, uv[0] + 30, uv[1] + 30), "conf": 0.8, "pt": (float(uv[0]), float(uv[1]))}


def test_localize_step():
    cams = rig()
    owner = {}

    def new_frame(n):
        f = np.zeros((8, 8, 3), np.uint8); owner[id(f)] = n; return f

    persons = {"A": FakePerson(cams["A"]), "B": FakePerson(cams["B"])}
    balloon = FakeBalloon(cams, lambda f: owner[id(f)])
    prev_t = {"A": None, "B": None}
    P, B = (0.3, 0.2, 1.3), (0.0, 0.5, 1.7)
    results = []

    def check(name, cond, extra=""):
        results.append(cond); print(f"[step] {name}{'  ' + extra if extra else ''}  {'OK' if cond else 'FAIL'}")

    def calls():
        return (persons["A"].calls, persons["B"].calls, balloon.calls)

    def run(frames, now):
        return step(cams, persons, balloon, frames, prev_t, now)

    # 1. fresh pair
    for d in persons.values(): d.world = P
    balloon.world = B
    fr = {"A": (new_frame("A"), 1000), "B": (new_frame("B"), 1000)}
    msg, dets = run(fr, 1010)
    good = (msg is not None and {"t", "balloon", "person", "person_id", "src"} <= set(msg)
            and msg["src"] == "vision" and msg["t"] == 1000 and msg["lost"] is None and msg["person_id"] == 7
            and np.linalg.norm(np.float64(msg["person"]) - P) < 0.01
            and np.linalg.norm(np.float64(msg["balloon"]) - B) < 0.01 and dets is not None)
    try:
        json.dumps(msg)
    except TypeError:
        good = False
    check("fresh pair -> full PROTOCOL s4 message", good, f"{msg}")

    # 2. same frames again
    c0 = calls(); msg, _ = run(fr, 1020)
    check("same frames -> None, detectors not re-run", msg is None and calls() == c0)

    # 3. only A new
    fr["A"] = (new_frame("A"), 1033)
    msg, _ = run(fr, 1040)
    check("only A new -> None", msg is None and calls() == c0)

    # 4. both new
    fr["B"] = (new_frame("B"), 1035)
    msg, _ = run(fr, 1040)
    check("both new -> message, one call per detector-camera", msg is not None and msg["skew_ms"] == 2
          and calls() == (c0[0] + 1, c0[1] + 1, c0[2] + 2))

    # 5. skew
    fr = {"A": (new_frame("A"), 2000), "B": (new_frame("B"), 1800)}
    msg, _ = run(fr, 2010)
    check("skew 200 ms -> nulls, lost=skew", msg is not None and msg["lost"] == "skew" and msg["person"] is None
          and msg["balloon"] is None and msg["person_id"] == -1)

    # 6. stale
    c0 = calls()
    fr = {"A": (new_frame("A"), 3000), "B": (new_frame("B"), 3000)}
    msg, _ = run(fr, 3600)
    check("age 600 ms -> nulls, lost=stale, detectors not run", msg is not None and msg["lost"] == "stale"
          and msg["person"] is None and calls() == c0)

    # 7. person missing in B
    persons["B"].world = None
    fr = {"A": (new_frame("A"), 4000), "B": (new_frame("B"), 4000)}
    msg, _ = run(fr, 4010)
    check("person missing in B -> person None, id -1, balloon kept", msg["person"] is None and msg["person_id"] == -1
          and msg["balloon"] is not None)

    # 8a. height gate
    for d in persons.values(): d.world = (0.3, 0.2, 0.2)   # floor level: in both views, below PERSON_Z_M
    fr = {"A": (new_frame("A"), 5000), "B": (new_frame("B"), 5000)}
    msg, _ = run(fr, 5010)
    check("person at z=0.2 -> rejected by height gate", msg["person"] is None)
    # 8b. reproj gate
    persons["A"].world = P; persons["B"].world = (-0.8, -0.6, 0.6)
    fr = {"A": (new_frame("A"), 6000), "B": (new_frame("B"), 6000)}
    msg, _ = run(fr, 6010)
    check("different people in A and B -> rejected by reproj gate", msg["person"] is None)

    # 9. no balloon
    persons["B"].world = P; balloon.world = None
    fr = {"A": (new_frame("A"), 7000), "B": (new_frame("B"), 7000)}
    msg, _ = run(fr, 7010)
    check("no balloon -> balloon None, person present", msg["balloon"] is None and msg["person"] is not None)

    # 10. None frame
    msg, dets = run({"A": (None, None), "B": fr["B"]}, 7020)
    check("missing frame -> None", msg is None and dets is None)
    return all(results)


# ---------- (4) ----------
def test_mono():
    cam = synth_cam("A", (-1.0, -1.5, 1.4), (0.3, 0.3, 0.9), f=700)
    results = []

    def check(name, cond):
        results.append(bool(cond)); print(f"[mono] {name}  {'OK' if cond else 'FAIL'}")

    # geometry: ray/floor and vertical-line intersections are exact
    rng = np.random.default_rng(1)
    errs_xy, errs_z, errs_b = [], [], []
    for _ in range(100):
        feet = np.array([rng.uniform(-0.8, 1.2), rng.uniform(-0.8, 1.2), 0.0])
        h = rng.uniform(1.4, 2.0)
        uf, uh = observe(cam, feet), observe(cam, feet + [0, 0, h])
        if not (in_view(cam, uf) and in_view(cam, uh)):
            continue
        box = (uf[0] - 40, uh[1], uf[0] + 40, uf[1])   # YOLO-like box: head top .. feet, centred on the feet
        X, why = mono.person_from_box(cam, box)
        if X is None:
            errs_xy.append(9); continue
        errs_xy.append(np.linalg.norm(X[:2] - feet[:2])); errs_z.append(abs(X[2] - mono.CHEST_FRAC * h))
        # balloon: sphere of 0.4 m at a random spot; apparent size from pinhole depth
        B = np.array([rng.uniform(-0.5, 1.0), rng.uniform(-0.5, 1.0), rng.uniform(1.0, 2.2)])
        ub = observe(cam, B)
        depth = float((cam.R @ B + cam.t.ravel())[2])
        px = cam.K[0, 0] * 0.4 / depth
        Xb, _ = mono.balloon_from_box(cam, (ub[0] - px / 2, ub[1] - px / 2, ub[0] + px / 2, ub[1] + px / 2), 0.4)
        errs_b.append(np.linalg.norm(Xb - B))
    check(f"person feet XY err max {max(errs_xy) * 1000:.2f} mm ({len(errs_z)} pts)", len(errs_z) > 40 and max(errs_xy) < 0.002)
    check(f"person chest z err max {max(errs_z) * 1000:.1f} mm (box top is not exactly above the feet)", max(errs_z) < 0.03)
    check(f"balloon from size err max {max(errs_b) * 1000:.1f} mm", max(errs_b) < 0.005)
    # 2 px of noise on the feet at ~2.5 m range -> a few cm, not metres
    feet = np.array([0.4, 0.2, 0.0]); uf, uh = observe(cam, feet), observe(cam, feet + [0, 0, 1.7])
    X, _ = mono.person_from_box(cam, (uf[0] - 40, uh[1] + 2, uf[0] + 40, uf[1] - 2))
    check(f"2 px noise -> {np.linalg.norm(X[:2] - feet[:2]) * 100:.1f} cm XY error", np.linalg.norm(X[:2] - feet[:2]) < 0.10)
    # gates
    X, why = mono.person_from_box(cam, (600, 100, 700, cam.size[1] - 1))
    check("box touching the bottom edge -> rejected (feet not visible)", X is None and "cut" in why)
    feet = np.array([0.4, 0.2, 0.0]); uf = observe(cam, feet)
    X, why = mono.person_from_box(cam, (uf[0] - 40, 0, uf[0] + 40, uf[1]))
    check("head above the top edge -> feet XY kept, chest z assumed", X is not None and np.linalg.norm(X[:2] - feet[:2]) < 0.002 and X[2] == mono.CHEST_Z_M)
    X, why = mono.person_from_box(cam, (600, 300, 700, 305))
    check("tiny box (implausible height) -> rejected", X is None)
    X, why = mono.person_from_box(cam, (600, 0, 700, 5))   # ray through the top of the image never hits the floor
    check("ray above the horizon -> rejected", X is None)
    X, why = mono.balloon_from_box(cam, (600, 300, 606, 306), 0.4)
    check("balloon box below --min-balloon-px -> rejected", X is None)

    # step()
    class FakePerson:
        def __init__(self, out): self.out, self.calls = out, 0
        def detect(self, frame): self.calls += 1; return self.out

    class FakeBalloon(FakePerson):
        pass
    feet = np.array([0.4, 0.2, 0.0]); uf, uh = observe(cam, feet), observe(cam, feet + [0, 0, 1.7])
    pbox = (uf[0] - 40, uh[1], uf[0] + 40, uf[1])
    B = np.array([0.6, 0.4, 1.7]); ub = observe(cam, B); px = cam.K[0, 0] * 0.4 / float((cam.R @ B + cam.t.ravel())[2])
    bbox = (ub[0] - px / 2, ub[1] - px / 2, ub[0] + px / 2, ub[1] + px / 2)
    pd = FakePerson([{"id": 7, "box": pbox, "pt": (0, 0)}]); bd = FakeBalloon({"box": bbox, "pt": ub})
    prev = {"t": None}; img = np.zeros((720, 1280, 3), np.uint8)
    msg, dets = mono.step(cam, pd, bd, (img, 1000), prev, 1010, balloon_diam=0.4)
    ok = (msg is not None and msg["src"] == "vision" and msg["t"] == 1000 and msg["person_id"] == 7 and msg["lost"] is None
          and np.linalg.norm(np.array(msg["person"]) - [0.4, 0.2, mono.CHEST_FRAC * 1.7]) < 0.01
          and np.linalg.norm(np.array(msg["balloon"]) - B) < 0.01 and json.dumps(msg))
    check("fresh frame -> full message, person + balloon within 1 cm", ok)
    msg, dets = mono.step(cam, pd, bd, (img, 1000), prev, 1020, balloon_diam=0.4)
    check("same frame again -> None, detectors not re-run", msg is None and pd.calls == 1 and bd.calls == 1)
    msg, dets = mono.step(cam, pd, bd, (img, 1000), prev, 1700, balloon_diam=0.4)
    check("stale frame -> nulls with lost='stale', no detection", msg["lost"] == "stale" and msg["person"] is None and pd.calls == 1)
    pd2 = FakePerson([]); msg, dets = mono.step(cam, pd2, bd, (img, 1100), prev, 1110, balloon_diam=0.4)
    check("no person -> person None, person_id -1, balloon kept", msg["person"] is None and msg["person_id"] == -1 and msg["balloon"] is not None)
    msg, dets = mono.step(cam, pd, bd, (None, None), prev, 1200)
    check("missing frame -> None", msg is None and dets is None)
    return all(results)


# ---------- (5) ----------
def test_color_balloon():
    results = []

    def check(name, cond):
        results.append(bool(cond)); print(f"[balloon] {name}  {'OK' if cond else 'FAIL'}")
    rng = np.random.default_rng(3)
    img = np.full((720, 1280, 3), (110, 120, 125), np.uint8)                      # grey-beige floor/wall
    img += rng.integers(0, 12, img.shape, dtype=np.uint8)                        # sensor noise
    det = ColorBalloonDetector("white")
    check("empty room -> None", det.detect(img) is None)
    cv2.rectangle(img, (600, 500), (820, 650), (245, 245, 245), -1)              # sheet of paper (elongated)
    cv2.line(img, (100, 100), (1100, 120), (250, 250, 250), 3)                   # white cable
    cv2.rectangle(img, (0, 0), (1279, 40), (255, 255, 255), -1)                  # bright wall strip touching the border
    check("paper + cable + wall strip -> None", det.detect(img) is None)
    cv2.circle(img, (900, 250), 60, (250, 248, 246), -1)                        # the balloon
    d = det.detect(img)
    check("white disc -> found, centre within 2 px, diameter within 4 px",
          d is not None and abs(d["pt"][0] - 900) < 2 and abs(d["pt"][1] - 250) < 2 and abs(d["diam_px"] - 120) < 4)
    cv2.circle(img, (300, 400), 25, (250, 248, 246), -1)                        # smaller white blob (a cup)
    d = det.detect(img)
    check("two discs -> the larger one", d is not None and abs(d["pt"][0] - 900) < 2)
    red = ColorBalloonDetector("red")
    check("red detector ignores the white disc", red.detect(img) is None)
    cv2.circle(img, (500, 300), 50, (30, 30, 220), -1)
    d = red.detect(img)
    check("red disc -> found by the red detector", d is not None and abs(d["pt"][0] - 500) < 2)
    return all(results)


# ---------- (6) ----------
def test_preflight():
    results = []

    def check(name, cond, extra=""):
        results.append(bool(cond)); print(f"[preflight] {name}{'  ' + extra if extra else ''}  {'OK' if cond else 'FAIL'}")

    def statuses(checks, key):
        return [s for s, n, d in checks if key in n]

    S = 0.15
    posA, posB = (0.5, -0.9, 1.4), (-0.6, -0.8, 1.5)
    with tempfile.TemporaryDirectory() as d:
        A = synth_cam("A", posA, (0, 0, 0)); B = synth_cam("B", posB, (0, 0, 0))
        save_intrinsics(d, "A", A.K, A.dist, A.size, 0.4); save_extrinsics(d, "A", A.R, A.t, tag_size=S, rms=0.6)
        save_intrinsics(d, "B", B.K, B.dist, B.size, 0.4); save_extrinsics(d, "B", B.R, B.t, tag_size=S, rms=0.6)
        c = preflight.offline_checks(d, ("A", "B"), S, targets_dir=os.path.join(d, "none"))
        check("real intrinsics + sane extrinsics + matching tag size -> no FAIL, intrinsics/extrinsics/pair OK",
              "FAIL" not in [s for s, _, _ in c] and statuses(c, "intrinsics A") == ["OK"] and "OK" in statuses(c, "extrinsics A")
              and statuses(c, "camera pair") == ["OK"] and statuses(c, "printed target") == ["WARN"])
        c1 = preflight.offline_checks(d, ("A",), S, targets_dir=os.path.join(d, "none"))
        check("single camera (mono): offline checks run, no pair row, no FAIL",
              statuses(c1, "camera pair") == [] and "FAIL" not in [s for s, _, _ in c1] and statuses(c1, "intrinsics A") == ["OK"])
        save_intrinsics(d, "A", A.K, np.zeros(5), A.size, -1.0)
        check("nominal intrinsics (rms -1, no distortion) -> WARN", statuses(preflight.check_intrinsics("A", d), "intrinsics A") == ["WARN"])
        save_intrinsics(d, "A", A.K, A.dist, A.size, 1.5)
        check("rms 1.5 px -> FAIL", statuses(preflight.check_intrinsics("A", d), "intrinsics A") == ["FAIL"])
        save_intrinsics(d, "A", A.K, A.dist, A.size, 0.4)
        save_extrinsics(d, "A", A.R, A.t, tag_size=0.18, rms=0.6)
        check("extrinsics made for 180 mm, config 150 mm -> FAIL (scale error)", "FAIL" in statuses(preflight.check_extrinsics("A", d, S), "tag size"))
        save_extrinsics(d, "A", A.R, A.t)
        check("extrinsics without a recorded tag size -> WARN", statuses(preflight.check_extrinsics("A", d, S), "tag size") == ["WARN"])
        save_extrinsics(d, "A", A.R, A.t, tag_size=S)
        check("missing extrinsics file -> FAIL", statuses(preflight.check_extrinsics("C", d, S), "extrinsics C") == ["FAIL"])
        low = synth_cam("A", (0.5, -0.9, 0.2), (0, 0, 0)); save_extrinsics(d, "A", low.R, low.t, tag_size=S)
        check("camera at z=0.2 m -> FAIL", "FAIL" in statuses(preflight.check_extrinsics("A", d, S), "extrinsics A"))
        flat = synth_cam("A", posA, (0, 0, 1.4)); save_extrinsics(d, "A", flat.R, flat.t, tag_size=S)
        cf = preflight.check_extrinsics("A", d, S)
        check("camera looking horizontally -> aim WARN and tag-outside-image FAIL", statuses(cf, "aim") == ["WARN"] and "FAIL" in statuses(cf, "tag"))
        save_extrinsics(d, "A", A.R, A.t, tag_size=S)
        save_extrinsics(d, "B", A.R, A.t, tag_size=S)
        check("B copied from A -> pair FAIL", statuses(preflight.check_pair(d), "camera pair") == ["FAIL"])
        near = synth_cam("B", (0.45, -0.9, 1.4), (0, 0, 0)); save_extrinsics(d, "B", near.R, near.t, tag_size=S)
        check("baseline 5 cm -> pair FAIL", statuses(preflight.check_pair(d), "camera pair") == ["FAIL"])
        save_extrinsics(d, "B", B.R, B.t, tag_size=S)
        tdir = pathlib.Path(d) / "targets"; tdir.mkdir()
        (tdir / "apriltag36h11_id0_180mm_A4.png").write_bytes(b"")
        check("printed target 180 mm vs config 150 -> WARN", statuses(preflight.check_targets(str(tdir), S), "printed target") == ["WARN"])
        (tdir / "apriltag36h11_id0_150mm_A4.png").write_bytes(b"")
        check("a 150 mm target file -> OK (and the stale 180 still WARNs)", sorted(statuses(preflight.check_targets(str(tdir), S), "printed target")) == ["OK", "WARN"])
        check("venue check on the committed file -> no FAIL", "FAIL" not in statuses(preflight.check_venue(), "venue"))
        check("worst(): FAIL -> 1, WARN only -> 0", preflight.worst([("FAIL", "x", "")]) == 1 and preflight.worst([("WARN", "x", ""), ("OK", "y", "")]) == 0)

        # live checks on fake streams: distortion-free cams so render_tag is exact
        cams = {n: synth_cam(n, p, (0, 0, 0), dist=(0, 0, 0, 0, 0)) for n, p in (("A", posA), ("B", posB))}
        imgs = {n: render_tag(cams[n], S)[0] for n in cams}
        clock = [100.0]

        class FakeStream:
            def __init__(self, n, lag_ms=0.0): self.n, self.lag = n, lag_ms
            def latest(self): return imgs[self.n], int(clock[0] * 1000 - self.lag)
        fake_sleep = lambda s: clock.__setitem__(0, clock[0] + s)
        live = preflight.live_checks({"A": FakeStream("A"), "B": FakeStream("B")}, cams, seconds=2.0, tag_size=S, sleep=fake_sleep, clock=lambda: clock[0])
        check("fresh fake streams: fps OK, tag seen OK, drift OK, skew OK",
              statuses(live, "camera A fps") == ["OK"] and statuses(live, "camera A tag") == ["OK"] and statuses(live, "camera B drift") == ["OK"]
              and statuses(live, "A/B skew") == ["OK"], "; ".join(f"{s} {n}" for s, n, _ in live if "balloon" not in n))
        live = preflight.live_checks({"A": FakeStream("A"), "B": FakeStream("B", lag_ms=200)}, cams, seconds=1.0, tag_size=S, sleep=fake_sleep, clock=lambda: clock[0])
        check("one stream 200 ms behind -> skew FAIL", statuses(live, "A/B skew") == ["FAIL"])
        moved = synth_cam("A", (0.5, -0.9, 1.4), (0.05, 0, 0), dist=(0, 0, 0, 0, 0))     # ~2 deg aim change since extrinsics
        live = preflight.live_checks({"A": FakeStream("A")}, {"A": moved}, seconds=1.0, tag_size=S, sleep=fake_sleep, clock=lambda: clock[0])
        check("camera moved since extrinsics -> drift FAIL", statuses(live, "camera A drift") == ["FAIL"])
    return all(results)


if __name__ == "__main__":
    r = [test_triangulate(), test_extrinsics_roundtrip(), test_floor_mat(), test_localize_step(), test_mono(), test_color_balloon(), test_preflight()]
    print("PASS" if all(r) else "FAIL")
    sys.exit(0 if all(r) else 1)
