"""Single-camera fallback: ONE calibrated camera -> world positions on udp 5007 (same PROTOCOL s4 message as
localize.py, so follow_me does not care which one is running).

  python -m laptop.vision.mono                      # camera A from laptop/config.py
  python -m laptop.vision.mono --a 1 --show --device mps
  python -m laptop.vision.mono --a 1 --auto-calib   # solve the camera pose from the floor mat at start (and again if it moves)
Needs calib/A_intrinsics.npz, and calib/A_extrinsics.npz unless --auto-calib finds the mat. 'q' in the window or Ctrl+C quits.

No triangulation, so each object needs one extra assumption:
  person  : the bottom-centre of the YOLO box is the feet, on the floor (z = 0). Ray/floor intersection gives
            X, Y. Chest z comes from the box top (ray through the top intersected with the vertical line through
            the feet, times 0.65). A box cut by the bottom of the image means the feet are NOT visible: rejected.
            A box cut by the top (head out of frame) keeps X, Y and assumes chest z = CHEST_Z_M.
  camera  : aim it so the floor 1 m in front is at the bottom of the frame and a standing person 2 m away fits.
  balloon : apparent diameter -> depth (pinhole: depth = f * D / px) along the ray through the box centre.
            D = --balloon-diam (m). Boxes smaller than --min-balloon-px are rejected (depth too noisy).
Accuracy ~5-10 % of range. Good enough for a follow-me demo; use localize.py once a second camera exists.

The per-tick logic lives in step() (pure, no I/O) so tools/vision_test.py can drive it with fake detectors.
"""
import argparse, time
import cv2, numpy as np
from .. import config
from ..control.protocol import STATE_PORT, UdpJson, now_ms
from . import floor
from .calib_io import Camera, make_tag_detector
from .localize import PERSON_Z_M, STALE_MS, draw, nulls
from .streams import Stream

BALLOON_DIAM_M = 2 * config.R_BALLOON   # envelope diameter, m (config.R_BALLOON is the radius; measure after inflating)
CHEST_FRAC = 0.65        # chest height as a fraction of the person's height
CHEST_Z_M = 1.2          # assumed chest height when the head is out of frame (X, Y still come from the feet)


def ray(cam, uv):
    """World-frame origin (camera centre) and UNIT direction of the back-projected ray through pixel uv."""
    n = cv2.undistortPoints(np.float32([[uv]]), cam.K, cam.dist).ravel()   # normalised camera coords
    d = cam.R.T @ np.array([n[0], n[1], 1.0])
    return cam.position(), d / np.linalg.norm(d)


def hit_plane(cam, uv, z=0.0):
    """World point where the ray through uv meets the horizontal plane at height z, or None (ray never gets there)."""
    o, d = ray(cam, uv)
    if abs(d[2]) < 1e-9:
        return None
    s = (z - o[2]) / d[2]
    return None if s <= 0 else o + s * d


def height_above(cam, uv, xy):
    """z of the point on the ray through uv that is directly above/below world (x, y): the ray is intersected
    with the vertical plane through (x, y) facing the camera. None if the ray runs away from it."""
    o, d = ray(cam, uv)
    n = np.array([d[0], d[1], 0.0])
    if np.linalg.norm(n) < 1e-9:
        return None
    n /= np.linalg.norm(n)
    s = float(n @ (np.array([xy[0], xy[1], 0.0]) - np.array([o[0], o[1], 0.0]))) / float(n @ d)
    return None if s <= 0 else float(o[2] + s * d[2])


def point_from_size(cam, uv, px, diam):
    """World point of a sphere of diameter diam (m) seen at pixel uv with apparent diameter px."""
    depth = 0.5 * (cam.K[0, 0] + cam.K[1, 1]) * diam / px          # along the optical axis
    n = cv2.undistortPoints(np.float32([[uv]]), cam.K, cam.dist).ravel()
    return cam.position() + cam.R.T @ (depth * np.array([n[0], n[1], 1.0]))


def person_from_box(cam, box, person_z=PERSON_Z_M, edge_px=4, chest_z=CHEST_Z_M):
    """(X, Y, z_chest) from a person box, or (None, reason). Feet = bottom-centre of the box on the floor.
    Head above the top edge of the image: X, Y still come from the feet, z is assumed (chest_z)."""
    x1, y1, x2, y2 = box
    if y2 >= cam.size[1] - edge_px:
        return None, "feet cut off"
    feet = hit_plane(cam, ((x1 + x2) / 2, y2), 0.0)
    if feet is None:
        return None, "feet ray misses floor"
    if y1 <= edge_px:
        return np.array([feet[0], feet[1], chest_z]), f"ok, head out of frame, z assumed {chest_z}"
    top = height_above(cam, ((x1 + x2) / 2, y1), feet[:2])
    if top is None:
        return None, "bad head ray"
    h = top - feet[2]
    z = CHEST_FRAC * h
    if not person_z[0] <= z <= person_z[1]:
        return None, f"height {h:.2f} m implausible"
    return np.array([feet[0], feet[1], z]), f"ok h={h:.2f} m"


def balloon_from_box(cam, box, diam, min_px=12):
    x1, y1, x2, y2 = box
    px = max(x2 - x1, y2 - y1)
    if px < min_px:
        return None, f"box {px:.0f} px too small"
    X = point_from_size(cam, ((x1 + x2) / 2, (y1 + y2) / 2), px, diam)
    if X[2] < 0:
        return None, f"z={X[2]:.2f} below floor"
    return X, f"ok {px:.0f} px"


def step(cam, person_det, balloon_det, frame, prev_t, now, stale_ms=STALE_MS, person_z=PERSON_Z_M,
         balloon_diam=BALLOON_DIAM_M, min_balloon_px=12):
    """One localisation tick. Pure: no sockets, no sleeping.
    frame  (img, t_ms) as returned by Stream.latest();  prev_t {"t": last processed t}  MUTATED in place.
    Returns (msg, dets): msg None -> nothing new; msg PROTOCOL s4 dict (+ 'lost') -> send it.
    dets = {"persons", "balloon", "why"} or None when the detectors did not run."""
    img, t = frame
    if img is None or t is None:
        return None, None
    if now - t > stale_ms:
        return nulls(now, "stale"), None
    if t == prev_t["t"]:
        return None, None
    prev_t["t"] = t

    persons, balloon = person_det.detect(img), balloon_det.detect(img)
    person3 = balloon3 = None
    why = {"person": "none seen", "balloon": "none seen"}
    if persons:
        X, why["person"] = person_from_box(cam, persons[0]["box"], person_z)
        if X is not None:
            person3 = [round(float(v), 3) for v in X]
    if balloon:
        X, why["balloon"] = balloon_from_box(cam, balloon["box"], balloon_diam, min_balloon_px)
        if X is not None:
            balloon3 = [round(float(v), 3) for v in X]
    msg = {"t": int(t), "balloon": balloon3, "person": person3,
           "person_id": int(persons[0]["id"]) if person3 is not None else -1,
           "src": "vision", "skew_ms": 0, "lost": None}
    return msg, {"persons": persons, "balloon": balloon, "why": why}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", default=config.SOURCES["A"], help="camera source (see streams.py)")
    ap.add_argument("--name", default="A", help="calibration name (calib/<name>_*.npz)")
    ap.add_argument("--calib", default=config.CALIB_DIR)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--stale-ms", type=int, default=STALE_MS)
    ap.add_argument("--device", default=None, help="ultralytics device: 0, cpu, mps ... (default: auto)")
    ap.add_argument("--conf-person", type=float, default=0.4)
    ap.add_argument("--conf-balloon", type=float, default=0.3)
    ap.add_argument("--balloon-color", default=config.BALLOON_COLOR, help="colour-blob balloon detector; 'none' = YOLO only")
    ap.add_argument("--balloon-diam", type=float, default=BALLOON_DIAM_M, help="m; inflated balloon diameter")
    ap.add_argument("--tag-size", type=float, default=config.TAG_SIZE_M, help="m; for the tag-drift check (0 = off)")
    ap.add_argument("--drift-px", type=float, default=8.0)
    ap.add_argument("--rot", type=int, default=0, help="degrees clockwise to rotate the stream (phone in portrait: 90)")
    ap.add_argument("--auto-calib", action="store_true", help="pose from the floor mat at start; re-solve when the drift check trips 2 s running")
    ap.add_argument("--spacing", type=float, nargs="+", default=None, help="m between mat tag centres (one value or +X +Y); default: calib/mat.json from survey_mat.py, else config.MAT")
    args = ap.parse_args()
    from .detect import BalloonDetector, PersonTracker   # lazy: slow import, optional for --help

    layout = floor.get_layout(args.spacing, args.calib)
    print(f"[mat] layout from {floor.layout_source(args.spacing, args.calib)}")
    stream = Stream(args.a, args.name, rotate=args.rot).wait_first(timeout=20)
    cam = floor.load_camera(args.name, args.calib, stream, args.auto_calib, args.tag_size, layout)
    print(f"[cam {args.name}] at world {np.round(cam.position(), 2)} m  (single-camera mode)")
    person_det = PersonTracker(config.YOLO_PERSON, conf=args.conf_person, device=args.device)
    balloon_det = BalloonDetector(config.BALLOON_WEIGHTS, conf=args.conf_balloon, device=args.device,
                                  color=None if args.balloon_color in (None, "none") else args.balloon_color)
    print(f"[balloon] mode: {balloon_det.mode}, diameter {args.balloon_diam} m")
    out = UdpJson()
    n_frames, t_rate, prev_t = 0, time.monotonic(), {"t": None}
    why, msg = {"person": "-", "balloon": "-"}, nulls(now_ms(), None)
    tag_det, guard = make_tag_detector(), floor.DriftGuard(args.drift_px)
    try:
        while True:
            frame = stream.latest()
            if time.monotonic() - t_rate > 1:          # once a second: drift check (+ re-solve) and the status line
                if args.tag_size > 0 and frame[0] is not None:
                    verdict = guard.update(floor.drift_px(cam, frame[0], tag_det, args.tag_size, layout))
                    if verdict != "ok" and not args.auto_calib:
                        print(f"[mono] WARNING: floor mat is {guard.last:.0f} px from where extrinsics expect it: publishing lost. "
                              f"Camera (laptop lid?) or mat moved -> re-run tools/calib/extrinsics.py --name {args.name}, or use --auto-calib")
                    elif verdict == "resolve":
                        cam2, info = floor.auto_extrinsics(cam, stream, args.tag_size, layout, seconds=1.0, det=tag_det, calib_dir=args.calib)
                        if cam2 is not None:
                            cam = cam2; guard.reset()
                            print(f"[mono] camera moved ({guard.last:.0f} px): re-solved. " + floor.describe(cam, info))
                        else:
                            print(f"[mono] camera moved ({guard.last:.0f} px) but cannot re-solve: {info['why']}")
                print(f"[mono] {n_frames} Hz  cam {stream.fps:.0f} fps  lost={msg['lost']}  person={msg['person']} "
                      f"({why['person']})  balloon={msg['balloon']} ({why['balloon']})  "
                      f"tag drift px={guard.last and round(guard.last, 1)}")
                n_frames, t_rate = 0, time.monotonic()
            if guard.bad:                              # stale pose: say so instead of publishing wrong fixes
                msg = nulls(now_ms(), "drift"); out.send(msg, ("127.0.0.1", STATE_PORT))
                time.sleep(0.05); continue
            msg2, dets = step(cam, person_det, balloon_det, frame, prev_t, now_ms(), args.stale_ms,
                              balloon_diam=args.balloon_diam)
            if msg2 is None:
                time.sleep(0.005); continue
            msg = msg2
            out.send(msg, ("127.0.0.1", STATE_PORT))
            if msg["lost"]:
                time.sleep(0.05)
            else:
                n_frames += 1
            if dets: why = dets["why"]
            if args.show and dets:
                img = frame[0].copy()
                draw(img, dets["persons"], dets["balloon"], f"{args.name}  balloon={msg['balloon']}  person={msg['person']}")
                cv2.imshow(f"cam {args.name}", cv2.resize(img, None, fx=0.6, fy=0.6))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop(); cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
