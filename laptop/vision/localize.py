"""Vision main loop: two cameras -> YOLO person + balloon -> triangulate -> world state on 5007.

  python -m laptop.vision.localize                     # sources from laptop/config.py
  python -m laptop.vision.localize --a 0 --b 1 --show
  python -m laptop.vision.localize --auto-calib        # camera poses from the floor mat at start; re-solved if a camera moves
Needs calib/{A,B}_intrinsics.npz, and the extrinsics files unless --auto-calib finds the mat. 'q' in a window or Ctrl+C quits.

Several people in view (judges, teammates): each camera lists its persons largest first, and the pair that is the
SAME person is the one whose rays meet (smallest reprojection error) at a plausible chest height (PERSON_Z_M).
The chest point (35 % down the box) is not the same physical 3D point from two viewpoints, so expect a few px
of reprojection error; --max-reproj is loose for that reason. With two views the reprojection error only measures
distance to the epipolar line, so a mismatch lying near that line passes --max-reproj; the height band is the
second line of defence. The person reported is that matched pair (person_id from camera A's tracker).

The per-tick logic lives in step() (pure, no I/O) so tools/vision_test.py can drive it with fake detectors.
"""
import argparse, time
import cv2, numpy as np
from .. import config
from ..control.protocol import STATE_PORT, UdpJson, now_ms
from . import floor
from .calib_io import Camera, make_tag_detector
from .streams import Stream, label
from .triangulate import triangulate

STALE_MS = 500          # a frame older than this -> the camera is considered lost
PERSON_Z_M = (0.5, 2.5) # plausible chest height band, metres above the floor
MAX_PAIR = 4            # people per camera considered when matching A's persons to B's
CHEST_Z_M = 1.2         # reported chest height when it comes from the feet (z ~ 0) or the head (z - HEAD_TO_CHEST_M)
HEAD_TO_CHEST_M = 0.35
FEET_Z_M = (-0.35, 0.35)   # triangulated feet must be on the floor
HEAD_Z_M = (1.1, 2.3)      # triangulated head top must be at a plausible height


def person_point(a, b):
    """Which point of two person detections is the SAME physical point seen from both cameras, in preference order:
    feet (ground contact) when neither box is cut at the bottom, head top when neither is cut at the top, else the
    35 %-down chest point. -> (uv_a, uv_b, z band, chest z from the triangulated z). Detections without feet/head/cut
    keys (fakes, old detectors) use the chest point."""
    if "feet" in a and "feet" in b:
        if not a["cut"]["bottom"] and not b["cut"]["bottom"]:
            return a["feet"], b["feet"], FEET_Z_M, lambda z: CHEST_Z_M
        if not a["cut"]["top"] and not b["cut"]["top"]:
            return a["head"], b["head"], HEAD_Z_M, lambda z: z - HEAD_TO_CHEST_M
    return a["pt"], b["pt"], PERSON_Z_M, lambda z: z


def draw(frame, persons, balloon, text):
    for p in persons:                                   # everybody (the roster's label when there is one: mono.py)
        x1, y1, x2, y2 = map(int, p["box"])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, tuple(map(int, p["pt"])), 6, (0, 255, 0), -1)
        label(frame, p.get("pid") or f"person id={p['id']}", (x1, max(18, y1 - 6)), 0.6, (0, 255, 0))
    if balloon:
        x1, y1, x2, y2 = map(int, balloon["box"])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 128, 0), 2)
        cv2.circle(frame, tuple(map(int, balloon["pt"])), 6, (255, 128, 0), -1)
    label(frame, text, (10, 25))


def tag_drift_px(cam, frame, det, obj, tag_id=0):
    """Mean px between where the floor tag's corners are and where the saved extrinsics say they should be.
    None if the tag is not visible. Big value = camera (or tag) moved since extrinsics: re-run extrinsics."""
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None or tag_id not in ids.ravel():
        return None
    seen = corners[list(ids.ravel()).index(tag_id)][0]
    rvec, _ = cv2.Rodrigues(cam.R)
    pred, _ = cv2.projectPoints(obj, rvec, cam.t, cam.K, cam.dist)
    return float(np.linalg.norm(pred.reshape(-1, 2) - seen, axis=1).mean())


def nulls(t, lost, skew_ms=None):
    """PROTOCOL s4 message saying 'nothing seen', with the reason in the extra 'lost' key."""
    return {"t": int(t), "balloon": None, "person": None, "person_id": -1, "src": "vision",
            "skew_ms": skew_ms, "lost": lost}


def step(cams, persons, balloon, frames, prev_t, now, max_skew=120, max_reproj=15.0,
         stale_ms=STALE_MS, person_z=PERSON_Z_M):
    """One localisation tick. Pure: no sockets, no sleeping.
    cams    {"A": Camera, "B": Camera}
    persons {"A": det, "B": det}   det.detect(frame) -> [{"id", "pt", ...}] largest first
    balloon det.detect(frame) -> {"pt", ...} | None   (one object shared by both cameras)
    frames  {"A": (frame, t_ms), "B": (frame, t_ms)}   as returned by Stream.latest()
    prev_t  {"A": t, "B": t}  timestamps last processed; MUTATED in place
    now     ms on the same clock as t_ms (protocol.now_ms)
    Returns (msg, dets):
      msg  None                 -> nothing new from BOTH cameras yet; send nothing
      msg  PROTOCOL s4 dict      -> send it. Extra keys: skew_ms, lost (None | "stale" | "skew"; main() adds "drift")
      dets {"A": (persons, balloon), "B": ...} for drawing, or None when the detectors did not run
    """
    (fA, tA), (fB, tB) = frames["A"], frames["B"]
    if fA is None or fB is None or tA is None or tB is None:
        return None, None
    if now - min(tA, tB) > stale_ms:
        return nulls(now, "stale"), None
    if tA == prev_t["A"] or tB == prev_t["B"]:
        return None, None
    prev_t["A"], prev_t["B"] = tA, tB
    skew = abs(tA - tB)
    t = (tA + tB) // 2
    if skew > max_skew:
        return nulls(t, "skew", skew), None

    pa, pb = persons["A"].detect(fA), persons["B"].detect(fB)
    ba, bb = balloon.detect(fA), balloon.detect(fB)
    person3 = balloon3 = None
    why = {"person": "A:%d B:%d" % (len(pa), len(pb)), "balloon": "A:%d B:%d" % (ba is not None, bb is not None)}
    ia = 0
    if pa and pb:
        # several people (judges, the friend at the desk): each camera lists its own largest-first; the pair that is
        # the SAME person is the one whose rays meet (smallest reprojection error) at a plausible chest height
        best = None
        for i, a in enumerate(pa[:MAX_PAIR]):
            for j, b in enumerate(pb[:MAX_PAIR]):
                uva, uvb, band, chest = person_point(a, b)
                if band is PERSON_Z_M:
                    band = person_z
                X, err = triangulate(cams["A"], cams["B"], uva, uvb)
                if err < max_reproj and band[0] <= X[2] <= band[1] and (best is None or err < best[1]):
                    best = (X, err, i, j, chest, "feet" if band is FEET_Z_M else "head" if band is HEAD_Z_M else "chest")
        if best is None:
            uva, uvb, band, _ = person_point(pa[0], pb[0])
            X, err = triangulate(cams["A"], cams["B"], uva, uvb)
            why["person"] = f"reproj {err:.0f} px" if err >= max_reproj else f"z={X[2]:.2f} m outside band {band}"
            if len(pa) > 1 or len(pb) > 1:
                why["person"] += f" (A:{len(pa)} B:{len(pb)}, no pair matched)"
        else:
            X, err, ia, j, chest, kind = best
            person3 = [round(float(X[0]), 3), round(float(X[1]), 3), round(float(chest(X[2])), 3)]
            why["person"] = f"ok {err:.1f} px via {kind}" + (f" (A#{ia} B#{j})" if (ia or j) else "")
    if ba and bb:
        X, err = triangulate(cams["A"], cams["B"], ba["pt"], bb["pt"])
        if err < max_reproj:
            balloon3 = [round(float(v), 3) for v in X]; why["balloon"] = f"ok {err:.1f} px"
        else:
            why["balloon"] = f"reproj {err:.0f} px"
    msg = {"t": int(t), "balloon": balloon3, "person": person3,
           "person_id": int(pa[ia]["id"]) if person3 is not None else -1,
           "src": "vision", "skew_ms": int(skew), "lost": None}
    return msg, {"A": (pa, ba), "B": (pb, bb), "why": why}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", default=config.SOURCES["A"]); ap.add_argument("--b", default=config.SOURCES["B"])
    ap.add_argument("--names", nargs=2, default=[config.CALIB_NAMES["A"], config.CALIB_NAMES["B"]], metavar=("NAME_A", "NAME_B"),
                    help="calibration names (calib/<name>_*.npz) for the A and B roles")
    ap.add_argument("--calib", default=config.CALIB_DIR)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--max-skew", type=int, default=120, help="ms between the two frames before we skip")
    ap.add_argument("--max-reproj", type=float, default=15.0, help="px; reject triangulations worse than this")
    ap.add_argument("--stale-ms", type=int, default=STALE_MS, help="ms; a frame older than this = camera lost")
    ap.add_argument("--device", default=None, help="ultralytics device: 0, cpu, mps ... (default: auto)")
    ap.add_argument("--conf-person", type=float, default=0.4)
    ap.add_argument("--conf-balloon", type=float, default=0.3)
    ap.add_argument("--balloon-color", default=config.BALLOON_COLOR, help="colour-blob balloon detector; 'none' = YOLO only")
    ap.add_argument("--tag-size", type=float, default=config.TAG_SIZE_M, help="m; for the tag-drift check (0 = off)")
    ap.add_argument("--drift-px", type=float, default=8.0, help="px; warn when the floor tag has moved more than this")
    ap.add_argument("--auto-calib", action="store_true", help="poses from the floor mat at start; re-solve a camera when its drift check trips 2 s running")
    ap.add_argument("--spacing", type=float, nargs="+", default=None, help="m between mat tag centres (one value or +X +Y); default: calib/mat.json from survey_mat.py, else config.MAT")
    ap.add_argument("--rot-a", type=int, default=config.ROTATE["A"]); ap.add_argument("--rot-b", type=int, default=config.ROTATE["B"], help="degrees clockwise to rotate each stream")
    ap.add_argument("--lag-b", type=int, default=0, help="ms of transport latency on camera B (WiFi phone); subtracted from its timestamps. Read it off vision_check --live 'A/B skew'")
    args = ap.parse_args()
    from .detect import BalloonDetector, PersonTracker   # lazy: ultralytics is slow to import and optional for --help

    layout = floor.get_layout(args.spacing, args.calib)
    print(f"[mat] layout from {floor.layout_source(args.spacing, args.calib)}")
    names = {"A": args.names[0], "B": args.names[1]}
    streams = {"A": Stream(args.a, names["A"], rotate=args.rot_a).wait_first(timeout=20),
               "B": Stream(args.b, names["B"], lag_ms=args.lag_b, rotate=args.rot_b).wait_first(timeout=20)}
    cams = {n: floor.load_camera(names[n], args.calib, streams[n], args.auto_calib, args.tag_size, layout) for n in ("A", "B")}
    for n, c in cams.items():
        print(f"[cam {n}] at world {np.round(c.position(), 2)} m")
    persons = {n: PersonTracker(config.YOLO_PERSON, conf=args.conf_person, device=args.device) for n in ("A", "B")}
    balloon = BalloonDetector(config.BALLOON_WEIGHTS, conf=args.conf_balloon, device=args.device,
                                  color=None if args.balloon_color in (None, "none") else args.balloon_color)
    print(f"[balloon] mode: {balloon.mode}")
    out = UdpJson()
    n_frames, t_rate = 0, time.monotonic()
    prev_t = {"A": None, "B": None}
    msg, why = nulls(now_ms(), None), {"person": "-", "balloon": "-"}
    tag_det, guards = make_tag_detector(), {n: floor.DriftGuard(args.drift_px) for n in ("A", "B")}
    try:
        while True:
            frames = {n: s.latest() for n, s in streams.items()}
            if time.monotonic() - t_rate > 1:          # once a second: drift checks (+ re-solves) and the status line
                if args.tag_size > 0:
                    for n in ("A", "B"):
                        if frames[n][0] is None:
                            continue
                        g = guards[n]
                        verdict = g.update(floor.drift_px(cams[n], frames[n][0], tag_det, args.tag_size, layout))
                        if verdict != "ok" and not args.auto_calib:
                            print(f"[localize] WARNING cam {n}: floor mat is {g.last:.0f} px from where extrinsics expect it: publishing lost. "
                                  f"Camera or mat moved -> re-run tools/calib/extrinsics.py --name {names[n]}, or use --auto-calib")
                        elif verdict == "resolve":
                            cam2, info = floor.auto_extrinsics(cams[n], streams[n], args.tag_size, layout, seconds=1.0, det=tag_det, calib_dir=args.calib)
                            if cam2 is not None:
                                cams[n] = cam2; g.reset()
                                print(f"[localize] cam {n} moved ({g.last:.0f} px): re-solved. " + floor.describe(cam2, info))
                            else:
                                print(f"[localize] cam {n} moved ({g.last:.0f} px) but cannot re-solve: {info['why']}")
                d = {n: guards[n].last and round(guards[n].last, 1) for n in ("A", "B")}
                print(f"[localize] {n_frames} Hz  camA {streams['A'].fps:.0f} fps  camB {streams['B'].fps:.0f} fps  "
                      f"skew {msg['skew_ms']} ms  lost={msg['lost']}  person={msg['person']} ({why['person']})  "
                      f"balloon={msg['balloon']} ({why['balloon']})  tag drift px A={d['A']} B={d['B']}")
                n_frames, t_rate = 0, time.monotonic()
            if any(g.bad for g in guards.values()):    # a stale pose: say so instead of publishing wrong fixes
                msg = nulls(now_ms(), "drift"); out.send(msg, ("127.0.0.1", STATE_PORT))
                time.sleep(0.05); continue
            msg2, dets = step(cams, persons, balloon, frames, prev_t, now_ms(),
                              args.max_skew, args.max_reproj, args.stale_ms)
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
                for n in ("A", "B"):
                    ps, bl = dets[n]
                    img = frames[n][0].copy()
                    draw(img, ps, bl, f"{n}  balloon={msg['balloon']}  person={msg['person']}")
                    cv2.imshow(f"cam {n}", cv2.resize(img, None, fx=0.6, fy=0.6))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        for s in streams.values(): s.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
