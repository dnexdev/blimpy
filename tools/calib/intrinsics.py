"""Intrinsic calibration: ONCE per camera, at the LOCKED focus/zoom it will fly with.

  python tools/calib/intrinsics.py --source 0 --name A
  python tools/calib/intrinsics.py --source http://192.168.137.21:4747/video --name B

Print targets/checkerboard_9x6_25mm_A4.png (make_targets.py) on stiff card. Show it to the camera:
near, far, tilted, in every corner of the image. A shot is auto-captured every 1.5 s when the board
is found (SPACE forces one). Calibrates automatically at --shots (default 20); 'c' calibrates early
(>= 12 shots); 'q' quits. Saves calib/<name>_intrinsics.npz. Good RMS < 0.5 px; redo if > 1 px.
Mac webcam: turn OFF Center Stage / auto-framing (Control Center -> Video Effects) first, it re-crops the image
between shots and no single K fits. If calibration keeps failing, --nominal 1280 720 writes a plausible pinhole
(no distortion) so extrinsics/localize can run; range error ~5 %, fine for the demo.
"""
import argparse, time
import cv2, numpy as np
import _bootstrap  # noqa: F401
from laptop import config
from laptop.vision.calib_io import Camera, save_intrinsics
from laptop.vision.streams import Stream, label


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default=None, help="camera index or URL (not needed with --nominal)"); ap.add_argument("--name", required=True, help="A or B")
    ap.add_argument("--cols", type=int, default=config.CHECKER_COLS); ap.add_argument("--rows", type=int, default=config.CHECKER_ROWS)
    ap.add_argument("--square", type=float, default=config.SQUARE_M, help="square edge in metres")
    ap.add_argument("--shots", type=int, default=20); ap.add_argument("--interval", type=float, default=1.5)
    ap.add_argument("--calib", default=config.CALIB_DIR)
    ap.add_argument("--min-sharp", type=float, default=60.0, help="Laplacian variance; frames blurrier than this are skipped")
    ap.add_argument("--nominal", nargs=2, type=int, metavar=("W", "H"),
                    help="skip calibration: save a nominal pinhole (f = 0.85*W, centre, no distortion) for a WxH stream")
    args = ap.parse_args()
    if args.nominal:
        w, h = args.nominal
        cam = Camera.nominal(args.name, (w, h), args.calib)
        print(f"nominal intrinsics saved to {args.calib}/{args.name}_intrinsics.npz: f={cam.K[0, 0]:.0f} px, centre ({w / 2:.0f}, {h / 2:.0f}). Expect ~5 % range error.")
        return
    if args.source is None:
        ap.error("--source is required unless --nominal is given")

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_pts, img_pts = [], []
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK

    s = Stream(args.source, args.name).wait_first()
    next_cap, flash, size = 0.0, 0, None
    print(f"[intrinsics {args.name}] source={args.source} pattern={pattern} square={args.square} m")
    while True:
        frame, _ = s.latest()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY); size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, pattern, flags)
        key = cv2.waitKey(1) & 0xFF
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
            cv2.drawChessboardCorners(frame, pattern, corners, found)
            sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
            if sharp < args.min_sharp:
                found = False          # motion blur: do not use this frame
            elif time.monotonic() >= next_cap or key == ord(" "):
                obj_pts.append(objp); img_pts.append(corners)
                next_cap, flash = time.monotonic() + args.interval, 3
                print(f"  shot {len(obj_pts)}/{args.shots}")
        if flash:
            frame[:] = 255; flash -= 1
        label(frame, f"{args.name}: shots {len(obj_pts)}/{args.shots}  {'BOARD OK' if found else 'move board into view / hold still'}  "
              f"[space]=shot [c]=calibrate [q]=quit", (10, 30), 0.7, (0, 255, 255))
        cv2.imshow(f"intrinsics {args.name}", frame)
        if key == ord("q"):
            print("aborted"); break
        if (key == ord("c") and len(obj_pts) >= 12) or len(obj_pts) >= args.shots:
            print("calibrating...")
            # square pixels, no tangential term, no k3: far more stable with 20 hand-held shots than the free model
            K0 = np.array([[size[0], 0, size[0] / 2], [0, size[0], size[1] / 2], [0, 0, 1]], float)
            cflags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO
                      | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
            rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, size, K0, None, flags=cflags)
            # per-shot error; drop the worst shots (mis-detected corners wreck the fit) and re-solve
            for _ in range(3):
                errs = []
                for o, i, r, t in zip(obj_pts, img_pts, rvecs, tvecs):
                    proj, _ = cv2.projectPoints(o, r, t, K, dist)
                    errs.append(float(np.sqrt(((proj.reshape(-1, 2) - i.reshape(-1, 2)) ** 2).sum(1).mean())))
                print("  per-shot RMS px:", np.round(errs, 2))
                bad = [k for k, e in enumerate(errs) if e > max(2.0, 2.5 * float(np.median(errs)))]
                if not bad or len(obj_pts) - len(bad) < 8:
                    break
                print(f"  dropping shots {[k + 1 for k in bad]} and re-solving")
                obj_pts = [o for k, o in enumerate(obj_pts) if k not in bad]
                img_pts = [i for k, i in enumerate(img_pts) if k not in bad]
                rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, size, K0, None, flags=cflags)
            p = save_intrinsics(args.calib, args.name, K, dist, size, rms)
            print(f"RMS = {rms:.3f} px  ({'good' if rms < 0.6 else 'meh, consider redoing'})")
            cx, cy = K[0, 2], K[1, 2]
            if not (0.3 * size[0] < cx < 0.7 * size[0] and 0.3 * size[1] < cy < 0.7 * size[1]):
                print(f"WARNING: principal point ({cx:.0f}, {cy:.0f}) far from the image centre ({size[0] / 2:.0f}, {size[1] / 2:.0f}). "
                      f"Bad fit: redo with the board tilted in all directions and near every edge/corner of the frame.")
            if abs(dist.ravel()[4]) > 1.0 or abs(dist.ravel()[1]) > 1.0:
                print("WARNING: large k2/k3 distortion terms = overfit. Redo with more varied board poses (tilt!) and no motion blur.")
            print("K =\n", np.round(K, 1)); print("dist =", np.round(dist.ravel(), 4)); print("saved", p)
            break
    s.stop(); cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
