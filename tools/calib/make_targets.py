"""Generate printable calibration targets (300 DPI PNGs, A4 canvas). Print at 100% / "actual size".

  python tools/calib/make_targets.py                # targets/checkerboard_9x6_25mm_A4.png + apriltag36h11_id{0,1,2,3}_<mm>mm_A4.png (the floor mat)
  python tools/calib/make_targets.py --tag-mm 300   # bigger tags (poster/tiled print; measure afterwards)

After printing, MEASURE: one checkerboard square (should be 25.0 mm) and a tag's black square edge (printers scale!).
Put the measured values in laptop/config.py (SQUARE_M, TAG_SIZE_M). Tape the four tags flat on the floor at the
corners of a square, every page the same way up (arrow on the page), tag 0 at the origin, tag 1 along +X, tag 3 along
+Y; measure the centre-to-centre spacing and put it in config.MAT_SPACING_M.
"""
import argparse, os
import cv2, numpy as np
from PIL import Image
import _bootstrap  # noqa: F401
from laptop import config

DPI = 300
A4 = (int(210 / 25.4 * DPI), int(297 / 25.4 * DPI))   # (w, h) portrait px


def canvas(w, h):
    return np.full((h, w), 255, np.uint8)


def paste_center(bg, img):
    y = (bg.shape[0] - img.shape[0]) // 2; x = (bg.shape[1] - img.shape[1]) // 2
    bg[y:y + img.shape[0], x:x + img.shape[1]] = img
    return bg


def save(path, img):
    Image.fromarray(img).save(path, dpi=(DPI, DPI))
    print("wrote", path, f"({img.shape[1]}x{img.shape[0]} px @ {DPI} dpi)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="targets")
    ap.add_argument("--cols", type=int, default=9); ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--tag-mm", type=float, default=round(config.TAG_SIZE_M * 1000, 1),
                    help="black-square edge; default = config.TAG_SIZE_M so the print matches what extrinsics.py assumes")
    ap.add_argument("--tag-ids", type=int, nargs="+", default=sorted(config.MAT), help="mat tags to generate (default: config.MAT)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    # checkerboard: (cols+1) x (rows+1) squares -> cols x rows inner corners
    sq = int(round(args.square_mm / 25.4 * DPI))
    nc, nr = args.cols + 1, args.rows + 1
    board = canvas(nc * sq, nr * sq)
    for r in range(nr):
        for c in range(nc):
            if (r + c) % 2 == 0:
                board[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = 0
    page = paste_center(canvas(A4[1], A4[0]), board)   # landscape
    cv2.putText(page, f"checkerboard {args.cols}x{args.rows} inner corners, {args.square_mm:.1f} mm squares - print at 100%",
                (60, A4[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
    save(os.path.join(args.out, f"checkerboard_{args.cols}x{args.rows}_{int(args.square_mm)}mm_A4.png"), page)

    # AprilTag 36h11: generated image edge == black square edge == TAG_SIZE
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    px = int(round(args.tag_mm / 25.4 * DPI))
    for tag_id in args.tag_ids:
        tag = cv2.aruco.generateImageMarker(d, tag_id, px, borderBits=1)
        if px + 2 * sq > A4[0]:
            print(f"NOTE: a {args.tag_mm:.0f} mm tag does not fit on A4; saving the bare tag - print poster/tiled and tape.")
            page = paste_center(canvas(px + 2 * sq, px + 2 * sq), tag)
        else:
            page = paste_center(canvas(*A4), tag)
        spot = config.MAT.get(tag_id)
        where = f"mat spot x={spot[0]:.2f} y={spot[1]:.2f} m" if spot else "not in config.MAT"
        cv2.putText(page, f"^ THIS EDGE UP = +Y  (all mat pages the same way)", (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
        cv2.putText(page, f"AprilTag 36h11 id {tag_id}   black edge = {args.tag_mm:.0f} mm (MEASURE)   {where}",
                    (40, page.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 3)
        save(os.path.join(args.out, f"apriltag36h11_id{tag_id}_{int(args.tag_mm)}mm_A4.png"), page)


if __name__ == "__main__":
    main()
