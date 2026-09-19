"""Vision smoke test: one camera -> YOLO person tracker + balloon detector, drawn live.

  python tools/webcam_test.py                 # webcam 0
  python tools/webcam_test.py --source 1      # second webcam / DroidCam URL / udp://@:5000 / clip.mp4
  python tools/webcam_test.py --no-show       # headless: just print detections + fps (for SSH / CI)

Green box = person (id from the tracker), magenta = balloon: models/balloon.pt (site fine-tune) if it exists, else the
committed models/balloon_web.pt, else colour blob / COCO 'sports ball'. q or ESC quits. First run downloads config.YOLO_PERSON.
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
from laptop import config
from laptop.vision.streams import Stream
from laptop.vision.detect import PersonTracker, BalloonDetector


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N s (0 = until q)")
    ap.add_argument("--device", default=None, help="'cpu' or '0' (GPU); default = ultralytics picks")
    args = ap.parse_args()

    people = PersonTracker(config.YOLO_PERSON, device=args.device)
    balloon = BalloonDetector(config.BALLOON_WEIGHTS, device=args.device, color=config.BALLOON_COLOR)
    print(f"[test] balloon mode: {balloon.mode}")
    cam = Stream(args.source, "cam").wait_first()
    print(f"[test] camera {args.source!r} up, frame {cam.latest()[0].shape[1]}x{cam.latest()[0].shape[0]}")
    t0 = last_t = time.monotonic(); n = 0; infer_ms = 0.0
    try:
        while True:
            frame, t_ms = cam.latest()
            if frame is None or t_ms == last_t:
                time.sleep(0.005); continue
            last_t = t_ms
            ta = time.monotonic()
            ps, b = people.detect(frame), balloon.detect(frame)
            infer_ms = 0.9 * infer_ms + 0.1 * (time.monotonic() - ta) * 1000
            n += 1
            if n % 15 == 0:
                who = ", ".join(f"person#{p['id']} {p['conf']:.2f}" for p in ps) or "no person"
                bb = f"balloon {b['conf']:.2f} at ({b['pt'][0]:.0f},{b['pt'][1]:.0f})" if b else "no balloon"
                print(f"[test] cam {cam.fps:4.1f} fps  infer {infer_ms:5.1f} ms  | {who} | {bb}")
            if not args.no_show:
                for p in ps:
                    x1, y1, x2, y2 = map(int, p["box"])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, tuple(map(int, p["pt"])), 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"person {p['id']} {p['conf']:.2f}", (x1, max(15, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if b:
                    x1, y1, x2, y2 = map(int, b["box"])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    cv2.putText(frame, f"balloon {b['conf']:.2f}", (x1, max(15, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                cv2.putText(frame, f"{cam.fps:.0f} fps  {infer_ms:.0f} ms", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow("blimpy vision test (q quits)", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            if args.seconds and time.monotonic() - t0 > args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop(); cv2.destroyAllWindows()
        print(f"[test] {n} frames, last infer {infer_ms:.1f} ms")


if __name__ == "__main__":
    main()
