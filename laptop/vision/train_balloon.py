"""Balloon detector training, two stages. BalloonDetector (detect.py) loads models/balloon.pt automatically.

  Stage "web"  (done once, ~40 min on the 5080; you should not need to redo it):
      python tools/dataset/build_balloon_dataset.py --matterport balloon_dataset.zip --openimages --negatives
      python -m laptop.vision.train_balloon --stage web          -> models/balloon_web.pt (+ copied to models/balloon.pt)
      Learns "balloon" in general from ~1.7k public photos: party balloons, hot-air balloons with baskets (our gondola),
      strings, clusters, every colour, plus 128 balloon-free room/street photos as negatives.

  Stage "site" (on the day, ~3 min):
      python tools/dataset/label_site.py --source 0             # ~50 frames of THE balloon (fins, gondola, your light)
      python -m laptop.vision.train_balloon --stage site        -> models/balloon.pt
      Fine-tunes balloon_web.pt on web + site frames (site repeated x4). Backbone frozen, low LR: fast and it cannot
      forget the web data. Validation is reported on web val AND on your held-out site frames separately.

  python -m laptop.vision.train_balloon --eval [--weights models/balloon.pt]   # metrics only
"""
import argparse, glob, os, shutil, sys

DATA_YAML = "datasets/balloon/data.yaml"
SITE_YAML = "datasets/balloon/site_val.yaml"


def site_val_yaml():
    """A data.yaml whose val split is ONLY the site frames (so the number that matters is reported on its own)."""
    imgs = sorted(p for p in glob.glob("datasets/balloon/images/val/site_*"))
    if not imgs: return None
    lst = "datasets/balloon/site_val.txt"
    open(lst, "w").write("\n".join(os.path.abspath(p) for p in imgs) + "\n")
    open(SITE_YAML, "w").write(f"path: {os.path.abspath('datasets/balloon')}\ntrain: images/train\nval: {os.path.abspath(lst)}\nnames:\n  0: balloon\n")
    return SITE_YAML


def evaluate(weights, device):
    from ultralytics import YOLO
    m = YOLO(weights)
    r = m.val(data=DATA_YAML, device=device, verbose=False, plots=False, quantize=16)
    print(f"[eval] {weights} on web val: mAP50 {r.box.map50:.3f}  mAP50-95 {r.box.map:.3f}  P {r.box.mp:.3f}  R {r.box.mr:.3f}")
    sy = site_val_yaml()
    if sy:
        r = m.val(data=sy, device=device, verbose=False, plots=False, quantize=16)
        print(f"[eval] {weights} on SITE val: mAP50 {r.box.map50:.3f}  mAP50-95 {r.box.map:.3f}  P {r.box.mp:.3f}  R {r.box.mr:.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", choices=["web", "site"], default="site")
    ap.add_argument("--base", default=None, help="web: COCO weights (default yolo11s.pt); site: default models/balloon_web.pt")
    ap.add_argument("--epochs", type=int, default=None); ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16); ap.add_argument("--device", default="0")
    ap.add_argument("--eval", action="store_true"); ap.add_argument("--weights", default="models/balloon.pt")
    ap.add_argument("--tag", default="", help="web: run name suffix + models/balloon_web_<tag>.pt (compare candidates)")
    ap.add_argument("--no-promote", action="store_true", help="web: do not overwrite models/balloon.pt")
    args = ap.parse_args()
    if args.eval:
        return evaluate(args.weights, args.device)
    if not os.path.exists(DATA_YAML):
        sys.exit(f"{DATA_YAML} missing: run tools/dataset/build_balloon_dataset.py first")
    from ultralytics import YOLO
    os.makedirs("models", exist_ok=True)
    common = dict(data=DATA_YAML, imgsz=args.imgsz, batch=args.batch, device=args.device, project="runs", exist_ok=True,
                  plots=False, workers=4,
                  # a balloon is a balloon in any colour / light: strong hue + brightness jitter; no vertical flips
                  hsv_h=0.2, hsv_s=0.7, hsv_v=0.5, degrees=10, scale=0.5, fliplr=0.5, flipud=0.0, mosaic=1.0, close_mosaic=5)
    if args.stage == "web":
        base = args.base or "yolo11s.pt"
        m = YOLO(base)
        res = m.train(name="balloon_web" + (f"_{args.tag}" if args.tag else ""), epochs=args.epochs or 40, patience=12, **common)
        best = os.path.join(res.save_dir, "weights", "best.pt")
        out = f"models/balloon_web{'_' + args.tag if args.tag else ''}.pt"
        shutil.copy(best, out); print(f"saved {out}")
        if not args.no_promote:
            shutil.copy(best, "models/balloon.pt"); print("promoted to models/balloon.pt")
        evaluate(out, args.device)
    else:
        base = args.base or ("models/balloon_web.pt" if os.path.exists("models/balloon_web.pt") else "yolo11s.pt")
        if not glob.glob("datasets/balloon/images/train/site_*"):
            sys.exit("no site frames in the dataset: python tools/dataset/label_site.py ... then "
                     "python tools/dataset/build_balloon_dataset.py --site data/site")
        m = YOLO(base)
        res = m.train(name="balloon_site", epochs=args.epochs or 15, freeze=10, lr0=0.002, lrf=0.1, warmup_epochs=1,
                      mosaic=0.5, **{k: v for k, v in common.items() if k != "mosaic"})
        best = os.path.join(res.save_dir, "weights", "best.pt")
        shutil.copy(best, "models/balloon.pt")
        print("saved models/balloon.pt")
        evaluate("models/balloon.pt", args.device)


if __name__ == "__main__":
    main()
