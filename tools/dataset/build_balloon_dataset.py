"""Assemble datasets/balloon (YOLO layout, one class 0 = balloon) from public sources + your own frames.

  python tools/dataset/build_balloon_dataset.py --matterport <balloon_dataset.zip> --openimages --negatives --site data/site
  python tools/dataset/build_balloon_dataset.py --stats               # what is in datasets/balloon now

Sources (each optional, idempotent, files are prefixed by source so they can be rebuilt independently):
  --matterport ZIP   Matterport Mask R-CNN "balloon" set (74 photos, VIA polygons -> boxes). Party balloons, close-ups.
  --openimages       Open Images V7 "Balloon" boxes from the public CSVs + S3 (party AND hot-air balloons: baskets =
                     our gondola, strings, all colours; cluster/drawing boxes dropped). val split -> our val,
                     test split -> our train, train split streamed until --oi-max images (default 1500).
  --negatives        Background images with NO balloons (coco128) so the detector learns what "no balloon" looks like.
  --hard-negatives N N Open Images photos that contain balls / cups / plates / lamps / heads / kites and NO balloon:
                     the things a half-trained detector mistakes for a balloon. YOLO treats unlabelled images as negatives.
  --site DIR         Your own labelled frames of THE balloon (from tools/dataset/label_site.py). Weighted x --site-repeat
                     (default 4) so 50 site images matter against 1500 web images.
Train/val split is 90/10 per source, deterministic (hash of the file name), so re-running does not leak val into train.
"""
import argparse, glob, hashlib, json, os, shutil, sys, zipfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

ROOT = "datasets/balloon"
YAML = f"{ROOT}/data.yaml"
YAML_TEXT = "path: datasets/balloon\ntrain: images/train\nval: images/val\nnames:\n  0: balloon\n"


def split_of(name, val_frac=0.1):
    h = int(hashlib.md5(name.encode()).hexdigest(), 16) % 1000
    return "val" if h < val_frac * 1000 else "train"


def put(src_img, name, boxes, split, wh=None):
    """Copy an image into the split and write its YOLO label (boxes: list of (x1,y1,x2,y2) in pixels; [] = negative)."""
    os.makedirs(f"{ROOT}/images/{split}", exist_ok=True); os.makedirs(f"{ROOT}/labels/{split}", exist_ok=True)
    ext = os.path.splitext(src_img)[1].lower() or ".jpg"
    dst = f"{ROOT}/images/{split}/{name}{ext}"
    if not os.path.exists(dst):
        shutil.copy(src_img, dst)
    if wh is None:
        import cv2
        im = cv2.imread(dst); wh = (im.shape[1], im.shape[0])
    W, H = wh
    lines = []
    for x1, y1, x2, y2 in boxes:
        x1, x2 = max(0, min(x1, x2)), min(W, max(x1, x2)); y1, y2 = max(0, min(y1, y2)), min(H, max(y1, y2))
        if x2 - x1 < 4 or y2 - y1 < 4: continue
        lines.append(f"0 {(x1 + x2) / 2 / W:.6f} {(y1 + y2) / 2 / H:.6f} {(x2 - x1) / W:.6f} {(y2 - y1) / H:.6f}")
    with open(f"{ROOT}/labels/{split}/{name}.txt", "w") as f:
        f.write("\n".join(lines) + ("\n" if lines else ""))
    return len(lines)


def do_matterport(zip_path):
    tmp = f"{ROOT}/_src/matterport"
    if not os.path.isdir(tmp):
        with zipfile.ZipFile(zip_path) as z:
            z.extractall(tmp)
    n_img = n_box = 0
    for sub in ("train", "val"):
        d = os.path.join(tmp, "balloon", sub)
        meta = json.load(open(os.path.join(d, "via_region_data.json")))
        for v in meta.values():
            fn = v["filename"]; p = os.path.join(d, fn)
            if not os.path.exists(p): continue
            regs = v["regions"].values() if isinstance(v["regions"], dict) else v["regions"]
            boxes = []
            for r in regs:
                xs, ys = r["shape_attributes"]["all_points_x"], r["shape_attributes"]["all_points_y"]
                boxes.append((min(xs), min(ys), max(xs), max(ys)))
            name = "mp_" + os.path.splitext(fn)[0]
            n_box += put(p, name, boxes, split_of(name)); n_img += 1
    print(f"[matterport] {n_img} images, {n_box} boxes")


OI_CLASS = "/m/01j51"                    # "Balloon" in oidv7-class-descriptions-boxable.csv
OI_CSV = {"validation": "https://storage.googleapis.com/openimages/v5/validation-annotations-bbox.csv",   # 25 MB
          "test": "https://storage.googleapis.com/openimages/v5/test-annotations-bbox.csv",               # 77 MB
          "train": "https://storage.googleapis.com/openimages/v6/oidv6-train-annotations-bbox.csv"}       # 2.2 GB, streamed
OI_IMG = "https://open-images-dataset.s3.amazonaws.com/{split}/{id}.jpg"


def oi_boxes(split, max_images, cache_dir):
    """{image_id: [(xmin,xmax,ymin,ymax) normalised]} for images with >= 1 clean balloon box (no group / drawing boxes).
    Small CSVs are cached; the 2.2 GB train CSV is streamed through and never stored."""
    import csv, io, urllib.request
    cache = os.path.join(cache_dir, f"oi_{split}_balloon.csv")
    if os.path.exists(cache):
        src = open(cache, encoding="utf-8")
    else:
        print(f"[openimages] reading {split} annotations ({'streaming 2.2 GB' if split == 'train' else 'download'})...")
        src = io.TextIOWrapper(urllib.request.urlopen(OI_CSV[split]), encoding="utf-8")
    good, bad = {}, set()
    keep = []
    for row in csv.reader(src):
        if len(row) < 12 or row[2] != OI_CLASS: continue
        keep.append(row)
        iid = row[0]
        if row[10] == "1" or row[11] == "1":            # IsGroupOf (a cluster boxed as one) / IsDepiction (drawing)
            bad.add(iid); continue
        good.setdefault(iid, []).append((float(row[4]), float(row[5]), float(row[6]), float(row[7])))
        if split == "train" and len(good) >= max_images * 2: break      # enough candidates, stop the stream early
    if not os.path.exists(cache):
        os.makedirs(cache_dir, exist_ok=True)
        with open(cache, "w", encoding="utf-8", newline="") as f:
            csv.writer(f).writerows(keep)
    return {k: v for k, v in good.items() if k not in bad}


def do_openimages(max_train, max_val):
    """Open Images V7 'Balloon' boxes, straight from the public CSVs + S3 bucket (no FiftyOne / MongoDB needed)."""
    import random, urllib.request
    from concurrent.futures import ThreadPoolExecutor
    import cv2
    cache_dir = f"{ROOT}/_src/openimages"
    plan = [("validation", "val", max_val), ("test", "val", max_val), ("train", "train", max_train)]   # OI val+test = our val (~115 imgs)
    n_img = n_box = 0
    for split, dst_split, cap in plan:
        boxes = oi_boxes(split, cap or 10 ** 9, cache_dir)
        ids = sorted(boxes); random.Random(0).shuffle(ids)
        if cap: ids = ids[:cap]
        img_dir = os.path.join(cache_dir, split); os.makedirs(img_dir, exist_ok=True)

        def fetch(iid):
            p = os.path.join(img_dir, iid + ".jpg")
            if not os.path.exists(p):
                try: urllib.request.urlretrieve(OI_IMG.format(split=split, id=iid), p)
                except Exception as e: return None
            return p
        with ThreadPoolExecutor(16) as ex:
            paths = list(ex.map(fetch, ids))
        k = 0
        for iid, p in zip(ids, paths):
            if not p: continue
            im = cv2.imread(p)
            if im is None: os.remove(p); continue
            H, W = im.shape[:2]
            bx = [(x0 * W, y0 * H, x1 * W, y1 * H) for x0, x1, y0, y1 in boxes[iid]]
            n_box += put(p, "oi_" + iid, bx, dst_split, (W, H)); k += 1
        n_img += k
        print(f"[openimages] {split}: {k} images -> {dst_split}")
    print(f"[openimages] total {n_img} images, {n_box} boxes")


# Open Images classes that look like a balloon to a half-trained detector. Images containing any of these and NO
# balloon become hard negatives (no label file): the demo hall is full of heads, lamps, cups and balls.
OI_HARD = {"Ball": "/m/018xm", "Football": "/m/01226z", "Tennis ball": "/m/05ctyq", "Volleyball (Ball)": "/m/02rgn06",
           "Coffee cup": "/m/02p5f1q", "Mug": "/m/02jvh9", "Bowl": "/m/04kkgm", "Plate": "/m/050gv4", "Clock": "/m/01x3z",
           "Lamp": "/m/0dtln", "Light bulb": "/m/0h8mzrc", "Orange": "/m/0cyhj_", "Apple": "/m/014j1m", "Egg (Food)": "/m/033cnk",
           "Human head": "/m/04hgtk", "Helmet": "/m/0zvk5", "Kite": "/m/02zt3", "Frisbee": "/m/02wmf", "Globe": "/m/0hqkz"}


def oi_hard_negative_ids(split, n, cache_dir):
    """Image ids in an OI split that contain a hard-negative class and no balloon (full CSV cached, 25/77 MB)."""
    import csv, urllib.request, random
    full = os.path.join(cache_dir, f"{split}_bbox_full.csv")
    if not os.path.exists(full):
        os.makedirs(cache_dir, exist_ok=True)
        print(f"[hard-neg] downloading {split} annotation CSV...")
        urllib.request.urlretrieve(OI_CSV[split], full)
    hard, balloon = set(), set()
    want = set(OI_HARD.values())
    for row in csv.reader(open(full, encoding="utf-8")):
        if len(row) < 3: continue
        if row[2] == OI_CLASS: balloon.add(row[0])
        elif row[2] in want: hard.add(row[0])
    ids = sorted(hard - balloon); random.Random(1).shuffle(ids)
    return ids[:n]


def do_hard_negatives(n):
    import urllib.request
    from concurrent.futures import ThreadPoolExecutor
    import cv2
    cache_dir = f"{ROOT}/_src/openimages"
    per = {"validation": n // 3, "test": n - n // 3}
    total = 0
    for split, k in per.items():
        ids = oi_hard_negative_ids(split, k, cache_dir)
        img_dir = os.path.join(cache_dir, split); os.makedirs(img_dir, exist_ok=True)

        def fetch(iid):
            p = os.path.join(img_dir, iid + ".jpg")
            if not os.path.exists(p):
                try: urllib.request.urlretrieve(OI_IMG.format(split=split, id=iid), p)
                except Exception: return None
            return p
        with ThreadPoolExecutor(16) as ex:
            paths = list(ex.map(fetch, ids))
        for iid, p in zip(ids, paths):
            if not p or cv2.imread(p) is None: continue
            name = "hneg_" + iid
            put(p, name, [], split_of(name)); total += 1
    print(f"[hard-neg] {total} balloon-free images with balls / cups / lamps / heads / kites")


def do_negatives():
    """coco128 (ultralytics sample set): 128 COCO images, none of which contain balloons -> pure background."""
    from pathlib import Path
    from ultralytics.utils.downloads import safe_download
    tmp = f"{ROOT}/_src"
    os.makedirs(tmp, exist_ok=True)
    if not os.path.isdir(f"{tmp}/coco128"):
        safe_download("https://github.com/ultralytics/assets/releases/download/v0.0.0/coco128.zip", dir=Path(tmp), unzip=True, delete=True)
    n = 0
    for p in sorted(glob.glob(f"{tmp}/coco128/images/train2017/*.jpg")):
        name = "neg_" + os.path.splitext(os.path.basename(p))[0]
        put(p, name, [], split_of(name)); n += 1
    print(f"[negatives] {n} background images")


def do_site(d, repeat):
    """YOLO-format folder produced on site: DIR/images/*.jpg + DIR/labels/*.txt (class 0)."""
    imgs = sorted(glob.glob(os.path.join(d, "images", "*.jpg")) + glob.glob(os.path.join(d, "images", "*.png")))
    n = 0
    for p in imgs:
        stem = os.path.splitext(os.path.basename(p))[0]
        lab = os.path.join(d, "labels", stem + ".txt")
        split = split_of("site_" + stem)
        for k in range(repeat if split == "train" else 1):
            name = f"site_{stem}_r{k}"
            ext = os.path.splitext(p)[1]
            os.makedirs(f"{ROOT}/images/{split}", exist_ok=True); os.makedirs(f"{ROOT}/labels/{split}", exist_ok=True)
            shutil.copy(p, f"{ROOT}/images/{split}/{name}{ext}")
            shutil.copy(lab, f"{ROOT}/labels/{split}/{name}.txt") if os.path.exists(lab) else open(f"{ROOT}/labels/{split}/{name}.txt", "w").close()
        n += 1
    print(f"[site] {n} images (train copies x{repeat})")


def stats():
    for split in ("train", "val"):
        imgs = glob.glob(f"{ROOT}/images/{split}/*")
        by = {}
        for p in imgs:
            src = os.path.basename(p).split("_")[0]
            lab = f"{ROOT}/labels/{split}/{os.path.splitext(os.path.basename(p))[0]}.txt"
            nb = sum(1 for l in open(lab) if l.strip()) if os.path.exists(lab) else 0
            a = by.setdefault(src, [0, 0]); a[0] += 1; a[1] += nb
        print(f"{split}: {len(imgs)} images  " + "  ".join(f"{k}={v[0]} imgs/{v[1]} boxes" for k, v in sorted(by.items())))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--matterport", metavar="ZIP"); ap.add_argument("--openimages", action="store_true")
    ap.add_argument("--oi-max", type=int, default=1500); ap.add_argument("--oi-max-val", type=int, default=200)
    ap.add_argument("--negatives", action="store_true"); ap.add_argument("--site", metavar="DIR")
    ap.add_argument("--hard-negatives", type=int, default=0, metavar="N", help="N Open Images photos of balls/cups/lamps/heads with no balloon")
    ap.add_argument("--site-repeat", type=int, default=4); ap.add_argument("--stats", action="store_true")
    args = ap.parse_args()
    os.makedirs(ROOT, exist_ok=True)
    if not os.path.exists(YAML): open(YAML, "w").write(YAML_TEXT)
    if args.matterport: do_matterport(args.matterport)
    if args.openimages: do_openimages(args.oi_max, args.oi_max_val)
    if args.negatives: do_negatives()
    if args.hard_negatives: do_hard_negatives(args.hard_negatives)
    if args.site: do_site(args.site, args.site_repeat)
    stats()


if __name__ == "__main__":
    main()
