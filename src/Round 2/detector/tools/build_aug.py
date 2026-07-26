"""build_aug.py — manufacture the two things the dataset has ZERO of.

Audited holes (2026-07-25), both mapping onto a reported failure mode:
  * 0 of 597 images contain NO pillar   -> nothing teaches "this is not a pillar"
  * 0 of 597 images contain BOTH classes -> the co-occurrence that dominates the
    real field is a case the model has literally never seen

This script produces three derived sets from the images you already have:
  negatives/  real pillar-free crops + synthetic colour distractors, empty labels
  cooc/       real images with an opposite-class pillar composited in
  sprites/    the extracted pillar cut-outs (kept for inspection/reuse)

LEAKAGE RULE, enforced: sprites harvested from TRAIN groups are only ever pasted
into TRAIN images; VAL sprites only into VAL images. Violating this re-creates
the exact train/val contamination make_split.py just removed.

    python build_aug.py --splits splits

Outputs new manifests: train_aug.txt, val_neg.txt, val_cooc.txt.
val.txt is left untouched so the headline metric stays comparable to baseline.
"""

import argparse
import os
import random

import cv2
import numpy as np

# OpenCV hue is 0-179. Red wraps, so it needs two bands.
HSV_BANDS = {
    1: [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (179, 255, 255))],   # red
    0: [((35, 60, 40), (85, 255, 255))],                                    # green
}
# Distractor hues that are NOT a class: orange, yellow, blue, and magenta.
# Magenta matters specifically -- it is the WRO parking-zone colour, so the
# detector must see it and stay silent rather than firing on it.
DISTRACTOR_HUES = [(12, 22), (25, 33), (100, 125), (145, 165)]


def load_labels(img_path):
    lab = img_path.replace(os.sep + "images" + os.sep, os.sep + "labels" + os.sep)
    lab = os.path.splitext(lab)[0] + ".txt"
    rows = []
    if os.path.exists(lab):
        for line in open(lab):
            p = line.split()
            if len(p) >= 5:
                rows.append((int(p[0]), *(float(x) for x in p[1:5])))
    return rows


def write_pair(out_dir, name, img, rows):
    """Write image + YOLO label into the images/ labels/ convention."""
    idir, ldir = os.path.join(out_dir, "images"), os.path.join(out_dir, "labels")
    os.makedirs(idir, exist_ok=True)
    os.makedirs(ldir, exist_ok=True)
    p = os.path.join(idir, name + ".jpg")
    cv2.imwrite(p, img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    with open(os.path.join(ldir, name + ".txt"), "w") as fh:
        fh.write("".join(f"{c} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n"
                         for c, cx, cy, w, h in rows))
    return p


def extract_sprite(img, box, cls):
    """Cut a pillar out of its box using an HSV band + largest component.

    Pillars are saturated solid colour, so a threshold beats GrabCut here and
    costs nothing. Returns (bgr, alpha) or None if the cut looks unreliable.
    """
    H, W = img.shape[:2]
    _, cx, cy, bw, bh = box
    x1, y1 = int((cx - bw / 2) * W), int((cy - bh / 2) * H)
    x2, y2 = int((cx + bw / 2) * W), int((cy + bh / 2) * H)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(W, x2), min(H, y2)
    if x2 - x1 < 12 or y2 - y1 < 12:
        return None
    crop = img[y1:y2, x1:x2]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    m = np.zeros(crop.shape[:2], np.uint8)
    for lo, hi in HSV_BANDS[cls]:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    m = (lbl == k).astype(np.uint8) * 255
    # reject cuts that clearly failed: a real pillar fills much of its own box
    if m.mean() / 255.0 < 0.25:
        return None
    ys, xs = np.where(m > 0)
    return crop[ys.min():ys.max() + 1, xs.min():xs.max() + 1], \
        m[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def paste(dst, sprite, alpha, x, y, feather):
    """Alpha-composite at (x, y). Feather is randomised per paste on purpose.

    Ghiasi et al. found blanket Gaussian blending gave no benefit, and a single
    fixed blending style becomes an artifact the detector can learn as a cue.
    Varying it means the seam is not a consistent signal.
    """
    h, w = alpha.shape
    H, W = dst.shape[:2]
    if x < 0 or y < 0 or x + w > W or y + h > H:
        return False
    a = alpha.astype(np.float32) / 255.0
    if feather:
        a = cv2.GaussianBlur(a, (0, 0), feather)
    a = a[..., None]
    roi = dst[y:y + h, x:x + w]
    dst[y:y + h, x:x + w] = (a * sprite + (1 - a) * roi).astype(np.uint8)
    return True


def iou_xyxy(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def mine_negatives(files, out_dir, tag, want, rng):
    """Crop pillar-free regions out of real images.

    These are REAL backgrounds from the real capture environment, which is the
    honest ceiling of what can be produced without new footage. They are NOT
    venue clutter -- no other robots, spectators, banners or reflections exist
    anywhere in this dataset. Expect them to suppress generic texture firing,
    not competition-specific false positives.
    """
    made = 0
    order = list(files)
    rng.shuffle(order)
    for path in order:
        if made >= want:
            break
        img = cv2.imread(path)
        if img is None:
            continue
        H, W = img.shape[:2]
        boxes = []
        for _, cx, cy, bw, bh in load_labels(path):
            boxes.append(((cx - bw / 2) * W, (cy - bh / 2) * H,
                          (cx + bw / 2) * W, (cy + bh / 2) * H))
        for _ in range(25):
            if made >= want:
                break
            cw = rng.randint(int(W * 0.35), int(W * 0.75))
            ch = rng.randint(int(H * 0.35), int(H * 0.75))
            x = rng.randint(0, W - cw)
            y = rng.randint(0, H - ch)
            r = (x, y, x + cw, y + ch)
            # margin: any overlap at all disqualifies, a sliver of pillar in a
            # "background" image is a mislabel, not a hard negative
            if any(iou_xyxy(r, b) > 0.0 or (b[0] < r[2] and b[2] > r[0]
                   and b[1] < r[3] and b[3] > r[1]) for b in boxes):
                continue
            crop = img[y:y + ch, x:x + cw].copy()
            # half of them get a non-class coloured distractor blob so the model
            # learns "saturated colour" alone is not the trigger
            if rng.random() < 0.5:
                lo, hi = DISTRACTOR_HUES[rng.randrange(len(DISTRACTOR_HUES))]
                bh_ = rng.randint(int(ch * 0.15), int(ch * 0.5))
                bw_ = int(bh_ * rng.uniform(0.3, 0.8))
                if bw_ >= 4 and bh_ >= 4 and bw_ < cw and bh_ < ch:
                    patch = np.zeros((bh_, bw_, 3), np.uint8)
                    patch[:] = (rng.randint(lo, hi), rng.randint(140, 255),
                                rng.randint(110, 255))
                    patch = cv2.cvtColor(patch, cv2.COLOR_HSV2BGR)
                    px = rng.randint(0, cw - bw_)
                    py = rng.randint(int(ch * 0.35), ch - bh_)
                    a = np.full((bh_, bw_), 255, np.uint8)
                    paste(crop, patch, a, px, py, rng.choice([0, 0, 0.8]))
            write_pair(out_dir, f"{tag}_neg_{made:05d}", crop, [])
            made += 1
    return made


def build_cooc(files, sprites, out_dir, tag, want, rng):
    """Composite the opposite class into real single-class images.

    Placement rules exist for a reason: a pillar floating in the upper third of
    the frame is not a case the robot will ever see, and training on it teaches
    a prior that is wrong on the field.
    """
    made, skipped = 0, 0
    order = [f for f in files if load_labels(f)]
    rng.shuffle(order)
    while made < want and order:
        path = order[rng.randrange(len(order))]
        img = cv2.imread(path)
        if img is None:
            continue
        rows = load_labels(path)
        have = {c for c, *_ in rows}
        other = [c for c in (0, 1) if c not in have]
        if not other or not sprites.get(other[0]):
            skipped += 1
            if skipped > want * 5:
                break
            continue
        cls = other[0]
        spr, alp = sprites[cls][rng.randrange(len(sprites[cls]))]
        H, W = img.shape[:2]
        exist = [((cx - bw / 2) * W, (cy - bh / 2) * H,
                  (cx + bw / 2) * W, (cy + bh / 2) * H) for _, cx, cy, bw, bh in rows]

        placed = False
        for _ in range(40):
            # sample target height from the real distribution (median box is
            # ~21% of the linear dimension; span the observed range)
            th = int(H * rng.uniform(0.10, 0.45))
            tw = max(4, int(th * spr.shape[1] / spr.shape[0]))
            if tw >= W or th >= H:
                continue
            s = cv2.resize(spr, (tw, th), interpolation=cv2.INTER_AREA)
            a = cv2.resize(alp, (tw, th), interpolation=cv2.INTER_NEAREST)
            x = rng.randint(0, W - tw)
            # bottom edge in the lower half: pillars stand on the floor
            ybot = rng.randint(int(H * 0.50), H - 1)
            y = ybot - th
            if y < 0:
                continue
            cand = (x, y, x + tw, y + th)
            # Kisantal et al.: overlapping pasted objects hurts. Keep them apart.
            if any(iou_xyxy(cand, b) > 0.05 for b in exist):
                continue
            # match exposure to the host frame so the paste is not a brightness cue
            gain = float(np.clip(img.mean() / max(1.0, s.mean()), 0.75, 1.35))
            s = np.clip(s.astype(np.float32) * gain, 0, 255).astype(np.uint8)
            if paste(img, s, a, x, y, rng.choice([0, 0, 0.7, 1.2])):
                rows = rows + [(cls, (x + tw / 2) / W, (y + th / 2) / H,
                                tw / W, th / H)]
                placed = True
            break
        if not placed:
            skipped += 1
            if skipped > want * 5:
                break
            continue
        write_pair(out_dir, f"{tag}_cooc_{made:05d}", img, rows)
        made += 1
    return made


def harvest_sprites(files, rng, cap=400):
    out = {0: [], 1: []}
    for path in files:
        rows = load_labels(path)
        if not rows:
            continue
        img = cv2.imread(path)
        if img is None:
            continue
        for box in rows:
            if len(out[box[0]]) >= cap:
                continue
            r = extract_sprite(img, box, box[0])
            if r is not None:
                out[box[0]].append(r)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--splits", default="splits")
    ap.add_argument("--out", default="derived")
    ap.add_argument("--neg-frac", type=float, default=0.20,
                    help="negatives as a fraction of the final train set")
    ap.add_argument("--cooc-train", type=int, default=180)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    rng = random.Random(args.seed)

    tr = [l.strip() for l in open(os.path.join(args.splits, "train.txt")) if l.strip()]
    va = [l.strip() for l in open(os.path.join(args.splits, "val.txt")) if l.strip()]

    spr_tr = harvest_sprites(tr, rng)
    spr_va = harvest_sprites(va, rng)
    print(f"sprites  train g{len(spr_tr[0])}/r{len(spr_tr[1])}   "
          f"val g{len(spr_va[0])}/r{len(spr_va[1])}")

    # n_neg / (len(tr) + n_cooc + n_neg) = neg_frac
    base = len(tr) + args.cooc_train
    n_neg = int(round(args.neg_frac * base / (1 - args.neg_frac)))

    n1 = mine_negatives(tr, os.path.join(args.out, "neg_train"), "tr", n_neg, rng)
    n2 = build_cooc(tr, spr_tr, os.path.join(args.out, "cooc_train"), "tr",
                    args.cooc_train, rng)
    n3 = mine_negatives(va, os.path.join(args.out, "neg_val"), "va", 60, rng)
    n4 = build_cooc(va, spr_va, os.path.join(args.out, "cooc_val"), "va", 60, rng)
    print(f"generated  train: {n1} neg, {n2} cooc   |   val: {n3} neg, {n4} cooc")

    def listing(d):
        p = os.path.join(args.out, d, "images")
        return [os.path.abspath(os.path.join(p, f)) for f in sorted(os.listdir(p))] \
            if os.path.isdir(p) else []

    aug = tr + listing("neg_train") + listing("cooc_train")
    rng.shuffle(aug)
    for name, rows in (("train_aug", aug),
                       ("val_neg", listing("neg_val")),
                       ("val_cooc", listing("cooc_val"))):
        with open(os.path.join(args.splits, name + ".txt"), "w", encoding="utf-8") as fh:
            fh.write("\n".join(rows) + "\n")

    neg_pct = n1 / max(1, len(aug))
    print(f"\ntrain_aug: {len(aug)} images  ({neg_pct:.0%} negatives, "
          f"{n2} with both classes)")
    print("val.txt UNCHANGED -- headline metric stays comparable to baseline")
    print("val_cooc is synthetic-on-synthetic: a proxy, NOT evidence. "
          "Only real co-occurrence footage settles that question.")


if __name__ == "__main__":
    main()
