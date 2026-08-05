"""make_split.py — build a LEAKAGE-FREE train/val split for the pillar dataset.

Why this exists: the split shipped inside `dataset/` is not usable as a
measurement. Audited 2026-07-25:

  * 25 image stems appear in BOTH images/train and images/val (exact dupes)
  * the 181 files named 20260701_HHMMSS_uuuuuu are frames from ONE video
    session, several per second; 27 of val's 29 one-second buckets also
    appear in train. Adjacent video frames are near-identical, so val was
    measuring memorisation.

Any mAP computed on the shipped split is inflated and says nothing about the
venue.

This regroups by SOURCE, then splits whole groups:
  * video frames    -> contiguous runs, cut wherever the gap exceeds --gap sec
  * numbered stills -> (prefix, index // --still-group)

Output: two manifests of absolute image paths. Nothing is copied or moved.

    python make_split.py --root "C:/Users/ANT PC/Downloads/drive-download-20260725T171902Z-1-001/dataset"

If red_####/green_#### are also video-derived (consecutive frames of the same
pillar), raise --still-group until val stops looking easy.
"""

import argparse
import os
import random
import re
from collections import Counter, defaultdict

IMG_EXT = (".jpg", ".jpeg", ".png")
VID = re.compile(r"^(\d{8})_(\d{2})(\d{2})(\d{2})_(\d+)$")
STILL = re.compile(r"^([A-Za-z]+)_(\d+)$")


def collect(root):
    """stem -> (image_path, label_path), de-duplicated across the old splits."""
    found = {}
    for split in ("train", "val"):
        idir = os.path.join(root, "images", split)
        ldir = os.path.join(root, "labels", split)
        if not os.path.isdir(idir):
            continue
        for f in sorted(os.listdir(idir)):
            if not f.lower().endswith(IMG_EXT):
                continue
            stem = os.path.splitext(f)[0]
            if stem in found:
                continue  # first occurrence wins -- this is the 25-dupe case
            lab = os.path.join(ldir, stem + ".txt")
            found[stem] = (os.path.join(idir, f), lab if os.path.exists(lab) else None)
    return found


def build_video_runs(stems, gap):
    """Assign each video stem a run id; a new run starts after a gap > `gap` s."""
    parsed = []
    for s in stems:
        m = VID.match(s)
        if m:
            d, hh, mm, ss, frac = m.groups()
            t = int(hh) * 3600 + int(mm) * 60 + int(ss) + float("0." + frac)
            parsed.append((d, t, s))
    parsed.sort()
    runs, rid, prev = {}, 0, None
    for d, t, s in parsed:
        if prev is None or prev[0] != d or (t - prev[1]) > gap:
            rid += 1
        runs[s] = rid
        prev = (d, t)
    return runs


def group_key(stem, still_group, runs):
    if stem in runs:
        return ("vid", runs[stem])
    m = STILL.match(stem)
    if m:
        return ("still", m.group(1), int(m.group(2)) // still_group)
    return ("solo", stem)


def read_classes(lab):
    if not lab:
        return []
    return [int(l.split()[0]) for l in open(lab) if l.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", default="splits")
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--gap", type=float, default=2.0,
                    help="seconds; larger gap starts a new video run")
    ap.add_argument("--still-group", type=int, default=5,
                    help="consecutive numbered stills per group")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    items = collect(args.root)
    runs = build_video_runs(list(items), args.gap)

    groups = defaultdict(list)
    for stem, (img, lab) in items.items():
        groups[group_key(stem, args.still_group, runs)].append((stem, img, lab))

    keys = sorted(groups)
    random.Random(args.seed).shuffle(keys)

    n_val_target = args.val_frac * len(items)
    val_keys, n = set(), 0
    for k in keys:
        if n >= n_val_target:
            break
        val_keys.add(k)
        n += len(groups[k])

    os.makedirs(args.out, exist_ok=True)
    stats = {}
    for name, sel in (("train", [k for k in keys if k not in val_keys]),
                      ("val", [k for k in keys if k in val_keys])):
        rows, cls, empty = [], Counter(), 0
        for k in sel:
            for stem, img, lab in sorted(groups[k]):
                rows.append(img)
                cs = read_classes(lab)
                if not cs:
                    empty += 1
                cls.update(cs)
        with open(os.path.join(args.out, name + ".txt"), "w", encoding="utf-8") as fh:
            fh.write("\n".join(rows) + "\n")
        stats[name] = (len(rows), len(sel), dict(sorted(cls.items())), empty)

    print(f"unique images after de-dup: {len(items)}  (shipped total was 622)")
    print(f"source groups: {len(groups)}  "
          f"[video runs: {len(set(runs.values()))}, gap>{args.gap}s]")
    for name, (n_img, n_grp, cls, empty) in stats.items():
        print(f"  {name:5s} {n_img:4d} imgs / {n_grp:3d} groups  classes={cls}  "
              f"negatives={empty}")
    print(f"\nmanifests -> {os.path.abspath(args.out)}")
    print("\nclass ids come from data.yaml: 0=green, 1=red  <-- verify before training")


if __name__ == "__main__":
    main()
