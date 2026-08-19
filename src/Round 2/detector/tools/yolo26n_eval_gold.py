"""Evaluate a trained model against gold_30 — same harness as the old-detector
test, so numbers are directly comparable (old: red recall 21.4%, 6 magenta swaps)."""
import sys
import numpy as np
from pathlib import Path
from ultralytics import YOLO

GOLD = Path(r"C:\Users\ANT PC\Downloads\gold_30")
WEIGHTS = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\ANT PC\wro_train\runs\y26n_320_v1\weights\best.pt"
NAMES = {0: "green", 1: "red", 2: "magenta"}

def iou(A, B):
    ix0, iy0 = max(A[0], B[0]), max(A[1], B[1])
    ix1, iy1 = min(A[2], B[2]), min(A[3], B[3])
    inter = max(0, ix1 - ix0) * max(0, iy1 - iy0)
    u = (A[2]-A[0])*(A[3]-A[1]) + (B[2]-B[0])*(B[3]-B[1]) - inter
    return inter / u if u > 0 else 0.0

model = YOLO(WEIGHTS)
golds = []
for lf in sorted((GOLD / "labels").glob("*.txt")):
    img = GOLD / "images" / (lf.stem + ".jpg")
    if not img.exists():
        continue
    import cv2
    H, W = cv2.imread(str(img)).shape[:2]
    g = []
    for ln in lf.read_text().strip().splitlines():
        c, cx, cy, bw, bh = ln.split()
        c = int(c); cx, cy, bw, bh = map(float, (cx, cy, bw, bh))
        g.append((c, (cx-bw/2)*W, (cy-bh/2)*H, (cx+bw/2)*W, (cy+bh/2)*H))
    golds.append((str(img), g))

print(f"weights: {WEIGHTS}")
print(f"gold frames {len(golds)}, boxes {sum(len(g) for _, g in golds)}\n")

for conf in (0.10, 0.25, 0.40):
    st = {c: dict(gold=0, det=0, tp=0, ious=[], swap=0) for c in NAMES.values()}
    fp = 0
    for imgp, gold in golds:
        for g in gold:
            st[NAMES[g[0]]]["gold"] += 1
        r = model.predict(imgp, imgsz=320, conf=conf, verbose=False)[0]
        used = set()
        for b in r.boxes:
            c = int(b.cls.item())
            xy = b.xyxy[0].tolist()
            st[NAMES[c]]["det"] += 1
            best, bi = 0.0, -1
            for gi, g in enumerate(gold):
                v = iou(xy, g[1:5])
                if v > best:
                    best, bi = v, gi
            if best >= 0.5 and bi not in used:
                used.add(bi)
                if gold[bi][0] == c:
                    st[NAMES[c]]["tp"] += 1
                    st[NAMES[c]]["ious"].append(best)
                else:
                    st[NAMES[c]]["swap"] += 1
            elif best < 0.3:
                fp += 1
    print(f"--- conf {conf:.2f} ---")
    print(f"{'class':9s}{'gold':>6s}{'det':>6s}{'TP':>5s}{'recall':>9s}{'prec':>8s}{'medIoU':>8s}{'swap':>6s}")
    for c in ("green", "red", "magenta"):
        s = st[c]
        rc = s["tp"]/s["gold"] if s["gold"] else 0
        pr = s["tp"]/s["det"] if s["det"] else 0
        mi = float(np.median(s["ious"])) if s["ious"] else 0
        print(f"{c:9s}{s['gold']:6d}{s['det']:6d}{s['tp']:5d}{100*rc:8.1f}%{100*pr:7.1f}%{mi:8.3f}{s['swap']:6d}")
    print(f"  FPs (IoU<0.3): {fp}\n")
