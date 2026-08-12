import cv2, numpy as np, os, glob
FR = r"C:\Users\daisy\Downloads\wro_magenta_frames_2026-08-12"
files = sorted(glob.glob(os.path.join(FR, "*.jpg")))
K = np.ones((3,3), np.uint8)
rows = []  # (tag, blob_idx, area, L, a, b) medians per blob per frame
miss = 0
for f in files:
    img = cv2.imread(f)
    if img is None: continue
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    L, a, b = lab[:,:,0], lab[:,:,1], lab[:,:,2]
    mask = ((a > 150) & (b < 125) & (L > 30)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, K)
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    tag = os.path.basename(f)[:4]
    blobs = sorted(range(1, n), key=lambda i: -stats[i, cv2.CC_STAT_AREA])[:2]
    blobs = [i for i in blobs if stats[i, cv2.CC_STAT_AREA] >= 150]
    if not blobs: miss += 1; continue
    for rank, i in enumerate(blobs):
        m = lbl == i
        rows.append((tag, rank, int(stats[i, cv2.CC_STAT_AREA]),
                     float(np.median(L[m])), float(np.median(a[m])),
                     float(np.median(b[m])),
                     stats[i, cv2.CC_STAT_WIDTH] / max(1, stats[i, cv2.CC_STAT_HEIGHT])))

import statistics as st
def rpt(name, rs):
    if not rs: print(f"{name}: no blobs"); return
    A  = [r[4] for r in rs]; B = [r[5] for r in rs]; Ls = [r[3] for r in rs]
    AR = [r[6] for r in rs]
    med = lambda v: st.median(v)
    mad = lambda v: st.median([abs(x - st.median(v)) for x in v])
    print(f"{name}: n={len(rs)}  L med {med(Ls):.1f}  "
          f"a med {med(A):.1f} (MAD {mad(A):.1f})  b med {med(B):.1f} (MAD {mad(B):.1f})  "
          f"signed a*={med(A)-128:+.1f} b*={med(B)-128:+.1f}  w/h med {med(AR):.2f}")
print(f"frames: {len(files)}, no-detection frames: {miss}")
rpt("ALL blobs         ", rows)
rpt("primary (largest) ", [r for r in rows if r[1] == 0])
rpt("secondary         ", [r for r in rows if r[1] == 1])
rpt("vidA primary      ", [r for r in rows if r[1] == 0 and r[0] == "vidA"])
rpt("vidB primary      ", [r for r in rows if r[1] == 0 and r[0] == "vidB"])
# distance to the two RED session centers from eval (signed a*,b*)
import math
for rn, (ra, rb) in [("red-stills", (29.9, 23.5)), ("red-video", (48.5, 38.7))]:
    if rows:
        pa = st.median([r[4] for r in rows if r[1] == 0]) - 128
        pb = st.median([r[5] for r in rows if r[1] == 0]) - 128
        print(f"D(a,b) magenta-primary vs {rn}: {math.hypot(pa - ra, pb - rb):.1f}")
