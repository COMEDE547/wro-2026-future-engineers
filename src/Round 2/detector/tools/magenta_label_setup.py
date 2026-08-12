import re, pathlib, glob, os
# --- Phase A: patch labelImg 1.8.6 float->int crashes (Py3.10+ / PyQt5 5.15) ---
import libs.canvas as C
cp = pathlib.Path(C.__file__)
s = cp.read_text(encoding='utf-8'); orig = s
def cast4(m):
    args = [a.strip() for a in m.group(2).split(',')]
    if len(args) != 4 or args[0].startswith('int('): return m.group(0)
    return m.group(1) + '(' + ', '.join('int(%s)' % a for a in args) + ')'
s = re.sub(r'(p\.draw(?:Line|Rect))\(([^()]*)\)', cast4, s)
if s != orig: cp.write_text(s, encoding='utf-8'); print("canvas.py patched:", cp)
else: print("canvas.py unchanged (no 4-scalar draw calls found)")
import labelImg as LI
lp = pathlib.Path(LI.__file__).parent / 'labelImg.py'
t = lp.read_text(encoding='utf-8'); o2 = t
t = re.sub(r'(?m)^(\s*)bar\.setValue\((?!int\()(.+)\)\s*$',
           r'\1bar.setValue(int(\2))', t)
t = re.sub(r'(?m)^(\s*)(self\.)?zoom_widget\.setValue\((?!int\()(.+)\)\s*$',
           r'\1\2zoom_widget.setValue(int(\3))', t)
if t != o2: lp.write_text(t, encoding='utf-8'); print("labelImg.py patched:", lp)
else: print("labelImg.py unchanged")

# --- Phase B: auto-generate YOLO labels (class 2 = magenta) for review ---
import cv2, numpy as np
FR = r"C:\Users\daisy\Downloads\wro_magenta_frames_2026-08-12"
K = np.ones((3,3), np.uint8); PAD = 0.08
n_lbl = n_box = 0
for f in sorted(glob.glob(os.path.join(FR, "*.jpg"))):
    img = cv2.imread(f); H, W = img.shape[:2]
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    m = ((lab[:,:,1] > 150) & (lab[:,:,2] < 125) & (lab[:,:,0] > 30)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, K)
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    lines = []
    for i in sorted(range(1, n), key=lambda i: -stats[i, cv2.CC_STAT_AREA])[:2]:
        if stats[i, cv2.CC_STAT_AREA] < 150: break
        x, y, w, h = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], \
                     stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        px, py = w * PAD, h * PAD
        x0, y0 = max(0, x - px), max(0, y - py)
        x1, y1 = min(W, x + w + px), min(H, y + h + py)
        cx, cy = (x0 + x1) / 2 / W, (y0 + y1) / 2 / H
        bw, bh = (x1 - x0) / W, (y1 - y0) / H
        lines.append(f"2 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        n_box += 1
    open(f[:-4] + ".txt", "w").write("\n".join(lines) + ("\n" if lines else ""))
    n_lbl += 1
open(os.path.join(FR, "classes.txt"), "w").write("green\nred\nmagenta\n")
print(f"auto-labels: {n_lbl} files, {n_box} boxes, classes.txt written (0=green 1=red 2=MAGENTA)")
