import cv2, numpy as np, glob, os, shutil, random, zipfile
random.seed(42); np.random.seed(42)
FR  = r"C:\Users\daisy\Downloads\wro_magenta_frames_2026-08-12"
PKG = r"C:\Users\daisy\Downloads\magenta_yolo_pkg"
ZIP = r"C:\Users\daisy\Downloads\magenta_yolo_2026-08-12.zip"
NV  = 4  # augmented variants per TRAIN image only
if os.path.exists(PKG): shutil.rmtree(PKG)
for s in ("train", "val"):
    for k in ("images", "labels"): os.makedirs(os.path.join(PKG, "magenta", s, k))
def boxes_of(p):
    return [l.split() for l in open(p).read().split("\n") if l.strip()] if os.path.exists(p) else []
def aug_once(img, boxes):
    out = img.copy(); bx = [b[:] for b in boxes]
    if random.random() < 0.5:
        out = cv2.flip(out, 1)
        for b in bx: b[1] = f"{1.0 - float(b[1]):.6f}"
    hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV).astype(np.float32)   # NO hue jitter
    hsv[:,:,1] *= random.uniform(0.70, 1.30); hsv[:,:,2] *= random.uniform(0.75, 1.25)
    out = cv2.cvtColor(np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR)
    g = random.uniform(0.55, 1.60)
    lut = np.clip(((np.arange(256)/255.0)**(1.0/g))*255, 0, 255).astype(np.uint8)
    out = cv2.LUT(out, lut)
    out = cv2.convertScaleAbs(out, alpha=random.uniform(0.85,1.15), beta=random.uniform(-15,15))
    if random.random() < 0.30:
        out = cv2.GaussianBlur(out, (random.choice([3,5]),)*2, 0)
    if random.random() < 0.40:
        H, W = out.shape[:2]
        pts = np.array([[random.randint(0,W), random.randint(0,H)] for _ in range(4)])
        ov = out.copy(); cv2.fillPoly(ov, [pts], (0,0,0))
        out = cv2.addWeighted(ov, 0.35, out, 0.65, 0)
    return out, bx

def wb(p, bx): open(p, "w").write("\n".join(" ".join(b) for b in bx) + ("\n" if bx else ""))
counts = {"train": 0, "val": 0, "aug": 0, "neg": 0}
for f in sorted(glob.glob(os.path.join(FR, "*.jpg"))):
    stem = os.path.splitext(os.path.basename(f))[0]
    split = "train" if stem.startswith("vidA") else "val"   # whole-video split, no leakage
    bx = boxes_of(os.path.join(FR, stem + ".txt"))
    if not bx: counts["neg"] += 1
    shutil.copy2(f, os.path.join(PKG, "magenta", split, "images", stem + ".jpg"))
    wb(os.path.join(PKG, "magenta", split, "labels", stem + ".txt"), bx)
    counts[split] += 1
    if split == "train" and bx:
        img = cv2.imread(f)
        for k in range(NV):
            a_img, a_bx = aug_once(img, bx)
            cv2.imwrite(os.path.join(PKG, "magenta", "train", "images", f"{stem}_aug{k}.jpg"),
                        a_img, [cv2.IMWRITE_JPEG_QUALITY, 95])
            wb(os.path.join(PKG, "magenta", "train", "labels", f"{stem}_aug{k}.txt"), a_bx)
            counts["aug"] += 1
if os.path.exists(ZIP): os.remove(ZIP)
with zipfile.ZipFile(ZIP, "w", zipfile.ZIP_DEFLATED) as z:
    for root, _, files in os.walk(PKG):
        for fn in files:
            p = os.path.join(root, fn)
            z.write(p, os.path.relpath(p, PKG))
print(f"train originals {counts['train']} + aug {counts['aug']} | val {counts['val']} "
      f"(val NEVER augmented) | empty-label frames kept as background: {counts['neg']}")
print("ZIP:", ZIP, f"({os.path.getsize(ZIP)/1e6:.1f} MB) -> upload this to Google Drive")
