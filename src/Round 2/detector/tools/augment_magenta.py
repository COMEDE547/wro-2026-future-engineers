import cv2, numpy as np, glob, os, shutil, random
random.seed(42); np.random.seed(42)
FR  = r"C:\Users\daisy\Downloads\wro_magenta_frames_2026-08-12"
OUT = r"C:\Users\daisy\Downloads\wro_magenta_aug_2026-08-12"
NV  = 5   # augmented variants per original
os.makedirs(OUT, exist_ok=True)
def read_boxes(p):
    if not os.path.exists(p): return []
    return [l.split() for l in open(p).read().split("\n") if l.strip()]
def write_boxes(p, boxes):
    open(p, "w").write("\n".join(" ".join(b) for b in boxes) + ("\n" if boxes else ""))
def aug_once(img, boxes):
    out = img.copy(); bx = [b[:] for b in boxes]
    if random.random() < 0.5:                       # horizontal flip
        out = cv2.flip(out, 1)
        for b in bx: b[1] = f"{1.0 - float(b[1]):.6f}"
    hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV).astype(np.float32)  # NO hue jitter
    hsv[:,:,1] *= random.uniform(0.70, 1.30)
    hsv[:,:,2] *= random.uniform(0.75, 1.25)
    out = cv2.cvtColor(np.clip(hsv,0,255).astype(np.uint8), cv2.COLOR_HSV2BGR)
    g = random.uniform(0.55, 1.60)                  # gamma
    lut = np.clip(((np.arange(256)/255.0)**(1.0/g))*255, 0, 255).astype(np.uint8)
    out = cv2.LUT(out, lut)
    a, b2 = random.uniform(0.85,1.15), random.uniform(-15,15)   # contrast/brightness
    out = cv2.convertScaleAbs(out, alpha=a, beta=b2)
    if random.random() < 0.30:                      # mild blur
        out = cv2.GaussianBlur(out, (random.choice([3,5]),)*2, 0)
    if random.random() < 0.40:                      # soft cast shadow
        H, W = out.shape[:2]
        pts = np.array([[random.randint(0,W), random.randint(0,H)] for _ in range(4)])
        ov = out.copy(); cv2.fillPoly(ov, [pts], (0,0,0))
        out = cv2.addWeighted(ov, 0.35, out, 0.65, 0)
    return out, bx
n_img = n_aug = 0
for f in sorted(glob.glob(os.path.join(FR, "*.jpg"))):
    stem = os.path.splitext(os.path.basename(f))[0]
    boxes = read_boxes(os.path.join(FR, stem + ".txt"))
    if not boxes:
        print("SKIP (no boxes after review):", stem); continue
    shutil.copy2(f, os.path.join(OUT, stem + ".jpg"))
    write_boxes(os.path.join(OUT, stem + ".txt"), boxes)
    img = cv2.imread(f); n_img += 1
    for k in range(NV):
        a_img, a_bx = aug_once(img, boxes)
        cv2.imwrite(os.path.join(OUT, f"{stem}_aug{k}.jpg"), a_img,
                    [cv2.IMWRITE_JPEG_QUALITY, 95])
        write_boxes(os.path.join(OUT, f"{stem}_aug{k}.txt"), a_bx)
        n_aug += 1
shutil.copy2(os.path.join(FR, "classes.txt"), os.path.join(OUT, "classes.txt"))
print(f"originals kept: {n_img}, augmented written: {n_aug}, total {n_img+n_aug} -> {OUT}")
print("transforms: hflip(boxes mirrored), S/V jitter, gamma, contrast/brightness,")
print("blur p=0.3, shadow p=0.4 -- HUE UNTOUCHED (hue is the class label)")
