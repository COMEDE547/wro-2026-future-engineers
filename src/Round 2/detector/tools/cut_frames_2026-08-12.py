import cv2, os
vids = [
    (r"C:\Users\daisy\Downloads\WhatsApp Video 2026-08-12 at 4.38.27 PM.mp4", "vidA"),
    (r"C:\Users\daisy\Downloads\WhatsApp Video 2026-08-12 at 4.38.49 PM.mp4", "vidB"),
]
out = r"C:\Users\daisy\Downloads\wro_magenta_frames_2026-08-12"
os.makedirs(out, exist_ok=True)
STEP = 20
total = 0
for path, tag in vids:
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print("FAIL open:", path); continue
    fps = cap.get(cv2.CAP_PROP_FPS)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    i = wrote = 0
    while True:
        ok, frame = cap.read()
        if not ok: break
        if i % STEP == 0:
            fn = os.path.join(out, f"{tag}_f{i:05d}.jpg")
            cv2.imwrite(fn, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            wrote += 1
        i += 1
    cap.release()
    print(f"{tag}: {w}x{h} @ {fps:.2f} fps, {i} frames read (header {n}), wrote {wrote} (every {STEP}th)")
    total += wrote
print("TOTAL:", total, "->", out)
