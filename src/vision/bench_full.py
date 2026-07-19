"""bench_full.py - digital benchmark: full pipeline FPS ceiling, NO camera.
Rotates through 64 pre-generated demo frames so caches see varied data."""
import sys
import time

sys.path.insert(0, r"C:\Users\ANT PC\wro_vision")
import cv2
import numpy as np
from pillar_ab import (OldPipe, NewPipe, Demo, pane,
                       ROI_TOP_FRAC, PROC_H, PROC_W)

cv2.setNumThreads(1)
demo = Demo()
frames = [demo.read()[0] for _ in range(64)]
y0 = int(PROC_H * ROI_TOP_FRAC)
old = OldPipe(frames[0][y0:].shape)
new = NewPipe(frames[0][y0:].shape)


def bench(fn, n=2000):
    for i in range(64):
        fn(frames[i % 64])
    t = time.perf_counter()
    for i in range(n):
        fn(frames[i % 64])
    dt = (time.perf_counter() - t) / n
    return dt * 1e3, 1.0 / dt


ob, on = old.detect(frames[0][y0:])


def ab_frame(f):
    """Everything the A/B harness does per frame except imshow/pollKey."""
    o = old.detect(f[y0:])
    nn = new.detect(f[y0:])
    left = pane(f, o[0], o[1], y0, "OLD", 1.0)
    right = pane(f, nn[0], nn[1], y0, "NEW", 1.0)
    body = cv2.hconcat([left, right])
    return body


rows = [
    ("OLD full detect          ", lambda f: old.detect(f[y0:])),
    ("NEW full detect          ", lambda f: new.detect(f[y0:])),
    ("render one pane          ", lambda f: pane(f, ob, on, y0, "x", 1.0)),
    ("A/B frame (2x detect+draw)", ab_frame),
]
print("DIGITAL BENCHMARK - no camera, 2000 iters, 64 rotating frames, 320x240")
print("-" * 66)
for name, fn in rows:
    ms, fps = bench(fn)
    print(f"{name}: {ms:7.3f} ms/frame  ->  {fps:8.0f} fps ceiling")
print("-" * 66)
print("camera-free verdict: compute ceiling vs the 30 fps webcam cap above")
