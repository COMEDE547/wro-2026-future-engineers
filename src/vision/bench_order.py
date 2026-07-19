"""bench_order.py - falsify the 'cache ordering' claim + test content dependence."""
import sys
import time

sys.path.insert(0, r"C:\Users\ANT PC\wro_vision")
import cv2
import numpy as np
from pillar_ab import OldPipe, NewPipe, Demo, ROI_TOP_FRAC, PROC_H, PROC_W

cv2.setNumThreads(1)
demo = Demo()
clean = [demo.read()[0] for _ in range(64)]
rng = np.random.default_rng(1)
noisy = [rng.integers(0, 256, (PROC_H, PROC_W, 3), np.uint8) for _ in range(64)]
y0 = int(PROC_H * ROI_TOP_FRAC)
old = OldPipe(clean[0][y0:].shape)
new = NewPipe(clean[0][y0:].shape)


def run(frames, first, second, n=2000):
    t1 = t2 = 0.0
    for i in range(64):
        first(frames[i % 64][y0:]); second(frames[i % 64][y0:])
    for i in range(n):
        f = frames[i % 64][y0:]
        a = time.perf_counter(); first(f)
        b = time.perf_counter(); second(f)
        c = time.perf_counter()
        t1 += b - a; t2 += c - b
    return t1 / n * 1e3, t2 / n * 1e3


oa, nb = run(clean, old.detect, new.detect)
na, ob = run(clean, new.detect, old.detect)
print("CLEAN frames (2 pillars on grey):")
print(f"  order OLD,NEW:  OLD {oa:.3f}  NEW {nb:.3f} ms")
print(f"  order NEW,OLD:  NEW {na:.3f}  OLD {ob:.3f} ms")
print(f"  second-position advantage: NEW {nb-na:+.3f} ms  OLD {ob-oa:+.3f} ms")
oa2, nb2 = run(noisy, old.detect, new.detect)
na2, ob2 = run(noisy, new.detect, old.detect)
print("NOISY frames (full random color, worst-case content):")
print(f"  order OLD,NEW:  OLD {oa2:.3f}  NEW {nb2:.3f} ms")
print(f"  order NEW,OLD:  NEW {na2:.3f}  OLD {ob2:.3f} ms")
print(f"  content cost:   OLD {oa2-oa:+.3f} ms  NEW {na2-na:+.3f} ms vs clean")
