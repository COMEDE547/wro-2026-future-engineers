"""bench_classify.py - classify-variant shootout on THIS machine."""
import time
import cv2
import numpy as np

cv2.setNumThreads(1)
H, W = 144, 320
rng = np.random.default_rng(0)
f = np.full((H, W, 3), 200, np.uint8)
f[20:90, 60:84] = (55, 39, 238)
f[40:96, 200:226] = (44, 214, 68)
roi = cv2.convertScaleAbs(f.astype(np.int16) + rng.integers(-8, 9, f.shape, np.int16))

HUE_SHIFT = 90
RED_LO, RED_HI = (80, 120, 80), (100, 255, 255)
GRN_LO, GRN_HI = (130, 80, 60), (175, 255, 255)
MAG_LO, MAG_HI = (50, 80, 80), (75, 255, 255)
_hue = ((np.arange(256) + HUE_SHIFT) % 180).astype(np.uint8)
_idn = np.arange(256, dtype=np.uint8)
HSV_LUT = np.dstack((_hue, _idn, _idn))
K3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

g = ((np.arange(32) << 3) + 4).astype(np.uint8)
B_, G_, R_ = np.meshgrid(g, g, g, indexing="ij")
img = np.stack([B_, G_, R_], -1).reshape(1, -1, 3)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(np.int16)
h_, s_, v_ = hsv[:, 0], hsv[:, 1], hsv[:, 2]
hs = (h_ + HUE_SHIFT) % 180
def band(lo, hi):
    return (hs >= lo[0]) & (hs <= hi[0]) & (s_ >= lo[1]) & (v_ >= lo[2])
red = band(RED_LO, RED_HI) & ~band(MAG_LO, MAG_HI)
grn = band(GRN_LO, GRN_HI)
FLAT = np.zeros(32768, np.uint8); FLAT[red] = 1; FLAT[grn] = 2
LUT3D = FLAT.reshape(32, 32, 32)

hsvb = np.empty((H, W, 3), np.uint8)
rm = np.empty((H, W), np.uint8); gm = np.empty((H, W), np.uint8)
mm = np.empty((H, W), np.uint8)
q = np.empty((H, W, 3), np.uint8); i32 = np.empty((H, W), np.int32)
cls_out = np.empty(H * W, np.uint8)

def a_old():
    cv2.cvtColor(roi, cv2.COLOR_BGR2HSV, dst=hsvb)
    cv2.LUT(hsvb, HSV_LUT, dst=hsvb)
    cv2.inRange(hsvb, RED_LO, RED_HI, dst=rm)
    cv2.inRange(hsvb, MAG_LO, MAG_HI, dst=mm)
    cv2.bitwise_not(mm, dst=mm); cv2.bitwise_and(rm, mm, dst=rm)
    cv2.inRange(hsvb, GRN_LO, GRN_HI, dst=gm)
    cv2.morphologyEx(rm, cv2.MORPH_OPEN, K3, dst=rm)
    cv2.morphologyEx(gm, cv2.MORPH_OPEN, K3, dst=gm)
    return rm, gm

def b_flat():
    global i32
    np.right_shift(roi, 3, out=q)
    i32[:] = q[..., 0]; i32 <<= 5; i32 |= q[..., 1]; i32 <<= 5; i32 |= q[..., 2]
    cls = FLAT.take(i32)
    return (cls == 1).view(np.uint8), (cls == 2).view(np.uint8)

def c_split3d():
    np.right_shift(roi, 3, out=q)
    b, gg, r = cv2.split(q)
    cls = LUT3D[b, gg, r]
    return (cls == 1).view(np.uint8), (cls == 2).view(np.uint8)

def d_stride3d():
    np.right_shift(roi, 3, out=q)
    cls = LUT3D[q[..., 0], q[..., 1], q[..., 2]]
    return (cls == 1).view(np.uint8), (cls == 2).view(np.uint8)

def e_flat_out():
    global i32
    np.right_shift(roi, 3, out=q)
    b, gg, r = cv2.split(q)
    np.multiply(b, 1024, out=i32, dtype=np.int32)
    i32 += gg.astype(np.int32) * 32; i32 += r
    FLAT.take(i32.ravel(), out=cls_out, mode="clip")
    cls = cls_out.reshape(H, W)
    return (cls == 1).view(np.uint8), (cls == 2).view(np.uint8)

def cc(m):
    return cv2.connectedComponentsWithStats(m, 4, cv2.CV_32S)

def bench(fn, n=3000):
    fn()
    t = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t) / n * 1e3

ra, ga = a_old(); rb, gb = b_flat(); rc, gc = c_split3d()
assert (rb == rc).all() and (gb == gc).all(), "variant mismatch"
for name, fn in [("A old cv2+morph", a_old), ("B flat-take     ", b_flat),
                 ("C split+3Dindex ", c_split3d), ("D stride+3Dindex", d_stride3d),
                 ("E split+take-out", e_flat_out)]:
    print(f"{name}: {bench(fn):7.4f} ms classify")
r0, g0 = a_old()
t = time.perf_counter()
for _ in range(3000):
    cc(r0); cc(g0)
print(f"2x CCL shared   : {(time.perf_counter()-t)/3000*1e3:7.4f} ms")
