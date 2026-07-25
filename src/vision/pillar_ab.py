"""
pillar_ab.py — OLD (v3 HSV+morphology) vs NEW (v4 fused-LUT) side by side.
ONE capture source, BOTH pipelines run on the SAME frame every iteration,
annotated views shown left (OLD) / right (NEW) with live per-pipeline timings.

Usage:
  python pillar_ab.py              # webcam 0 (falls back to demo if absent)
  python pillar_ab.py video.mp4    # video file
  python pillar_ab.py --demo       # synthetic moving pillars (no camera needed)
Keys: q quit
"""

import sys
import time

import cv2
import numpy as np

cv2.setNumThreads(1)

# ---------------- shared config ----------------
PROC_W, PROC_H = 320, 240
ROI_TOP_FRAC   = 0.40
MIN_AREA       = 80
AR_MIN, AR_MAX = 1.1, 3.5
EXTENT_MIN     = 0.5

HUE_SHIFT = 90
RED_H_LO,  RED_H_HI = 170, 10
GRN_H_LO,  GRN_H_HI = 40,  85
MAG_H_LO,  MAG_H_HI = 140, 165
RED_SV, GRN_SV, MAG_SV = (120, 80), (80, 60), (80, 80)

def _sh(h):
    return (h + HUE_SHIFT) % 180

RED_LO, RED_HI = (_sh(RED_H_LO), *RED_SV), (_sh(RED_H_HI), 255, 255)
GRN_LO, GRN_HI = (_sh(GRN_H_LO), *GRN_SV), (_sh(GRN_H_HI), 255, 255)
MAG_LO, MAG_HI = (_sh(MAG_H_LO), *MAG_SV), (_sh(MAG_H_HI), 255, 255)

BOX_BGR = ((60, 60, 255), (60, 255, 60))


def blobs(mask_u8, color_id):
    """Shared box extraction: ccWithStats + vectorized gates."""
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask_u8, 4, cv2.CV_32S)
    if n <= 1:
        return None
    s = stats[1:]
    x, y, w, h, a = s[:, 0], s[:, 1], s[:, 2], s[:, 3], s[:, 4]
    keep = ((a >= MIN_AREA)
            & (h > w * AR_MIN) & (h < w * AR_MAX)
            & (a >= EXTENT_MIN * w * h))
    if not keep.any():
        return None
    k = np.flatnonzero(keep)
    out = np.empty((k.size, 6), np.int32)
    out[:, 0], out[:, 1], out[:, 2], out[:, 3] = x[k], y[k], w[k], h[k]
    out[:, 4] = y[k] + h[k]
    out[:, 5] = color_id
    return out


def gather(parts):
    parts = [p for p in parts if p is not None]
    if not parts:
        return None, None
    allb = parts[0] if len(parts) == 1 else np.vstack(parts)
    return allb, allb[allb[:, 4].argmax()]


# ------------- OLD: v3 pipeline (HSV + per-channel LUT + morphology) -------------
_hue = ((np.arange(256) + HUE_SHIFT) % 180).astype(np.uint8)
_idn = np.arange(256, dtype=np.uint8)
HSV_LUT = np.dstack((_hue, _idn, _idn))
K3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


class OldPipe:
    NAME = "OLD  hsv+inRange+morph"

    def __init__(self, roi_shape):
        h, w = roi_shape[:2]
        self.hsv = np.empty((h, w, 3), np.uint8)
        self.red = np.empty((h, w), np.uint8)
        self.grn = np.empty((h, w), np.uint8)
        self.mag = np.empty((h, w), np.uint8)

    def detect(self, roi):
        cv2.cvtColor(roi, cv2.COLOR_BGR2HSV, dst=self.hsv)
        cv2.LUT(self.hsv, HSV_LUT, dst=self.hsv)
        cv2.inRange(self.hsv, RED_LO, RED_HI, dst=self.red)
        cv2.inRange(self.hsv, MAG_LO, MAG_HI, dst=self.mag)
        cv2.bitwise_not(self.mag, dst=self.mag)
        cv2.bitwise_and(self.red, self.mag, dst=self.red)
        cv2.inRange(self.hsv, GRN_LO, GRN_HI, dst=self.grn)
        cv2.morphologyEx(self.red, cv2.MORPH_OPEN, K3, dst=self.red)
        cv2.morphologyEx(self.grn, cv2.MORPH_OPEN, K3, dst=self.grn)
        return gather([blobs(self.red, 0), blobs(self.grn, 1)])


# ------------- NEW: v4 fused 32^3 LUT, no cvtColor, no morphology -------------
def build_lut_flat():
    """32x32x32 BGR->class {0 bg, 1 red, 2 green}; magenta baked in as 0."""
    g = ((np.arange(32) << 3) + 4).astype(np.uint8)          # cell centers
    B, G, R = np.meshgrid(g, g, g, indexing="ij")
    img = np.stack([B, G, R], -1).reshape(1, -1, 3)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(np.int16)
    h, s, v = hsv[:, 0], hsv[:, 1], hsv[:, 2]
    hs = (h + HUE_SHIFT) % 180

    def band(lo, hi, sv):
        return (hs >= lo[0]) & (hs <= hi[0]) & (s >= sv[0]) & (v >= sv[1])

    red = band(RED_LO, RED_HI, RED_SV) & ~band(MAG_LO, MAG_HI, MAG_SV)
    grn = band(GRN_LO, GRN_HI, GRN_SV)
    cls = np.zeros(32 * 32 * 32, np.uint8)
    cls[red] = 1
    cls[grn] = 2
    return cls                                               # index b<<10|g<<5|r


LUT_FLAT = build_lut_flat()
LUT3D = LUT_FLAT.reshape(32, 32, 32)


class NewPipe:
    NAME = "NEW  fused 32^3 LUT"

    def __init__(self, roi_shape):
        h, w = roi_shape[:2]
        self.q   = np.empty((h, w, 3), np.uint8)             # roi >> 3
        self.i32 = np.empty((h, w), np.int32)                # flat LUT index

    def detect(self, roi):
        np.right_shift(roi, 3, out=self.q)                   # one pass, all 3 ch
        q = self.q                                           # 3D fancy index:
        cls = LUT3D[q[..., 0], q[..., 1], q[..., 2]]         # fastest variant (D)
        red = (cls == 1).view(np.uint8)                      # zero-copy bool->u8
        grn = (cls == 2).view(np.uint8)
        return gather([blobs(red, 0), blobs(grn, 1)])


# ---------------- capture / demo ----------------
class Demo:
    """Synthetic moving pillars in rulebook colors, mild sensor noise."""
    def __init__(self):
        self.t = 0.0
        self.rng = np.random.default_rng(0)

    def read(self):
        self.t += 0.03
        f = np.full((PROC_H, PROC_W, 3), 200, np.uint8)
        rw = int(20 + 5 * np.sin(self.t * 1.7))
        rx = int(70 + 50 * np.sin(self.t))
        gx = int(210 + 45 * np.sin(self.t * 0.6 + 2))
        f[128:128 + 2 * rw, rx:rx + rw]      = (55, 39, 238)   # red pillar
        f[150:150 + 56,     gx:gx + 26]      = (44, 214, 68)   # green pillar
        noise = self.rng.integers(-8, 9, f.shape, np.int16)
        return cv2.convertScaleAbs(f.astype(np.int16) + noise), True


def open_source():
    if "--demo" in sys.argv:
        return Demo(), "demo"
    src = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 0
    cap = cv2.VideoCapture(int(src) if isinstance(src, str) and src.isdigit() else src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ok, _ = cap.read()
    if not ok:
        print("no camera -> demo mode")
        cap.release()
        return Demo(), "demo"
    return cap, "cv"


def read_frame(src, kind):
    if kind == "demo":
        f, ok = src.read()
        return f if ok else None
    ok, f = src.read()
    if not ok:
        return None
    return cv2.resize(f, (PROC_W, PROC_H), interpolation=cv2.INTER_AREA)


# ---------------- render ----------------
PANE = 2                                                     # x2 upscale per pane
HDR = 46


def pane(frame, allb, nearest, y0, name, ms):
    big = cv2.resize(frame, (PROC_W * PANE, PROC_H * PANE),
                     interpolation=cv2.INTER_NEAREST)
    if allb is not None:
        for x, y, w, h, _, cid in allb:
            cv2.rectangle(big, (x * PANE, (y + y0) * PANE),
                          ((x + w) * PANE, (y + y0 + h) * PANE), BOX_BGR[cid], 2)
    if nearest is not None:
        x, y, w, h, _, cid = nearest
        cv2.rectangle(big, (x * PANE, (y + y0) * PANE),
                      ((x + w) * PANE, (y + y0 + h) * PANE), (0, 255, 255), 2)
    cv2.putText(big, f"{name}  {ms:5.2f} ms  (~{1000.0/max(ms,1e-3):4.0f} fps ceiling)",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return big


POLL = cv2.pollKey if hasattr(cv2, "pollKey") else (lambda: cv2.waitKey(1))


def main():
    src, kind = open_source()
    old = new = None
    ema_o = ema_n = 1e-3
    fps, fwin, twin = 0.0, 0, time.perf_counter()

    while True:
        frame = read_frame(src, kind)
        if frame is None:
            break
        y0 = int(frame.shape[0] * ROI_TOP_FRAC)
        roi = frame[y0:]
        if old is None:
            old, new = OldPipe(roi.shape), NewPipe(roi.shape)

        t0 = time.perf_counter()
        ob, on = old.detect(roi)
        t1 = time.perf_counter()
        nb, nn = new.detect(roi)
        t2 = time.perf_counter()
        ema_o = 0.9 * ema_o + 0.1 * (t1 - t0)
        ema_n = 0.9 * ema_n + 0.1 * (t2 - t1)

        left  = pane(frame, ob, on, y0, OldPipe.NAME, ema_o * 1e3)
        right = pane(frame, nb, nn, y0, NewPipe.NAME, ema_n * 1e3)
        body = cv2.hconcat([left, right])
        hdr = np.full((HDR, body.shape[1], 3), 28, np.uint8)
        cv2.putText(hdr, f"same frames | NEW is {ema_o/max(ema_n,1e-9):4.1f}x faster"
                    f" | loop {fps:5.1f} fps | q quits",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("pillar A/B - OLD (left) vs NEW (right)", cv2.vconcat([hdr, body]))
        if POLL() & 0xFF == ord("q"):
            break

        fwin += 1
        now = time.perf_counter()
        if now - twin >= 0.5:
            fps, fwin, twin = fwin / (now - twin), 0, now

    if kind == "cv":
        src.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
