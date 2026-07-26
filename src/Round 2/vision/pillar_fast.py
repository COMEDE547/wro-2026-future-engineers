"""
WRO FE 2026 — pillar detection v4.1: dual classifier w/ startup auto-select.
Lesson from the 7700X A/B: the winning classifier is MICROARCHITECTURE-
DEPENDENT (Zen4 AVX-512 -> cv2 path wins; container -> LUT won). So v4.1
ships BOTH, benchmarks them for ~0.1 s at startup on THIS machine, and runs
the faster one. The Pi's Cortex-A76 picks its own winner on first run.

Usage:
  python pillar_fast.py                  # webcam test (laptop; view on)
  python pillar_fast.py video.mp4        # video file test
  python pillar_fast.py --pi             # Pi 5: IMX708 120fps mode, headless
Flags:
  --show [N]  live view every Nth frame (laptop default 1; --pi default off)
  --bench     1 Hz console timings
  --classifier {auto,cv2,lut}   force a classifier (default auto)
Keys: q quits the view window.

Pi one-time setup:
  /boot/firmware/cmdline.txt: append  isolcpus=3
  run:  taskset -c 3 chrt -f 80 python pillar_fast.py --pi --bench
  cooling: Active Cooler. Camera Module 2? sensor -> (640,480), frame 9709 us.

v4.1 vs v3:
  [S1] IMX708 1536x864@120 sensor mode, FrameDurationLimits (8333,8333)
  [S1] capture_request + MappedArray zero-copy; inline classify, no Grabber
  [S2] classifier auto-select: cv2 path (cvtColor+inRange+morph) vs fused
       32^3 LUT (one gather, no morph) — measured at startup, faster wins
  kept: per-class ccWithStats + vectorized gates, nearest = lowest bottom
        edge, --show HUD with pollKey, magenta exclusion in both paths
"""

import argparse
import sys
import time

import cv2
import numpy as np

cv2.setNumThreads(1)

# ---------------- config ----------------
PROC_W, PROC_H = 320, 240
ROI_TOP_FRAC   = 0.40
MIN_AREA       = 80
AR_MIN, AR_MAX = 1.1, 3.5
EXTENT_MIN     = 0.5
VIEW_SCALE     = 3
SENSOR_MODE    = {"output_size": (1536, 864), "bit_depth": 10}   # IMX708 120 fps
FRAME_US       = 8333

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

DECISION = ("RED -> PASS RIGHT", "GREEN -> PASS LEFT")
BOX_BGR  = ((60, 60, 255), (60, 255, 60))
POLL = cv2.pollKey if hasattr(cv2, "pollKey") else (lambda: cv2.waitKey(1))

_hue = ((np.arange(256) + HUE_SHIFT) % 180).astype(np.uint8)
_idn = np.arange(256, dtype=np.uint8)
HSV_LUT = np.dstack((_hue, _idn, _idn))
K3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


def build_lut3d():
    """32^3 BGR->class {0 bg, 1 red, 2 green}; magenta baked in as 0."""
    g = ((np.arange(32) << 3) + 4).astype(np.uint8)
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
    return cls.reshape(32, 32, 32)


LUT3D = build_lut3d()


class Bufs:
    def __init__(self, roi_shape):
        h, w = roi_shape[:2]
        self.hsv = np.empty((h, w, 3), np.uint8)
        self.red = np.empty((h, w), np.uint8)
        self.grn = np.empty((h, w), np.uint8)
        self.mag = np.empty((h, w), np.uint8)
        self.q   = np.empty((h, w, 3), np.uint8)


def classify_cv2(roi, B):
    """cv2 path: cvtColor + hue-shift LUT + inRange + open (wins on Zen4)."""
    cv2.cvtColor(roi, cv2.COLOR_BGR2HSV, dst=B.hsv)
    cv2.LUT(B.hsv, HSV_LUT, dst=B.hsv)
    cv2.inRange(B.hsv, RED_LO, RED_HI, dst=B.red)
    cv2.inRange(B.hsv, MAG_LO, MAG_HI, dst=B.mag)
    cv2.bitwise_not(B.mag, dst=B.mag)
    cv2.bitwise_and(B.red, B.mag, dst=B.red)
    cv2.inRange(B.hsv, GRN_LO, GRN_HI, dst=B.grn)
    cv2.morphologyEx(B.red, cv2.MORPH_OPEN, K3, dst=B.red)
    cv2.morphologyEx(B.grn, cv2.MORPH_OPEN, K3, dst=B.grn)
    return B.red, B.grn


def classify_lut(roi, B):
    """LUT path: one 3D gather, no cvtColor, no morphology."""
    np.right_shift(roi, 3, out=B.q)
    q = B.q
    cls = LUT3D[q[..., 0], q[..., 1], q[..., 2]]
    return (cls == 1).view(np.uint8), (cls == 2).view(np.uint8)


CLASSIFIERS = {"cv2": classify_cv2, "lut": classify_lut}


def pick_classifier(roi_shape, forced="auto"):
    """~0.1 s startup micro-bench on THIS machine; faster classifier wins."""
    if forced in CLASSIFIERS:
        print(f"classifier: {forced} (forced)")
        return CLASSIFIERS[forced]
    B = Bufs(roi_shape)
    rng = np.random.default_rng(0)
    # dummy = full random noise = WORST CASE on purpose: speckle-heavy input
    # penalizes the no-morphology LUT path (CCL labels every fleck), so the
    # winner is the one with the flattest cost profile, not best-case speed
    dummy = rng.integers(0, 256, (*roi_shape[:2], 3), np.uint8)
    times = {}
    for name, fn in CLASSIFIERS.items():
        fn(dummy, B)
        t = time.perf_counter()
        for _ in range(200):
            fn(dummy, B)
        times[name] = (time.perf_counter() - t) / 200 * 1e3
    win = min(times, key=times.get)
    print(f"classifier auto-select: cv2 {times['cv2']:.3f} ms | "
          f"lut {times['lut']:.3f} ms -> {win}")
    return CLASSIFIERS[win]


def blobs(mask, color_id):
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
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


def detect(frame, B, classify):
    y0 = int(frame.shape[0] * ROI_TOP_FRAC)
    roi = frame[y0:]
    red_m, grn_m = classify(roi, B)
    parts = [b for b in (blobs(red_m, 0), blobs(grn_m, 1)) if b is not None]
    if not parts:
        return None, None, y0
    allb = parts[0] if len(parts) == 1 else np.vstack(parts)
    return allb, allb[allb[:, 4].argmax()], y0


def render(frame, allb, nearest, y0, fps, w_ms, v_ms):
    big = cv2.resize(frame, (PROC_W * VIEW_SCALE, PROC_H * VIEW_SCALE),
                     interpolation=cv2.INTER_NEAREST)
    S = VIEW_SCALE
    if allb is not None:
        for x, y, w, h, _, cid in allb:
            cv2.rectangle(big, (x * S, (y + y0) * S),
                          ((x + w) * S, (y + y0 + h) * S), BOX_BGR[cid], 2)
    if nearest is not None:
        x, y, w, h, _, cid = nearest
        cv2.rectangle(big, (x * S, (y + y0) * S),
                      ((x + w) * S, (y + y0 + h) * S), (0, 255, 255), 3)
        cv2.putText(big, DECISION[cid], (12, big.shape[0] - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.putText(big, f"{fps:5.1f} FPS", (12, 34),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.putText(big, f"cap+proc {w_ms:5.2f}  view {v_ms:4.1f} ms",
                (12, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    return big


def open_source(args):
    if args.pi:
        from picamera2 import Picamera2, MappedArray
        picam2 = Picamera2()
        cfg = picam2.create_video_configuration(
            lores={"size": (PROC_W, PROC_H), "format": "RGB888"},   # Pi 5 RGB lores
            sensor=SENSOR_MODE,
            buffer_count=3,
            controls={"FrameDurationLimits": (FRAME_US, FRAME_US),
                      "NoiseReductionMode": 3},
        )
        picam2.align_configuration(cfg)
        picam2.configure(cfg)
        picam2.start()
        time.sleep(1.0)
        picam2.set_controls({"AeEnable": False, "AwbEnable": False})
        # venue: picam2.set_controls({"ExposureTime": 4000, "AnalogueGain": 4.0})

        def raw_read():
            req = picam2.capture_request()
            return req
        return picam2, None

    cap = cv2.VideoCapture(args.source if args.source is not None else 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return None, cap


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("source", nargs="?", default=None)
    p.add_argument("--pi", action="store_true")
    p.add_argument("--show", nargs="?", const=1, default=None, type=int, metavar="N")
    p.add_argument("--bench", action="store_true")
    p.add_argument("--classifier", choices=["auto", "cv2", "lut"], default="auto")
    a = p.parse_args()
    if a.show is None:
        a.show = 0 if a.pi else 1
    if a.source is not None and a.source.isdigit():
        a.source = int(a.source)
    return a


def main():
    args = parse_args()
    picam2, cap = open_source(args)
    if args.pi:
        from picamera2 import MappedArray

    # PRIME: read one frame for the REAL geometry — align_configuration may
    # adjust the lores size; Bufs/auto-select must match actual, not constants
    if args.pi:
        req = picam2.capture_request()
        with MappedArray(req, "lores") as m:
            fshape = m.array.shape
        req.release()
    else:
        ok, _f0 = cap.read()
        if not ok:
            print("no frames from source")
            return
        fshape = (PROC_H, PROC_W, 3)
    y0p = int(fshape[0] * ROI_TOP_FRAC)
    roi_shape = (fshape[0] - y0p, fshape[1], 3)
    classify = pick_classifier(roi_shape, args.classifier)
    B = Bufs(roi_shape)

    last_decision = None
    ema_w = ema_v = 0.0
    fps, fwin, twin = 0.0, 0, time.perf_counter()
    t_rep, idx = twin, 0

    while True:
        showing = args.show and idx % args.show == 0
        t0 = time.perf_counter()
        if args.pi:
            req = picam2.capture_request()
            with MappedArray(req, "lores") as m:
                allb, nearest, y0 = detect(m.array, B, classify)
                frame = m.array.copy() if showing else None
            req.release()
        else:
            ok, f = cap.read()
            if not ok:
                break
            frame = cv2.resize(f, (PROC_W, PROC_H), interpolation=cv2.INTER_AREA)
            allb, nearest, y0 = detect(frame, B, classify)
        t2 = time.perf_counter()
        ema_w = 0.9 * ema_w + 0.1 * (t2 - t0)

        if showing:
            big = render(frame, allb, nearest, y0, fps, ema_w * 1e3, ema_v * 1e3)
            cv2.imshow("pillars", big)
            if POLL() & 0xFF == ord("q"):
                break
            ema_v = 0.9 * ema_v + 0.1 * (time.perf_counter() - t2)
        idx += 1
        fwin += 1

        now = time.perf_counter()
        if now - twin >= 0.5:
            fps, fwin, twin = fwin / (now - twin), 0, now
        if args.bench and now - t_rep >= 1.0:
            print(f"{fps:6.1f} fps | cap+proc {ema_w*1e3:6.2f} ms | "
                  f"view {ema_v*1e3:5.2f} ms")
            t_rep = now

        decision = DECISION[nearest[5]] if nearest is not None else None
        if decision != last_decision:
            print(decision or "no pillar")
            last_decision = decision

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
