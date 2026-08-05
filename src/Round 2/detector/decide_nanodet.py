"""decide_nanodet.py — pass-side decision from the trained NanoDet. Prints "left" or "right".

WRO Future Engineers Obstacle Challenge:
    red pillar   -> pass RIGHT
    green pillar -> pass LEFT

The mapping is keyed on the class NAME, never the integer id. cfg.py shipped
once with ["red_pillar","green_pillar"] (0=red), the inverse of data.yaml; a
name lookup survives that kind of reorder, an index lookup silently inverts
every decision on the field.

    python decide_nanodet.py --image frame.jpg
    python decide_nanodet.py --dir some/folder --verbose
    python decide_nanodet.py --camera 0 --window 5
    python decide_nanodet.py --sweep

Default output is one word per frame and nothing else.
"""

import argparse
import os
import sys

import cv2
import torch

sys.path.insert(0, r"C:\Users\ANT PC\wro_vision")
from nanodet_lite import cfg as C
from nanodet_lite.train import build_model
from eval_nanodet import SPLITS, decode, prep, run

PASS_SIDE = {"green": "left", "red": "right"}
CKPT = r"C:\Users\ANT PC\wro_vision\nanodet_runs\best.pt"

# A pillar shorter than this fraction of frame height is too far to act on;
# committing to a steer from a 12 px blob is how you swerve at noise.
MIN_H = 0.08

# Operating point from --sweep on the nanodet_lite ep86 checkpoint:
#
#    thr  minH | val acc  wrong  nocall | cooc acc  wrong | neg/img
#   0.30  0.08 |   0.941   5.9%    0.0% |    0.883  11.7% |   0.350
#   0.35  0.08 |   0.932   5.9%    0.8% |    0.867  13.3% |   0.283
#   0.45  0.08 |   0.941   5.1%    0.8% |    0.817  18.3% |   0.083  <- default
#   0.55  0.08 |   0.856   0.0%   14.4% |    0.733  20.0% |   0.050
#
# 0.45 is joint-best on real-val accuracy, lowest non-zero wrong-side, and
# cuts empty-frame false alarms 4x versus 0.35. It pays for that on the
# co-occurrence set -- but those are SYNTHETIC composites, while the
# false-alarm and real-val columns are measured on real frames.
#
# Lowering thr helps co-occurrence (more detections -> less chance of missing
# the nearest pillar) and floods empty frames; raising it does the reverse.
# thr 0.55 gives ZERO wrong-side calls on real frames at the cost of holding
# course 14.4% of the time -- use it if a spurious steer is worse than
# hesitation on your track.
THR = 0.45


def load(ckpt=CKPT):
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ck = torch.load(ckpt, map_location=dev, weights_only=False)
    if ck["classes"] != C.CLASS_NAMES:
        raise SystemExit(f"class mismatch: ckpt {ck['classes']} vs "
                         f"cfg {C.CLASS_NAMES} -- refusing to run")
    if set(ck["classes"]) - set(PASS_SIDE):
        raise SystemExit(f"no pass-side rule for {set(ck['classes']) - set(PASS_SIDE)}")
    m = build_model().to(dev)
    m.load_state_dict(ck["model"])
    m.eval()
    return m, dev, ck


def decide_frame(model, bgr, dev, thr=THR, min_h=MIN_H):
    """-> (side or None, detection or None). Nearest = lowest bottom edge.

    Bottom edge beats box height as a distance proxy because occlusion and
    frame-clipping eat the top of a pillar while its base stays put.
    """
    x, _, _, _ = prep(bgr)
    dets = decode(model, x.to(dev), thr)
    act = [d for d in dets if (d[5] - d[3]) >= min_h]
    if not act:
        return None, None
    d = max(act, key=lambda d: d[5])
    return PASS_SIDE[C.CLASS_NAMES[d[0]]], d


class Decider:
    """Majority vote over the last N frames.

    A single-frame decision flaps: one bad frame flips the steer and flips
    back, which is worse on the field than committing. Costs N-1 frames of
    latency. window=1 disables.
    """

    def __init__(self, window=1):
        self.window, self.buf = window, []

    def __call__(self, side):
        self.buf.append(side)
        if len(self.buf) > self.window:
            self.buf.pop(0)
        votes = [s for s in self.buf if s]
        return max(set(votes), key=votes.count) if votes else None


def sweep(model, dev):
    """Calibrate the operating point.

    The asymmetry that matters: a NO CALL means hold course, which is
    recoverable. A WRONG SIDE is scored against you. So the threshold should
    be tuned to crush wrong-side calls, accepting more no-calls -- up to the
    point where the detector stops being useful.
    """
    print(f"{'thr':>5} {'minH':>5} | {'val acc':>8} {'wrong':>6} {'nocall':>7} | "
          f"{'cooc acc':>9} {'wrong':>6} | {'neg/img':>8}")
    print("-" * 74)
    best = None
    import eval_nanodet as EN
    for thr in (0.30, 0.35, 0.45, 0.55):
        for min_h in (0.08, 0.14):
            # run() reads MIN_H from its own module global, so set it there
            # rather than passing it -- otherwise both rows print identically.
            EN.MIN_H = min_h
            v = run(model, os.path.join(SPLITS, "val.txt"), dev, thr)
            c = run(model, os.path.join(SPLITS, "val_cooc.txt"), dev, thr)
            n = run(model, os.path.join(SPLITS, "val_neg.txt"), dev, thr)
            va = v["dec_ok"] / max(1, v["dec_n"])
            vw = v["dec_wrong"] / max(1, v["dec_n"])
            vn = v["dec_none"] / max(1, v["dec_n"])
            ca = c["dec_ok"] / max(1, c["dec_n"])
            cw = c["dec_wrong"] / max(1, c["dec_n"])
            print(f"{thr:5.2f} {min_h:5.2f} | {va:8.3f} {vw:6.1%} {vn:7.1%} | "
                  f"{ca:9.3f} {cw:6.1%} | {n['det_per_img']:8.3f}")
            # score: total wrong-side across both sets, tie-broken by accuracy
            s = (vw + cw, -(va + ca))
            if best is None or s < best[0]:
                best = (s, thr, min_h, va, vw, ca, cw)
    print(f"\nlowest combined wrong-side: thr {best[1]:.2f} minH {best[2]:.2f}"
          f"  (val acc {best[3]:.3f} wrong {best[4]:.1%} | "
          f"cooc acc {best[5]:.3f} wrong {best[6]:.1%})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default=CKPT)
    ap.add_argument("--image")
    ap.add_argument("--dir")
    ap.add_argument("--video")
    ap.add_argument("--camera", type=int)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--thr", type=float, default=THR)
    ap.add_argument("--min-h", type=float, default=MIN_H)
    ap.add_argument("--window", type=int, default=1)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()

    model, dev, ck = load(a.ckpt)
    if a.verbose or a.sweep:
        print(f"ckpt epoch {ck['epoch']}  classes {ck['classes']}", flush=True)

    if a.sweep:
        sweep(model, dev)
        return

    dec = Decider(a.window)

    def emit(bgr, tag=None):
        side, d = decide_frame(model, bgr, dev, a.thr, a.min_h)
        side = dec(side) if a.window > 1 else side
        if a.verbose:
            extra = (f"  [{C.CLASS_NAMES[d[0]]} {d[1]:.2f} "
                     f"x{d[2]:.2f}-{d[4]:.2f}]") if d else "  [no pillar]"
            print(f"{tag + '  ' if tag else ''}{side or '-'}{extra}", flush=True)
        elif side:
            print(side, flush=True)

    if a.image:
        emit(cv2.imread(a.image), os.path.basename(a.image) if a.verbose else None)
    elif a.dir:
        for f in sorted(os.listdir(a.dir)):
            if f.lower().endswith((".jpg", ".jpeg", ".png")):
                emit(cv2.imread(os.path.join(a.dir, f)), f if a.verbose else None)
    elif a.video or a.camera is not None:
        cap = cv2.VideoCapture(a.video if a.video else a.camera)
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            emit(frame)
        cap.release()
    else:
        ap.error("give one of --image / --dir / --video / --camera / --sweep")


if __name__ == "__main__":
    main()
