"""decide.py — pass-side decision. Prints "left" or "right".

WRO Future Engineers Obstacle Challenge:
    red pillar   -> pass RIGHT
    green pillar -> pass LEFT

The mapping below is keyed on the class NAME, never on the integer id.
The dataset ships 0=green/1=red while an earlier config used 0=red; if the
index order is ever changed again, a name lookup follows it and an id lookup
silently inverts every decision. Inverting this is a scored failure.

    python decide.py --image frame.jpg
    python decide.py --dir  some/folder
    python decide.py --camera 0
    python decide.py --eval splits/val.txt        # decision accuracy vs labels

Default output is one word per frame and nothing else.
"""

import argparse
import os
import sys

import cv2
import torch

import tiny_pillar as T

PASS_SIDE = {"green": "left", "red": "right"}

# Operating point, chosen from the sweep in _sweep.py. The asymmetry that
# matters: a NO CALL means hold course, which is recoverable; a WRONG SIDE is
# scored against you. Measured on val / val_cooc / val_neg:
#
#    thr  minH | val acc  wrong  nocall | cooc acc  wrong | neg fire
#   0.35  0.08 |   0.879   2.4%    9.7% |    0.617  31.7% |   11/60   <- default
#   0.45  0.08 |   0.718   1.6%   26.6% |    0.533  31.7% |    6/60
#   0.45  0.20 |   0.371   0.0%   62.9% |    0.350  16.7% |    3/60
#   0.55  0.20 |   0.177   0.0%   82.3% |    0.117  15.0% |    1/60
#
# Raising the threshold buys wrong-side reduction on real frames but pays for it
# in no-calls very fast. Note that it does NOT fix the co-occurrence case: wrong
# side stays ~30% almost everywhere, because the cause is a MISSED nearest
# pillar, not an over-confident one. You cannot threshold your way out of a
# detection that never happened -- that needs real dual-pillar training footage.
THR = 0.35

# A pillar occupying less than this fraction of frame height is too far away to
# act on; committing to a steer from a 12 px blob is how you swerve at noise.
MIN_H = 0.08


def load(ckpt="runs_final/best.pt", device=None):
    dev = device or torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ck = torch.load(ckpt, map_location=dev, weights_only=False)
    if ck.get("classes") != T.CLASS_NAMES:
        raise SystemExit(f"class mismatch: ckpt {ck.get('classes')} vs "
                         f"tiny_pillar {T.CLASS_NAMES} -- refusing to run")
    if set(ck["classes"]) - set(PASS_SIDE):
        raise SystemExit(f"no pass-side rule for {set(ck['classes']) - set(PASS_SIDE)}")
    T.PREPROC = ck.get("preproc", False)      # must match how it was trained
    m = T.TinyPillar().to(dev)
    m.load_state_dict(ck["model"])
    m.eval()
    return m, dev


def target(dets, min_h=MIN_H):
    """Pick the pillar to act on: the nearest one.

    Nearest = lowest bottom edge, matching pillar_fast.py v4.1. Bottom edge
    beats box height because a partially occluded or clipped pillar loses
    height from the top while its base stays put.
    """
    ok = [d for d in dets if (d[5] - d[3]) >= min_h]
    return max(ok, key=lambda d: d[5]) if ok else None


def decide_frame(model, bgr, device, thr=0.35, min_h=MIN_H):
    """-> (side or None, detection or None). side is the literal 'left'/'right'."""
    img, _, _, _ = T.letterbox(bgr)
    if T.PREPROC:
        img = T.normalize_photometric(img)
    x = torch.from_numpy(img.transpose(2, 0, 1)[None].astype("float32") / 255.0)
    with torch.no_grad():
        dets = T.decode(*model(x.to(device)), thr=thr)[0]
    d = target(dets, min_h)
    return (PASS_SIDE[T.CLASS_NAMES[d[0]]], d) if d else (None, None)


class Decider:
    """Majority vote over the last N frames.

    A single-frame decision flaps: one bad frame at 40 fps flips the steer and
    flips back, which is worse on the field than committing to one side. The
    vote costs N-1 frames of latency -- at ~40 fps and N=5 that is ~125 ms.
    Set window=1 to disable.
    """

    def __init__(self, window=5):
        self.window = window
        self.buf = []

    def __call__(self, side):
        self.buf.append(side)
        if len(self.buf) > self.window:
            self.buf.pop(0)
        votes = [s for s in self.buf if s]
        if not votes:
            return None
        return max(set(votes), key=votes.count)


def gt_side(path):
    """Expected side from the label file, using the same nearest-pillar rule."""
    rows = T.load_labels(path)
    if not rows:
        return None
    c, cx, cy, w, h = max(rows, key=lambda r: r[2] + r[4] / 2)   # lowest bottom edge
    if h < MIN_H:
        return None
    return PASS_SIDE[T.CLASS_NAMES[c]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="runs_final/best.pt")
    ap.add_argument("--image")
    ap.add_argument("--dir")
    ap.add_argument("--video")
    ap.add_argument("--camera", type=int)
    ap.add_argument("--eval", help="manifest; report decision accuracy vs labels")
    ap.add_argument("--thr", type=float, default=THR)
    ap.add_argument("--window", type=int, default=1,
                    help="vote window; >1 for video/camera")
    ap.add_argument("--verbose", action="store_true",
                    help="also print class, score and box")
    a = ap.parse_args()

    model, dev = load(a.ckpt)
    dec = Decider(a.window)

    def emit(bgr, tag=None):
        side, d = decide_frame(model, bgr, dev, a.thr)
        side = dec(side) if a.window > 1 else side
        if a.verbose:
            extra = (f"  [{T.CLASS_NAMES[d[0]]} {d[1]:.2f} "
                     f"x{d[2]:.2f}-{d[4]:.2f}]") if d else "  [no pillar]"
            print(f"{tag + '  ' if tag else ''}{side or '-'}{extra}", flush=True)
        elif side:
            print(side, flush=True)

    if a.eval:
        files = [l.strip() for l in open(a.eval, encoding="utf-8") if l.strip()]
        n = ok = miss = wrong = spur = 0
        for p in files:
            g = gt_side(p)
            bgr = cv2.imread(p)
            if bgr is None:
                continue
            s, _ = decide_frame(model, bgr, dev, a.thr)
            if g is None:
                spur += 1 if s else 0
                continue
            n += 1
            if s is None:
                miss += 1
            elif s == g:
                ok += 1
            else:
                wrong += 1
        print(f"decision accuracy {ok}/{n} = {ok / max(1, n):.3f}")
        print(f"  wrong side {wrong}  ({wrong / max(1, n):.1%})   <-- these lose points")
        print(f"  no call    {miss}  ({miss / max(1, n):.1%})")
        if spur:
            print(f"  spurious calls on {spur} frames with no actionable pillar")
        return

    if a.image:
        emit(cv2.imread(a.image), os.path.basename(a.image) if a.verbose else None)
    elif a.dir:
        for f in sorted(os.listdir(a.dir)):
            if f.lower().endswith((".jpg", ".jpeg", ".png")):
                emit(cv2.imread(os.path.join(a.dir, f)), f if a.verbose else None)
    elif a.video or a.camera is not None:
        cap = cv2.VideoCapture(a.video if a.video else a.camera)
        while True:
            ok_, frame = cap.read()
            if not ok_:
                break
            emit(frame)
        cap.release()
    else:
        ap.error("give one of --image / --dir / --video / --camera / --eval")


if __name__ == "__main__":
    main()
