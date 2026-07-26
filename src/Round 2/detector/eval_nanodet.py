"""eval_nanodet.py — score nanodet_lite on the SAME metrics as tiny_pillar.

Val loss is not comparable across architectures, so this converts NanoDet's
detections into the identical three numbers used for tiny_pillar:
  val F1        per-class precision/recall/F1 at IoU 0.5 on real val
  NEG           detections per empty frame (the clutter failure mode)
  COOC          F1 on synthetic dual-pillar frames (proxy only)
plus pass-side decision accuracy, which is what actually scores at WRO.
"""

import os
import sys

import cv2
import numpy as np
import torch

sys.path.insert(0, r"C:\Users\ANT PC\wro_vision")
from nanodet_lite import cfg as C
from nanodet_lite.train import build_model

SPLITS = r"C:\Users\ANT PC\wro_vision\tiny\splits"
PASS_SIDE = {"green": "left", "red": "right"}
MIN_H = 0.08
MEAN = np.array([103.53, 116.28, 123.675], np.float32)
STD = np.array([57.375, 57.12, 58.395], np.float32)


def letterbox(img, size=320):
    h, w = img.shape[:2]
    r = size / max(h, w)
    nh, nw = int(round(h * r)), int(round(w * r))
    out = np.full((size, size, 3), 114, np.uint8)
    top, left = (size - nh) // 2, (size - nw) // 2
    out[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh))
    return out, r, left, top


def prep(img):
    lb, r, l, t = letterbox(img)
    x = (lb.astype(np.float32) - MEAN) / STD
    return torch.from_numpy(x.transpose(2, 0, 1))[None], r, l, t


def load_labels(p):
    lp = p.replace(os.sep + "images" + os.sep, os.sep + "labels" + os.sep)
    lp = os.path.splitext(lp)[0] + ".txt"
    out = []
    if os.path.exists(lp):
        for line in open(lp):
            q = line.split()
            if len(q) >= 5:
                out.append((int(q[0]), *(float(v) for v in q[1:5])))
    return out


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def decode(model, x, thr=0.35):
    """NanoDetPlusHead -> [(cls, score, x1,y1,x2,y2)] in normalised letterbox coords."""
    with torch.no_grad():
        preds = model(x)
    nc, reg = C.NUM_CLASSES, 4 * (C.REG_MAX + 1)
    cls_p, reg_p = preds.split([nc, reg], dim=-1)
    scores = cls_p.sigmoid()[0]                       # (A, nc)

    # rebuild the anchor centres the head used
    centers, strides = [], []
    for s in C.STRIDES:
        f = C.INPUT_SIZE[0] // s
        yy, xx = np.meshgrid(np.arange(f), np.arange(f), indexing="ij")
        centers.append(np.stack([(xx.ravel() + 0.5) * s, (yy.ravel() + 0.5) * s], 1))
        strides.append(np.full(f * f, s, np.float32))
    centers = np.concatenate(centers, 0)
    strides = np.concatenate(strides, 0)

    # DFL expectation -> ltrb distances
    d = reg_p[0].reshape(-1, 4, C.REG_MAX + 1).softmax(-1)
    proj = torch.arange(C.REG_MAX + 1, dtype=d.dtype, device=d.device)
    ltrb = (d * proj).sum(-1).cpu().numpy() * strides[:, None]

    best_s, best_c = scores.max(-1)
    keep = (best_s > thr).cpu().numpy().nonzero()[0]
    dets = []
    S = C.INPUT_SIZE[0]
    for i in keep:
        cx, cy = centers[i]
        x1, y1 = (cx - ltrb[i, 0]) / S, (cy - ltrb[i, 1]) / S
        x2, y2 = (cx + ltrb[i, 2]) / S, (cy + ltrb[i, 3]) / S
        dets.append((int(best_c[i]), float(best_s[i]), x1, y1, x2, y2))
    # tiny NMS: few objects per frame, so O(n^2) is fine
    dets.sort(key=lambda d: -d[1])
    out = []
    for d0 in dets:
        if all(iou(d0[2:6], k[2:6]) < 0.5 or d0[0] != k[0] for k in out):
            out.append(d0)
    return out[:10]


def gt_norm(path):
    """GT in letterboxed normalised coords, matching the detector's frame."""
    img = cv2.imread(path)
    h0, w0 = img.shape[:2]
    _, r, l, t = letterbox(img)
    S = C.INPUT_SIZE[0]
    g = []
    for c, cx, cy, bw, bh in load_labels(path):
        X, Y = (cx * w0 * r + l) / S, (cy * h0 * r + t) / S
        W, H = bw * w0 * r / S, bh * h0 * r / S
        g.append((c, X - W/2, Y - H/2, X + W/2, Y + H/2))
    return img, g


def run(model, manifest, dev, thr=0.35):
    files = [l.strip() for l in open(manifest, encoding="utf-8") if l.strip()]
    tp = [0]*C.NUM_CLASSES; fp = [0]*C.NUM_CLASSES; fn = [0]*C.NUM_CLASSES
    n_img = n_det = 0
    dec_n = dec_ok = dec_wrong = dec_none = 0
    for p in files:
        img, gt = gt_norm(p)
        x, _, _, _ = prep(img)
        dets = decode(model, x.to(dev), thr)
        n_img += 1; n_det += len(dets)

        used = set()
        for c, s, *bb in dets:
            best, bi = 0.0, -1
            for gi, g in enumerate(gt):
                if gi in used or g[0] != c:
                    continue
                v = iou(bb, g[1:])
                if v > best:
                    best, bi = v, gi
            if best >= 0.5:
                tp[c] += 1; used.add(bi)
            else:
                fp[c] += 1
        for gi, g in enumerate(gt):
            if gi not in used:
                fn[g[0]] += 1

        # pass-side decision: nearest = lowest bottom edge, gated on size
        act = [d for d in dets if (d[5]-d[3]) >= MIN_H]
        pred = PASS_SIDE[C.CLASS_NAMES[max(act, key=lambda d: d[5])[0]]] if act else None
        gact = [g for g in gt if (g[4]-g[2]) >= MIN_H]
        truth = PASS_SIDE[C.CLASS_NAMES[max(gact, key=lambda g: g[4])[0]]] if gact else None
        if truth is not None:
            dec_n += 1
            if pred is None:
                dec_none += 1
            elif pred == truth:
                dec_ok += 1
            else:
                dec_wrong += 1

    rows, f1s = [], []
    for c in range(C.NUM_CLASSES):
        pr = tp[c] / max(1, tp[c] + fp[c]); rc = tp[c] / max(1, tp[c] + fn[c])
        f = 2*pr*rc / max(1e-9, pr+rc); f1s.append(f)
        rows.append(f"{C.CLASS_NAMES[c]:6s} P {pr:.3f} R {rc:.3f} F1 {f:.3f} "
                    f"(tp {tp[c]} fp {fp[c]} fn {fn[c]})")
    return dict(f1=sum(f1s)/C.NUM_CLASSES, rows=rows, det_per_img=n_det/max(1, n_img),
                n_img=n_img, dec_n=dec_n, dec_ok=dec_ok, dec_wrong=dec_wrong,
                dec_none=dec_none)


if __name__ == "__main__":
    ck_path = sys.argv[1] if len(sys.argv) > 1 else \
        r"C:\Users\ANT PC\wro_vision\nanodet_runs\best.pt"
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    m = build_model().to(dev).eval()
    ck = torch.load(ck_path, map_location=dev, weights_only=False)
    m.load_state_dict(ck["model"])
    print(f"ckpt epoch {ck['epoch']}  val-loss {ck['val']:.4f}  classes {ck['classes']}")

    r = run(m, os.path.join(SPLITS, "val.txt"), dev)
    print(f"\nVAL   macroF1 {r['f1']:.3f}")
    for x in r["rows"]:
        print("   ", x)
    print(f"      decision {r['dec_ok']}/{r['dec_n']} = "
          f"{r['dec_ok']/max(1,r['dec_n']):.3f}  wrong "
          f"{r['dec_wrong']/max(1,r['dec_n']):.1%}  nocall "
          f"{r['dec_none']/max(1,r['dec_n']):.1%}")

    n = run(m, os.path.join(SPLITS, "val_neg.txt"), dev)
    print(f"\nNEG   {n['det_per_img']:.3f} false det/img over {n['n_img']} empty frames")

    c = run(m, os.path.join(SPLITS, "val_cooc.txt"), dev)
    print(f"\nCOOC  macroF1 {c['f1']:.3f}   decision "
          f"{c['dec_ok']}/{c['dec_n']} = {c['dec_ok']/max(1,c['dec_n']):.3f}  "
          f"wrong {c['dec_wrong']/max(1,c['dec_n']):.1%}   (synthetic proxy)")
