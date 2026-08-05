"""tiny_pillar.py — barebones anchor-free pillar detector, single file.

Why not NanoDet-Plus for a first run: WRO pillars are large, high-contrast,
near-rectangular, and there are two classes. That does not need 1.17M params,
a dual-head training scheme, a dynamic soft-label assigner, or NMS. This is a
CenterNet-style head: one heatmap peak per object, plus width/height and a
sub-cell offset. Loss is ~15 lines. Peak extraction is a 3x3 maxpool.

Keep `nanodet_lite` as the fallback if this plateaus below what you need.

    python make_split.py --root <dataset>          # do this first
    python tiny_pillar.py train
    python tiny_pillar.py eval  --ckpt runs/best.pt
    python tiny_pillar.py export --ckpt runs/best.pt

CLASS IDS COME FROM data.yaml AND ARE  0=green, 1=red.
That is the opposite of nanodet_lite/cfg.py. Getting this backwards makes the
robot pass every pillar on the wrong side, which is a scored failure, not a
cosmetic one. CLASS_NAMES below is the single source of truth.
"""

import argparse
import math
import os
import random

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

CLASS_NAMES = ["green", "red"]   # data.yaml order -- do not reorder
NUM_CLASSES = len(CLASS_NAMES)
INP = 320                        # network input (square, letterboxed)
STRIDE = 8
OUT = INP // STRIDE              # 40x40
LUT_CACHE = {}

# Fixed photometric normalisation, applied IDENTICALLY at train and inference.
# The flag is written into the checkpoint and re-checked at export, because a
# train/serve mismatch here is silent: the model still runs, just badly.
PREPROC = True


def normalize_photometric(img):
    """Grey-world white balance + CLAHE on V only.

    CLAHE must never touch BGR directly -- equalising the channels
    independently shifts hue, and hue is the class label for this task. Working
    on V leaves the H channel untouched by construction.
    Cost is ~1-2 ms at 320x320, which fits the Pi 5 budget.
    """
    b, g, r = cv2.split(img.astype(np.float32))
    mb, mg, mr = b.mean() + 1e-6, g.mean() + 1e-6, r.mean() + 1e-6
    k = (mb + mg + mr) / 3.0
    img = cv2.merge([np.clip(b * k / mb, 0, 255),
                     np.clip(g * k / mg, 0, 255),
                     np.clip(r * k / mr, 0, 255)]).astype(np.uint8)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(v)
    return cv2.cvtColor(cv2.merge([h, s, v]), cv2.COLOR_HSV2BGR)


# ---------------------------------------------------------------- data

def letterbox(img, size=INP):
    """Resize keeping aspect, pad to square. The dataset mixes 640x480 (4:3)
    and 1920x1080 (16:9); stretching instead of padding would give the model a
    free shortcut, since aspect ratio correlates perfectly with which capture
    session -- and therefore with the class -- in this dataset."""
    h, w = img.shape[:2]
    r = size / max(h, w)
    nh, nw = int(round(h * r)), int(round(w * r))
    out = np.full((size, size, 3), 114, np.uint8)
    top, left = (size - nh) // 2, (size - nw) // 2
    out[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh),
                                                   interpolation=cv2.INTER_LINEAR)
    return out, r, left, top


def load_labels(img_path):
    """YOLO txt sitting in a sibling `labels` dir -> [(cls, cx, cy, w, h)] normalised."""
    lab = img_path.replace(os.sep + "images" + os.sep, os.sep + "labels" + os.sep)
    lab = os.path.splitext(lab)[0] + ".txt"
    rows = []
    if os.path.exists(lab):
        for line in open(lab):
            p = line.split()
            if len(p) >= 5:
                rows.append((int(p[0]), *(float(x) for x in p[1:5])))
    return rows


def gaussian(hm, cx, cy, radius):
    """Splat an unnormalised gaussian peak, keeping the max where peaks overlap."""
    d = 2 * radius + 1
    sigma = d / 6.0
    ax = np.arange(-radius, radius + 1, dtype=np.float32)
    g = np.exp(-(ax[None, :] ** 2 + ax[:, None] ** 2) / (2 * sigma * sigma))
    H, W = hm.shape
    l, r = min(cx, radius), min(W - cx, radius + 1)
    t, b = min(cy, radius), min(H - cy, radius + 1)
    if r <= -l or b <= -t:
        return
    np.maximum(hm[cy - t:cy + b, cx - l:cx + r],
               g[radius - t:radius + b, radius - l:radius + r],
               out=hm[cy - t:cy + b, cx - l:cx + r])


class PillarDS(torch.utils.data.Dataset):
    def __init__(self, manifest, train):
        self.files = [l.strip() for l in open(manifest, encoding="utf-8") if l.strip()]
        self.train = train

    def __len__(self):
        return len(self.files)

    def _aug(self, img, boxes):
        # Colour: jitter V and S only. HUE IS THE LABEL for this task -- rotating
        # it teaches the model to ignore the one cue that separates the classes.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[..., 1] *= random.uniform(0.6, 1.4)
        hsv[..., 2] *= random.uniform(0.5, 1.5)
        img = cv2.cvtColor(np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR)

        if random.random() < 0.5:                      # gamma / exposure curve
            g = random.uniform(0.6, 1.6)
            img = LUT_CACHE.setdefault(
                round(g, 2),
                np.clip(((np.arange(256) / 255.0) ** (1.0 / round(g, 2))) * 255,
                        0, 255).astype(np.uint8))[img]

        if random.random() < 0.4:                      # synthetic cast shadow
            # Directly targets the reported "misses under shadow" failure. A
            # soft-edged darkened polygon is a closer match to a real cast
            # shadow than uniform brightness reduction, which never happens.
            ov = np.ones(img.shape[:2], np.float32)
            pts = np.array([[random.randint(-40, INP + 40),
                             random.randint(-40, INP + 40)] for _ in range(4)],
                           np.int32)
            cv2.fillConvexPoly(ov, cv2.convexHull(pts), random.uniform(0.35, 0.75))
            ov = cv2.GaussianBlur(ov, (0, 0), random.uniform(4, 14))
            img = np.clip(img.astype(np.float32) * ov[..., None], 0, 255).astype(np.uint8)

        if random.random() < 0.5:                      # hflip: red stays red
            img = img[:, ::-1].copy()
            boxes = [(c, 1.0 - cx, cy, w, h) for c, cx, cy, w, h in boxes]

        if random.random() < 0.8:                      # scale + shift, in norm space
            s = random.uniform(0.7, 1.3)
            dx, dy = random.uniform(-.1, .1), random.uniform(-.1, .1)
            M = np.float32([[s, 0, dx * INP + (1 - s) * INP / 2],
                            [0, s, dy * INP + (1 - s) * INP / 2]])
            img = cv2.warpAffine(img, M, (INP, INP), borderValue=(114, 114, 114))
            nb = []
            for c, cx, cy, w, h in boxes:
                cx = cx * s + dx + (1 - s) / 2
                cy = cy * s + dy + (1 - s) / 2
                w, h = w * s, h * s
                if 0 < cx < 1 and 0 < cy < 1 and w > 0.01 and h > 0.01:
                    nb.append((c, cx, cy, w, h))
            boxes = nb
        return img, boxes

    def __getitem__(self, i):
        path = self.files[i]
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"unreadable: {path}")
        h0, w0 = img.shape[:2]
        img, r, left, top = letterbox(img)

        # normalised-in-original -> normalised-in-letterboxed
        boxes = []
        for c, cx, cy, w, h in load_labels(path):
            boxes.append((c, (cx * w0 * r + left) / INP, (cy * h0 * r + top) / INP,
                          w * w0 * r / INP, h * h0 * r / INP))
        if self.train:
            img, boxes = self._aug(img, boxes)
        # AFTER augmentation, never before: augmentation plays the role of the
        # scene/sensor varying, normalisation plays the role of the fixed
        # on-robot preprocessing that has to undo it. Same order as inference.
        if PREPROC:
            img = normalize_photometric(img)

        hm = np.zeros((NUM_CLASSES, OUT, OUT), np.float32)
        wh = np.zeros((2, OUT, OUT), np.float32)
        off = np.zeros((2, OUT, OUT), np.float32)
        mask = np.zeros((OUT, OUT), np.float32)
        for c, cx, cy, w, h in boxes:
            fx, fy = cx * OUT, cy * OUT
            ix, iy = int(fx), int(fy)
            if not (0 <= ix < OUT and 0 <= iy < OUT):
                continue
            rad = max(1, int(0.18 * min(w, h) * OUT))
            gaussian(hm[c], ix, iy, rad)
            wh[:, iy, ix] = (w, h)          # normalised 0-1, direct regression
            off[:, iy, ix] = (fx - ix, fy - iy)
            mask[iy, ix] = 1.0

        x = torch.from_numpy(img.transpose(2, 0, 1).astype(np.float32) / 255.0)
        return x, torch.from_numpy(hm), torch.from_numpy(wh), \
            torch.from_numpy(off), torch.from_numpy(mask)


# ---------------------------------------------------------------- model

def dw(cin, cout, stride=1):
    """Depthwise-separable conv. Cheap on Cortex-A76, which is what the Pi 5 is."""
    return nn.Sequential(
        nn.Conv2d(cin, cin, 3, stride, 1, groups=cin, bias=False),
        nn.BatchNorm2d(cin), nn.ReLU(inplace=True),
        nn.Conv2d(cin, cout, 1, bias=False),
        nn.BatchNorm2d(cout), nn.ReLU(inplace=True),
    )


class TinyPillar(nn.Module):
    """320 -> stride-8 heatmap. One tiny top-down connection for context, so a
    pillar's decision is not made from a 3x3 receptive field."""

    def __init__(self, nc=NUM_CLASSES):
        super().__init__()
        self.stem = nn.Sequential(
            nn.Conv2d(3, 16, 3, 2, 1, bias=False),
            nn.BatchNorm2d(16), nn.ReLU(inplace=True),
        )                                     # 160
        self.b1 = dw(16, 32, 2)               # 80
        self.b2 = dw(32, 64, 2)               # 40  <- stride 8
        self.b3 = dw(64, 64)
        self.down = dw(64, 96, 2)             # 20
        self.ctx = dw(96, 64)
        self.fuse = dw(64, 64)

        def head(out, bias=0.0):
            m = nn.Sequential(nn.Conv2d(64, 48, 3, 1, 1), nn.ReLU(inplace=True),
                              nn.Conv2d(48, out, 1))
            nn.init.constant_(m[-1].bias, bias)
            return m

        # -4.6 = logit(0.01): start pessimistic, or focal loss is swamped by the
        # ~1600 background cells per image at the first step.
        self.hm = head(nc, -4.6)
        self.wh = head(2)
        self.off = head(2)

    def forward(self, x):
        x = self.stem(x)
        x = self.b1(x)
        p = self.b3(self.b2(x))
        c = F.interpolate(self.ctx(self.down(p)), size=p.shape[-2:], mode="nearest")
        p = self.fuse(p + c)
        return self.hm(p), self.wh(p), self.off(p)


def focal(pred, gt):
    """CornerNet penalty-reduced focal loss on a gaussian target."""
    p = torch.clamp(torch.sigmoid(pred), 1e-4, 1 - 1e-4)
    pos = gt.eq(1).float()
    neg = 1.0 - pos
    pos_loss = -torch.log(p) * (1 - p) ** 2 * pos
    neg_loss = -torch.log(1 - p) * p ** 2 * (1 - gt) ** 4 * neg
    n = pos.sum()
    return (pos_loss.sum() + neg_loss.sum()) / torch.clamp(n, min=1.0)


def masked_l1(pred, gt, mask):
    m = mask.unsqueeze(1)
    return (torch.abs(pred - gt) * m).sum() / torch.clamp(m.sum() * pred.shape[1], min=1.0)


def decode(hm, wh, off, topk=10, thr=0.3):
    """sigmoid -> 3x3 peak suppression -> topk. No NMS needed: one peak = one object."""
    h = torch.sigmoid(hm)
    keep = (F.max_pool2d(h, 3, 1, 1) == h).float()
    h = h * keep
    B, C, H, W = h.shape
    # reshape, not view: after the peak mask the tensor can be non-contiguous,
    # and view() raises on it. Training batches happened to stay contiguous, so
    # this only surfaced at batch size 1 -- i.e. only at inference.
    scores, idx = h.reshape(B, -1).topk(topk, dim=1)
    cls = torch.div(idx, H * W, rounding_mode="floor")
    pix = idx % (H * W)
    ys, xs = torch.div(pix, W, rounding_mode="floor").float(), (pix % W).float()
    out = []
    for b in range(B):
        dets = []
        for k in range(topk):
            if scores[b, k] < thr:
                continue
            x, y = int(xs[b, k]), int(ys[b, k])
            ox, oy = off[b, 0, y, x].item(), off[b, 1, y, x].item()
            w, h_ = wh[b, 0, y, x].item(), wh[b, 1, y, x].item()
            cx, cy = (xs[b, k].item() + ox) / W, (ys[b, k].item() + oy) / H
            dets.append((int(cls[b, k]), scores[b, k].item(),
                         cx - w / 2, cy - h_ / 2, cx + w / 2, cy + h_ / 2))
        out.append(dets)
    return out


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


# ---------------------------------------------------------------- train / eval

def evaluate(model, dl, device, thr=0.3, iou_thr=0.5):
    """Per-class precision / recall / F1 at IoU 0.5. Deliberately not mAP:
    with 597 images and no negatives, a single number would hide more than it
    shows. Precision here is optimistic -- see the negatives note in README."""
    model.eval()
    tp = [0] * NUM_CLASSES
    fp = [0] * NUM_CLASSES
    fn = [0] * NUM_CLASSES
    with torch.no_grad():
        for x, hm, wh, off, mask in dl:
            dets = decode(*model(x.to(device)), thr=thr)
            for b in range(x.shape[0]):
                gt = []
                m = mask[b].nonzero()
                for yy, xx in m.tolist():
                    c = int(hm[b, :, yy, xx].argmax())
                    w, h_ = wh[b, 0, yy, xx].item(), wh[b, 1, yy, xx].item()
                    ox, oy = off[b, 0, yy, xx].item(), off[b, 1, yy, xx].item()
                    cx, cy = (xx + ox) / OUT, (yy + oy) / OUT
                    gt.append((c, cx - w / 2, cy - h_ / 2, cx + w / 2, cy + h_ / 2))
                used = set()
                for c, s, *box in sorted(dets[b], key=lambda d: -d[1]):
                    best, bi = 0.0, -1
                    for gi, g in enumerate(gt):
                        if gi in used or g[0] != c:
                            continue
                        v = iou(box, g[1:])
                        if v > best:
                            best, bi = v, gi
                    if best >= iou_thr:
                        tp[c] += 1
                        used.add(bi)
                    else:
                        fp[c] += 1
                for gi, g in enumerate(gt):
                    if gi not in used:
                        fn[g[0]] += 1
    rows, f1s = [], []
    for c in range(NUM_CLASSES):
        p = tp[c] / max(1, tp[c] + fp[c])
        r = tp[c] / max(1, tp[c] + fn[c])
        f = 2 * p * r / max(1e-9, p + r)
        f1s.append(f)
        rows.append(f"{CLASS_NAMES[c]:6s} P {p:.3f} R {r:.3f} F1 {f:.3f}  "
                    f"(tp {tp[c]} fp {fp[c]} fn {fn[c]})")
    return sum(f1s) / NUM_CLASSES, rows


def false_alarm_rate(model, dl, device, thr=0.3):
    """Detections per image on frames containing NO pillar.

    This is the metric that tracks the clutter failure mode. Precision on a
    dataset with zero negatives cannot see it at all, which is exactly why the
    original dataset could not have fixed that problem no matter how long you
    trained on it.
    """
    model.eval()
    n_img = n_det = n_any = 0
    with torch.no_grad():
        for batch in dl:
            x = batch[0].to(device)
            for dets in decode(*model(x), thr=thr):
                n_img += 1
                n_det += len(dets)
                n_any += 1 if dets else 0
    return n_det / max(1, n_img), n_any / max(1, n_img), n_img


def train(args):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    tr = torch.utils.data.DataLoader(
        PillarDS(os.path.join(args.splits, args.train_manifest), True),
        batch_size=args.bs, shuffle=True, num_workers=args.workers,
        drop_last=True, persistent_workers=args.workers > 0)
    va = torch.utils.data.DataLoader(
        PillarDS(os.path.join(args.splits, "val.txt"), False),
        batch_size=args.bs, shuffle=False, num_workers=args.workers)

    # diagnostics -- only loaded if build_aug.py has been run
    extra = {}
    for tag, fn in (("neg", "val_neg.txt"), ("cooc", "val_cooc.txt")):
        p = os.path.join(args.splits, fn)
        if os.path.exists(p):
            extra[tag] = torch.utils.data.DataLoader(
                PillarDS(p, False), batch_size=args.bs, shuffle=False)

    model = TinyPillar().to(device)
    n = sum(p.numel() for p in model.parameters())
    print(f"params {n:,}  ({n / 1_167_660:.2%} of nanodet_lite deploy graph)")
    print(f"train {len(tr.dataset)}  val {len(va.dataset)}  device {device}")

    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(
        opt, max_lr=args.lr, total_steps=args.epochs * len(tr), pct_start=0.25)
    os.makedirs(args.out, exist_ok=True)

    best = -1.0
    for ep in range(args.epochs):
        model.train()
        agg = np.zeros(3)
        for x, hm, wh, off, mask in tr:
            x, hm = x.to(device), hm.to(device)
            wh, off, mask = wh.to(device), off.to(device), mask.to(device)
            ph, pw, po = model(x)
            lh = focal(ph, hm)
            lw = masked_l1(pw, wh, mask)
            lo = masked_l1(po, off, mask)
            loss = lh + 5.0 * lw + 1.0 * lo
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            opt.step()
            sched.step()
            agg += [lh.item(), lw.item(), lo.item()]
        agg /= len(tr)
        line = f"ep {ep:3d}  hm {agg[0]:.3f} wh {agg[1]:.4f} off {agg[2]:.3f}"

        if (ep + 1) % args.eval_every == 0 or ep == args.epochs - 1:
            f1, rows = evaluate(model, va, device)
            print(line + f"  | macroF1 {f1:.3f}")
            for r in rows:
                print("        " + r)
            frac = 0.0
            if "neg" in extra:
                dpi, frac, n_i = false_alarm_rate(model, extra["neg"], device)
                print(f"        NEG   {dpi:.3f} false det/img, "
                      f"{frac:.1%} of {n_i} empty frames fire")
            if "cooc" in extra:
                cf1, crows = evaluate(model, extra["cooc"], device)
                print(f"        COOC  macroF1 {cf1:.3f}  (synthetic proxy only)")
            # Selecting on F1 alone picks checkpoints that fire on empty frames,
            # because val has no negatives for F1 to be penalised by. Price the
            # false-alarm rate into selection explicitly.
            score = f1 - args.neg_penalty * frac
            print(f"        SCORE {score:.3f}  (F1 - {args.neg_penalty}*neg_fire)")
            ck = {"model": model.state_dict(), "epoch": ep, "f1": f1,
                  "neg_fire": frac, "score": score, "classes": CLASS_NAMES,
                  "preproc": PREPROC, "input": INP}
            torch.save(ck, os.path.join(args.out, "last.pt"))  # every eval: power cuts
            if score > best:
                best = score
                torch.save(ck, os.path.join(args.out, "best.pt"))
        else:
            print(line)
    print(f"best macro F1 {best:.3f} -> {args.out}\\best.pt")


def export(args):
    model = TinyPillar()
    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    model.load_state_dict(ck["model"])
    model.eval()
    if ck.get("classes") != CLASS_NAMES:
        raise SystemExit(f"class order mismatch: ckpt {ck.get('classes')} "
                         f"vs file {CLASS_NAMES} -- refusing to export")
    torch.onnx.export(model, torch.randn(1, 3, INP, INP), args.out,
                      opset_version=11, input_names=["images"],
                      output_names=["hm", "wh", "off"],
                      do_constant_folding=True, dynamo=False)
    print(f"wrote {args.out}  (epoch {ck['epoch']}, val macroF1 {ck.get('f1', -1):.3f})")
    print("next: onnx2ncnn -> ncnnoptimize -> benchncnn ON THE PI 5")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["train", "eval", "export"])
    ap.add_argument("--splits", default="splits")
    ap.add_argument("--train-manifest", default="train_aug.txt",
                    help="train.txt for the un-augmented baseline")
    ap.add_argument("--no-preproc", action="store_true",
                    help="disable grey-world WB + CLAHE-on-V (ablation)")
    ap.add_argument("--out", default="runs")
    ap.add_argument("--ckpt", default="runs/best.pt")
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--bs", type=int, default=32)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--eval-every", type=int, default=5)
    ap.add_argument("--neg-penalty", type=float, default=0.25,
                    help="weight on empty-frame firing in checkpoint selection")
    a = ap.parse_args()
    PREPROC = not a.no_preproc
    globals()["PREPROC"] = PREPROC
    torch.manual_seed(0)
    random.seed(0)
    np.random.seed(0)
    if a.mode == "train":
        train(a)
    elif a.mode == "eval":
        dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        m = TinyPillar().to(dev)
        m.load_state_dict(torch.load(a.ckpt, map_location=dev,
                                     weights_only=False)["model"])
        dl = torch.utils.data.DataLoader(
            PillarDS(os.path.join(a.splits, "val.txt"), False), batch_size=a.bs)
        f1, rows = evaluate(m, dl, dev)
        print(f"macroF1 {f1:.3f}")
        for r in rows:
            print("  " + r)
    else:
        a.out = "tiny_pillar320.onnx" if a.out == "runs" else a.out
        export(a)
