"""Lightning-free NanoDet-Plus training loop, manifest-driven.

Upstream nanodet/trainer/task.py calls training_epoch_end /
validation_epoch_end, both removed in pytorch-lightning 2.0, and its
requirements pin torch<2.0. That pin applies to the TRAINER only -- the model
code is clean on torch 2.x, so this replaces the trainer.

    python -m nanodet_lite.train --epochs 120

Checkpoints every epoch: Mumbai power cuts.

NOTE ON PRETRAINING: this trains FROM SCRATCH unless --coco-ckpt is given.
Upstream COCO checkpoints are Google-Drive-only on a dormant repo. With 473
real training images, scratch training a 1.17M-param deploy graph is a genuine
handicap -- tiny_pillar (111K params, scratch) reached F1 0.834 on this same
split and is the number to beat.
"""

import argparse
import math
import os
import time

import torch
from torch.utils.data import DataLoader

from . import cfg as C
from .data.collate import collate_meta
from .data.dataset import ManifestDataset
from .data.transform.pipeline import Pipeline
from .model.arch.nanodet_plus import NanoDetPlus
from .model.weight_averager.ema import ExpMovingAverager

SPLITS = r"C:\Users\ANT PC\wro_vision\tiny\splits"


def build_model():
    return NanoDetPlus(
        backbone=C.MODEL["backbone"], fpn=C.MODEL["fpn"],
        head=C.MODEL["head"], aux_head=C.MODEL["aux_head"],
        detach_epoch=C.MODEL["detach_epoch"],
    )


def load_pretrained(model, path):
    raw = torch.load(path, map_location="cpu", weights_only=False)
    sd = raw.get("state_dict", raw)
    sd = {(k[6:] if k.startswith("model.") else k): v for k, v in sd.items()}
    own = model.state_dict()
    n = 0
    for k, v in sd.items():
        if k in own and own[k].shape == v.shape:
            own[k].copy_(v)
            n += 1
    model.load_state_dict(own)
    print(f"[pretrain] loaded {n} tensors from {os.path.basename(path)}")
    return model


def loaders():
    t = C.TRAIN
    tr = ManifestDataset(os.path.join(SPLITS, "train_aug.txt"), C.INPUT_SIZE,
                         Pipeline(C.TRAIN_PIPELINE, keep_ratio=False))
    va = ManifestDataset(os.path.join(SPLITS, "val.txt"), C.INPUT_SIZE,
                         Pipeline(C.VAL_PIPELINE, keep_ratio=False))
    dl = DataLoader(tr, batch_size=t["batch_size"], shuffle=True,
                    num_workers=t["num_workers"], collate_fn=collate_meta,
                    drop_last=True, pin_memory=True,
                    persistent_workers=t["num_workers"] > 0)
    dv = DataLoader(va, batch_size=t["batch_size"], shuffle=False,
                    num_workers=t["num_workers"], collate_fn=collate_meta,
                    pin_memory=True)
    return dl, dv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=C.TRAIN["epochs"])
    ap.add_argument("--coco-ckpt", default=None)
    ap.add_argument("--resume", default=None)
    a = ap.parse_args()

    t = C.TRAIN
    os.makedirs(t["save_dir"], exist_ok=True)
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    model = build_model()
    if a.coco_ckpt:
        load_pretrained(model, a.coco_ckpt)
    else:
        print("[pretrain] NONE - training from scratch")
    model.to(dev)

    n_all = sum(p.numel() for p in model.parameters())
    n_aux = sum(p.numel() for n, p in model.named_parameters()
                if n.startswith(("aux_head", "aux_fpn")))
    print(f"params: train {n_all:,} | deploy {n_all - n_aux:,}")

    dl, dv = loaders()
    print(f"train {len(dl.dataset)} imgs / {len(dl)} batches | val {len(dv.dataset)}")

    opt = torch.optim.AdamW(model.parameters(), lr=t["lr"],
                            weight_decay=t["weight_decay"])
    total = a.epochs * len(dl)
    ema = ExpMovingAverager(decay=t["ema_decay"])
    ema.load_from(model)
    scaler = torch.amp.GradScaler("cuda", enabled=t["amp"] and dev.type == "cuda")

    ep0, gstep, best = 0, 0, 1e9
    if a.resume:
        ck = torch.load(a.resume, map_location="cpu", weights_only=False)
        model.load_state_dict(ck["model"]); opt.load_state_dict(ck["optim"])
        ep0, gstep = ck["epoch"] + 1, ck["gstep"]
        print(f"resumed at epoch {ep0}")

    for ep in range(ep0, a.epochs):
        model.train(); model.epoch = ep
        t0, agg = time.time(), {}
        for i, b in enumerate(dl):
            lr = (t["lr"] * (gstep + 1) / t["warmup_iters"]) if gstep < t["warmup_iters"] \
                else t["lr"] * 0.5 * (1 + math.cos(math.pi * (gstep - t["warmup_iters"])
                                                   / max(1, total - t["warmup_iters"])))
            for g in opt.param_groups:
                g["lr"] = lr
            b["img"] = b["img"].to(dev, non_blocking=True)
            opt.zero_grad(set_to_none=True)
            with torch.amp.autocast("cuda", enabled=scaler.is_enabled()):
                _, loss, st = model.forward_train(b)
            scaler.scale(loss).backward()
            scaler.unscale_(opt)
            torch.nn.utils.clip_grad_norm_(model.parameters(), t["grad_clip"])
            scaler.step(opt); scaler.update(); ema.update(model, gstep); gstep += 1
            for k, v in st.items():
                agg[k] = agg.get(k, 0.0) + float(v)
        line = " ".join(f"{k} {v/len(dl):.3f}" for k, v in agg.items())

        model.eval(); vt = vn = 0
        with torch.no_grad():
            for b in dv:
                b["img"] = b["img"].to(dev, non_blocking=True)
                model.epoch = ep
                _, vl, _ = model.forward_train(b)
                vt += float(vl); vn += 1
        v = vt / max(1, vn)
        print(f"ep {ep:3d} | {line} | VAL {v:.4f} | {time.time()-t0:.0f}s", flush=True)

        ck = {"model": model.state_dict(), "optim": opt.state_dict(),
              "epoch": ep, "gstep": gstep, "val": v, "classes": C.CLASS_NAMES}
        torch.save(ck, os.path.join(t["save_dir"], "last.pt"))
        if v < best:
            best = v
            torch.save(ck, os.path.join(t["save_dir"], "best.pt"))
    print(f"done. best val loss {best:.4f} -> {t['save_dir']}")


if __name__ == "__main__":
    main()
