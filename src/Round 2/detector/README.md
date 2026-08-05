# Round 2 — Obstacle Challenge pillar detector

Detects the red and green traffic-sign pillars and outputs the pass side:
**red → `right`, green → `left`** (WRO Future Engineers General Rules §13).

## Quick start

```bash
python decide_nanodet.py --camera 0 --window 5     # prints "left" or "right"
python decide_nanodet.py --image frame.jpg
python decide_nanodet.py --sweep                   # re-calibrate the threshold
```

Deploy model: `models/pillar_nanodet320.onnx` — 4.52 MB, 1,167,660 parameters,
fixed 320×320 input, opset 11. Output is `(1, 2125, 34)` = 2 class logits +
32 regression channels per anchor.

## Results

Measured on a leakage-free, group-wise validation split (124 real images).
`nanodet_lite` is the shipping model; `tiny_pillar` is a from-scratch baseline
kept for comparison and as a lighter fallback.

| metric | tiny_pillar (111 K params) | **nanodet_lite (1.17 M)** |
|---|---|---|
| val macro F1 | 0.834 | **0.898** |
| pass-side decision accuracy | 0.879 | **0.941** |
| wrong side | **2.4 %** | 5.1 % |
| no call | 9.7 % | **0.8 %** |
| false detections per empty frame | 0.283 | **0.083** |
| decision accuracy, both pillars in frame | 0.617 | **0.867** |
| wrong side, both pillars in frame | 31.7 % | **13.3 %** |

The last row is the important one: when a red and a green pillar are both in
view — the normal condition on the track — wrong-side calls more than halved.

### Operating point

Confidence threshold defaults to **0.45**, chosen from the sweep below rather
than by eye. A *no call* means hold course and is recoverable; a *wrong side*
is scored against you, so the two errors are not weighted equally.

| thr | val acc | wrong | no call | both-pillar wrong | false det/empty frame |
|---|---|---|---|---|---|
| 0.30 | 0.941 | 5.9 % | 0.0 % | 11.7 % | 0.350 |
| 0.35 | 0.932 | 5.9 % | 0.8 % | 13.3 % | 0.283 |
| **0.45** | **0.941** | **5.1 %** | 0.8 % | 18.3 % | **0.083** |
| 0.55 | 0.856 | **0.0 %** | 14.4 % | 20.0 % | 0.050 |

Lowering the threshold helps the both-pillar case (fewer missed near pillars)
but fills empty frames with phantom detections; raising it does the reverse.
`thr 0.55` produces zero wrong-side calls on real frames at the cost of holding
course 14.4 % of the time — use it if a spurious steer is worse than hesitation.

## Known limitations

- Trained on **597 images from a single lighting session** with no venue
  clutter — no other robots, spectators, banners or reflective floors. The
  numbers above are indicative, not a venue prediction.
- The both-pillar figures come from **synthetically composited** frames,
  because no real image in the dataset contains both colours at once.
- **Raspberry Pi 5 latency for this model is unmeasured.** Run
  `onnx2ncnn` → `ncnnoptimize` → `benchncnn` on the Pi before relying on it.
- Remaining errors are dominated by *missed* detections of the nearest pillar,
  not by colour confusion (colour classification error is 0.0 %).

## Layout

| Path | Purpose |
|---|---|
| `models/` | exported ONNX (deploy) + the small baseline checkpoint |
| `nanodet_lite/` | NanoDet-Plus stripped to 45 files — no PyTorch Lightning, yacs, omegaconf, pycocotools or YAML |
| `nanodet_extra/` | hand-written replacements: config, training loop, manifest dataset, collate |
| `strip.py` | rebuilds `nanodet_lite` from an upstream clone plus `nanodet_extra` |
| `eval_nanodet.py` | scores a checkpoint on all four metrics above |
| `decide_nanodet.py` | pass-side output and threshold sweep |
| `export_nanodet.py` | deploy-graph ONNX export (drops the auxiliary head) |
| `tools/make_split.py` | builds the leakage-free group-wise train/val split |
| `tools/build_aug.py` | mines background negatives and composites both-pillar frames |
| `tools/tiny_pillar.py` | the 111 K-parameter baseline detector |

The 48 MB training checkpoint is **not** committed — it is reproducible from
`strip.py` + `nanodet_extra/train.py`, and belongs in a release rather than in
git history.

## Reproducing

```bash
git clone --depth 1 https://github.com/RangiLyu/nanodet.git nanodet_src
python strip.py                                  # -> nanodet_lite/
python tools/make_split.py --root <dataset>      # -> splits/
python tools/build_aug.py --splits splits        # negatives + both-pillar frames
python -m nanodet_lite.train --epochs 120
python export_nanodet.py
```

Split manifests are not committed because they hold absolute local paths;
regenerate them with `make_split.py`.

## Design notes

- **Class order is `["green", "red"]` — 0 = green, 1 = red**, matching the
  dataset's `data.yaml`. The pass-side lookup is keyed on the class *name*, not
  the index, so reordering the list cannot silently invert every steering
  decision. The loader refuses to run if a checkpoint's class list disagrees
  with the config.
- **Hue is the class label**, so hue-jitter augmentation is disabled. Value,
  saturation, gamma and synthetic cast shadows are used instead — those are
  what venue lighting actually varies.
- Images are **letterboxed, never stretched**: the dataset mixes 4:3 and 16:9
  sources, and stretching would let the model infer the class from aspect ratio.
- The nearest pillar is chosen by **lowest box bottom edge**, which survives
  occlusion better than box height, since clipping eats the top of a pillar
  while its base stays put.
- Magenta is deliberately present among the mined negatives so the detector
  learns to ignore the parking-zone walls.

## Attribution

`nanodet_lite/` is a derivative of [NanoDet](https://github.com/RangiLyu/nanodet)
by RangiLyu, used under the Apache License 2.0 — see
`LICENSE-nanodet-Apache-2.0`. Upstream file paths are preserved so future
upstream changes remain applicable. Modifications: registry builders replaced
with direct imports; the PyTorch Lightning trainer replaced with a plain
training loop (upstream calls `training_epoch_end`, removed in Lightning 2.0);
`data/collate.py` rewritten (upstream imports `torch._six`, removed in PyTorch
1.9); the COCO dataset and evaluator replaced with a manifest-driven YOLO-txt
loader; and an unused visualisation import removed from the head.
