# WRO 2026 pillar detector — 3-class model v2 (2026-08-19)

Drop-in replacement for `pillar_3class_yolo26n_320.onnx`.

## Files
| file | use |
|---|---|
| `pillar_3class_yolo26n_320_v2.onnx` | **deploy this** — 9.19 MB, what the Pi runs |
| `pillar_3class_yolo26n_320_v2.pt` | PyTorch weights, 5.10 MB — for retraining/further export |
| `detect_3class.py` | unmodified standalone runner (expects the ONNX beside it as `best.onnx`) |
| `training_args.yaml` / `training_results.csv` | full recipe + per-epoch metrics |

## Class map — LOCKED
```
0 = green, 1 = red, 2 = magenta
```
**Identical to the deployed model** and to `detect_3class.py`. No remap needed.

## Interface
- input `images` `[1,3,320,320]`, RGB, `/255.0`, letterboxed (pad value 114)
- output `output0` `[1,300,6]` → squeeze → `(300,6)` = `x1,y1,x2,y2,conf,cls`
- **end-to-end / NMS-free** — same layout as the current deployed ONNX, so
  `detect_3class.py` runs it with zero changes.

## Numbers
Validation (715 held-out images, 977 instances, whole-video holdout):

| class | instances | mAP50 | mAP50-95 |
|---|---|---|---|
| green | 137 | 0.994 | 0.989 |
| red | 643 | 0.994 | 0.963 |
| magenta | 197 | 0.995 | 0.988 |
| **all** | 977 | **0.994** | **0.980** |

Blind hand-labelled gold set (30 frames / 41 boxes), conf 0.25:
**41/41 — 100 % recall, 100 % precision, 0 class swaps, 0 false positives.**
Median IoU 0.937 green / 0.927 red / 0.946 magenta. Holds at conf 0.40.

Trained on 2,873 frames / 3,501 boxes, yolo26n @ 320, 100 epochs, seed 0.
Reproducible: three runs bit-identical, 708/708 weight tensors, max abs diff 0.

## Caveats — read before trusting the numbers
1. **Gold is 41 boxes.** Zero errors bounds the true error rate at roughly 8.6 %,
   not zero. It is the best yardstick available, not a large one.
2. **Validation is a whole-video holdout of the same footage**, so 0.980 measures
   in-distribution performance. It is not a claim about unseen lighting or venues.
3. **Magenta comes from a single capture session** — its validation number is
   optimistic; treat it as the least-trustworthy of the three.
4. **At conf 0.10 the model emits duplicate boxes** for one object (precision
   ~90 %, still 0 FPs). This is learned, not algorithmic, NMS-free behaviour and
   it weakens out-of-domain. **Use conf 0.25 or 0.40**, and dedup by IoU if you
   ever consume raw output as labels.
5. **Not yet benchmarked on the Pi.** 0.3 ms/image on an RTX 4060 says nothing
   about Pi 5 throughput or thermals.

## Swap procedure
1. Back up the current `pillar_3class_yolo26n_320.onnx`.
2. Copy the v2 ONNX in (rename to whatever the runner loads — `detect_3class.py`
   expects `best.onnx` beside it).
3. Verify on a handful of mat photos **before** trusting it in a run:
   `python detect_3class.py photo1.jpg photo2.jpg`
4. Confirm the printed class names read green/red/magenta correctly. If the
   runner ever prints `UNEXPECTED output shape`, stop — the decode is wrong and
   every box is garbage.
