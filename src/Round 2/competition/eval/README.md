# Local evaluation — committed competition weights

**What this is:** per-class detection metrics for the weights committed in this
folder (`best_ncnn.onnx` = a byte copy of
`../detector/models/pillar_3class_yolo26n_320_v2.onnx` / evaluated via the
paired `.pt`), measured locally so the numbers trace to a committed command and
raw outputs rather than to a training-notebook screenshot.

**Command (Ultralytics 8.4.122, torch 2.6.0+cu124, RTX 4060, 2026-08-24):**

```
yolo val model="src/Round 2/detector/models/pillar_3class_yolo26n_320_v2.pt" \
  data=<wro_dataset_2985>/data.yaml imgsz=320 split=val
```

Dataset: `wro_dataset_2985` (2,873 shipped frames, folder-scoped split;
class order locked `0=green 1=red 2=magenta` in its `data.yaml`).
Val split: **715 images / 977 instances**.

| Class | Images | Instances | P | R | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| all | 715 | 977 | 0.984 | 0.985 | 0.994 | 0.980 |
| green | 137 | 137 | 0.977 | 0.985 | 0.994 | 0.989 |
| red | 373 | 643 | 0.986 | 0.978 | 0.994 | 0.964 |
| magenta | 197 | 197 | 0.990 | 0.992 | 0.995 | 0.988 |

Speed at 320 px on the desktop GPU: 1.2 ms inference/image (Pi 5 CPU delivery
rate is separate and measured on-device).

**Read the numbers with the split in mind.** The Engineering Journal (§09)
quotes the more conservative per-class mAP50-95 figures from the original
training run on the earlier leakage-fixed 473/124 split (green 0.859 /
red 0.792 / magenta 0.820). The table above is a different, larger, held-out
split from the expanded dataset; frame-sequence datasets reward whichever split
is easier, which is exactly why this team withdrew a set of results once
already after a leakage audit (journal §09). Both numbers are reported, with
their splits, rather than the prettier one alone. For the pass decision only
green/red matter; the runtime's 2-class map decodes them at indices 0/1 and
ignores magenta.

Committed artifacts: `confusion_matrix_normalized.png`, `BoxPR_curve.png`.
