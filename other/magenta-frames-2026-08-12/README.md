# Magenta parking-marker frames — 2026-08-12

Source footage for the magenta measurements quoted in
[`docs/3_software.md`](../../docs/3_software.md) §3.1 and §4.2. Committed so the
numbers can be checked against the images that produced them, rather than taken
on trust.

## Provenance

Two phone videos of the improvised parking-lot markers on the OMOTEC practice
mat, shot by the team on 2026-08-12 (832×464, 60 fps). Every 20th frame was
extracted with [`cut_frames_2026-08-12.py`](../../src/Round%202/detector/tools/cut_frames_2026-08-12.py),
giving **85 frames** (`vidA_*` 39, `vidB_*` 46). One extra file,
`vidB_f00200_det.jpg`, is a detector overlay kept as a visual sanity check.

Labels are YOLO-format, **132 boxes across 85 frames**, class id `2`.
`classes.txt` preserves the project-wide order `green / red / magenta` — the
same order as `data.yaml`, so a config read against a different order inverts
every class. That ordering trap is why the file is committed alongside the
labels rather than assumed.

## What these frames measured

Run [`magenta_lab_stats.py`](../../src/Round%202/detector/tools/magenta_lab_stats.py)
over this folder to reproduce:

| Quantity | Value |
|---|---|
| Magenta Lab centre | a\* +55, b\* −16 (b\* MAD 2.0) |
| Separation from red in the (a,b) plane | **46.8 (stills) / 55.1 (video)** |
| Calibration tolerance cap | 15 (red↔green), 22 (magenta) |
| Magenta detection through full `process_frame` | **43/85 → 76/85** at tolerance 20 |

The separation figure is the point: at a tolerance of 15–22 against a measured
neighbour distance of 46.8–55.1, red/magenta confusion is **structurally
impossible on real footage**, not merely unlikely. That claim previously rested
on published RGB values; these frames are the measured form of it.

Magenta is also the only one of the three classes with **b\* < 0**, which the
picker uses as a free hard pre-gate (mirroring the `b* > 0` gate on red).

## Honest limits

- **One session, one venue, one phone.** Not the race camera, and not venue
  lighting.
- **Auto white balance drifted** between the two clips — measured a\* moved
  60 → 49 across 22 seconds. These frames are therefore good for tuning gates
  and bad as an absolute colour reference; the race procedure remains a live
  locked-camera calibration at the venue (key `3`).
- The **9 residual misses** are all close-approach frames where chroma
  physically collapses as the marker fills the frame. That range belongs to the
  forward TF-Luna by design, not to the colour picker. Distant and mid-range
  detection is 100%.
- Labels were auto-generated then reviewed, not hand-drawn from scratch.

## Related

`src/Round 2/detector/` holds the 3-class model trained partly on this data
(`pillar_3class_yolo26n_320.onnx`, in-domain mAP50-95 green .859 / red .792 /
magenta .820) together with `wro_3class_train.ipynb`. That model is committed
**NOT DEPLOYED** — the calibrated-Lab picker remains the stack of record until
`benchncnn` and a concurrent-load test on the Pi 5 say otherwise.
