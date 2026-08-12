# Testing workflow

What is tested, by what command, and what result blocks a release.

Anything below marked **not yet run** is named as such. No result is quoted for a
test that has not been executed.

---

## 1. Test levels

| Level | Runs on | Needs hardware | Frequency |
|---|---|---|---|
| **T1** Data integrity | Desktop | No | Every time the dataset changes |
| **T2** Model evaluation | Desktop | No | Every training run |
| **T3** Operating-point sweep | Desktop | No | Every training run |
| **T4** Deploy-graph parity | Desktop | No | Every ONNX export |
| **T5** On-target latency + thermal | Raspberry Pi 5 | Yes | Before any Pi latency claim |
| **T6** Firmware bench | ESP32 + sensors | Yes | Every firmware change |
| **T7** Track | Full vehicle | Yes | **Unblocked 2026-08-08** — drive integrated; six recordings in `other/`; logged run-to-stop pending |

---

## 2. T1 — Data integrity

**Why it exists:** it was written *after* a leakage audit found the validation set
was measuring memorisation — 25 duplicate image stems across the split boundary
and 27 of 29 validation second-buckets shared with training. See
[D5](4_systems_and_decisions.md#d5--data-leakage-audit-our-own-validation-numbers-invalidated-and-rebuilt).

```bash
cd "src/Round 2/detector"
python tools/make_split.py --root <dataset>
```

**Pass criteria**

| Check | Requirement |
|---|---|
| Duplicate stems across split | 0 |
| Validation capture-buckets also present in train | 0 |
| Split sizes | 473 train / 124 val, group-wise by capture bucket |

**A failure here invalidates every downstream number.** It does not adjust them.

---

## 3. T2 — Model evaluation

```bash
python eval_nanodet.py
```

Reports, on the leakage-free validation split: macro F1, pass-side decision
accuracy, wrong-side rate, no-call rate, false detections per empty frame, and
the both-pillar subset.

**Pass criteria (current shipping model as the floor):** macro F1 >= 0.898,
decision accuracy >= 0.941. A candidate that does not clear the shipping model on
both does not ship.

**Label discipline:** the both-pillar rows are measured on **synthetic
composites** — no real image in the dataset contains a red and a green pillar at
once. Every report of those rows must say so.

---

## 4. T3 — Operating-point sweep

```bash
python decide_nanodet.py --sweep
```

Sweeps the confidence threshold over 0.30-0.55 and reports accuracy, wrong-side,
no-call, both-pillar wrong-side and false detections per empty frame at each
point. The threshold is set from this table, never by eye. Current operating
point and the argument for it: [3 — Software](3_software.md#operating-point-045-and-why).

**Pass criterion:** the chosen threshold must be justified in writing against at
least one competing threshold in the same table.

---

## 5. T4 — Deploy-graph parity

```bash
python export_nanodet.py
```

The export drops NanoDet-Plus's auxiliary training head. Training graph and
deploy graph are therefore different objects, which is exactly the confusion that
produced a falsified prediction once already
([D4](4_systems_and_decisions.md#d4--a-prediction-we-made-twice-and-got-wrong-twice)).

**Pass criteria**

| Check | Requirement |
|---|---|
| ONNX output shape | `(1, 2125, 34)` |
| Input | fixed 320 x 320, opset 11 |
| Class list | `["green", "red"]` — loader must refuse a mismatch |
| Decision agreement, ONNX vs PyTorch on the val split | identical pass-side call on every frame |

---

## 6. T5 — On-target latency and thermal — **not yet run**

```bash
onnx2ncnn pillar_nanodet320.onnx pillar.param pillar.bin
ncnnoptimize pillar.param pillar.bin pillar-opt.param pillar-opt.bin 0
benchncnn 20 4 0 -1 1
```

Latency and **temperature and sustained clock** are captured in the *same* run.
A peak-throughput number taken from a cold board does not describe a three-lap
run — that is risk R5.

**Pass criteria:** end-to-end perception latency compatible with the pass-decision
look-ahead of 327-567 mm at race speed, and no thermal throttling observed across
a 15-minute sustained run.

**Until this test has run, no Raspberry Pi 5 latency figure exists for this
model.** The ~30 fps number in earlier notes is a different model (YOLO26s @ 224,
9.47 M params); the 24.6 ms figure is fp32 on x86.

If T5 fails, the fallback is `tiny_pillar` at 111 K parameters — identical
pass-side interface, measured accuracy cost documented in
[D2](4_systems_and_decisions.md#d2--tiny_pillar-111-k-params-rejected-in-favour-of-nanodet_lite-117-m-params).

---

## 7. T6 — Firmware bench

| Test | Method | Pass criterion |
|---|---|---|
| I2C mux channel select | Read all three TF-Luna channels with only one sensor connected | Only the connected channel returns a plausible range |
| BNO055 calibration gate | Power-on, watch calibration status registers | Heading capture does not occur until status is calibrated |
| Servo travel | Command 45 / 90 / 135 deg | Mechanical travel matches, no binding at either limit |
| Heading hold | Rotate the chassis by hand, release | Servo counter-steers proportionally; saturates at 30 deg of error |
| Corner edge detection | Present and remove a wall in front of a side LiDAR | Target heading steps 90 deg exactly **once** per edge, not repeatedly |

The last one is the important one: it is the test that distinguishes a rising-edge
implementation from a level test, and a level test spins the vehicle.

---

## 8. T7 — Track — **unblocked 2026-08-08**

The drivetrain is integrated ([1 — Mobility](1_mobility.md)) and the vehicle
drives: six test recordings are committed under [`other/`](../other) — the
2026-08-08 pair, the 2026-08-09 speed pair, and the 2026-08-09 Open-Challenge
controlled pair. None yet captures a complete run through the autonomous stop,
and **no serial-telemetry log has ever been recorded** — the first logged
run-to-stop (`tools/serial_log.py`) is the release-blocking T7 result.

Still planned: repeated corner-exit runs measuring understeer at the chosen
speed, and a full three-lap Open Challenge run. The corner-exit test is the one
expected to change the mechanical design.

---

## 9. Regression policy

When the model changes, re-run **T1 -> T2 -> T3 -> T4 -> T5** in that order. T2
and T3 results are not comparable across different splits, so T1 must pass first.

Tables and plots in `docs/` are currently transcribed from test output by hand,
which is exactly how documentation drifts away from data. The mitigation — a
generator under `docs/figures/` that re-derives every table from the raw results
— is specified but **not yet written**. Risk R9 stays open until it is.
