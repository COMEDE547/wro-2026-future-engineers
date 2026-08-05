# 4 — Systems Thinking & Engineering Decisions

How the vehicle got to its current state: the constraints it must satisfy, the
decisions taken, the alternatives that were built and then rejected, and the
risks that remain open.

Every figure quoted here is measured. Where a decision was later reversed, the
original reasoning is kept rather than deleted — the reversal is the useful part.

---

## 1. Design constraints

### Imposed by the rules

| Constraint | Value | Source |
|---|---|---|
| Vehicle envelope | 300 x 200 x 300 mm | WRO FE General Rules |
| Vehicle mass | <= 1.5 kg | WRO FE General Rules |
| Steering | one steered axle, Ackermann — no differential-drive skid steering | WRO FE General Rules |
| Autonomy | fully autonomous from start; no external control or intervention | WRO FE General Rules |
| Pass rule | red pillar -> pass on the **right**, green pillar -> pass on the **left** | General Rules §13 |
| Traffic-sign geometry | 50 x 50 x 100 mm pillars | Game description |
| Official sign colours | red (238, 39, 55) · green (68, 214, 44) · magenta (255, 0, 255) | Game description |

### Self-imposed

| Constraint | Value | Why |
|---|---|---|
| Perception compute | Raspberry Pi 5, CPU only | No accelerator in budget; forces a model small enough to run without one |
| Deploy model size | <= 5 MB | Keeps the ONNX in git rather than in a release, so the exact deployed weights are version-controlled |
| Motion compute | ESP32, separate from perception | Steering must keep running at ~50 Hz even if the vision process stalls |
| Wrong-side calls | minimised in preference to no-calls | A *no call* means hold course and is recoverable. A *wrong side* is scored against you. The two errors are not symmetric and the operating point is chosen accordingly. |
| Every number in this repo | reproducible from a committed script | Prevents documentation drifting away from the data it describes |

---

## 2. Decision log

### D1 — Neural pillar detector: deferred, then adopted the same day

**Context.** The Obstacle Challenge needs the nearest pillar identified and
classified red/green. A classical HSV + connected-components pipeline
(`src/Round 2/vision/pillar_fast.py`, v4.1) already did this at **0.43 ms/frame**.

**First decision — defer the neural detector.** Nothing beat HSV on speed, and a
learned model on a small dataset looked like the higher-risk option. The intended
fallback if HSV proved fragile was an *HSV-ROI proposal + small CNN verifier*:
keep the fast classical stage, add a learned second opinion.

**Reversal, same day.** Measurement showed HSV failing in **both** directions —
missed pillars as well as false positives. That single fact voids the verifier
architecture: a verifier sits **downstream** of HSV proposals, so it can suppress
a false positive but can never recover a pillar HSV never proposed. The fallback
was not a fallback. The neural detector became the primary path.

**Consequence.** `pillar_fast.py` is retained as a diagnostic and as a
zero-dependency sanity check, not as the competition detector.

**What this cost.** Roughly half a day. The decision was made on the wrong failure
model and corrected as soon as the failure was decomposed by direction rather
than by rate.

---

### D2 — `tiny_pillar` (111 K params) rejected in favour of `nanodet_lite` (1.17 M params)

Both were built and both were trained. `tiny_pillar` is a from-scratch detector
kept in the repo at `src/Round 2/detector/tools/tiny_pillar.py`.

Scored on the leakage-free, group-wise validation split (124 real images):

| Metric | `tiny_pillar` (111 K) | `nanodet_lite` (1.17 M) | Winner |
|---|---|---|---|
| val macro F1 | 0.834 | **0.898** | nanodet |
| pass-side decision accuracy | 0.879 | **0.941** | nanodet |
| wrong side | **2.4 %** | 5.1 % | **tiny** |
| no call | 9.7 % | **0.8 %** | nanodet |
| false detections per empty frame | 0.283 | **0.083** | nanodet |
| decision accuracy, both pillars in frame | 0.617 | **0.867** | nanodet |
| wrong side, both pillars in frame | 31.7 % | **18.3 %** | nanodet |

**`nanodet_lite` loses one row and it is not a trivial one.** `tiny_pillar` makes
fewer wrong-side calls (2.4 % vs 5.1 %) — but it buys that by refusing to call at
all 9.7 % of the time against nanodet 0.8 %. Its caution is abstention, not
accuracy, and abstention on the track means driving past a pillar without acting
on it.

**Decision:** ship `nanodet_lite`. The deciding row is the last one. Both pillars
visible at once is the normal condition on the track, and wrong-side calls in
that condition drop by 42 % relative.

`tiny_pillar` stays in the repo as a lighter fallback if Pi 5 latency for
`nanodet_lite` turns out to be unacceptable — see risk R6.

---

### D3 — HSV confidence rescoring: researched, quantified, rejected

**Proposal.** Re-weight detector confidences using HSV colour agreement, to
recover accuracy cheaply without retraining.

**Why it was rejected — a ceiling calculation, not an opinion.** The remaining
error was decomposed by stage:

| Failure stage | Share of remaining error |
|---|---|
| Detection — nearest pillar never proposed | **30.0 %** |
| Selection — wrong pillar chosen as nearest | 1.7 % |
| Classification — right pillar, wrong colour | **0.0 %** |

Colour classification error is already **zero**. HSV rescoring acts only on boxes
the detector has already proposed, so it cannot address the 30 % that dominates,
and it cannot improve a term already at its floor. Ceiling on the whole idea:
**<= 1 point of decision accuracy.**

**Kept as:** a cheap veto — an HSV disagreement may suppress a detection, which is
a legitimate use — never as the fix for the accuracy gap.

**Generalisation.** Decompose the error before choosing the remedy. The intuitive
fix targeted the term that was already solved.

---

### D4 — A prediction we made twice and got wrong twice

**Prediction.** A 1.17 M-parameter model will overfit 597 training images and lose
to a 111 K-parameter model. Argued twice, on parameter-count-versus-dataset-size
grounds.

**Outcome.** Falsified. `nanodet_lite` won six of seven metrics (D2).

**Mechanism, identified after the fact.** NanoDet-Plus trains with an **auxiliary
head** that is discarded at inference. It is a second supervision signal — a
regulariser aimed at precisely the low-data regime the prediction said would break
it. The parameter count that drove the prediction was a count of the *deploy*
graph and never described the *training* dynamics.

**Rule adopted:** judge an architecture by its training mechanism, not by its
parameter count. This is why `export_nanodet.py` explicitly drops the auxiliary
head at export — the training graph and the deploy graph are different objects,
and the documentation now names which one every figure refers to.

---

### D5 — Data leakage audit: our own validation numbers invalidated and rebuilt

**Trigger.** Validation accuracy looked better than the qualitative behaviour of
the detector on new frames.

**Finding.** The train/val split was contaminated:

- **25 duplicate image stems** across the split boundary.
- **27 of 29 validation second-buckets** shared a bucket with training images.
  Frames captured within the same second of a continuous recording are
  near-identical; splitting them at random puts a frame in val whose neighbour is
  in train.

The validation set was measuring memorisation, not generalisation.

**Action.** Every previously reported number was **withdrawn as invalid**, not
adjusted. `tools/make_split.py` was written to split **group-wise by capture
bucket** rather than by frame, producing a clean **473 / 124** split. Both models
were retrained and re-measured from scratch. Every figure in this repository comes
from the rebuilt split.

**Cost.** All prior results discarded. **Value:** every number that survived is now
trustworthy, and the split is reproducible by a committed script rather than by a
one-off shuffle.

---

### D6 — Neural detector superseded in the field; calibrated per-venue picker adopted

(2026-08-05.) The val-split winner did not transfer. `nanodet_lite` won six of
seven metrics on the leakage-free split, and the write-up said at the time that
every number rested on 597 images from one lighting session with zero venue
clutter. Field testing observed exactly that failure mode: accuracy on real
footage fell below usable (quantitative capture pending). The alternatives
failed on their own axes — a fixed-band HSV pipeline degraded under lighting and
brightness shifts, and YOLO26n failed under concurrent runtime load (suspected
OOM; kernel-log capture pending). The stack that survived reverses the losing
philosophy on both axes: **calibrated per-venue** instead of fixed
published-value bands, and **classical Lab chroma-distance** instead of learned
features. One (a,b) chroma disc per colour, sampled interactively at the venue
(median + MAD tolerance, capped), an L floor and a chroma gate, largest
connected component, 3-of-5 temporal vote. Its own sub-iterations are on
record: a 3-disc / 6-bucket brightness variant missed between levels, and a
brightness-ordered capsule chain produced fewer detections with degenerate
boxes — the single-disc form is the one that passed hardware testing. Next
measurement: the current stack's accuracy on the same footage and ms/frame on
the Pi 5, so this entry can carry the numbers its predecessors did.

## 3. Risk register

| ID | Risk | Likelihood | Impact | Mitigation | State |
|---|---|---|---|---|---|
| R1 | Class order silently inverted between `data.yaml`, `nanodet_lite/cfg.py` and `tools/tiny_pillar.py` — every steering decision flips | Low | **Critical** | Class list is `["green", "red"]` in all three files. The pass-side lookup is keyed on the class **name**, never the index, so a reordering cannot invert steering. The loader refuses to start if a checkpoint class list disagrees with the config. | **Closed by design** |
| R2 | Venue lighting differs from the capture session | **High** | High | Hue-jitter augmentation disabled (hue *is* the label); value, saturation, gamma and synthetic cast shadows used instead — those are what venue lighting actually varies. HSV bands calibrated against the official sign RGB values. | Mitigated, not closed |
| R3 | Magenta parking-zone walls sit between red and green in hue and are misread as pillars | Medium | Medium | Magenta surfaces deliberately included among the mined background negatives so the detector is trained to ignore them. | Mitigated |
| R4 | Dataset is 597 images from **one** lighting session with zero venue clutter | **Certain** | High | Stated openly wherever a number is quoted. A second capture session under different lighting, with clutter present, is the highest-priority data task. | **Open** |
| R5 | Pi 5 thermal throttling degrades inference latency during a sustained run | Medium | High | `benchncnn` must capture temperature and sustained clock in the same pass, not just peak throughput. | **Open — not yet measured** |
| R6 | `nanodet_lite` too slow on Pi 5 CPU | Medium | High | `tiny_pillar` (111 K params) retained as a drop-in lighter fallback; the pass-side interface is identical. | **Open — gated on R5** |
| R7 | Detector stalls and takes the steering loop down with it | Low | **Critical** | Perception (Pi 5) and motion (ESP32) sit on separate processors. Steering holds its last heading target and keeps correcting if vision stops. | Closed by architecture |
| R8 | TF-Luna address collision — all three units answer at `0x10` | Certain | Medium | PCA9548A 8-channel I2C multiplexer at `0x70`; channels 0/1/2 carry left/centre/right, channel 4 carries the BNO055. | Closed |
| R9 | Documented numbers drift away from the data as the model is retrained | Medium | Medium | Tables are currently hand-transcribed from test output. Planned: a generator under `docs/figures/` re-deriving every table from raw results. | **Open — generator not written** |
| R10 | No drive motor selected; propulsion is unbuilt | Certain | **Critical** | Nothing mitigates this. It is the critical-path hardware item. | **Open** |

---

## 4. Iteration cycles

| # | Cycle | What changed | Evidence that forced it |
|---|---|---|---|
| 1 | HSV v1 -> v4.1 | Dual classifier with startup auto-select; morphological opening removed | 0.43 ms/frame; connected-components cost was dominated by the opening step |
| 2 | HSV -> learned detector | Classical pipeline demoted to diagnostic | HSV failed in both directions, voiding the ROI-verifier fallback (D1) |
| 3 | Split rebuilt | Random split -> group-wise split by capture bucket, 473/124 | 25 duplicate stems, 27/29 contaminated val buckets (D5) |
| 4 | Model selected | `tiny_pillar` -> `nanodet_lite` | Six of seven metrics on the clean split (D2) |
| 5 | Operating point set | Confidence threshold fixed at 0.45 from a sweep | See [3 — Software Architecture](3_software.md) |

---

## 5. What we do not yet know

Stated here rather than left for a reader to discover.

- **No Raspberry Pi 5 latency figure exists for `nanodet_lite`.** The ~30 fps
  number in earlier notes is a *different* model (YOLO26s @ 224, 9.47 M params),
  and the 24.6 ms figure is fp32 on x86. Neither is a Pi number. No Pi latency
  will be quoted until `onnx2ncnn` -> `ncnnoptimize` -> `benchncnn` has been run
  on the target board.
- **Both-pillar figures are synthetic composites.** No real image in the dataset
  contains a red and a green pillar simultaneously, so those frames were
  composited. They are optimistic by construction and unreliable in both
  directions. Every such figure is labelled.
- **Every number rests on 597 images from a single lighting session** with no
  other robots, spectators, banners or reflective flooring present.
- **The Obstacle Challenge controller is implemented off-repo, not yet landed.**
  Strategy and state machine are in [3 — Software Architecture](3_software.md);
  the code lands after integration fixes.
- **The drivetrain is integrated in firmware but not built.** Motor and driver
  are chosen (N20 via TB6612); the working point and chassis are not. See
  [1 — Mobility](1_mobility.md).
