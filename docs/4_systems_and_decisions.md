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
| Motion compute | ESP32, separate from perception | Steering must keep running at its design rate (~50 Hz nominal) even if the vision process stalls |
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

### D7 - Parking: implement, not descope (2026-08-06)

The Obstacle state machine promised `PARK_SEARCH` / `PARALLEL_PARK` while the
controller carried neither, leaving an implement-or-descope decision open.
Decided: implement. The scoring table settles it - 1.8.2 pays 15 for a parallel
park, **1.8.3 pays 7 even for a partial or non-parallel park**, and 1.8.1 adds
7 for starting in the lot with a completed lap - so a crude, conservative
attempt strictly dominates a descope that forfeits up to 22 points to save
nothing but code. Constraints accepted with the decision: the runtime picker
excluded magenta by design, so bay detection was a new capability (a third
calibrated class, or TF-Luna geometry against the 20 mm limiters); and rule
9.24.7 ends the round on touching a limiter, which caps how aggressive the
manoeuvre may be.

**Update 2026-08-12 — first constraint closed.** The picker now carries a
calibrated magenta class and reports bay sightings as telemetry
([3 — Software](3_software.md), §3.1), validated at 76/85 on mat frames
(§4.2). The park controller is still unwritten: detection exists, behaviour
does not.

### D8 - Chassis: rebuild the frame after the wheel-shed failure (2026-08-06), resolved as a LEGO redesign (2026-08-09/10)

Driver: the first observed mechanical failure - under sustained drive the
LEGO-mounted wheels shed from their axles within seconds (1 - Mobility,
section 3). The fix chosen was frame-level rather than a retention patch at the
axle, on the reasoning that a frame flexing under drive torque is what works an
axle interface loose. **The option first written down was a fully 3D-printed
chassis** with proper hubs and axle retention.

**Outcome (2026-08-09/10): the printed rebuild was not needed and was cancelled.**
A ground-up redesign of the LEGO space-frame itself - structurally tighter and
more rigid than the first layout, and the same redesign recorded in
`models/README.md` - passed the sustained-drive test that killed the original:
**seconds to failure on 2026-08-06 became 28 s and 29 s of continuous driving
with no wheel loss on 2026-08-09, roughly six times the duration.** Every run
and every committed artifact since is on that frame.

Two things this decision is worth reading for. First, the frame-level diagnosis
was right and the *specific implementation* first chosen was wrong - fixing the
stiffness was what mattered, not the material, and the cheaper option turned out
to be sufficient. Second, one of D8's stated secondary payoffs does not
materialise: a LEGO answer produces a LEGO design file rather than a chassis
STL, so the CAD this repo commits is the design file plus the printed mounts the
frame carries, not a printed chassis. That payoff was claimed up front precisely
so that failing to deliver it would be visible here rather than quietly dropped.

## 3. Risk register

| ID | Risk | Likelihood | Impact | Mitigation | State |
|---|---|---|---|---|---|
| R1 | Class order silently inverted between `data.yaml`, `nanodet_lite/cfg.py` and `tools/tiny_pillar.py` — every steering decision flips | Low | **Critical** | Class list is `["green", "red"]` in all three files. The pass-side lookup is keyed on the class **name**, never the index, so a reordering cannot invert steering. The loader refuses to start if a checkpoint class list disagrees with the config. | **Closed by design** |
| R2 | Venue lighting differs from the capture session | **High** | High | **Now quantified (2026-08-08):** across our two acquisition sessions the fitted Lab (a,b) centre for red moves **24.0** units while the picker's tolerance is only 12–15, so a calibration from one session cannot cover the other — pooling the two saturates tolerance at the 15.00 cap and costs 3.2 % wrong-side calls, which condition-matched calibration reduces to 0.0 % (0 wrong in 33 committed calls). Hue-jitter augmentation stays disabled (hue *is* the label). **Procedural mitigation: calibrate at the venue in the venue's light during check time, save to `calib.json`, run headless from it; never reuse a calibration across lighting.** Raw: `docs/eval_raw/picker_eval_summary.txt`. | Mitigated by procedure, not closed |
| R3 | Magenta parking-zone walls sit between red and green in hue and are misread as pillars | Medium | Medium | Two independent mitigations. Superseded detector: magenta surfaces deliberately included among the mined background negatives. Current picker (2026-08-12): magenta is its own calibrated class, and the measured red↔magenta separation in (a,b) is 46.8–55.1 against tolerances of 15 and 22 — the misread is out of reach rather than merely trained against. | Mitigated (measured, single session) |
| R4 | Dataset is 597 images from **one** lighting session with zero venue clutter | **Certain** | High | Stated openly wherever a number is quoted. A second capture session under different lighting, with clutter present, is the highest-priority data task. | **Open** |
| R5 | Pi 5 thermal throttling degrades inference latency during a sustained run | Medium | High | `benchncnn` must capture temperature and sustained clock in the same pass, not just peak throughput. | **Open — not yet measured** |
| R6 | `nanodet_lite` too slow on Pi 5 CPU | Medium | High | `tiny_pillar` (111 K params) retained as a drop-in lighter fallback; the pass-side interface is identical. | **Open — gated on R5** |
| R7 | Detector stalls and takes the steering loop down with it | Low | **Critical** | Perception (Pi 5) and motion (ESP32) sit on separate processors. Steering holds its last heading target and keeps correcting if vision stops. | Closed by architecture |
| R8 | TF-Luna address collision — all three units answer at `0x10` | Certain | Medium | PCA9548A 8-channel I2C multiplexer at `0x70`; channels 0/1/2 carry left/centre/right, channel 4 carries the BNO055. | Closed |
| R9 | Documented numbers drift away from the data as the model is retrained | Medium | Medium | Raw output for every quoted table is now committed under `docs/eval_raw/`, each file the unedited stdout/JSON of a named command. The picker harness re-derives its table from `round2.py` itself and records that file's SHA-256. On 2026-08-08 the NanoDet tables were regenerated from the committed checkpoints and every sweep row matched the transcribed values exactly. | **Mitigated — raw artifacts committed and verified 2026-08-08** |
| R10 | ~~No drive motor selected; propulsion is unbuilt~~ **Superseded 2026-08-06:** propulsion is built — N20 through a TB6612 into a spur pinion and rear-axle differential, integrated in `406e315`. The live risk is now **wheel retention**: the LEGO wheels shed within ~5 s of sustained drive. | ~~Certain~~ **Low (2026-08-10)** | **Critical** | ~~Chassis rebuild to a 3D-printed frame is in fabrication~~ **Superseded 2026-08-10: the rebuild is cancelled — this chassis is final.** How retention was resolved on this frame is not recorded; what is on record is that it now holds: the two 2026-08-09 Round-1 runs are 28 s and 29 s of continuous driving with no wheel loss ([`other/test-runs-2026-08-09-round1/`](../other/test-runs-2026-08-09-round1/)), roughly six times the failure duration observed on 2026-08-06. | **Mitigated by evidence — a full three-lap run and the wall-strike stall case remain outstanding** |

---

## 4. Iteration cycles

| # | Cycle | What changed | Evidence that forced it |
|---|---|---|---|
| 1 | HSV v1 -> v4.1 | Dual classifier with startup auto-select; morphological opening removed | 0.43 ms/frame; connected-components cost was dominated by the opening step |
| 2 | HSV -> learned detector | Classical pipeline demoted to diagnostic | HSV failed in both directions, voiding the ROI-verifier fallback (D1) |
| 3 | Split rebuilt | Random split -> group-wise split by capture bucket, 473/124 | 25 duplicate stems, 27/29 contaminated val buckets (D5) |
| 4 | Model selected | `tiny_pillar` -> `nanodet_lite` | Six of seven metrics on the clean split (D2) |
| 5 | Operating point set | Confidence threshold fixed at 0.45 from a sweep | See [3 — Software Architecture](3_software.md) |
| 6 | Learned detector -> calibrated-Lab picker | Neural stack superseded; picker became the stack of record | NanoDet won the val split but scored under 42 % on real field footage, and YOLO collapsed to ~0.3 fps under concurrent load |
| 7 | **Calibration procedure changed** | Calibration is now fitted **per venue, in the venue's own light**, saved to `calib.json` and reused headless for the run — never pooled across lighting conditions | Measured 2026-08-08: red's fitted Lab (a,b) centre moves **24.0** between our two capture sessions against a tolerance of 12–15, so pooling saturates the tolerance cap. Condition-matched calibration took wrong-side calls from 3.2 % to **0.0 %** (0 of 33 committed) and colour accuracy from 76.5 % to 90.3 %. Raw: `docs/eval_raw/` |

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
