# WRO Future Engineers 2026 — Engineering Materials

Engineering documentation for an autonomous vehicle built for the **World Robot
Olympiad Future Engineers** category, season **2026**.

> **Team:** [TEAM NAME — TBD] · **Members:** [TBD] · **Coach:** [TBD] · **Country:** India

---

## At a glance

| | |
|---|---|
| **Architecture** | Two processors. ESP32 runs sensing and steering at ~50 Hz; Raspberry Pi 5 runs pillar detection. A stalled detector cannot stall the steering loop. |
| **Steering** | Single-servo Ackermann, 45-135 deg travel, proportional on heading error at 1.5 servo-deg/deg — saturating at 30 deg of error |
| **Heading** | BNO055 absolute yaw. Corner detection is a *rising-edge* test on a side TF-Luna crossing 150 cm |
| **Range sensing** | 3x TF-Luna behind a PCA9548A I2C multiplexer (all three share address `0x10`) |
| **Pillar detection** | **Calibrated-Lab colour picker** — one (a,b) chroma disc per colour, sampled at the venue; the prior neural stack was superseded 2026-08-05 |
| **Why the neural stack lost** | `nanodet_lite` val decision accuracy **0.941** did not transfer to real footage — single-session, zero-clutter training data (a limit stated at training time, then observed) |
| **Validation** | 473 / 124 group-wise split, leakage-audited. 597 source images, **one lighting session**. |
| **Open Challenge** | Implemented |
| **Obstacle Challenge** | **Implemented off-repo** — controller + Pi runtime bench-tested; repo landing after integration fixes (radio removal, I2C map verification) |
| **Drivetrain** | **Built (Round-1 configuration)** — N20 via TB6612 through a LEGO differential; Ackermann steering; bring-up in progress. Working point (ratio, diameters, mass) unmeasured. |

---

## Documentation index

Organised against the five criteria WRO uses to score engineering documentation.

| # | Criterion | Document | State |
|---|---|---|---|
| 1 | Mobility & Mechanical Design | [`docs/1_mobility.md`](docs/1_mobility.md) | Steering + drive integration documented; **working point (ratio / wheels / chassis) unchosen** |
| 2 | Power & Sensor Architecture | [`docs/2_power_and_sensors.md`](docs/2_power_and_sensors.md) | Sensors and camera geometry documented; **power budget unmeasured** |
| 3 | Software Architecture & Obstacle Strategy | [`docs/3_software.md`](docs/3_software.md) | Open implemented (drives); Obstacle implemented off-repo, pending landing |
| 4 | Systems Thinking & Engineering Decisions | [`docs/4_systems_and_decisions.md`](docs/4_systems_and_decisions.md) | Decision log, rejected alternatives, risk register |
| 5 | Reproducibility & Repository Quality | [`docs/5_reproducibility.md`](docs/5_reproducibility.md) | Reproduction steps, licensing, versioning policy |
| — | Testing workflow | [`docs/tests.md`](docs/tests.md) | T1-T4 running; T5-T7 blocked on hardware |
| — | Detector reference | [`src/Round 2/detector/README.md`](src/Round%202/detector/README.md) | Results, operating point, reproduction |
| — | Firmware reference | [`src/README.md`](src/README.md) | Sketch-by-sketch |

**Start with [4 — Systems Thinking](docs/4_systems_and_decisions.md)** if you only
read one. It carries the decision log, including the alternatives that were built
and rejected and the numbers that killed them.

---

## Repository layout

```
docs/           five criterion documents + testing workflow + figure scripts
src/Round 1/    ESP32 firmware — heading hold, corner detection, steering
src/Round 2/    superseded detector (kept as evidence) + classical HSV pipeline
models/         CAD / printable parts        — empty, no chassis exists yet
schemes/        electromechanical schematics — signal wiring v0.1; power tree pending
t-photos/       team photos                  — pending
v-photos/       vehicle photos               — 6 views present (Round-1 config)
video/          performance video links      — pending a driving vehicle
other/          datasheets, setup notes
```

Empty directories are empty because the artifact does not exist yet, not because
it was overlooked. Each one says which.

---

## Quick start

**Run the superseded neural detector (kept runnable as iteration evidence):**

```bash
cd "src/Round 2/detector"
python decide_nanodet.py --camera 0 --window 5     # prints "left" or "right"
python decide_nanodet.py --image frame.jpg
python decide_nanodet.py --sweep                   # re-derive the operating point
```

**Reproduce every number in this repository from scratch:**
see [`docs/5_reproducibility.md`](docs/5_reproducibility.md#3-reproducing-the-detector-from-zero).

**Build and flash the firmware:** Arduino IDE or PlatformIO, board = ESP32.
Libraries: Adafruit BNO055, Adafruit Unified Sensor, Adafruit BusIO, ESP32Servo.

---

## Competition artifacts

| Artifact | Required | Status |
|---|---|---|
| 6 vehicle photos — every side, top and bottom | Yes | **Present** — `v-photos/`, Round-1 configuration; refreshed once the Pi 5 + camera are mounted |
| 2 team photos | Yes | **Pending** |
| Performance video, >= 30 s autonomous driving, one per challenge | Yes | **Pending** — requires a driving vehicle |
| Electromechanical schematic | Yes | **Partial** — signal wiring v0.1 in `schemes/`; power tree pending battery selection |
| Control software | Yes | Present, `src/` |
| CAD / printable parts | Yes | **Pending** — no chassis |

These are gated on hardware that does not exist yet. They are tracked here so the
gap is visible rather than discovered late.

---

## Version history

| Version | Date | Change | Problems it created or exposed |
|---|---|---|---|
| — | 2026-06-26 | Initial repo; ESP32 firmware — IMU heading hold, TF-Luna corner detection, ultrasonic prototype | Corner detection needed a rising-edge test; a level test re-fires and spins the vehicle |
| — | 2026-07-19 | HSV pillar pipeline v4.1, 0.43 ms/frame | Measured to fail in **both** directions, which voided the planned ROI-verifier fallback |
| — | 2026-07-26 | Trained NanoDet-Plus detector; vision moved under `Round 2` | Validation split found contaminated — 25 duplicate stems, 27/29 shared capture buckets. All prior numbers withdrawn and re-measured on a clean 473/124 split. |
| — | 2026-07-28 | Documentation restructured against the five scoring criteria | Root README had described the vehicle as camera-free after the detector had shipped |
| — | 2026-08-05 | Drive integrated (N20 / TB6612, append-only); calibrated-Lab picker declared the Round 2 stack; NanoDet superseded after field testing; signal-wiring schematic v0.1 added | The val-split winner did not survive real footage — the single-session dataset limit, stated at training time, was observed in practice |

Full reasoning for each entry: [`docs/4_systems_and_decisions.md`](docs/4_systems_and_decisions.md).

---

## Known limits

Stated here rather than left for a reader to find.

- **Every detector number rests on 597 images from a single lighting session**,
  with no other robots, spectators, banners or reflective flooring present. The
  figures are indicative, not a venue prediction — and field testing subsequently
  observed exactly this transfer failure (see the version history).
- **Both-pillar figures are synthetic composites.** No real image in the dataset
  contains a red and a green pillar at once.
- **No Raspberry Pi 5 latency figure exists for this model.** It will not be
  quoted until `benchncnn` has run on the target board.
- **The Obstacle Challenge controller is not yet in this repository.** It is
  implemented and bench-tested off-repo; it lands after integration fixes. Until
  then [`docs/3_software.md`](docs/3_software.md) is the specification of record.
- **The camera is a USB UVC webcam; the exact model is not yet identified**, so
  no field-of-view or angular-resolution figure is quoted.
- **The vehicle is built to Round-1 configuration only.** The Raspberry Pi 5 and
  camera are not yet mounted, and drive bring-up (direction, duty, corner tests)
  is unfinished.

---

## Licence

MIT — see [`LICENSE`](LICENSE). `src/Round 2/detector/nanodet_lite/` is a
derivative of [NanoDet](https://github.com/RangiLyu/nanodet) and is Apache-2.0;
its licence is at `src/Round 2/detector/LICENSE-nanodet-Apache-2.0`.
