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
| **Pillar detector** | `nanodet_lite` — 1,167,660 params, 4.52 MB ONNX, 320x320, opset 11 |
| **Detector accuracy** | macro F1 **0.898** · pass-side decision accuracy **0.941** · wrong side **5.1 %** · no call **0.8 %** · **0.083** false detections per empty frame |
| **Validation** | 473 / 124 group-wise split, leakage-audited. 597 source images, **one lighting session**. |
| **Open Challenge** | Implemented |
| **Obstacle Challenge** | **Specified, not implemented** — state machine and strategy documented, code not written |
| **Drivetrain** | **Not chosen.** The vehicle steers but does not yet drive. Critical path. |

---

## Documentation index

Organised against the five criteria WRO uses to score engineering documentation.

| # | Criterion | Document | State |
|---|---|---|---|
| 1 | Mobility & Mechanical Design | [`docs/1_mobility.md`](docs/1_mobility.md) | Steering documented; **drivetrain unchosen** |
| 2 | Power & Sensor Architecture | [`docs/2_power_and_sensors.md`](docs/2_power_and_sensors.md) | Sensors and camera geometry documented; **power budget unmeasured** |
| 3 | Software Architecture & Obstacle Strategy | [`docs/3_software.md`](docs/3_software.md) | Open Challenge implemented; Obstacle Challenge specified |
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
src/Round 2/    detector (NanoDet-Plus derivative) + classical HSV pipeline
models/         CAD / printable parts        — empty, no chassis exists yet
schemes/        electromechanical schematics — pending
t-photos/       team photos                  — pending
v-photos/       vehicle photos               — pending a built vehicle
video/          performance video links      — pending a driving vehicle
other/          datasheets, setup notes
```

Empty directories are empty because the artifact does not exist yet, not because
it was overlooked. Each one says which.

---

## Quick start

**Run the pillar detector on a camera or an image:**

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
| 6 vehicle photos — every side, top and bottom | Yes | **Pending** — requires a built vehicle |
| 2 team photos | Yes | **Pending** |
| Performance video, >= 30 s autonomous driving, one per challenge | Yes | **Pending** — requires a driving vehicle |
| Electromechanical schematic | Yes | **Pending** — requires a fixed component list |
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

Full reasoning for each entry: [`docs/4_systems_and_decisions.md`](docs/4_systems_and_decisions.md).

---

## Known limits

Stated here rather than left for a reader to find.

- **Every detector number rests on 597 images from a single lighting session**,
  with no other robots, spectators, banners or reflective flooring present. The
  figures are indicative, not a venue prediction.
- **Both-pillar figures are synthetic composites.** No real image in the dataset
  contains a red and a green pillar at once.
- **No Raspberry Pi 5 latency figure exists for this model.** It will not be
  quoted until `benchncnn` has run on the target board.
- **The Obstacle Challenge is not implemented.** The strategy and state machine
  are specified in [`docs/3_software.md`](docs/3_software.md); the code is not
  written.
- **The camera module physically fitted is not yet fixed**, so no field-of-view
  or angular-resolution figure is quoted.
- **There is no drivetrain.** The vehicle holds a heading; it does not drive.

---

## Licence

MIT — see [`LICENSE`](LICENSE). `src/Round 2/detector/nanodet_lite/` is a
derivative of [NanoDet](https://github.com/RangiLyu/nanodet) and is Apache-2.0;
its licence is at `src/Round 2/detector/LICENSE-nanodet-Apache-2.0`.
