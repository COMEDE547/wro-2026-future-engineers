# Round 2 — development history

Earlier and parallel versions of the Round 2 software, kept because the path to
the current stack is part of the engineering record. Nothing here runs on the
robot; the shipped runtime is `../round2.py` with `../main.cpp` on the ESP32.

They fall into three lineages.

## 1. Colour-picker lineage — ancestors of the current stack

`ggg.py` · `object_det_new.py` · `team2_working.py` · `team3_working_round2.py`
· `team3_wro_round2.py`

All share the structure the shipped runtime still uses — `rgb_to_lab`,
`get_masks`, `extract_bounding_box`, `process_frame`, `sample_roi_lab`,
`calibrate_color`, `run_calibration_session` — so the current `round2.py` is the
end of this line rather than a rewrite. `object_det_new.py` is the largest (793
lines) and closest to the shipped version; `ggg.py` is the furthest back.

What changed between the last of these and the shipped runtime, and why, is
recorded in the patch commit `ef19e54` and in
[`docs/4_systems_and_decisions.md`](../../../docs/4_systems_and_decisions.md) —
state-gated serial protocol, wait-for-start, IMUPLUS instead of NDOF, gradient
visual steering, headless operation from a saved calibration.

## 2. Neural-detector lineage — the superseded path

`detect.py` · `detect_2026.py` · `detect_with_esp.py` · `capture.py`

ONNX inference runtimes for the trained NanoDet detector, `detect_with_esp.py`
being the one wired to the ESP32 serial protocol, `capture.py` the dataset
capture utility. This path was evaluated and dropped; the selection evidence and
the threshold sweep are in
[`docs/3_software.md`](../../../docs/3_software.md) with raw output under
[`docs/eval_raw/`](../../../docs/eval_raw).

## 3. Hybrid

`camera.py` — colour segmentation combined with a shape test
(`is_pillar_shaped`), a confidence score, gray-world white balance, and dodge
commands issued directly to the ESP32. The shape test is a false-positive
defence the shipped picker does not currently have, and the measured
false-alarm rate (0.35 per empty frame before the temporal vote, see
`docs/eval_raw/picker_eval_summary.txt`) is the weakest number in the current
stack — so this idea is a live candidate rather than a dead end.

## Why these are kept

Retaining the versions that did not work is deliberate. The failures they
represent — and the reasons each was replaced — are the actual content of the
iteration cycles in `docs/4_systems_and_decisions.md`. A repository that shows
only the final file hides the engineering.
