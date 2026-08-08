# Raw evaluation output

Every performance number quoted in `docs/` and in `src/Round 2/detector/README.md`
is transcribed from a file in this folder. Nothing here is edited by hand: each
file is the unmodified stdout or JSON of the command named beside it. This folder
exists to close risk **R9** in `docs/4_systems_and_decisions.md` ("documented
numbers drift away from the data"), which was open because tables were
transcribed from console output that was never committed.

| File | Produced by | Covers |
|---|---|---|
| `picker_eval_summary.txt` / `.json` | `python "src/Round 2/eval_picker.py"` | The **stack of record** - the calibrated-Lab colour picker |
| `picker_eval_frames.csv` | same | Per-frame record, one row per image (368 rows) |
| `picker_calibration.json` | same | Fitted calibration + colour separations |
| `nanodet_sweep_raw.txt` | `python decide_nanodet.py --sweep` | Operating-point sweep for the superseded NanoDet path |
| `nanodet_eval_raw.txt` | `python eval_nanodet.py` | NanoDet val/neg/co-occurrence metrics at its own default threshold |

## Reproducing

The picker harness needs only the dataset manifests and `round2.py`:

```
python "src/Round 2/eval_picker.py" --splits <path>/tiny/splits --out docs/eval_raw
```

It imports the detection functions from `round2.py` verbatim rather than
reimplementing them, so what is measured is the code that runs on the robot.
`round2.py` SHA-256 is recorded in the summary; if the runtime changes, rerun.

## Verification performed 2026-08-08

The two NanoDet raw files were regenerated from the committed checkpoints and
**every row of the threshold-sweep table in `docs/3_software.md` and in the
detector README matches the regenerated output exactly** (thr 0.45 = 0.941
accuracy / 5.1 % wrong-side / 18.3 % co-occurrence / 0.083 false detections per
empty frame). The hand-transcribed tables were correct.

One reading note that previously caused confusion inside this repo: the two
NanoDet files are taken at **different thresholds**. `eval_nanodet.py` runs at
its own default of 0.35 (0.932 / 13.3 %); the deployed operating point is 0.45
(0.941 / 18.3 %), which appears in the sweep. Rows from the two must never be
mixed in one table.
