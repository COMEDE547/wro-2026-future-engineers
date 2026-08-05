# Figures

**Empty by design, and this is a tracked defect — risk R9.**

Every table in `docs/` is currently hand-transcribed from the output of
`eval_nanodet.py` and `decide_nanodet.py --sweep`. That means a retrain can move
a number in the data without moving it in the documentation, and nothing catches
the divergence.

What belongs here once written:

| Artifact | Source |
|---|---|
| Confidence-threshold sweep table and plot | `decide_nanodet.py --sweep` |
| `tiny_pillar` vs `nanodet_lite` comparison table | `eval_nanodet.py` on both checkpoints |
| Failure-decomposition bar chart | `eval_nanodet.py` stage breakdown |
| Split-integrity summary | `tools/make_split.py` |

One command should regenerate all of them, so no figure in this repository is
hand-exported and no number can drift from the data it came from.
