# Detector tools

Scripts behind the magenta measurements and the 3-class model. Committed so the
numbers in `docs/3_software.md` are reproducible rather than asserted.

| Script | What it does |
|---|---|
| `cut_frames_2026-08-12.py` | Extracts every 20th frame from the two mat videos → the 85 frames in `other/magenta-frames-2026-08-12/` |
| `magenta_lab_stats.py` | Measures the magenta Lab centre, MAD spread, and the (a,b) distance to the fitted red centre — produces the 46.8 / 55.1 separation figures |
| `magenta_label_setup.py` | Generates YOLO labels (class 2) from the measured gate, preserving the `green / red / magenta` class order |
| `augment_magenta.py` | 5 variants per image for training. **Hue is deliberately untouched** — hue *is* the class label here, so jittering it would teach the model the opposite of the task |
| `package_magenta_yolo.py` | Builds the train/val split. Val is `vidB` only and is never augmented — a whole-video holdout, so near-duplicate frames cannot leak across the split |

Two of these encode decisions worth stating plainly, because both are easy to
get wrong and expensive to debug later:

**Hue augmentation is off.** Standard colour-jitter pipelines would destroy the
only signal that separates the three classes.

**The split is by video, not by frame.** Frames sampled 20 apart from one clip
are near-duplicates; a random split would put siblings on both sides and report
memorisation as accuracy. This project has already been burned by exactly that
— the original 597-image red/green dataset shipped with 25 duplicate stems and
27 of 29 validation second-buckets shared with train, which is why every metric
measured before the group-wise rebuild is unusable.
