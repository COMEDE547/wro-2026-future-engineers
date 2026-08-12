#!/usr/bin/env python3
"""
eval_picker.py - offline evaluation of the SHIPPED calibrated-Lab colour picker.

WHY THIS EXISTS
    docs/3_software.md declares the picker as the stack of record but carried no
    measured numbers of its own; the only metrics in the repo belonged to the
    superseded NanoDet path. This harness measures the picker itself.

METHOD (important - read before quoting any number)
    * It imports the detection functions from round2.py VERBATIM (rgb_to_lab,
      get_masks, extract_bounding_box, process_frame, calibrate_color,
      resize_frame) instead of reimplementing them, so what is measured is the
      code that runs on the robot. Hardware-only imports (serial, av) are stubbed
      because they are not reachable from the vision path.
    * Calibration is fitted on the TRAIN split only, using the same
      calibrate_color() estimator the interactive session uses, sampled from the
      central 50% of each labelled train box. Evaluation runs on the held-out
      val manifests. Split provenance: tiny/splits/*.txt (group-wise,
      leakage-free; see tools/make_split.py).
    * Frames are fed RGB at 240x240 via the shipped resize_frame(), matching the
      runtime path exactly (including its known aspect distortion).

WHAT THIS DOES NOT MEASURE
    * Venue performance. The dataset was shot on a different camera and lighting
      than the robot's Lenovo 300 FHD (team designation OMO/WCAM/11); the race procedure recalibrates on
      the day. These numbers validate the METHOD on labelled data, not the venue.
    * The 5-of-7 temporal vote. Still images cannot exercise it, so every
      false-alarm number here is a per-frame figure and therefore CONSERVATIVE
      relative to the runtime, which requires 5 hits in 7 frames.

Outputs (written to --out):
    picker_calibration.json    fitted calibration + provenance
    picker_eval_frames.csv     per-frame record, one row per image
    picker_eval_summary.json   aggregate metrics
    picker_eval_summary.txt    same, human-readable (this is the doc source)
"""
import argparse
import csv
import json
import os
import sys
import time
import types
import hashlib
from datetime import datetime, timezone

import numpy as np
import cv2

# --- import the shipped picker verbatim -------------------------------------
# round2.py imports serial/av at module scope for the hardware path. Neither is
# reachable from the vision functions, so they are stubbed to allow the import.
for _m in ("serial", "av"):
    if _m not in sys.modules:
        try:
            __import__(_m)
        except Exception:
            sys.modules[_m] = types.ModuleType(_m)

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import importlib.util

_spec = importlib.util.spec_from_file_location(
    "round2_shipped", os.path.join(_HERE, "round2.py"))
round2 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(round2)

process_frame = round2.process_frame
calibrate_color = round2.calibrate_color
resize_frame = round2.resize_frame
rgb_to_lab = round2.rgb_to_lab
MIN_SWERVE_HEIGHT = round2.MIN_SWERVE_HEIGHT
REVERSE_HEIGHT = round2.REVERSE_HEIGHT

# WRO 2026 General Rules 9.19: red pillar -> pass on its RIGHT, green -> LEFT.
PASS_SIDE = {"red": "right", "green": "left"}
# Dataset class order is fixed by data.yaml: 0=green, 1=red. Keyed by NAME
# downstream so a future reorder cannot silently invert every steering decision.
CLASS_NAMES = {0: "green", 1: "red"}
FRAME = 240


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def label_path(img_path):
    p = img_path.replace("\\", "/")
    if "/images/" in p:
        p = p.replace("/images/", "/labels/")
    return os.path.splitext(p)[0] + ".txt"


def read_labels(img_path):
    """Return [(class_name, xc, yc, w, h)] in normalised coords, or []."""
    lp = label_path(img_path)
    if not os.path.exists(lp):
        return []
    out = []
    with open(lp) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 5:
                continue
            cid = int(float(parts[0]))
            if cid not in CLASS_NAMES:
                continue
            out.append((CLASS_NAMES[cid],) + tuple(float(v) for v in parts[1:5]))
    return out


def load_rgb240(img_path):
    bgr = cv2.imread(img_path, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return resize_frame(rgb, FRAME, FRAME)


def nearest_gt(labels):
    """Nearest pillar = tallest labelled box, matching the runtime's
    taller-box-is-closer rule in round2.py."""
    if not labels:
        return None
    return max(labels, key=lambda t: t[4])


def fit_calibration(train_list, max_boxes_per_class=200):
    """Fit the shipped calibrate_color() on central-50% patches of train boxes."""
    patches = {"red": [], "green": []}
    used = {"red": 0, "green": 0}
    for img_path in train_list:
        if all(used[c] >= max_boxes_per_class for c in used):
            break
        labels = read_labels(img_path)
        if not labels:
            continue
        frame = load_rgb240(img_path)
        if frame is None:
            continue
        lab = rgb_to_lab(frame)
        for cname, xc, yc, w, h in labels:
            if used[cname] >= max_boxes_per_class:
                continue
            # central 50% of the box, to avoid edge bleed into wall/background
            x0 = int(max(0, (xc - w / 4) * FRAME))
            x1 = int(min(FRAME, (xc + w / 4) * FRAME))
            y0 = int(max(0, (yc - h / 4) * FRAME))
            y1 = int(min(FRAME, (yc + h / 4) * FRAME))
            if x1 - x0 < 2 or y1 - y0 < 2:
                continue
            patches[cname].append(lab[y0:y1, x0:x1])
            used[cname] += 1
    calib = {}
    for cname in ("red", "green"):
        if not patches[cname]:
            raise SystemExit(f"no {cname} training patches found")
        calib[cname] = calibrate_color(patches[cname])
    return calib, used


def decide(red_box, green_box):
    """Replicates round2.py's single-frame decision: pick the taller box, apply
    the MIN_SWERVE_HEIGHT / REVERSE_HEIGHT gates, emit a pass side."""
    if red_box is not None and green_box is not None:
        if red_box["height"] >= green_box["height"]:
            box, colour = red_box, "red"
        else:
            box, colour = green_box, "green"
    elif red_box is not None:
        box, colour = red_box, "red"
    elif green_box is not None:
        box, colour = green_box, "green"
    else:
        return None, None, "none"
    if box["height"] < MIN_SWERVE_HEIGHT:
        return box, colour, "hold_too_far"
    if box["height"] > REVERSE_HEIGHT:
        return box, colour, "reverse"
    return box, colour, PASS_SIDE[colour]


def run_split(name, files, calib, writer, timed=False):
    """Evaluate one manifest. Returns a metrics dict."""
    m = dict(split=name, frames=0, with_gt=0,
             detected=0, colour_correct=0, colour_wrong=0, missed=0,
             side_correct=0, side_wrong=0, side_nocall=0, side_reverse=0,
             empty_frames=0, empty_false_alarms=0, centre_in_gt=0,
             latency_ms=[])
    for img_path in files:
        frame = load_rgb240(img_path)
        if frame is None:
            continue
        labels = read_labels(img_path)
        gt = nearest_gt(labels)
        m["frames"] += 1

        t0 = time.perf_counter()
        red_box, green_box = process_frame(frame, calib)
        dt = (time.perf_counter() - t0) * 1000.0
        if timed:
            m["latency_ms"].append(dt)

        box, colour, action = decide(red_box, green_box)
        fired = box is not None

        if gt is None:
            # negative frame: any box at all is a false alarm
            m["empty_frames"] += 1
            if fired:
                m["empty_false_alarms"] += 1
            gt_colour, gt_side = "", ""
        else:
            m["with_gt"] += 1
            gt_colour = gt[0]
            gt_side = PASS_SIDE[gt_colour]
            if not fired:
                m["missed"] += 1
            else:
                m["detected"] += 1
                if colour == gt_colour:
                    m["colour_correct"] += 1
                else:
                    m["colour_wrong"] += 1
                cx = box["x"] + box["width"] / 2.0
                cy = box["y"] + box["height"] / 2.0
                gx0 = (gt[1] - gt[3] / 2) * FRAME
                gx1 = (gt[1] + gt[3] / 2) * FRAME
                gy0 = (gt[2] - gt[4] / 2) * FRAME
                gy1 = (gt[2] + gt[4] / 2) * FRAME
                if gx0 <= cx <= gx1 and gy0 <= cy <= gy1:
                    m["centre_in_gt"] += 1
            if action in ("left", "right"):
                if action == gt_side:
                    m["side_correct"] += 1
                else:
                    m["side_wrong"] += 1
            elif action == "reverse":
                m["side_reverse"] += 1
            else:
                m["side_nocall"] += 1

        writer.writerow([
            name, os.path.basename(img_path), gt_colour, gt_side,
            "" if gt is None else round(gt[4] * FRAME, 1),
            colour or "", "" if not fired else box["height"], action,
            round(dt, 3),
        ])
    return m


def pct(n, d):
    return None if not d else round(100.0 * n / d, 1)


def summarise(m):
    return {
        "split": m["split"],
        "frames": m["frames"],
        "frames_with_pillar": m["with_gt"],
        "empty_frames": m["empty_frames"],
        "detect_rate_pct": pct(m["detected"], m["with_gt"]),
        "miss_rate_pct": pct(m["missed"], m["with_gt"]),
        "colour_accuracy_given_detection_pct": pct(m["colour_correct"], m["detected"]),
        "centre_inside_gt_box_pct": pct(m["centre_in_gt"], m["detected"]),
        "pass_side_correct_pct": pct(m["side_correct"], m["with_gt"]),
        "pass_side_WRONG_pct": pct(m["side_wrong"], m["with_gt"]),
        "committed_calls_pct": pct(m["side_correct"] + m["side_wrong"], m["with_gt"]),
        "accuracy_among_committed_calls_pct": pct(
            m["side_correct"], m["side_correct"] + m["side_wrong"]),
        "no_call_hold_course_pct": pct(m["side_nocall"], m["with_gt"]),
        "reverse_called_pct": pct(m["side_reverse"], m["with_gt"]),
        "false_alarms_per_empty_frame": None if not m["empty_frames"] else round(
            m["empty_false_alarms"] / m["empty_frames"], 3),
        "_counts": {k: v for k, v in m.items() if k not in ("latency_ms", "split")},
    }


def source_family(img_path):
    """The dataset has two acquisition sources with different cameras and
    resolutions: hand-shot stills (red_*/green_*, 640x480) and one continuous
    video session (2026*, 1920x1080). Condition-matched calibration is fitted
    within a family; the race procedure calibrates in the venue's own conditions,
    so the per-family arm is the closer analogue of race day."""
    b = os.path.basename(img_path)
    return "stills" if (b.startswith("red_") or b.startswith("green_")) else "video"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--splits", default=r"C:\Users\ANT PC\wro_vision\tiny\splits")
    ap.add_argument("--out", default="docs/eval_raw")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    def load_manifest(fn):
        p = os.path.join(args.splits, fn)
        with open(p) as f:
            return [ln.strip() for ln in f if ln.strip()]

    train = load_manifest("train.txt")
    val = load_manifest("val.txt")
    val_neg = load_manifest("val_neg.txt")
    val_cooc = load_manifest("val_cooc.txt")

    print(f"train={len(train)} val={len(val)} val_neg={len(val_neg)} "
          f"val_cooc={len(val_cooc)}")

    t0 = time.time()
    calib, used = fit_calibration(train)
    print(f"calibration fitted on {used} boxes in {time.time()-t0:.1f}s")
    for c in ("red", "green"):
        print(f"  {c}: a={calib[c]['a_center']:.2f} b={calib[c]['b_center']:.2f} "
              f"tol={calib[c]['tol']:.2f} l_min={calib[c]['l_min']:.2f}")

    d_ab = float(np.hypot(calib["red"]["a_center"] - calib["green"]["a_center"],
                          calib["red"]["b_center"] - calib["green"]["b_center"]))
    # magenta reference, WRO 2026 rules 13: parking limitation RGB (255,0,255)
    mag_lab = rgb_to_lab(np.array([[[255, 0, 255]]], dtype=np.float32))[0, 0]
    d_red_mag = float(np.hypot(calib["red"]["a_center"] - mag_lab[1],
                               calib["red"]["b_center"] - mag_lab[2]))

    calib_out = {
        "fitted_on": "train.txt (473 real images, group-wise leakage-free split)",
        "estimator": "round2.calibrate_color(), central 50% of each labelled box",
        "boxes_sampled": used,
        "calibration": {c: {k: float(v) for k, v in calib[c].items()} for c in calib},
        "red_green_ab_separation": round(d_ab, 1),
        "red_magenta_ab_separation": round(d_red_mag, 1),
        "magenta_immune": bool(d_red_mag > max(calib["red"]["tol"],
                                               calib["green"]["tol"])),
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "round2_py_sha256": sha256(os.path.join(_HERE, "round2.py")),
    }
    with open(os.path.join(args.out, "picker_calibration.json"), "w") as f:
        json.dump(calib_out, f, indent=2)

    csv_path = os.path.join(args.out, "picker_eval_frames.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["split", "image", "gt_colour", "gt_pass_side", "gt_height_px",
                    "pred_colour", "pred_height_px", "action", "latency_ms"])
        res = [
            run_split("val_real", val, calib, w, timed=True),
            run_split("val_cooccurrence", val_cooc, calib, w),
            run_split("val_negatives", val_neg, calib, w),
        ]
        # ARM B: condition-matched calibration. Fit within each acquisition
        # family (train images only), evaluate that family's val images. This is
        # the analogue of race-day procedure, where calibration happens in the
        # venue's own lighting rather than pooled across sessions.
        matched = []
        matched_calib = {}
        for fam in ("stills", "video"):
            tr = [p for p in train if source_family(p) == fam]
            va = [p for p in val if source_family(p) == fam]
            if len(tr) < 10 or len(va) < 5:
                continue
            cal_f, used_f = fit_calibration(tr)
            matched_calib[fam] = {
                "calibration": {c: {k: float(v) for k, v in cal_f[c].items()}
                                for c in cal_f},
                "train_images": len(tr), "val_images": len(va),
                "boxes_sampled": used_f,
            }
            matched.append(run_split(f"val_real_matched_{fam}", va, cal_f, w))

    lat = res[0]["latency_ms"]
    summary = {
        "generated_utc": calib_out["generated_utc"],
        "harness": "src/Round 2/eval_picker.py",
        "system_under_test": "src/Round 2/round2.py (functions imported verbatim)",
        "round2_py_sha256": calib_out["round2_py_sha256"],
        "frame_size": FRAME,
        "gates": {"MIN_SWERVE_HEIGHT": MIN_SWERVE_HEIGHT,
                  "REVERSE_HEIGHT": REVERSE_HEIGHT},
        "calibration": calib_out["calibration"],
        "red_green_ab_separation": calib_out["red_green_ab_separation"],
        "red_magenta_ab_separation": calib_out["red_magenta_ab_separation"],
        "latency_ms_per_frame_desktop": {
            "median": round(float(np.median(lat)), 3),
            "p95": round(float(np.percentile(lat, 95)), 3),
            "note": "Ryzen 7700X desktop, single thread. NOT a Raspberry Pi 5 number.",
        },
        "splits": [summarise(m) for m in res] + [summarise(m) for m in matched],
        "arm_b_condition_matched_calibration": matched_calib,
        "caveats": [
            "Calibration fitted on train split only; val never used to fit.",
            "Dataset camera and lighting differ from the robot's Lenovo 300 FHD; "
            "the race procedure recalibrates on the day. Validates method, not venue.",
            "Single images cannot exercise the 5-of-7 temporal vote, so the "
            "false-alarm figures are per-frame and conservative vs runtime.",
            "Co-occurrence frames are composited, not photographed; treat as "
            "indicative in both directions.",
        ],
    }
    with open(os.path.join(args.out, "picker_eval_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    lines = [f"picker eval  {summary['generated_utc']}",
             f"SUT round2.py sha256 {summary['round2_py_sha256'][:16]}",
             ""]
    for s in summary["splits"]:
        lines.append(f"[{s['split']}] frames={s['frames']} "
                     f"with_pillar={s['frames_with_pillar']} empty={s['empty_frames']}")
        for k, v in s.items():
            if k in ("split", "_counts", "frames", "frames_with_pillar",
                     "empty_frames") or v is None:
                continue
            lines.append(f"    {k:42s} {v}")
        lines.append("")
    lines.append(f"latency median {summary['latency_ms_per_frame_desktop']['median']} ms "
                 f"p95 {summary['latency_ms_per_frame_desktop']['p95']} ms (desktop)")
    txt = "\n".join(lines)
    with open(os.path.join(args.out, "picker_eval_summary.txt"), "w") as f:
        f.write(txt + "\n")
    print("\n" + txt)


if __name__ == "__main__":
    main()
