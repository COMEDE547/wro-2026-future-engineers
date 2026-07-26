"""Manifest-driven YOLO-txt dataset for NanoDet-Plus.

Reads the SAME leakage-free manifests tiny_pillar uses
(splits/train_aug.txt, splits/val.txt), not a directory. This matters: the
shipped dataset/ split leaked badly (25 duplicate stems across train/val, 27
of 29 val one-second video buckets also present in train), so any model
trained off the raw directories is measuring memorisation.

Labels are resolved by swapping /images/ -> /labels/, the convention every
derived folder already follows. Images with an empty or absent .txt are valid
negatives and are kept -- they are how the false-positive rate comes down.
"""

import os

import cv2
import numpy as np
import torch


class ManifestDataset(torch.utils.data.Dataset):
    def __init__(self, manifest, input_size, pipeline):
        self.files = [l.strip() for l in open(manifest, encoding="utf-8") if l.strip()]
        if not self.files:
            raise RuntimeError(f"empty manifest: {manifest}")
        self.input_size = tuple(input_size)   # (w, h)
        self.pipeline = pipeline

    def __len__(self):
        return len(self.files)

    @staticmethod
    def _label_path(img_path):
        p = img_path.replace(os.sep + "images" + os.sep, os.sep + "labels" + os.sep)
        return os.path.splitext(p)[0] + ".txt"

    def _load(self, img_path, w, h):
        """YOLO normalised cxcywh -> absolute xyxy."""
        boxes, labels = [], []
        lp = self._label_path(img_path)
        if os.path.exists(lp):
            for line in open(lp):
                p = line.split()
                if len(p) < 5:
                    continue
                c, cx, cy, bw, bh = int(p[0]), *(float(x) for x in p[1:5])
                boxes.append([(cx - bw / 2) * w, (cy - bh / 2) * h,
                              (cx + bw / 2) * w, (cy + bh / 2) * h])
                labels.append(c)
        if boxes:
            return (np.array(boxes, dtype=np.float32),
                    np.array(labels, dtype=np.int64))
        return np.zeros((0, 4), np.float32), np.zeros((0,), np.int64)

    def __getitem__(self, i):
        path = self.files[i]
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"cv2 failed to decode {path}")
        h, w = img.shape[:2]
        gt_bboxes, gt_labels = self._load(path, w, h)
        meta = {
            "img": img,
            "img_info": {"file_name": os.path.basename(path),
                         "height": h, "width": w, "id": i},
            "gt_bboxes": gt_bboxes,
            "gt_labels": gt_labels,
        }
        meta = self.pipeline(self, meta, self.input_size)
        meta["img"] = torch.from_numpy(meta["img"].transpose(2, 0, 1))
        return meta
