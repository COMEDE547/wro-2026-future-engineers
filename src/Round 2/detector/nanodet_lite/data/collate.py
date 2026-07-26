"""Collate for NanoDet meta dicts.

Upstream nanodet/data/collate.py imports torch._six.string_classes, removed in
torch 1.9 and gone in 2.x -- it dies on import regardless of the Lightning pin.

GT stays as numpy on purpose: NanoDetPlusHead.target_assign_single_img calls
torch.from_numpy(gt_bboxes) itself and raises TypeError if handed a Tensor.
"""

import numpy as np
import torch


def collate_meta(batch):
    out = {k: [d[k] for d in batch] for k in batch[0]}
    out["img"] = torch.stack([im.float() for im in out["img"]], dim=0)
    out["gt_bboxes"] = [np.ascontiguousarray(b, dtype=np.float32)
                        for b in out["gt_bboxes"]]
    out["gt_labels"] = [np.ascontiguousarray(l, dtype=np.int64)
                        for l in out["gt_labels"]]
    out["gt_bboxes_ignore"] = None
    return out
