"""export_nanodet.py — deploy graph only, to ONNX for the Pi 5.

Drops aux_head and aux_fpn: they exist purely to supervise training and are
what makes the 4.17M training graph collapse to 1.17M at inference.

    python export_nanodet.py
"""

import sys

import torch

sys.path.insert(0, r"C:\Users\ANT PC\wro_vision")
from nanodet_lite import cfg as C
from nanodet_lite.model.arch.one_stage_detector import OneStageDetector

CKPT = r"C:\Users\ANT PC\wro_vision\nanodet_runs\best.pt"
OUT = r"C:\Users\ANT PC\wro_vision\nanodet_runs\pillar_nanodet320.onnx"

model = OneStageDetector(backbone_cfg=C.MODEL["backbone"],
                         fpn_cfg=C.MODEL["fpn"],
                         head_cfg=C.MODEL["head"])
ck = torch.load(CKPT, map_location="cpu", weights_only=False)
if ck["classes"] != C.CLASS_NAMES:
    raise SystemExit(f"class mismatch {ck['classes']} vs {C.CLASS_NAMES}")
sd = {k: v for k, v in ck["model"].items()
      if not k.startswith(("aux_head", "aux_fpn"))}
missing, unexpected = model.load_state_dict(sd, strict=False)
model.eval()

n = sum(p.numel() for p in model.parameters())
print(f"deploy params {n:,} | missing {len(missing)} unexpected {len(unexpected)}")

w, h = C.INPUT_SIZE
with torch.no_grad():
    out = model(torch.randn(1, 3, h, w))
print(f"output {tuple(out.shape)} = {C.NUM_CLASSES} cls + {4*(C.REG_MAX+1)} reg")

# fixed shape + legacy tracer: ncnn's onnx2ncnn chokes on dynamo graphs and
# dynamic axes block int8 graph optimisation
torch.onnx.export(model, torch.randn(1, 3, h, w), OUT, opset_version=11,
                  input_names=["images"], output_names=["preds"],
                  do_constant_folding=True, dynamo=False)
print("wrote", OUT)
print("next: onnx2ncnn -> ncnnoptimize -> benchncnn ON THE PI 5")
