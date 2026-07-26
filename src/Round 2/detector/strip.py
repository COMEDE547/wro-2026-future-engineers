import os, re, shutil
SRC = r"C:\Users\ANT PC\wro_vision\nanodet_src\nanodet"
DST = r"C:\Users\ANT PC\wro_vision\nanodet_lite"
EXTRA = r"C:\Users\ANT PC\wro_vision\nanodet_extra"
if os.path.exists(DST): shutil.rmtree(DST)

KEEP = [
 "model/module/conv.py","model/module/activation.py","model/module/norm.py",
 "model/module/init_weights.py","model/module/scale.py","model/module/nms.py",
 "model/backbone/shufflenetv2.py","model/backbone/ghostnet.py",
 "model/fpn/ghost_pan.py",
 "model/head/nanodet_plus_head.py","model/head/simple_conv_head.py","model/head/gfl_head.py",
 "model/head/assigner/dsl_assigner.py","model/head/assigner/base_assigner.py",
 "model/head/assigner/assign_result.py",
 "model/loss/gfocal_loss.py","model/loss/iou_loss.py","model/loss/utils.py",
 "model/arch/one_stage_detector.py","model/arch/nanodet_plus.py",
 "model/weight_averager/ema.py",
 "data/transform/warp.py","data/transform/color.py","data/transform/pipeline.py",
 "data/batch_process.py",
 "util/box_transform.py","util/misc.py","util/util_mixins.py",
]
for rel in KEEP:
    d = os.path.join(DST, *rel.split("/"))
    os.makedirs(os.path.dirname(d), exist_ok=True)
    shutil.copy2(os.path.join(SRC, *rel.split("/")), d)

for pkg in ["", "model","model/module","model/backbone","model/fpn","model/head",
            "model/head/assigner","model/loss","model/arch","model/weight_averager",
            "data","data/transform","util"]:
    p = os.path.join(DST, *(pkg.split("/") if pkg else []), "__init__.py")
    os.makedirs(os.path.dirname(p), exist_ok=True)
    open(p,"w").close()

def patch(rel, subs, must=True):
    p = os.path.join(DST, *rel.split("/")); s = open(p, encoding="utf-8").read(); orig = s
    for a,b in subs:
        if a not in s and must: raise SystemExit(f"ANCHOR MISS {rel}: {a[:70]!r}")
        s = s.replace(a,b)
    if s != orig: open(p,"w",encoding="utf-8").write(s)

def w(rel, text):
    open(os.path.join(DST, *rel.split("/")), "w", encoding="utf-8").write(text)

w("model/backbone/__init__.py",
  "from .shufflenetv2 import ShuffleNetV2\n\n__all__ = ['ShuffleNetV2']\n")
w("model/fpn/__init__.py",
  "from .ghost_pan import GhostPAN\n\n__all__ = ['GhostPAN']\n")
w("model/head/__init__.py",
  "from .nanodet_plus_head import NanoDetPlusHead\n"
  "from .simple_conv_head import SimpleConvHead\n\n"
  "__all__ = ['NanoDetPlusHead', 'SimpleConvHead']\n")
w("util/__init__.py",
  "from .box_transform import bbox2distance, distance2bbox\n"
  "from .misc import multi_apply, images_to_levels, unmap\n\n"
  "__all__ = ['bbox2distance','distance2bbox','multi_apply','images_to_levels','unmap']\n")

patch("model/head/assigner/assign_result.py", [
 ("from nanodet.util import util_mixins", "from ....util import util_mixins"),
])
patch("model/head/nanodet_plus_head.py", [
 ("from nanodet.util import bbox2distance, distance2bbox, multi_apply, overlay_bbox_cv",
  "from ...util import bbox2distance, distance2bbox, multi_apply"),
])
p = os.path.join(DST, "model","head","nanodet_plus_head.py")
s = open(p, encoding="utf-8").read()
s = re.sub(r"\n    def show_result\(.*?(?=\n    def |\Z)", "\n", s, flags=re.S)
open(p,"w",encoding="utf-8").write(s)

gfl = open(os.path.join(DST,"model","head","gfl_head.py"), encoding="utf-8").read()
keep = [b for b in re.split(r"\n(?=class |def )", gfl)
        if b.startswith("class Integral") or b.startswith("def reduce_mean")]
assert len(keep)==2, [k[:40] for k in keep]
w("model/head/gfl_head.py",
  "import torch\nimport torch.distributed as dist\nimport torch.nn as nn\n"
  "import torch.nn.functional as F\n\n\n" + "\n\n".join(k.rstrip() for k in keep) + "\n")

HELPER = ("\n\ndef _kw(cfg):\n"
          "    \"\"\"cfg dict -> kwargs, dropping the upstream registry 'name' key.\"\"\"\n"
          "    return {k: v for k, v in dict(cfg).items() if k != 'name'}\n")
patch("model/arch/one_stage_detector.py", [
 ("from ..backbone import build_backbone\nfrom ..fpn import build_fpn\nfrom ..head import build_head",
  "from ..backbone import ShuffleNetV2\nfrom ..fpn import GhostPAN\nfrom ..head import NanoDetPlusHead, SimpleConvHead"
  + HELPER),
 ("self.backbone = build_backbone(backbone_cfg)", "self.backbone = ShuffleNetV2(**_kw(backbone_cfg))"),
 ("self.fpn = build_fpn(fpn_cfg)", "self.fpn = GhostPAN(**_kw(fpn_cfg))"),
 ("self.head = build_head(head_cfg)", "self.head = NanoDetPlusHead(**_kw(head_cfg))"),
])
patch("model/arch/nanodet_plus.py", [
 ("from ..head import build_head", "from ..head import SimpleConvHead\nfrom .one_stage_detector import _kw"),
 ("self.aux_head = build_head(aux_head)", "self.aux_head = SimpleConvHead(**_kw(aux_head))"),
])

for rel in ["cfg.py", "train.py", "data/dataset.py", "data/collate.py"]:
    src = os.path.join(EXTRA, *rel.split("/"))
    if os.path.exists(src):
        shutil.copy2(src, os.path.join(DST, *rel.split("/")))
        print("  +", rel)

n = sum(len(f) for _,_,f in os.walk(DST))
print("staged", n, "files ->", DST)
