"""NanoDet-Plus-m 320, 2-class WRO pillar task. Plain dicts, no yacs/omegaconf.

CLASS ORDER IS data.yaml ORDER: 0=green, 1=red.
An earlier draft of this file had ["red_pillar","green_pillar"] i.e. 0=red,
which is INVERTED. Training on the wrong mapping makes the robot pass every
pillar on the wrong side -- a scored failure, not cosmetic.
"""


class D(dict):
    """dict with attribute access.

    Upstream reaches into config with dots (loss_cfg.loss_qfl.beta) while other
    call sites treat the same objects as plain dicts (norm_cfg.pop("type")).
    Supporting both means neither yacs nor omegaconf is needed.
    """

    def __getattr__(self, k):
        try:
            return self[k]
        except KeyError as e:
            raise AttributeError(k) from e

    __setattr__ = dict.__setitem__


def dot(o):
    if isinstance(o, dict):
        return D({k: dot(v) for k, v in o.items()})
    if isinstance(o, (list, tuple)):
        return type(o)(dot(v) for v in o)
    return o


CLASS_NAMES = ["green", "red"]      # data.yaml order -- do not reorder
NUM_CLASSES = len(CLASS_NAMES)
INPUT_SIZE = (320, 320)             # (w, h)
STRIDES = [8, 16, 32, 64]
REG_MAX = 7

MODEL = dict(
    detach_epoch=10,
    backbone=dict(model_size="1.0x", out_stages=[2, 3, 4],
                  activation="LeakyReLU", pretrain=False),
    fpn=dict(in_channels=[116, 232, 464], out_channels=96, kernel_size=5,
             num_extra_level=1, use_depthwise=True, activation="LeakyReLU"),
    head=dict(num_classes=NUM_CLASSES, input_channel=96, feat_channels=96,
              stacked_convs=2, kernel_size=5, strides=STRIDES,
              activation="LeakyReLU", reg_max=REG_MAX, norm_cfg=dict(type="BN"),
              loss=dict(loss_qfl=dict(beta=2.0, loss_weight=1.0),
                        loss_dfl=dict(loss_weight=0.25),
                        loss_bbox=dict(loss_weight=2.0))),
    aux_head=dict(num_classes=NUM_CLASSES, input_channel=192, feat_channels=192,
                  stacked_convs=4, strides=STRIDES, activation="LeakyReLU",
                  reg_max=REG_MAX),
)
MODEL = dot(MODEL)

# Hue is the class label here, so no hue rotation anywhere.
# Brightness/contrast/saturation are the knobs venue lighting actually varies.
TRAIN_PIPELINE = dict(
    perspective=0.0, scale=[0.6, 1.4], stretch=[[0.8, 1.2], [0.8, 1.2]],
    rotation=8, shear=0, translate=0.15, flip=0.5,
    brightness=0.25, contrast=[0.7, 1.3], saturation=[0.7, 1.3],
    normalize=[[103.53, 116.28, 123.675], [57.375, 57.12, 58.395]],
)
VAL_PIPELINE = dict(
    normalize=[[103.53, 116.28, 123.675], [57.375, 57.12, 58.395]],
)

TRAIN = dict(
    epochs=120, batch_size=32, num_workers=4, lr=1e-3, weight_decay=0.05,
    warmup_iters=200, grad_clip=35.0, amp=True, ema_decay=0.9998,
    save_dir=r"C:\Users\ANT PC\wro_vision\nanodet_runs",
)
