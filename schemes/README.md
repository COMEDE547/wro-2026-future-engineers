# Electromechanical schematics

**`wiring-v0.2.png` / `.pdf` — CURRENT (2026-08-10): signal + power on one
page.** Everything v0.1 drew, plus the power tree as physically identified:
3S 11.1 V 2600 mAh pack (Tamiya-style plug) -> dual-rail buck (barrel-jack in;
XL4015/LM2596-class, chip and rating TBD; 5 V screw out + USB-A out) -> ESP32
VIN, with the servo drawing through the ESP32's 5 V chain (the named brownout
risk in [docs/2 §5](../docs/2_power_and_sensors.md); a dedicated rail is
pending), pack 11.1 V direct to TB6612 VM, and the buck's USB-A drawn as the
**candidate** Pi 5 supply — TBD, because a plain USB-A source has no PD
negotiation, which makes the Pi 5 cap its USB-peripheral budget at 600 mA.
Unknowns are drawn as TBD rather than guessed. The missing rule-9.10-mandatory
main power switch and the absent fuse are called out on-sheet.

The diagram is generated, not hand-drawn: edit
[`make_wiring_v0_2.py`](make_wiring_v0_2.py) and rerun
(`py -3 schemes/make_wiring_v0_2.py` from the repo root, needs matplotlib) —
do not edit the PNG.

**`wiring-signal-v0.1.png` / `.pdf`** — superseded 2026-08-10, retained as
history. Signal wiring only; left the power tree as "pending battery selection"
(the pack was read off the hardware on 2026-08-06) and carried a CH4-vs-CH5
BNO055 harness question that closed when the Round 2 firmware landed on
**CH4**.
