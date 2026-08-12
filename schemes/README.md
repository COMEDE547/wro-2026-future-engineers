# Electromechanical schematics

**`wiring-v0.2.png` / `.pdf` — signal-wiring authority (2026-08-10); its power
tree is HISTORICAL, superseded by the 2026-08-11 rework recorded in the hand
diagram below.** Everything v0.1 drew, plus the power tree as identified on
2026-08-10: 3S 11.1 V pack (label read 2026-08-11: **2200 mAh 60C** — the
frozen v0.2 render predates the read and shows 2600) -> dual-rail buck
(barrel-jack in; XL4015/LM2596-class, chip and rating TBD; 5 V screw out +
USB-A out) -> ESP32 VIN, with the servo drawn through the ESP32's 5 V chain
(the named brownout risk in [docs/2 §5](../docs/2_power_and_sensors.md) —
**fixed 2026-08-11**: servo power moved to Buck-2; the v0.2 render predates
the rework), pack 11.1 V direct to TB6612 VM, and the buck's USB-A drawn as the
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

**`circuit_diagram_complete_2026-08-11.jpg`** — full power + control diagram
(hand-drawn, photographed at the bench 2026-08-11). Companion to wiring-v0.2,
which remains the generated signal-wiring authority. Every firmware-relevant
pin on this diagram was cross-checked against `src/Round 2/main.cpp` the same
day and matched 6/6: I2C SDA/SCL `21/22`, servo `13`, TB6612 `25/26/33/27`,
PCA9548A `0x70` (channels 0/1/2 = TF-Luna `0x10`, CH4 = BNO055 `0x28`). It also
records the as-built power tree confirmed on the vehicle: servo **power** from
Buck-2 (signal only from the ESP32), 3× TF-Luna on Buck-2, ESP32 from a Pi USB
port, Pi via the fast-charging module — see `docs/2_power_and_sensors.md` §5/§6
for the rework note this supersedes. **Known omission:** the diagram does not
show the external start button on **GPIO32** that both firmwares use — the
button was wired after the diagram was drawn; treat GPIO32 as present per
`src/`.
