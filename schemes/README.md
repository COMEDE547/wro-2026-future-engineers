# Electromechanical schematics

**`wiring-signal-v0.1.png` / `.pdf`** — the fixed signal wiring, drawn 2026-08-05:
ESP32 + PCA9548A I2C multiplexer + 3x TF-Luna + BNO055 + steering servo +
TB6612 / N20 drive, plus the Raspberry Pi 5 serial link and USB camera.

**Still pending:** the power tree — a battery pack and dual-rail buck are now
fitted but unlabelled / undocumented; the tree gets drawn once their specs are
read off the hardware.
See the power-budget section of
[`docs/2_power_and_sensors.md`](../docs/2_power_and_sensors.md#6-power-budget).

**Open harness question, drawn as ch4:** Round 1 firmware and the original
wiring notes put the BNO055 on mux **channel 4**; the off-repo Round 2 firmware
currently addresses **channel 5**. The diagram shows ch4; the discrepancy is
being verified against the physical harness before the Round 2 code lands.
