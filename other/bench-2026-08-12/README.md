# Bench session — 2026-08-12 (OMOTEC)

Voltage-only measurement session, Fluke 15B+ (DC-V autorange), 14 photos.
ESP32 was running `sketch_aug12c.ino` (serial motor-test, visible in-frame)
via laptop USB; wheels off ground; Pi state not recorded — NOT the as-raced
condition the power budget specifies. Values feed
`docs/2_power_predicted_budget.md` (row 1; footnote).

Readings:
- Pack / VIN: **12.47–12.49 V** (photo 03 also shows the pack label —
  Champion **2200 mAh 60C** — the capacity read of 2026-08-11, photo-backed).
- 5 V node: **5.05–5.08 V** (photo 07). Probe point not conclusively
  identifiable as Buck-2 output vs the Pi-USB 5 V feeding the ESP32; both
  nodes read ~5.0–5.1 V, so this number is deliberately NOT entered as the
  row-4 Buck-2 measurement.
- Motor terminals at 0 / half / full commanded speed: 0 / 5.95–5.96 /
  12.47 V — PWM duty-cycle means (~0 / 48 / 100 %), not rail voltages.

Excluded as artifacts: 1.248 / 0.506 / 0.595 V frames — the same digits
decade-shifted (autorange transition / probe-contact flake).

Not measurable with this meter: all currents (voltage-only), servo transient
dip (no MIN capture). Those rows stay empty until a better meter surfaces.
