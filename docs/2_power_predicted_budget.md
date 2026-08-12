# 2a — Predicted Power Budget (datasheet-derived)

**Status: PREDICTED ONLY.** Every number here is derived from datasheets and
cited independent measurements. Predictions score nothing and prove nothing —
they exist so the multimeter session has sanity bands and so the measured
column lands next to the prediction it confirms or refutes. The measured
column fills only from meter readings — first entry 2026-08-12 (row 1).

## Two discrepancies — RESOLVED on the vehicle 2026-08-11 (record kept)

1. **Pack capacity.** The 2026-08-06 note recorded 2600 mAh; the label was
   re-read on 2026-08-11 — the misread is corrected in
   [2 — Power](2_power_and_sensors.md) §6 and [1 — Mobility](1_mobility.md).
   This file's math already assumed the correct value.
   → **Label says: 2200 mAh, 60C** (read 2026-08-11).
2. **Topology.** The rework is real and confirmed on the vehicle: §5's
   servo-through-ESP32-chain failure point is FIXED (dated 2026-08-11), and
   §6 plus `schemes/circuit_diagram_complete_2026-08-11.jpg` now describe
   the as-built tree.
   → **As-built topology is:** pack → fast-charging module → Pi 5; ESP32 from
   a Pi USB port; pack → Buck-2 → 3× TF-Luna + servo **power** (signal only
   from the ESP32); pack → TB6612 VM → N20.

## Predicted vs measured

Conditions: full system as raced (Pi booted, vision script running unless
labelled otherwise), wheels off ground unless labelled. Buck efficiency
assumed **88% (85–92% band)** — measure it (P_out/P_in per buck) and replace.

| Row | Node / condition | Predicted | **Measured** | Meter reads outside band → investigate |
|---|---|---|---|---|
| 1 | Pack V, rested, all off | 11.7–12.6 V | **12.47–12.49 V** † | <11.1 V: pack discharged/aged |
| 2 | Pack V, idle (system up, motors still) | ~rested −0.05 V | | drop >0.3 V: harness/connector resistance |
| 3 | Pack V, driving (N20 @ STRAIGHT_SPEED, servo sweeping) | ~rested −0.1 V | | drop >0.3 V: harness, not cells (60C pack sags mV at these currents) |
| 4 | 5 V rail (Buck-2), driving | 4.9–5.1 V | | <4.8 V: buck overloaded or input sag |
| 5 | Total pack I, idle | **0.7–1.0 A** (band 0.5–1.4) | | <0.5: vision not running / wrong node. >1.4: Pi pegged or fault |
| 6 | Total pack I, driving (free-run) | **0.8–1.3 A** (band 0.6–1.8) | | >1.8 sustained: N20 loaded/stalling or buck losses |
| 7 | Servo transient adder | +0.3–0.5 A, ~100–300 ms | | >2.5 A total or >0.5 s: servo bound / N20 co-stall |
| 8 | (optional) loaded pass, wheels pressed to mat | above row 6 | | label condition honestly; use the higher of 6/8 for runtime |

> † Measured 2026-08-12, bench (Fluke 15B+, DC-V autorange): ESP32 running a
> serial motor-test sketch via laptop USB, wheels off ground, Pi state not
> recorded — *not* the as-raced condition this table specifies. Value sits
> inside the predicted band. Rows 2–3 and a row-4-while-driving re-measure
> during the first logged run. A 5 V-node reading of 5.05–5.08 V from the same
> session is deliberately **not entered in row 4**: the probe point (Buck-2
> output vs the Pi-USB 5 V feeding the ESP32) is unconfirmed pending the bench
> photos, and both nodes read ~5.0–5.1 V, so the number cannot disambiguate
> itself — entering it would be a false measured claim. Motor-terminal PWM
> means at 0 / half / full commanded speed: 0 / 5.95–5.96 / 12.47 V ≈
> 0 / 48 / 100 % duty (duty-cycle characterisation, not a rail voltage).
> All currents and the servo-dip row stay empty — the meter is voltage-only
> and has no MIN capture.

**Derived rows (fill after 1–7):** idle P = V₂×I₅ · driving P = V₃×I₆ ·
sag = V₁−V₃ · runtime = (label mAh × 0.8) ÷ I₆ · runs/charge = runtime ÷ 3 min.
Predicted: **runtime 1.4–2.2 h, ~25–40 runs/charge** (at 2200 mAh).

## Per-component basis (each figure sourced)

| Component | Rail | Figure used | Source |
|---|---|---|---|
| Raspberry Pi 5 8 GB | 5 V | 1.8–2.7 W idle; ~5–7 W under this vision load (well under the 8.8 W four-core stress figure) | CNX Software Pi 5 review (2023-11-05); Geerling 1.8 W idle |
| Pi 5 USB budget | 5 V | **600 mA total** without a negotiated 5 V/5 A supply (1.6 A only with one) | Official Raspberry Pi documentation, power-supplies |
| Lenovo 300 FHD webcam | Pi USB | nameplate 5 V/150 mA; comparable 1080p cams stream ~200–220 mA | Lenovo GXC1B34793 spec; Tripp Lite AWC-001; C270 bench |
| ESP32-WROOM-32, radios off | Pi USB | 30–50 mA core @240 MHz; dev boards 40–80 mA with regulator/LED | Espressif ESP32 datasheet Table 6 |
| TF-Luna ×3 | Buck-2 5 V | ≤70 mA avg / 150 mA peak **per unit** → ~210 mA avg aggregate | Benewake TF-Luna datasheet A05 |
| BNO055 (fusion) | 3.3 V | 12.3 mA typ | Bosch BST-BNO055-DS000 |
| MG90S | Buck-2 5 V | ~10 mA idle / 120–250 mA moving / 700–800 mA stall | TowerPro spec + bench sources |
| TB6612FNG | 3.3 V / VM | ICC ~1.1 mA; ~0.5 Ω total on-resistance | Toshiba TB6612FNG datasheet |
| N20 12 V (variant TBD) | pack | 40–60 mA no-load; 0.75–1.1 A stall (variant-dependent; 100 rpm class assumed) | Sharvi N20-12V-100; Pololu 12 V HPCB analog |

## Predicted failure-relevant notes
- **Pack utilization <2% of the 60C rating at predicted peak** — any measured
  sag implicates connectors/wire gauge/buck input, never the cells.
- **Highest-consequence single point:** if the Pi's supply doesn't negotiate
  5 A, the Pi caps USB at 600 mA — webcam (+ ESP32 if USB-powered) both live
  inside that cap. Predicted draw 0.18–0.27 A fits, but verify enumeration and
  check `vcgencmd pmic_read_adc` for the restriction warning.
- Servo + N20 transients can stack to ~2–2.5 A at the pack briefly — harmless
  to the pack, capable of tripping a marginal buck. The 470–1000 µF bulk cap
  recommendation stands.
