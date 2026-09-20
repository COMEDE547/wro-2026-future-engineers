# APAC 2026 vehicle — changes since Nationals

**Status 2026-09-20.** After the WRO India National Championship (26–28 Aug 2026) the vehicle was rebuilt for APAC 2026. This page records what changed. *Measured* values were measured on the vehicle; *team* values are the team's figures and are still to be measured. Documents 1–5 remain the record of the Nationals configuration.

## Specification

| | Nationals (Aug 2026) | APAC 2026 (Sep 2026) | Basis |
|---|---|---|---|
| Length × width × height | length 23.0 cm | 19.1 × 14.3 × 27.2 cm (±0.5 cm) | measured |
| Mass | 785 g | 862 g on 2026-09-16; race-ready re-weigh pending | measured |
| Chassis | LEGO Technic space-frame, 69-step build | LEGO Technic frame, 84-step build ([instructions](../models/chassis/chassis_build_instructions_apac_2026-09.pdf)), plus 3D-printed holders for the motor and components (files to follow) | build file |
| Steering | single-servo Ackermann (MG90S) | non-Ackermann: an MG90 servo drives a vertical steering column that turns the front axle, 60° each side | team |
| Wheelbase | — | 13 cm | team |
| Wheels | front 41 mm, rear 55.6 mm | front 45 mm, rear 55 mm (rear tyres chosen for grip); two 30 mm horizontal side rollers | team |
| Drive motor | N20 gearmotor, 12 V, 600 rpm, via TB6612FNG | 400 rpm gearmotor via TB6612FNG; rated voltage and part number to be confirmed | team |
| Gearing | 20T pinion → 28T crown on a LEGO differential (5:7) | unchanged | team |
| Battery | 3S LiPo, 11.1 V, 2200 mAh | unchanged | team |
| Power | pack → fast-charge module → Pi 5; pack → buck converter → 5 V rail (TF-Lunas, servo); TB6612FNG from the pack ([2 — Power & Sensors](2_power_and_sensors.md)) | unchanged, plus the WS2812 LEDs on the 5 V rail; power switch on the battery lead; start button on the rear beams | team, wiring draft |
| Sensors | 3 × TF-Luna on a PCA9548A, BNO055, USB camera | same set; the camera (Lenovo 300 FHD) now sits on top of a rear tower, lens height and tilt to be measured | photos |
| Lighting | none | 6 × WS2812 LEDs (three 2-LED strips) on the front frame | team, wiring draft |

## Why each change was made

Reasons as the team gives them. The measurement or before/after run behind each one is still to be added.

| Change | Stated reason |
|---|---|
| Horizontal side rollers | keep the car moving if it touches a wall |
| Non-Ackermann steering column | a smaller turning circle |
| Rear tyres | more grip |
| Camera raised | a better view of the track |
| New frame | a sturdier structure |
| Front LEDs | fewer colour mistakes |

## ESP32 pin map

From the 2026-09-20 wiring draft and the committed Nationals firmware ([`round2_ino.ino`](../src/Round%202/round2_ino/round2_ino.ino)); only GPIO4 is new. To be re-checked against the APAC firmware when it is committed.

| Signal | GPIO |
|---|---|
| I²C SDA / SCL — PCA9548A at `0x70`: ch0 left, ch1 centre, ch2 right TF-Luna (`0x10`), ch4 BNO055 (`0x28`) | 21 / 22 |
| Steering servo | 13 |
| TB6612FNG AIN1 / AIN2 / PWMA / STBY | 25 / 26 / 33 / 27 |
| WS2812 data | 4 |

## Still to add

- Race-ready six-view photos. The shots in [`v-photos/apac-2026-09-20/`](../v-photos/apac-2026-09-20/) are working photos with cables attached.
- Open Challenge and Obstacle Challenge videos of this vehicle.
- The APAC firmware and Raspberry Pi code.
- STL files for the 3D-printed holders.
- A corrected wiring diagram.
- Measurements: race-ready mass and axle loads, turning circle, camera height and tilt, motor speed and current, current on each supply rail.
