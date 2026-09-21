# APAC 2026 vehicle — changes since Nationals

**Status 2026-09-22.** After the WRO India National Championship (26–28 Aug 2026) the vehicle was rebuilt for APAC 2026. This page records what changed and why. Every figure is marked by its basis: *measured* on the vehicle, *team* figure not yet measured, *derived* by calculation from named inputs, or *vendor* specification. Documents 1–5 remain the record of the Nationals configuration.

## 1. Specification

| | Nationals (Aug 2026) | APAC 2026 (Sep 2026) | Basis |
|---|---|---|---|
| Length × width × height | length 23.0 cm | 19.1 × 14.3 × 27.2 cm (±0.5 cm) | measured |
| Mass | 785 g | 862 g race-ready, Raspberry Pi 5 included (2026-09-16) | measured |
| Axle loads | — | front 201 g, rear 661 g (23 % / 77 %) — see §2 | measured |
| Chassis | LEGO Technic space-frame, 69-step build | LEGO Technic frame, 84-step build ([instructions](../models/chassis/chassis_build_instructions_apac_2026-09.pdf)); 332 g bare | build file; measured |
| Printed parts | TF-Luna holders, servo horn beam, motor clamp ([`models/`](../models/)) | the same STL files | team |
| Steering | single-servo Ackermann (MG90S) | non-Ackermann: an MG90 servo drives a vertical steering column that turns the front axle, 60° each side | team |
| Wheelbase | — | 13 cm | team |
| Wheels | front 41 mm, rear 55.6 mm | front 45 mm, rear 55 mm (rear tyres chosen for grip); two 30 mm horizontal side rollers (build steps 71–76) | team |
| Drive motor | N20 gearmotor, 12 V, 600 rpm, via TB6612FNG | MEX motor (Avishkaar), rated 6 V, 400 rpm, via TB6612FNG — see §3 | team |
| Gearing | 20T pinion → 28T crown on a LEGO differential (5:7) | unchanged | team |
| Battery | 3S LiPo, 11.1 V, 2200 mAh | unchanged | team |
| Power | pack → fast-charge module → Pi 5; pack → Buck-2 → 5 V rail (TF-Lunas, servo); TB6612FNG from the pack ([2 — Power & Sensors](2_power_and_sensors.md)) | unchanged, plus the WS2812 LEDs on the 5 V rail; power switch on the battery lead; start button on the rear beams | team |
| Sensors | 3 × TF-Luna on a PCA9548A, BNO055, USB camera on a front arm at ~10 cm | same sensors; the camera (Lenovo 300 FHD) now sits on top of a rear tower — see §4 | photos; measured |
| Lighting | none | 6 × WS2812 LEDs (three 2-LED strips) on the front frame — see §5 | team |

Wiring: [`schemes/wiring_apac_2026.md`](../schemes/wiring_apac_2026.md).

## 2. Weight and balance

| Quantity | Value | Basis |
|---|---|---|
| Total, race-ready | 862 g | measured |
| Front / rear axle | 201 g / 661 g (23 % / 77 %); the two sum to the total | measured |
| Centre of mass | 10.0 cm behind the front axle (13 cm × 661 / 862) | derived |
| Bare chassis | 332 g, split 31 % / 69 % | measured |
| Everything added to the chassis | 530 g, about 82 % on the rear axle | derived |

Assuming the added parts share the chassis's split would have put 67 g too much on the front, which is why the full car was weighed per axle.

The driven rear wheels carry 77 % of the weight, which helps traction. The steered front wheels carry 23 %, on the smaller tyres, so front grip is what limits cornering. Planned check: measure the turning circle at walking pace and at race speed; a much wider circle at speed means the front is sliding.

## 3. Drive

- **Motor:** MEX motor (Avishkaar), rated 6 V and 400 rpm no-load (team).
- **Speed at the rated voltage:** 400 rpm × 20/28 = 286 rpm at the rear wheels; × π × 55 mm = **0.82 m/s no-load** (derived).
- **Voltage the motor sees:** the TB6612FNG switches the pack (11.1 V nominal, 12.6 V full) with PWM, so the motor's average voltage is duty × pack voltage. With the Nationals firmware, full commanded speed was 100 % duty (12.47 V measured at the motor terminals) and half speed was 48 % duty (5.95 V) — see [2a](2_power_predicted_budget.md). On a 6 V motor, duty above about 48 % on a full pack averages more than its rating. The APAC firmware writes the duty directly with an 8-bit `analogWrite` and no cap: `SPEED = 210` is 82 % duty, 9.1 V average on the nominal pack and 10.4 V on a full one, 1.5–1.7 times the motor's 6 V rating (derived). Turning (`100`, 39 %), parking (`70`, 27 %) and the collision escape (`80`, 31 %) stay under the rating. The motor temperature after a full 3-minute run has not been measured yet.
- **Measured 2026-09-21** (pack at 12.3 V, `SPEED = 210`, so 10.1 V average; ammeter in the motor wire unless stated):

| Test | Result | Basis |
|---|---|---|
| Rear wheels lifted, free-running | 0.33 A; rear wheel 422 rpm (slow-motion video of a taped wheel, counted over one minute) = **1.22 m/s** | measured |
| Driving on the mat | 0.599 A | measured |
| Wheels held (stall), ammeter on the battery lead | 1.24 A, which is 1.51 A through the motor at 82 % duty | measured; motor figure derived |
| 3 m straight line from a standing start | 3.36 s, 0.89 m/s on average with the launch included | measured |

- **Speed check.** Scaling the rated 400 rpm linearly to 10.1 V predicts 482 rpm at the rear wheel. The measured 422 rpm is 12.5 % lower, because linear scaling ignores the voltage lost in the winding and the drivetrain friction that shows up as the 0.33 A free-running current.
- **Motor model** (derived). Treating the motor as a standard DC motor (voltage = current × resistance + speed × motor constant) and fitting it to the free-running and stall points gives 6.71 Ω including the driver and 0.128 V·s/rad at the gearmotor output. At the measured 0.599 A on the mat it predicts 0.94 m/s, and 3.37 s for the 3 m standing start (launch time constant 0.17 s). The measured time is 3.36 s.
- **Torque and grip** (derived; drivetrain efficiency beyond the free-running losses assumed 0.8). The rear axle gets about 0.04 N·m while cruising (1.4 N at the tyres, the rolling and drivetrain load on the mat) and 0.17 N·m at stall (6.2 N at the tyres). The rear tyres carry 6.5 N (661 g), so 6.2 N would need a friction coefficient of 0.95: a hard launch or a push against a wall spins the rear wheels on most surfaces before the motor stalls.

## 4. Camera placement

- **Mount:** lens centre 24.8 ± 0.5 cm above the floor; optical axis 12° below horizontal (78° from straight down); the hinge is glued at that angle (measured).
- **Field of view:** Lenovo lists a 95° diagonal field of view ([product page](https://store.lenovo.com/in/en/lenovo-300-fhd-webcam-gxc1b34793-387.html)), which is about 56° vertical and 87° horizontal at 16:9 (derived; not yet measured on this camera).

| Quantity | Formula | Value |
|---|---|---|
| Where the optical axis meets the floor | h / tan α | 117 cm ahead of the lens; about ±10 cm per 1° of angle error |
| Nearest floor in view | h / tan(α + VFOV/2) | ~29 cm from the lens; the lens sits ~10 cm behind the front wheels, so ~19 cm ahead of them |
| Separation of pillar bases at 0.5 m and 1.0 m | atan(h / 0.5 m) − atan(h / 1.0 m) | 12.5° (5.6° with the Nationals 10 cm mount) |
| Tops of walls 1–3 m away | α − atan((h − 10 cm) / D) | 3.6°–9.2° above the image centre |

h = lens height, α = pitch below horizontal, VFOV = vertical field of view, D = distance to the wall.

**Why the camera went up.** Reading range from the image depends on how low a pillar's base sits (the Nationals rule, [2 — Power & Sensors §3](2_power_and_sensors.md)). Raising the lens from ~10 cm to 24.8 cm makes that signal about 2.2 times stronger (12.5° against 5.6° between pillars at 0.5 m and 1.0 m), and the centre of the image now lands ~1.2 m ahead instead of 0.33–0.57 m. The separation keeps growing with height up to h = √(Z_near · Z_far), 55 cm for pillars at 0.3 m and 1.0 m, so the earlier statement in §3 of that document, that a higher mount compresses it, was wrong; it is corrected there.

**What it costs.**

1. A blind zone: a pillar within ~19 cm of the front wheels has its base below the frame, so the nearest-pillar rule needs a tie-break for boxes cut off at the bottom edge.
2. A view over the 100 mm walls: off-field objects can appear between the wall tops and the horizon, so the pillar search has to stay below the black wall band, not a fixed image row.

Planned checks: a tape on the floor slid until it just enters the bottom of the image (measures the blind zone directly); a start-position reference frame compared before each round (the hinge is glued, but this catches knocks).

## 5. Lighting and power additions

- **LEDs:** 6 × WS2812 on the front frame; data from ESP32 GPIO4; power from the 5 V Buck-2 rail (team). The firmware lights them white at `setBrightness(50)`, about 20 %: roughly 12 mA per LED and 0.07 A for all six by the 60 mA-per-LED full-white rule of thumb in the [Adafruit NeoPixel Überguide](https://learn.adafruit.com/adafruit-neopixel-uberguide) (derived). Full white would be 0.36 A.
- **Switch and button:** one power switch on the battery lead; the start button on the rear beams (team). The rules require exactly one power switch and one start button ([2 — Power & Sensors §6](2_power_and_sensors.md)).

### Power budget, measured 2026-09-21

Each branch was measured on its own lead from the pack (ammeter and voltmeter, pack at 12.3 V). Pack currents for the two 5 V converters are given at the nominal 11.1 V, the conservative case used in [2a](2_power_predicted_budget.md), with the converters assumed 88 % efficient; the motor branch is as measured.

| Branch | Feeds | Basis | Idle | Driving | Worst case |
|---|---|---|---|---|---|
| Buck-2 (5 V) | 3 TF-Luna, servo, 6 LEDs | measured at its input | 0.13 A | 0.23–0.32 A (servo sweeping) | 0.32 A |
| Fast-charge module (5 V) | Pi 5, camera, ESP32 | measured peak, 11.8 W at the Pi | 1.21 A | 1.21 A | 1.21 A |
| TB6612FNG | drive motor | measured | 0 | 0.49 A (0.599 A × 82 % duty) | 1.24 A (stall) |
| **Pack total** | | | **1.34 A** | **1.93–2.02 A** | **2.77 A** |

With the Pi's peak treated as its average, a 2200 mAh pack used to 80 % gives about 52 minutes of driving, about 17 three-minute runs (derived). The prediction for the Nationals car in [2a](2_power_predicted_budget.md) was 0.95–1.40 A driving; the difference is the Pi (an 11.8 W peak against 6.2–8.5 W predicted as an average) and the larger motor.

| Part | Load | Rating (vendor) | Use |
|---|---|---|---|
| TB6612FNG, one channel | 1.51 A through the bridge at stall; 0.60 A driving | 1.2 A continuous, 3.2 A peak | **126 % at stall**; 50 % driving |
| Pi 5 USB ports without USB-PD | camera + ESP32 | 600 mA in total | see §6 |
| Battery, 3S 2200 mAh 60C | 2.77 A worst case | 132 A | 2 % |

## 6. New failure points

| Risk | Effect | Check or mitigation |
|---|---|---|
| Pi 5 fed from a USB-A fast-charge module (no USB-PD) | The Pi peaked at 11.8 W (2.36 A at 5 V, measured) and the module's rating has not been read; without USB-PD the Pi limits its USB ports to 600 mA in total, and the camera and the ESP32 both draw from them | Run `vcgencmd get_throttled` after a full run; move to a 5 A USB-C PD supply if it reports problems |
| WS2812 data at the ESP32's 3.3 V | The LED input-high threshold is 0.7 × VDD (3.5 V at 5 V); a marginal signal can flicker or show the wrong colour, tinting the scene | Watch for flicker in the camera image; add a level shifter if seen |
| 6 V motor on the 11.1 V pack | Heat and brush wear: `SPEED = 210` averages 9.1–10.4 V, above the 6 V rating (§3) | Temperature check after a 3-minute run |
| Drive motor held against a wall | 1.51 A through the TB6612FNG at stall, above its 1.2 A continuous rating (§5); the driver's thermal shutdown or the motor's heating would stop the car | The Pi's distance logic backs away (escape at 31 % duty for 0.30 s, at least 3 cm of progress, at most 2 attempts); on most surfaces the rear wheels spin before a full stall (§3) |
| Firmware front stop: the ESP32 stopped the drive motor when the front TF-Luna read under 18 cm or its reading went stale | **Observed:** it stopped the car and ended a run (team) | Removed from the firmware on 2026-09-10; collision handling moved to the Pi (§7) |
| Camera angle | Pixel-based settings shift if the angle moves | Hinge glued; reference frame before each round |
| Background above the walls | Off-field red or green objects read as pillars | Search only below the wall band |
| Blind zone ahead of the wheels | The nearest-pillar rule ties for boxes cut off at the bottom | Tie-break rule in the code |

## 7. Why each change was made

Reasons as the team gives them, with the numbers available so far. The before/after test behind each one is still to be added.

| Change | Stated reason | Numbers so far |
|---|---|---|
| Horizontal side rollers | keep the car moving if it touches a wall | — |
| Non-Ackermann steering column | a smaller turning circle | turning circle still to be measured |
| Rear tyres | more grip | — |
| Camera raised | a better view of the track | range signal about 2.2 times stronger; look-ahead ~1.2 m (§4) |
| New frame | a sturdier structure | — |
| Front LEDs | fewer colour mistakes | — |
| Front stop removed from the firmware (2026-09-10) | stopping the drive motor ends a competition run, and it ended one | collision handling now on the Pi's distance logic (§6) |

**The 2026-09-10 front-stop decision.** The firmware used to stop the drive motor when the front TF-Luna read under 18 cm (released at 24 cm) or when its reading went stale, and that stop ended a run. On 2026-09-10 `forwardHardStopIsActive()` was changed to return `false`, with the comment "Disabled: stopping the drive motor here ends a competition run". Collisions are now handled on the Pi: back away at speed 80 for 0.30 s, require at least 3 cm of progress, at most 2 attempts, release at 30 cm. The team kept the earlier versions as backups, which are not committed: `obstacle-challenge-final-code_withLED.ino.before-remove-front-brake-20260910.bak` (MD5 `a0f20dd94d2dce1a27c2fc72e3700922`) and `obstacle-challenge-final-code_withLED.ino.before-parking-override-20260910.bak` (MD5 `c93dd67690d2153980442369aa3df41d`). The same day added a `PARKING_OVERRIDE_ON` serial command to the firmware; `main.py` never sends it, so it has no effect in a run.

## 8. ESP32 pin map

From the Nationals firmware ([`round2_ino.ino`](../src/Round%202/round2_ino/round2_ino.ino)) and the 2026-09-20 wiring draft; only GPIO4 is new. Checked against the APAC firmware ([`src/apac-2026/`](../src/apac-2026/)) on 2026-09-21: it reads the centre TF-Luna on mux channel 3 (channel 1 at Nationals).

| Signal | GPIO |
|---|---|
| I²C SDA / SCL — PCA9548A at `0x70`: ch0 left, ch3 centre, ch2 right TF-Luna (`0x10`), ch4 BNO055 (`0x28`) | 21 / 22 |
| Steering servo | 13 |
| TB6612FNG AIN1 / AIN2 / PWMA / STBY | 25 / 26 / 33 / 27 |
| WS2812 data | 4 |
| Start button | 32 |

## 9. Still to add

- Race-ready six-view photos (front, back, left, right, top, bottom). The shots in [`v-photos/apac-2026-09-20/`](../v-photos/apac-2026-09-20/) are working photos with cables attached.
- Open Challenge and Obstacle Challenge videos of this vehicle.
- The Open Challenge code for this vehicle; [`src/apac-2026/`](../src/apac-2026/) holds the Obstacle Challenge stack.
- Measurements: turning circle at walking pace and race speed, the blind-zone tape check, motor temperature after a full run, `vcgencmd get_throttled` after a full run.
- A before/after test for each change in §7.
