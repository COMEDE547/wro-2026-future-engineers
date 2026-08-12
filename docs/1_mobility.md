# 1 — Mobility & Mechanical Design

**Honest status: the vehicle is built to its Round-1 configuration — chassis,
single-servo Ackermann steering, and an N20 drive through a LEGO differential
are physically fitted, with all three TF-Lunas and the IMU mounted. The
Raspberry Pi 5 and camera are not yet on the vehicle. The gear-ratio, wheel and
battery specs are now read off the hardware and the vendor listing (2026-08-06,
tables below); the working point is still unmeasured, and the first sustained
drive exposed a wheel-retention failure (§3).** The critical-path gap (risk R10)
has narrowed from "no build" to "no measured working point", and this document
does not write around it.

---

## 1. What exists

### Steering — single-servo Ackermann

| Parameter | Value | Source |
|---|---|---|
| Steered axles | 1 (front) | WRO FE requires steered, not skid, steering |
| Actuator | Servo on `GPIO13` | `src/Round 1/round 1/round 1.ino` |
| Servo model | **MG90, metal gears** | read off hardware 2026-08-06 |
| Pulse range | 500-2400 us at 50 Hz | firmware |
| Angle range | 64-136 deg, 106 = straight — asymmetric about center (−42/+30); retuned 2026-08-10 (`58adb1c`), was 45-135 / 90 | firmware, `constrain()` |
| Control law | proportional on heading error, `STEER_GAIN = 1.0` servo-deg per deg of error + 2.0 deg deadband; retuned 2026-08-10, was 1.5 / no deadband | firmware |
| Loop rate | ~50 Hz nominal (20 ms delay; effective rate unmeasured) | firmware |

### A number that falls out of those two

`STEER_GAIN = 1.0` with the asymmetric −42/+30 deg travel about center means the
steering **saturates at 42 deg of heading error toward one side and 30 deg toward
the other** (retuned 2026-08-10; the original 1.5-gain / ±45 tune saturated at
30 deg both ways). Beyond saturation the vehicle is at full lock and the
controller is open-loop until the error falls back under the limit.

That is intentional and it sets the corner behaviour: a 90 deg heading step at a
corner puts the controller into saturation immediately and holds full lock through
the first two thirds of the turn, which is the fastest legal way through it. It
also means `STEER_GAIN` cannot be tuned for corner response — corners are
saturated regardless — so the gain is free to be tuned purely for straight-line
stability.

### Why proportional only

No derivative term: the heading signal is differentiated on-chip already and a D
term on a 50 Hz loop amplifies quantisation into servo chatter.
No integral term: the target heading is a **step** function that jumps 90 deg at
every corner, and an integrator winds up across each step and then overshoots the
recovery. Steady-state heading error on a straight is bounded by servo resolution,
not by the absence of an I term.

### Drive — N20 via TB6612 (chosen 2026-08-01, integrated 2026-08-03)

| Parameter | Value | Source |
|---|---|---|
| Motor | **N20 (GA12-N20-600 class): 12 V rated, 600 RPM no-load, 1:50 gearbox; 0.18 kg-cm rated / 0.65 kg-cm stall torque; 0.06 A rated / 0.75 A stall** — datasheet-class figures, pending our own §6 measurement | vendor listing (robu.in), read 2026-08-06 |
| Reduction | **20T pinion -> 28T ring gear on the LEGO differential (62821; 12T internal half-bevels) = 5:7 external reduction; wheels ~71 % of motor speed** | teeth counted off hardware 2026-08-11 |
| Transmission | spur pinion into a LEGO differential on the rear axle; external pinion:crown tooth ratio **20:28, counted 2026-08-11** — closes the speed calculation (was the one missing number) | bottom view below + hardware count |
| Driven axle / wheels | rear axle, both wheels through the differential; **rear 55.6 x 14 mm, front 41 x 21 mm** | measured 2026-08-06 |
| Battery | **3S Li-Po, 11.1 V nominal, 2200 mAh 60C** (the 2026-08-06 note said 2600 — label re-read 2026-08-11, 2200 is what is printed on the pack) | read off pack 2026-08-11 |
| Driver | TB6612FNG, channel A | `src/Round 1/round 1/round 1.ino` |
| Pins | AIN1 `GPIO25` · AIN2 `GPIO26` · PWMA `GPIO33` · STBY `GPIO27` (or tied 3V3) | firmware |
| PWM | 20 kHz, 10-bit (0-1023), cruise duty 1000 — retuned 2026-08-10 (`58adb1c`), was 550; 98 % duty leaves no headroom and drifts as the pack sags | firmware |
| Stop logic | an observer counts the 90-deg heading steps the corner logic makes; after 12 turns and heading settled within 15 deg (4 s failsafe), a 500 ms timed run-on (was 1500 ms; retuned 2026-08-10), then short-circuit brake | firmware |

Integration is append-only: the original steering / corner logic is
byte-untouched, and the module observes `targetHeading` steps rather than
modifying the trigger code. Physical bring-up — direction check, `MOTOR_INVERT`,
duty tune on the mat — ~~is pending the build~~ **happened 2026-08-08/09**
(test footage under `other/`); cruise duty retuned to 1000 in `58adb1c`.

![Chassis prototype during steering-centre calibration](img/chassis-prototype-steering-bringup-1.jpg)
*Chassis prototype during steering bring-up — servo tester holding the 90-deg
centre position. Prototype hardware; the competition chassis configuration is
not yet frozen. — Superseded 2026-08-10: this frame IS the final competition
chassis (rebuild cancelled, see below), and the mat-tuned centre moved to 106.*

![Underside: N20 into the LEGO differential](../v-photos/vehicle-bottom.jpg)
*Underside of the built vehicle: the N20's pinion drives a LEGO differential on
the rear axle; the front Ackermann linkage and printed servo mount are at the
bottom of frame.*

---

## 2. What does not exist

| Item | State | Blocks |
|---|---|---|
| Drive motor + driver | **Chosen — N20 via TB6612**, integrated in firmware; specs on record above | Flash test + direction check + duty tune still pending |
| Chassis | **Built and FINAL** — Lego Technic hybrid (plastic, printed PLA brackets for sensors / motor), Round-1 configuration; ~~rebuild to a fully 3D-printed chassis decided 2026-08-06~~ **rebuild CANCELLED 2026-08-10 on retention evidence** (section 3); team CAD in `models/` since 2026-08-11 | Mass measurement, camera mounting |
| Gearing | **Closed 2026-08-11** — integrated ratio 1:50 (datasheet); external stage counted on the vehicle: **20T pinion -> 28T crown = 5:7 (R_ext = 1.4)** | Working point now computable — see the speed calculation |
| Wheels and tyres | **Measured** — rear 55.6 x 14 mm, front 41 x 21 mm | Traction limit; total mass still unmeasured |

`models/` holds the team CAD since 2026-08-11/12: TF-Luna enclosure, steering
horn beam and N20 clamp (originals, Fusion 360), the initial-design LXFML and
a 69-page build-instructions PDF. Third-party camera-mount parts are cited in
`models/README.md`, not redistributed.

---

## 3. How the drivetrain will be chosen

The method was fixed before the motor family was chosen and still governs the
open part of the choice: the N20 + TB6612 pick constrains the space, but ratio,
wheels and speed remain to be derived. Writing it down first means the selection
can be criticised before money is spent.

### Hard bounds

| Bound | Value |
|---|---|
| Envelope | 300 x 200 x 300 mm |
| Mass | <= 1.5 kg total, including battery |
| Steering | Ackermann, single steered axle |

### Working point to be derived, in this order

1. **Target speed** from the run budget — three laps within the time limit, minus
   the time cost of the pass manoeuvres and the parking routine.
2. **Wheel diameter** -> required wheel RPM at that speed.
3. **Traction-limited torque** from mass and tyre-surface friction on the mat.
   This is the ceiling; torque above it spins the wheels and buys nothing.
4. **Motor + gear ratio** selected so the *continuous* operating point sits inside
   the motor's efficient band, not at its stall end. Stall torque is a
   specification, not an operating point.
5. **Current draw measured**, not summed from datasheets, and fed into
   [2 — Power & Sensors](2_power_and_sensors.md#6-power-budget).

**Where the derivation stands (2026-08-06):** at the pack's 11.1 V nominal the
no-load gearbox output is ~600 x (11.1 / 12) ≈ **555 RPM**; with 55.6 mm rear
wheels the theoretical no-load ceiling is `(555 / 60) x pi x 0.0556 / R_ext` ≈
**1.6 m/s / R_ext**; with the counted external stage (20T:28T, R_ext = 1.4)
this gives **~1.15 m/s** as the unloaded upper bound on top speed.
Counting two gears closes the calculation; a tape-measure speed run on the mat
replaces it with a measured number.

### Torque, traction and acceleration (derived 2026-08-13)

Speed above answers "how fast can it go." This answers "can it get there, and
what stops it first." Every input is either measured on the vehicle (ratio,
wheel diameter) or the datasheet-class motor figure from §1 — none is a bench
measurement of this motor, and the section is labelled accordingly.

**Torque at the wheel.** The external stage multiplies motor torque by
`R_ext = 1.4` (20T pinion into a 28T ring, counted on the vehicle 2026-08-11):

| | Motor (datasheet-class) | × 1.4 → wheel | Tractive force at r = 27.8 mm |
|---|---|---|---|
| Rated | 0.18 kg·cm = 0.0177 N·m | 0.0247 N·m | **0.89 N** |
| Stall | 0.65 kg·cm = 0.0637 N·m | 0.0892 N·m | **3.21 N** |

**What stops the vehicle first — and it is genuinely close.** Traction limit is
`μ × N_rear`. Total mass is capped at 1.5 kg by rule and **has never been
weighed**; taking a 1.0–1.5 kg band and 50–60 % of weight on the driven axle
gives `N_rear ≈ 4.9–8.8 N`, and rubber on a painted mat is roughly μ = 0.5–0.9:

`F_traction ≈ 2.5 N (light vehicle, slick mat) … 7.9 N (heavy, grippy)`

Against a stall tractive force of **3.21 N**, the design sits *on the boundary*.
At the pessimistic end the wheels break traction before the motor stalls; at the
optimistic end the motor stalls before the tyres let go. **Weighing the vehicle
and noting the front/rear split settles which regime it is in — that single
measurement is worth more to this chapter than any other.** The distinction is
not academic: it decides whether a wall strike stalls the motor (current spike,
TB6612 thermal event) or spins the wheels (no spike, but no recovery either).

**Acceleration.** Motor torque falls roughly linearly from stall toward rated as
speed rises, so a mid-ramp average of ~2.0 N on 1.25 kg gives ~1.6 m/s². Reaching
the derived 1.15 m/s ceiling therefore takes **~0.7 s over ~0.40 m** — about half
a corridor width, which is why the vehicle is at speed well before the first
corner and why `FINAL_RUN_MS` behaves as a distance rather than a ramp.

**Does the ratio give enough speed?** The driving corridor is ~1000 mm inside a
3200 mm mat, so a lap centre-line is roughly `4 × 2.2 m ≈ 8.8 m` and three laps
≈ **26 m**. At the derived 1.15 m/s that is **~23 s of driving** before corner
slowdowns and pass manoeuvres. The 5:7 external stage is therefore not the
binding constraint on round time — which is the justification for leaving it at
5:7 rather than re-cutting for speed.

**What would overturn this section:** a weighed vehicle (settles the traction
regime), a measured stall current (converts the datasheet stall torque into a
figure for *this* motor), or a tape-measure speed run (replaces the 1.15 m/s
derivation outright). All three are single-session measurements and all three
are listed in §4.

### The test that will change the design

Corner exit at full steering saturation is the binding mechanical case: it
combines maximum lateral load with the highest steering-rate demand. The
acceptance test is a repeated corner-exit run measuring understeer at the chosen
speed. If the vehicle cannot hold the post-corner line, the fix is mechanical —
weight distribution, tyre compound, or Ackermann geometry — not a gain change,
because the controller is saturated through that phase and gain has no authority
there.

This is written before the build so that the result cannot be rationalised
afterwards.

**First observed failure (2026-08-06), logged before any fix exists:** under
sustained drive the LEGO-mounted wheels shed from their axles within seconds.
Wheel retention is therefore the first mechanical acceptance gate — ahead of
the corner-exit test above, which cannot even be attempted until the wheels
stay on. The retention fix is **decided (2026-08-06): a full chassis rebuild to
a 3D-printed frame** with proper hubs and axle retention, replacing the LEGO
axle interface that sheds under load; when the printed chassis lands with a
before/after drive test, it becomes the drivetrain's first test-caused design
change - and puts committable CAD in `models/`.

**Superseded 2026-08-10: the *printed* rebuild is cancelled — it was not needed.**
Retention was resolved instead by a **ground-up redesign of the LEGO space-frame
itself**, built structurally tighter and more rigid than the first layout (the
same redesign recorded in [`models/README.md`](../models/README.md), which
replaced an initial layout that could not package motor and battery together,
carried too high a centre of mass, and would not hold a curve). The wheels were
never the root cause: a frame that flexes under drive torque works the axle
interface loose, and stiffening the frame removed the load path that was
shedding them.

The before/after is the test that changed the design, and it is on record: the
failure was seconds of sustained drive on 2026-08-06; the two 2026-08-09 Round-1
runs on the redesigned frame are 28 s and 29 s of continuous driving with no
wheel loss
([`other/test-runs-2026-08-09-round1/`](../other/test-runs-2026-08-09-round1/)),
roughly **six times** the failure duration. Every run and every committed
artifact since then is on this frame, and it is final.

Because the answer was a LEGO redesign rather than a printed one, the
committable CAD is the LEGO design file plus the printed mounts the frame
carries (3× TF-Luna, servo, N20), not a chassis STL — there is no printed
chassis and there never was. Residual gates: one full three-lap run, and the
wall-strike stall case.

**Impact path (answered 2026-08-06):** there is no sacrificial element and no
bumper — a hard wheel strike feeds force directly into the steering servo
(MG90, metal gears), and a head-on puts the chassis first against the wall.
Nothing has broken so far and no crash-driven design change exists yet;
whether a cheap fuse-part or bumper is worth its mass against the 1.5 kg
budget is an open trade.

---

## 4. Open

- **Wheel retention under drive torque — critical path** (observed failure, §3).
- The drivetrain working point: ~~count the pinion:crown teeth~~ counted
  2026-08-11 (20:28 = 5:7); measure speed and stall current on the mat. Motor,
  driver, wheels and pack are all now on record.
- Camera mount at ~100 mm height and 10-17 deg pitch has a geometric
  justification ([2 — Power & Sensors](2_power_and_sensors.md#3-camera-placement-justified-by-field-geometry))
  but no physical bracket; it depends on the chassis.
- ~~CAD for `models/`~~ — **team CAD committed 2026-08-11/12** (enclosure, horn, N20 clamp, LXFML, build-instructions PDF). The signal-wiring schematic is now in `schemes/`; the
  power-tree schematic was committed 2026-08-10/11 (`schemes/` — generated wiring v0.2 plus the as-built hand diagram).
- ~~Six vehicle photos for `v-photos/`~~ — **done 2026-08-05**, six views
  committed.
