# 1 — Mobility & Mechanical Design

**Honest status: steering is implemented and tuned; the drive motor and driver
are chosen and integrated in firmware (N20 via TB6612, append-only module). The
working point — gear ratio, wheels, chassis, battery — is not chosen, and no
vehicle is built.** The critical-path hardware gap (risk R10) has narrowed from
"no propulsion" to "no build", and this document does not write around it.

---

## 1. What exists

### Steering — single-servo Ackermann

| Parameter | Value | Source |
|---|---|---|
| Steered axles | 1 (front) | WRO FE requires steered, not skid, steering |
| Actuator | Servo on `GPIO13` | `src/Round 1/round 1/round 1.ino` |
| Pulse range | 500-2400 us at 50 Hz | firmware |
| Angle range | 45-135 deg, 90 deg = straight | firmware, `constrain()` |
| Control law | proportional on heading error, `STEER_GAIN = 1.5` servo-deg per deg of error | firmware |
| Loop rate | ~50 Hz | firmware |

### A number that falls out of those two

`STEER_GAIN = 1.5` with a +/-45 deg travel limit means the steering **saturates at
30 degrees of heading error** (45 / 1.5). Beyond that the vehicle is at full lock
and the controller is open-loop until the error falls back under 30 deg.

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
| Motor | N20 gearmotor — gear ratio and rated voltage pending spec | decision log |
| Driver | TB6612FNG, channel A | `src/Round 1/round 1/round 1.ino` |
| Pins | AIN1 `GPIO25` · AIN2 `GPIO26` · PWMA `GPIO33` · STBY `GPIO27` (or tied 3V3) | firmware |
| PWM | 20 kHz, 10-bit (0-1023), cruise duty 550 | firmware |
| Stop logic | an observer counts the 90-deg heading steps the corner logic makes; after 12 turns and heading settled within 15 deg (4 s failsafe), a 1500 ms timed run-on, then short-circuit brake | firmware |

Integration is append-only: the original steering / corner logic is
byte-untouched, and the module observes `targetHeading` steps rather than
modifying the trigger code. Physical bring-up — direction check, `MOTOR_INVERT`,
duty tune on the mat — is pending the build.

![Chassis prototype during steering-centre calibration](img/chassis-prototype-steering-bringup-1.jpg)
*Chassis prototype during steering bring-up — servo tester holding the 90-deg
centre position. Prototype hardware; the competition chassis configuration is
not yet frozen.*

---

## 2. What does not exist

| Item | State | Blocks |
|---|---|---|
| Drive motor + driver | **Chosen — N20 via TB6612**, integrated in firmware; bring-up pending build | Gear-ratio / voltage spec feeds the power budget |
| Chassis | **Unchosen** | Mass budget, camera mounting, CAD in `models/` |
| Gearing | **Unchosen** — the N20's integrated ratio, spec pending | Speed / torque working point |
| Wheels and tyres | **Unchosen** | Traction limit, effective gear ratio |

`models/` is empty for this reason. It is not an oversight — there is nothing to
put in it yet.

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

---

## 4. Open

- Chassis selection and the drivetrain working point (ratio / wheels) —
  **critical path**. Motor and driver are chosen.
- Camera mount at ~100 mm height and 10-17 deg pitch has a geometric
  justification ([2 — Power & Sensors](2_power_and_sensors.md#3-camera-placement-justified-by-field-geometry))
  but no physical bracket; it depends on the chassis.
- CAD for `models/`. The signal-wiring schematic is now in `schemes/`; the
  power-tree schematic is pending battery selection.
- Six vehicle photos for `v-photos/` — blocked on a built vehicle.
