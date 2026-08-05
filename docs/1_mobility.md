# 1 — Mobility & Mechanical Design

**Honest status: steering is implemented and tuned. Propulsion is not chosen.**
The vehicle holds and corrects a heading but cannot yet drive itself forward.
This is the critical-path hardware gap (risk R10) and this document does not
write around it.

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

---

## 2. What does not exist

| Item | State | Blocks |
|---|---|---|
| Drive motor | **Unchosen** | Everything below, plus the power budget |
| Chassis | **Unchosen** | Mass budget, camera mounting, CAD in `models/` |
| Gearing | **Unchosen** | Speed / torque working point |
| Wheels and tyres | **Unchosen** | Traction limit, effective gear ratio |

`models/` is empty for this reason. It is not an oversight — there is nothing to
put in it yet.

---

## 3. How the drivetrain will be chosen

The method is fixed even though the choice is not. Writing it down now means the
selection can be criticised before money is spent.

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

- Drive motor and chassis selection — **critical path**.
- Camera mount at ~100 mm height and 10-17 deg pitch has a geometric
  justification ([2 — Power & Sensors](2_power_and_sensors.md#3-camera-placement-justified-by-field-geometry))
  but no physical bracket; it depends on the chassis.
- CAD for `models/`, wiring schematic for `schemes/`.
- Six vehicle photos for `v-photos/` — blocked on a built vehicle.
