# Round 1 (Open Challenge) test runs - 2026-08-09 evening

Two runs on the Open Challenge layout, recorded the same evening. **The firmware
was the same in both** - the revision committed alongside this directory - and
the only deliberate variable is the **inner-wall configuration**: how many of the
extendable centre wall sections were pushed out. That makes this pair the first
controlled comparison in the repository. The
[2026-08-09 obstacle runs](../test-runs-2026-08-09) varied speed at fixed
firmware; the [2026-08-08 runs](../test-runs-2026-08-08) varied firmware and
cannot be compared to anything.

**These are not the competition demonstration videos.** Those are linked from
[`video/video.md`](../../video/video.md) as required. What follows is test
footage retained as engineering record.

| File | Inner-wall configuration | Length | Frame rate |
|---|---|---|---|
| `run-1-two-wall-extensions.mp4` | Two extended sections, on adjacent sides of the centre block | 28.1 s | 31.6 fps |
| `run-2-one-wall-extension.mp4` | One extended section | 29.2 s | 30 fps |

The configuration labels describe what is visible in the footage. They have not
been cross-checked against a measured field layout, so they identify the two
runs relative to each other and are not a claim about which official
randomisation each corresponds to.

No traffic-sign pillars are placed on the driving surface in either run. The
coloured blocks visible inside the centre block are stored there, as in the
2026-08-08 footage.

## What the footage establishes

- Both vehicles circulate the track under their own control for the whole
  recording. No frame at 0.5 s sampling shows the vehicle stalled, wedged, or
  being handled after the start.
- The corridor width differs between the two runs by construction, and the same
  steering constants were used in both. That is the comparison worth having.

## What the footage does not establish

**Neither recording captures the end of a run.** In the final frame of each, the
vehicle is still driving. Nothing here shows the twelfth turn, the
`FINAL_RUN_MS` run-on, or the `motorBrake()` stop, so the lap counter and the
stopping logic remain untested by this evidence. A run is only demonstrated when
the recording continues past the vehicle coming to rest.

Wall contact between sampled frames is also not excluded. Sampling was 2 s over
the body of each run and 1 s over the last eight seconds.

## Measurement attempt that failed, recorded so it is not repeated

Automated trajectory extraction was attempted on both clips and abandoned. Two
methods were tried:

1. **Median-background subtraction.** Fails because the camera is handheld and
   moving. The background model blurs and every static edge registers as motion.
2. **Colour segmentation on the vehicle, with ORB homography stabilisation.**
   Fails because the stored red and green blocks sitting on the centre block
   occupy the same hue range as the vehicle's own colouring, and the tracker
   locks onto them. Stabilisation quality also degrades whenever the operator
   walks through frame.

Both produced trajectories that were plainly wrong on inspection, so no number
derived from them appears above. This is the same conclusion the 2026-08-08
notes reached from the other direction: **video is the wrong instrument for
these measurements.** Serial telemetry is the right one. The firmware already
prints heading, target, servo angle and a `[turn] n/12` line every loop; a
captured log settles lap count, stop behaviour and steering residual directly
and does not depend on camera work.

If video measurement is wanted later, the cheap fixes are a tripod and a strip
of tape on the vehicle in a colour that appears nowhere else on the field.

## Firmware delta in this revision

Relative to the previous `main`:

| Constant | Was | Now |
|---|---|---|
| `SERVO_CENTER` | 90 | 106 |
| `SERVO_MIN_ANGLE` / `SERVO_MAX_ANGLE` | 45 / 135 | 64 / 136 |
| `STEER_GAIN` | 1.5 | 1 |
| `STEER_DEADBAND` | absent | 2.0 deg |
| `DRIVE_SPEED` | 550 | 1000 |
| `FINAL_RUN_MS` | 1500 | 500 |

**The steering correction also changed sign**, from
`SERVO_CENTER + STEER_GAIN * error` to `SERVO_CENTER - STEER_GAIN * error`. That
is the largest change in the diff and it is not a retune: with the previous sign
the loop drove the heading error further open rather than closing it, so the
Round 1 sketch as it stood on `main` could not hold a heading after a turn.

This does **not** explain the failure recorded in the
[2026-08-09 obstacle runs](../test-runs-2026-08-09). Those ran the Round 2
controller, `src/Round 2/main.cpp`, which is a separate implementation with its
own `SERVO_CENTER`, an explicit `INVERT_STEERING` flag and hand-branched signs at
each call site. It does not contain the expression corrected here, so the
`snapToCardinal` boot-heading hypothesis recorded there stands untouched. What
the two implementations do share is that steering sign is chosen by hand in both,
and it has now been found wrong in one of them.

Alongside the sign, `SERVO_CENTER` moved 90 -> 106 and the limits became
asymmetric about it (64 / 136, i.e. -42 / +30), which says the linkage geometry
changed too. Which of the two came first is not recorded.

Behavioural changes: the side LiDARs are muted for 1 s after a corner is taken,
so a single opening cannot re-trigger; heading is read and steering is applied on
every loop including during that mute; a start-button block is present but
commented out.

## Open

1. **Run-completion footage.** Both clips need a retake that runs past the stop.
2. **Serial log.** No telemetry was captured during either run. Until it is, the
   turn count reached in each configuration is unknown.
3. **`FINAL_RUN_MS` is stale.** The `~150 cm` in its inline comment was measured
   at `DRIVE_SPEED 550`. Speed is now 1000 and the value was cut to 500 ms, so
   the run-on distance has changed and has not been re-measured.
4. **`DRIVE_SPEED 1000`** is 98 % duty. There is no headroom left for the
   steering loop to ask for more, and the setting will drift as the pack sags.
5. **No sanity bound on the LiDAR reading.** `readDistance` returns the raw
   16-bit value and only `-1` is treated as a fault. A garbled I2C read that
   decodes large passes `> OPENING_CM` and books a 90 deg turn that never
   happened. The 1 s mute suppresses a repeat, not the first one. A reject band
   outside roughly 2-600 cm costs one line.
6. **Simultaneous left and right openings cancel.** If both fire in the same
   iteration, `targetHeading` nets to zero change, so `trackTurnsAndStop` sees no
   change and counts no turn while the vehicle keeps driving. Whether that can
   occur depends on the inner-wall configuration, which is exactly what these two
   runs varied.
7. **The commented-out start-button block does not work as written.**
   `while (btn == 1) { digitalRead(32); }` never reassigns `btn`, so enabling it
   hangs at boot. It needs `btn = digitalRead(32);` inside the loop. Worth fixing
   before it is needed, since the round requires a no-touch start.
