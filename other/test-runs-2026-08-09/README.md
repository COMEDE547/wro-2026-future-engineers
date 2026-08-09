# Drive-speed test runs — 2026-08-09

Two runs on the Obstacle Challenge mat, recorded on the same day at two motor
speed settings. They are kept together because the pair is the evidence: the
same firmware and the same track, changed in one variable.

**These are not the competition demonstration videos.** Those are linked from
[`video/video.md`](../../video/video.md) as required. What follows is test
footage retained as engineering record.

| File | Speed setting | Length | Outcome |
|---|---|---|---|
| `speed-400-clean-run.mp4` | 400 | 50.0 s | Runs without intervention for the whole recording |
| `speed-700-leg2-wall-strike.mp4` | 700 | 15.1 s (driving from ≈9.3 s) | Strikes the inner block after the first turn |

## What the failing run shows

Measured from the recording (30 fps):

| Time | Event |
|---|---|
| ≈9.3 s | Motion begins; setup and handling occupy everything before this |
| 10.2 – 12.2 s | Leg 1, straight and stable, tracking parallel to the wall |
| 12.4 – 13.0 s | First turn executes at the corner |
| ≈13.3 s | Heading continues past the new leg direction instead of settling |
| ≈13.6 s | Contact with the inner block corner |
| 13.6 – 15.0 s | Stationary against the block |

The vehicle reaches the inner wall roughly 1.1 s after the turn. The corridor is
1000 mm wide, so from mid-corridor there is about 500 mm of lateral margin to
lose. Crossing that in 1.1 s implies a post-turn heading error in the region of
30–40°, which is the useful part of the measurement: a steering loop with too
much gain oscillates by a few degrees and diverges gradually, whereas an error of
tens of degrees held steadily is a wrong heading *target*, not a poorly damped
one.

## Candidate causes, in the order they are worth testing

1. **Heading reference.** If `snapToCardinal` is resolving to absolute compass
   points while the IMU booted at an arbitrary magnetic heading, leg 1 is correct
   by construction — it *is* the boot heading — and every later leg inherits the
   offset between the boot orientation and the absolute grid. That predicts
   precisely what the recording shows: a clean first leg, then a wall strike on
   the second. It also explains why speed matters without being about speed: the
   error exists at both settings, but at 400 the wall-following correction has
   the time and distance to recover, and at 700 it does not.
2. **Speed-invariant steering gain.** Yaw response scales with forward speed, so
   400 → 700 raises effective loop gain by about 1.75×. Adequate damping at the
   lower setting can become oscillatory at the higher one with no code change.
3. **Loop latency.** Any fixed delay costs proportionally more distance at higher
   speed. The known contributor is the per-loop telemetry line: roughly 120
   characters at 115200 baud is about 10 ms of blocking transmission inside a
   nominally 20 ms loop.

## The test that separates them

Start the vehicle deliberately rotated 30–40° away from the first leg direction
and run at **400**. Cause 1 depends on boot orientation and not on speed, so if
the vehicle fails at the low setting when started rotated, the heading reference
is confirmed and causes 2 and 3 are not what is being observed. If it runs
cleanly, attention moves to gain and latency.

Serial telemetry captured during a failing run would also resolve it directly,
since the heading target is printed each loop and a target that does not match
the wall direction is visible in the log.

## Open

The firmware revision flashed during these runs is not recorded. Until that is
confirmed, the first cause above is a hypothesis consistent with the recording
rather than a diagnosis, and no entry has been made in the risk register on the
strength of it.
