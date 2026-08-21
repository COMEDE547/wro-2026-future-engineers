# Round 2 firmware — `round2_ino.ino`

ESP32 obstacle-challenge firmware, built directly on `src/Round 1/round 1/round 1.ino`.
All Round 1 behaviour is preserved — heading-hold P-steering off the BNO055, corner
detection from a side TF-Luna opening, the TB6612/N20 drive module, the GPIO32 no-touch
start, and the 12-turn finish. Everything added for the obstacle round is marked
`// === R2 ===`.

**Separate lineage from `../main.cpp`.** That file is untouched.

## What the obstacle logic does

| Situation | Behaviour |
|---|---|
| Pillar on a straight | Heading target shifts `SWERVE_DEG` (12°) toward the pass side, ramped in at ~30°/s — a gentle lane change over a lot of track, not a turn. Side lidar zeroes the drift within `WALL_MIN_CM` of a wall. |
| Pillar at a corner, pass side **outside** the turn | **Wide:** hold straight `WIDE_DELAY_MS` past the opening, then the normal 90°. The later turn swings the arc out toward the outer wall. Front wall inside `WIDE_ABORT_FRONT_CM` cancels the delay and turns immediately. |
| Pillar at a corner, pass side **inside** the turn | **Narrow:** turn 90° at once plus a temporary `NARROW_CUT_DEG` bias into the corner, expiring on time or once the heading settles. |
| Front blocked with a pillar active | Brake, reverse straight (blind, short), drive forward aimed toward the pass side, resume. |
| Start of the run | Parking-bay exit: full-lock forward arc toward the track alternated with opposite-lock reverse arcs (parallel-park kinematics — rotation accumulates one way), until the nose is `EXIT_HEADING_DEG` off the lane, then an angled drive-out and heading-hold pulls it straight. |

Wide and narrow are **derived at runtime** from pillar colour × turn direction, so the same
binary works whichever way the lap runs. Red is passed on the right, green on the left.

Swerve and corner-cut offsets are applied at the steering call and never written into
`targetHeading`, so the lap counter cannot be corrupted by them.

## Serial input

Newline-terminated `R`/`RED`, `G`/`GREEN`, `C`/`CLEAR` at 115200 from the Pi
(`../pi_sender/pi_sender.py`). Whole tokens are matched — a character-level parser would
read the second letter of `GREEN` as red. No valid line for `PI_TIMEOUT_MS` (1.5 s) falls
back to `CLEAR`, so a dead Pi degrades the run to Round-1 driving rather than freezing on a
stale command.

## Before flashing

- **`EXIT_DIR`** — `+1` if the track is to the robot's right at the start, `-1` if left.
  Compile-time, so a direction change means a reflash.
- **`BOT_LENGTH_CM`** — measured at 23 cm. The bay is 1.5× that, so there are only ~11.5 cm
  of free space ahead of the robot at the start. `EXIT_FRONT_STOP_CM` derives from it and
  **must stay below the gap** — a front-stop larger than the gap breaks the forward leg on
  its first check and the robot never leaves the bay.
- **`EXIT_FWD_MS_MAX`** — the bay is governed by time, not by the lidar (a TF-Luna is
  unreliable under ~20 cm). Calibrate it: drive straight at `EXIT_SPEED` for one second,
  measure the distance, and set the cap so one forward leg covers roughly 8 cm.
- Bench the servo lock direction against the sign of `EXIT_DIR`, and the reverse polarity,
  before putting the robot in a bay.

## Known limits

- No wall-crash recovery when no pillar is active — inherited from Round 1; a missed corner
  is still unrecoverable.
- Both reverse legs are blind. There is no rear sensor, so their durations are the only
  thing stopping them.
- A corner opening that passes during the escape manoeuvre is a missed turn.
- Parking and the final run shape are not implemented — the run ends with the Round 1
  12-turn stop, marked `TODO phase 2` at the brake.
