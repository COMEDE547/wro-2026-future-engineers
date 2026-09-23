# APAC 2026 test logs

**Scope.** What the car's own logs record from 15 to 23 September 2026: 260 runs of `main.py` (`round2_run_*.csv`, one row per telemetry sample and one per event, written when `ENABLE_RUN_CSV_LOGGING = True`) and 32 attempts of `parallel_parking.py` (`parking_run_*.csv`, `ENABLE_RUN_LOGGING = True`). Every figure here is computed from those files by [`run_metrics.py`](../src/apac-2026/tools/run_metrics.py). The per-run summary is [`data/run_summary_2026-09.csv`](data/run_summary_2026-09.csv), the parking attempts are in [`data/parking_runs_2026-09-20/`](data/parking_runs_2026-09-20/), and [`data/example_full_run_20260919_192712.csv`](data/example_full_run_20260919_192712.csv) is the fastest full run, whole. The code was tuned from day to day, so each day's figures belong to that day's settings.

## 1. Full runs from the parking lot

A run counts here when it starts with the parking exit (first state PARKING_EXIT_DIRECTION). It reached the twelfth corner when `corner_counter` reached 12; the time is counted from the first drive command. Runs the team stopped by hand are included, so the counts are attempts, not a success rate.

| Day | Runs | Reached the 12th corner | Time to the 12th corner, s: median (range) |
|---|---|---|---|
| 15 Sep | 22 | 1 | 241 (241–241) |
| 16 Sep | 53 | 2 | 225 (201–248) |
| 17 Sep | 15 | 2 | 206 (204–208) |
| 18 Sep | 39 | 9 | 180 (165–190) |
| 19 Sep | 51 | 10 | 167 (154–184) |
| 20 Sep | 7 | 3 | 176 (168–180) |

From 15–17 to 18–20 September the time to the twelfth corner fell from 201–248 s to 154–190 s (median 176 s over 22 runs); 14 of those 22 runs reached the twelfth corner inside 180 s, the length of a round.

## 2. Why full runs stopped, 18–20 September

The code prints a reason with every FAULT. Reasons in the full runs of 18–20 September:

| Reason | Runs |
|---|---|
| corner clearance reverse made no reliable progress | 7 |
| parking-exit forward heading alignment timeout | 4 |
| front reached 5 cm before parking alignment | 1 |
| unknown Round 2 state STRAIGHT | 1 |
| parking-exit reverse heading alignment timeout | 1 |
| red approach exceeded 4s without reaching distance trigger | 1 |

The most frequent, the corner-clearance reverse that made no progress, is the stall of a car pressed against a wall at a corner.

## 3. Parking

**U-turn parking-in in `main.py`, 17 September.** 24 test runs of the U-turn routine; 0 reached COMPLETE. The faults were clearance limits during the U-turn (U-turn forward clearance only N cm: 5; U-turn side clearance only N cm: 3; U-turn heading did not complete; progress=N: 1). Earlier that evening, 10 runs of the timed routine (reverse at full left lock, then right) reached COMPLETE in 9; the log cannot show whether the car ended inside the space.

**`parallel_parking.py`, 20 September.** 32 attempts from 14:36 to 17:35; 15 reached COMPLETE, in a median of 31 s (20–88 s). In the last session, from 16:30, 7 of 11. The attempts that did not finish stopped in TRACK_OPENING (5), ALIGN_VIRTUAL_CUTOUT (5), ENTER_SPACE (3), PARALLEL_ALIGN (2), PASS_LAST_PILLAR (1), FAULT (1).

**Three laps, then parallel parking, 20 September.** In 3 runs `main.py` handed over after the twelfth corner (PARKING_PARALLEL_HANDOFF) and `parallel_parking.py` took over; 2 of them ended parked:

| Start | Time to the 12th corner, s | Parking | From the first drive command to the end, s |
|---|---|---|---|
| 17:12 | 168 | COMPLETE after 58 s | 226 |
| 17:26 | 180 | TRACK_OPENING after 13 s | 193 |
| 17:32 | 176 | COMPLETE after 88 s | 264 |

At those speeds three laps plus parking took longer than a three-minute round.

## 4. After the side rollers came off, 22 September

7 short runs in the evening, each started on a straight (DETECT_DIRECTION) with a 2-corner limit, as the counters show; 5 reached COMPLETE. CORNER_CLEARANCE_RECOVERY ran in 5 of them, and in 5 the car went on to make its turn.

## 5. What the logs do not show

- Whether a pillar or a wall was touched: the logs record states, distances and commands, not contacts.
- How squarely the car ended in the parking space.

## 6. Performance videos

| Challenge | Video | Run |
|---|---|---|
| Open Challenge | [youtu.be/PbOQJ54wr-0](https://youtu.be/PbOQJ54wr-0), 33 s | India National Championship, August 2026 (first vehicle) |
| Obstacle Challenge | [youtu.be/dBjqMQ-jRBA](https://youtu.be/dBjqMQ-jRBA), 100 s | India National Championship, August 2026 (first vehicle) |

Additional footage: [`video/apac-2026-obstacle-run.mp4`](../video/apac-2026-obstacle-run.mp4), 3 min 19 s, shows the APAC vehicle on the practice mat, about 20 September: from the parking lot, three laps (12 corners), then a stop after the twelfth corner.
