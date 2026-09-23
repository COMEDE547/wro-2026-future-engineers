# Parallel-park controller for the Nationals firmware (never run)

Written on 12-13 August 2026 on the branch `feat/round2-park` for the Nationals (Round 2) ESP32 firmware, and merged on 23 September 2026 as a record. It never ran on a mat and is not part of any build: the Nationals controller is still [`src/Round 2/main.cpp`](../../src/Round%202/main.cpp), which [3 - Software](../../docs/3_software.md) and the Nationals video describe.

| File | What it is |
|---|---|
| `main_park.cpp` | The branch's version of `src/Round 2/main.cpp`, with the parking states switched on (`PARK_ENABLED 1`) |
| `park_shimmy.h` | The shimmy parallel-park controller (branch commit `fe9f60c`) |
| `bench_teleop_park.ino` | A bench-only teleoperation sketch, never flashed at a venue |
| `PARK_NOTES.md` | The author's notes: the state machine, the rule facts, and the rule that kept it off `main` until a mat run |

The APAC vehicle parks with [`parallel_parking.py`](../../src/apac-2026/obstacle-challenge/parallel_parking.py) instead.
