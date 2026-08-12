# PARK CONTROLLER — main_park_2026-08-12.cpp (norm-hash e5ac30a23aca9e5a)

Base: committed `src/Round 2/main.cpp` @ ethan-dev `18d7edb`.
Patch: `round2_park_vs_18d7edb.patch` (applies clean to pristine 18d7edb).
Compiles: ESP32 core 2.0.17 + Adafruit_BNO055 1.5.3 — 302,553 B (23%) flash / 22,424 B (6%) RAM.
Pi side: NO changes. Parser accepts both `MAG,cx,h` (committed v2) and `MAG,cx,h,w` (v3).

## HARD RULE
Does NOT go into tonight's push. Tonight's 3_software patch states in three places
that no state consumes MAG and the park states do not exist. Committing this
alongside it is a self-made A5. Branch `feat/round2-park`; merge only after a mat
run, together with the 3_software/D7 supersession paragraph.

## State machine
DRIVING_STRAIGHT (parkArmed after turn 12)
  -> limiter passages counted two ways:
     (a) dropout: established -> close (h>=MAG_NEAR_H) -> stale >700ms
     (b) HANDOFF: fresh h collapses below 0.55x running max, 2 samples confirm
         (near limiter exits frame while far one is already tracked - no dropout ever fires)
  -> 2 passages -> PARK_PASS (900ms clearance)  |  1 passage + 2.5s quiet -> PARK_PASS solo (1600ms)
PARK_PASS   -> heading-held straight, rear axle clears the far limiter
PARK_ARC_IN -> reverse, full lock TOWARD wall; exit on |dyaw|>=42deg | wall-Luna<14cm | 3s
PARK_ARC_OUT-> reverse, counter lock; exit PARKED on |dyaw|<=8deg (after 400ms floor)
               | wall-Luna<5cm | 3s  -> ROBOT_STOPPED
Fallbacks from ANY state: 15s total cap, +2 extra corners, 6s no-limiter -> legacy wall stop.
Kill switch: PARK_ENABLED 0 = byte-identical legacy build.

## Rule facts (verified against the official 2026 PDF, Jan 15 2026 version)
- Lot ALWAYS in the starting section (sec.8 step 4, Fig 8d). Width 20cm from outer
  wall; length 1.5 x vehicle length; limiters 200x20x100mm magenta jut INTO the track.
- That section's pillars are moved toward the inner wall after lot placement (Fig 8e);
  side rules are OFF post-lap-3 (App A sec.5). Moving a sign is still banned.
- 1.8.2 = 15 (fully inside + wheel-to-wall delta <= 2cm both wheels); 1.8.3 = 7
  (partly in, or in but crooked); touching a limiter = round over, park = 0 (9.24.7).
- Stop must hold: judges need 30s stationary (App A sec.2). ROBOT_STOPPED holds forever. OK.
- Stopping in the finish section after 3 laps also banks 1.3 = 3 pts on its own.

## Tune order (Aug 22 bench, serial log is the instrument)
1. MAG_NEAR_H (30) — read h at ~30-40cm abeam from [mag] lines. CAN BE DONE TONIGHT
   OFFLINE: run v3 round2.py over Downloads\wro_magenta_frames_2026-08-12\ and read
   h by distance band. Same source can sanity-check MAG_HANDOFF_RATIO (0.55).
2. PARK_PASS_MS (900) — rear axle must clear the far limiter: watch where ARC_IN
   starts vs the limiter. Too short = tail clips limiter = round over. Err long.
3. PARK_CUT_IN_DEG (42) — lateral tuck of the S-curve ~ 2R(1-cos42) ~ 0.26R.
   If the vehicle parks proud of the 20cm lot, RAISE toward 50-55 before touching speed.
4. PARK_ALIGNED_DEG (8) — the 15-pt bar is wheel-to-wall delta <=2cm. Over the
   ~26cm wheelbase, 2cm ~ 4.4deg. 8deg ships the 7-pt attempt; tighten toward 4-5
   only if arcs prove repeatable.
5. Wall floors (14/5cm) are belt-only: black wall at grazing angle reads garbage,
   lunaValid just skips them. Yaw + time exits govern.

## Bench gates before this build ever runs
GPIO32 polarity -> steer direction at center 106 (INVERT_STEERING) -> reverse
direction sign (BACKWARD_SPEED actually reverses) -> one straight + one corner ->
then a park rep on the mat with limiters from the WhatsApp-video session.
