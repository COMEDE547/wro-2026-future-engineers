# Test runs — 2026-08-08

Two runs recorded within a minute of each other. **They are two different
software revisions**, and neither matches the revision used in the
[2026-08-09 runs](../test-runs-2026-08-09). Nothing here can be compared against
anything else: each is a record of how one build behaved, and the only variable
is not held constant between them.

| File | Length | Behaviour |
|---|---|---|
| `slow-wobble-first-turn-wall-strike.mp4` | 13.8 s | Drives the first leg slowly, wobbles, and strikes the wall at the first turn |
| `slow-continuous-readjust.mp4` | 20.5 s | Moves very slowly and corrects its steering continuously without ever settling |

Both were shot at 60 fps, which is useful: it is fast enough to resolve steering
corrections if the vehicle is ever filmed close enough to measure them.

## `slow-wobble-first-turn-wall-strike.mp4`

The start button is pressed at about 1.3 s and motion begins around 2.5 s. The
vehicle covers the leg, reaches the first corner toward the end of the
recording, and does not come out of the turn cleanly. The pillars visible along
the outer edge are stored there, not placed on the track.

## `slow-continuous-readjust.mp4`

The distinguishing feature is that the vehicle never holds a heading — it is
correcting continuously for the whole run rather than tracking straight and
occasionally adjusting.

Measured over a 5.1 s window where the camera is steady (10.6–15.7 s), forward
speed is **96 px/s**, which against a vehicle roughly 200 mm long gives about
**0.2 m/s, ±25 %**. That is slow enough to be worth stating as a number rather
than an impression.

**The oscillation frequency could not be measured from this recording, and the
attempt is recorded here so it is not repeated.** The vehicle occupies about
100 px and the lateral residual after removing the path curvature is about
4.5 px RMS, which is the same order as the tracking noise. Two independent
estimators disagreed by a factor of six, and the apparent spectral peak at
1.2 Hz turned out to be the corner frequency of the one-second detrending
window used in the analysis rather than anything the vehicle did.

The frequency is worth having, because a steering limit cycle encodes the loop
delay that produces it. The instrument for that is the serial telemetry log —
steering command against time, sampled at loop rate — not video.

## Open

Which software revision produced each of these runs has been requested from the
team and is not yet recorded. Until it is, these are observations of behaviour
and cannot be attributed to any particular change in the code.
