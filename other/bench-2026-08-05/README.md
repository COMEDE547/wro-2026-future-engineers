# Bench session — 2026-08-05

Build and bring-up photographs from the OMOTEC lab.

| File | What it shows |
|---|---|
| `bench-1.jpeg` | Steering bring-up: the Round-1 vehicle in hand with the servo-angle calibration page open on the laptop |
| `bench-2.jpeg` | Wiring the power board — ESP32, buck converter and perfboard harness |
| `bench-3.jpeg` | The same board being landed onto the chassis |
| `bench-4.jpeg` | The detector running live on the Raspberry Pi 5, both pillars in frame |
| `bench-5.jpeg` | Close view of the Pi 5 detector output against the physical pillars |

`bench-5.jpeg` is referenced from [`docs/3_software.md`](../../docs/3_software.md)
§4.1: it shows the runtime resolving a red and a green pillar **simultaneously
and correctly** on the deployed camera, at 57×91 px and 64×99 px respectively.
Both heights sit above the 80 px reverse threshold, which is the correct
response at this range — the pillars are roughly 30 cm from the lens.

That frame matters because the co-occurrence case is the weakest measured result
in the repository (47.1 % among committed calls). That figure was produced on
**composited** two-pillar images; this photograph is real two-pillar input on
the real camera, resolving correctly, which suggests the composite is a
pessimistic proxy. It is a single frame and proves nothing on its own — the
proper measurement is a logged run against real footage, which is why it is
described here as an indication rather than a result.

The camera visible in `bench-4`/`bench-5` is the lab unit `OMO/WCAM/11` (a Lenovo 300 FHD).
