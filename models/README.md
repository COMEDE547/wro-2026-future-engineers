# models/ — CAD for TED drive (WRO FE 2026)

The vehicle is a **LEGO Technic chassis**; every non-LEGO component is held by a
3D-printed bracket. This folder holds the team's own design files and printable
parts. Third-party parts used on the vehicle are attributed below and NOT
redistributed here.

## Chassis — the current frame

| File | What it is |
|---|---|
| `chassis/chassis_lego_current.lxfml` | The LEGO design for the frame the vehicle runs on today — 91 elements, 19 distinct part types |
| `chassis/chassis_build_instructions.pdf` | 69-step build instructions generated from that model, enough to rebuild the frame from parts |

**This is the redesigned frame, not a superseded one.** The first layout could
not package motor and battery together, carried its mass too high, and would not
hold a curve; under drive load it also flexed enough to work the rear axle
interface loose, and the wheels shed within seconds on 2026-08-06. The answer was
a ground-up redesign of the space-frame, built stiffer — the frame documented
here. It ran 28 s and 29 s with no wheel loss on 2026-08-09, roughly six times
the failure duration, and the planned 3D-printed rebuild was cancelled as
unnecessary. Full account: [1 — Mobility](../docs/1_mobility.md) §3 and
[D8](../docs/4_systems_and_decisions.md).

Because the answer was a LEGO redesign rather than a printed one, the chassis
artifact here is a design file and a build sequence rather than a chassis STL.
There is no printed chassis and there never was.

## Team-designed printed parts (as fitted, 2026-08-11)

| Part | File | Fitted as | Bounding box (mm) | Designed by / in |
|---|---|---|---|---|
| TF-Luna holder | `tf-luna/TF_luna_enclosure_lego.stl` | LEGO-pitch face frame for a TF-Luna (56 mm = exactly 7 LEGO modules; printed 3×, orange PLA) | 9.0 × 56.0 × 26.3 | TED drive, Fusion 360 |
| Servo horn beam, long | `steering/Horn_Beam_LONG.stl` | MG90S horn → LEGO beam steering link | 45.5 × 11.8 × 7.8 | TED drive, Fusion 360 |
| N20 motor mount | `drive/n20_clamp_lego.stl` | Printed clamp holding the 12 V N20 gearmotor to the LEGO frame | 38.8 × 26.0 × 12.0 | TED drive, Fusion 360 (re-modelled; .f3d export still owed) |

Parametric sources: Fusion 360 — **export and commit the .f3d/.step for each
team part alongside its STL** (Fusion cloud retains them).

## Third-party printed parts — sources

The Lenovo 300 FHD webcam is held by four printed parts of a published mount
kit. The kit's files are linked, not redistributed, in this repository — that
is safe under every possible license, and attribution is the requirement
either way.

**[1]** cncplasticfactory, *"Lenovo 300 FHD Webcam Mount for 3D Printers with
2020 Extrusions,"* Printables.com, model 352447, updated 29 Dec 2022.
https://www.printables.com/model/352447 — accessed 11 Aug 2026.
Parts as fitted on the vehicle: front clamp, rear clamp, base bracket, spacer
arm (white PLA). License: stated on the model page (not machine-readable at
citation time) — confirm there before ever redistributing the files.
Parts used as-printed from the published geometry; no modifications.

**[2]** PsychoShaft, *"C92X Mount,"* VoronDesign/VoronUsers `printer_mods`
(the upstream design that [1] remixes).
https://github.com/VoronDesign/VoronUsers/tree/master/printer_mods/PsychoShaft/C92X_PsycHoShafts_Mount
— lineage credit only; no files taken from it directly.

**[3]** Lenovo, *"Lenovo 300 FHD WebCam — Overview and Service Parts,"*
acc500192.
https://support.lenovo.com/in/en/accessories/acc500192-lenovo-300-fhd-webcam-overview-and-service-parts
— the mounted camera hardware (1080p, 95° FOV, USB). Team designation for the
same unit: **OMO/WCAM/11** (see `docs/2_power_and_sensors.md`).

## Not printed

MG90S servo body is LEGO-integrated. The N20 sits in the printed clamp above;
the .f3d source for it is still owed.
