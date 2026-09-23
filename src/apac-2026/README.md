# APAC 2026 competition code

The code the vehicle runs at WRO Future Engineers APAC 2026: the Obstacle Challenge stack and the Open Challenge firmware. The vehicle is described in [`docs/apac_2026_vehicle.md`](../../docs/apac_2026_vehicle.md), and how `main.py` drives the run, with its state diagrams, in [`docs/apac_2026_software.md`](../../docs/apac_2026_software.md). The two parking modules are described in its [section 8](../../docs/apac_2026_software.md#8-parking-modules).

| File | Runs on | What it does |
|---|---|---|
| [`obstacle-challenge/main.py`](obstacle-challenge/main.py) | Raspberry Pi 5 | Camera, pillar and line detection, and the driving decisions; sends drive commands to the ESP32 |
| [`obstacle-challenge/parallel_parking.py`](obstacle-challenge/parallel_parking.py) | Raspberry Pi 5 | Parallel parking at the end of the run (`BUILD_ID = "slot-guided-entry-v29"`): finds the two parking-lot limitations with the camera, drives into the gap and straightens up. Imports `main.py`; called by `main.py` after the twelfth corner when `PARALLEL_PARKING = True`, or run on its own |
| [`obstacle-challenge/partial_parking.py`](obstacle-challenge/partial_parking.py) | Raspberry Pi 5 | Direct entry into the parking space on a locked heading, reusing the parallel module's approach and block search. Called by `main.py` after the twelfth corner when `PARTIAL_PARKING = True` (as committed), or run on its own |
| [`obstacle-challenge/esp32/obstacle-challenge-final-code/obstacle-challenge-final-code.ino`](obstacle-challenge/esp32/obstacle-challenge-final-code/obstacle-challenge-final-code.ino) | ESP32 | Drive motor, steering servo, LEDs, the three TF-Luna sensors and the BNO055 through the PCA9548A; sends telemetry to the Pi |
| [`open-challenge/open-challenge.ino`](open-challenge/open-challenge.ino) | ESP32 | The Open Challenge on the ESP32 alone: direction detection, wall following with heading hold, twelve 90° turns on the BNO055 heading, and a final stop at 150 cm from the wall ahead ([software §11](../../docs/apac_2026_software.md#11-open-challenge-firmware)) |
| [`obstacle-challenge/parking_camera_calibration.json`](obstacle-challenge/parking_camera_calibration.json) | Raspberry Pi 5 | The team's image-to-floor calibration for the parking modules (1280 × 720 image, 100 × 150 cm rectangle) |
| [`tools/lab-calibration.py`](tools/lab-calibration.py) | Pi or laptop | Interactive Lab colour-threshold picker (trackbars), with the same image preprocessing as `main.py`; how it is used at a venue is in [vehicle §4](../../docs/apac_2026_vehicle.md) |
| [`tools/camera-settings.py`](tools/camera-settings.py) | Pi or laptop | Interactive brightness, contrast and gamma preview |

## Serial link

115200 baud. `main.py` opens `/dev/ttyUSB0` on the Pi (`COM3` is its Windows bench setting).

- Pi to ESP32: `speed,direction,steer` and a newline. `speed` is the 8-bit PWM duty (0 to 255); `steer` is a correction of -60 to +60 around the servo centre of 85.
- ESP32 to Pi: telemetry lines `T,seq,ms,heading,left,center,right,validMask,hardStop,appliedSpeed`.

## Settings as committed

| Setting | Value | File |
|---|---|---|
| Drive speed | `SPEED = 215` (84 % duty; 210 in v0.4.0) | `main.py` |
| Turning, parking, collision escape | `100`, `70`, `80` | `main.py` |
| Leave the parking area at the start | `ENABLE_PARKING_EXIT = True` | `main.py` |
| Park at the end | `ENABLE_PARKING_IN = True`: after the twelfth corner `main.py` hands over to a parking module, chosen by `PARTIAL_PARKING = True` and `PARALLEL_PARKING = False` as committed (exactly one must be on; [software §8](../../docs/apac_2026_software.md#8-parking-modules)) | `main.py` |
| Run log | `ENABLE_RUN_CSV_LOGGING = False`; set to `True`, every run writes `run_logs/round2_run_<date_time>.csv` beside `main.py`, one row per telemetry sample and one per event | `main.py` |
| Parking modules | run on their own, e.g. `python parallel_parking.py --direction clockwise --last-pillar auto` | `parallel_parking.py`, `partial_parking.py` |
| Parking run log (CSV and video per run) | `ENABLE_RUN_LOGGING = False` | `parallel_parking.py` |
| Servo centre | `SERVO_CENTER = 85` in the Obstacle firmware; 106 in the Open Challenge firmware | firmware |
| Mux channels | TF-Luna left 0, centre 3, right 2; BNO055 4 | firmware |
| Front hard stop | disabled since 2026-09-10 (`forwardHardStopIsActive()` returns `false`); see [vehicle §7](../../docs/apac_2026_vehicle.md) | firmware |

Both parking modules read `parking_camera_calibration.json` beside them. It depends on how the camera is mounted: the team's file, made on the practice mat with the September camera mount, is committed; `python parallel_parking.py --calibrate` makes a new one if the mount changes.

On the Pi, `main.py` sets GPIO3 high once it has initialised and is waiting for the start signal, and low when it exits.

## Provenance and checksums

Copied on 2026-09-21 from the team coach's working repository (private; commit `63c2b31`). The two parking modules arrived on 2026-09-23 as a Google Drive download (`drive-download-20260922T184733Z-1-001.zip`, which dates them 2026-09-22 03:49 and 04:26) and are stored unchanged. The ESP32 file was `obstacle-challenge-final-code_withLED.ino`; it is renamed here and placed in a folder of the same name, as the Arduino IDE expects. The files are stored byte-for-byte (see [`.gitattributes`](.gitattributes)), so these MD5s can be checked on any checkout:

| File | MD5 |
|---|---|
| `main.py` | `77187817b99846d4c7d8766d5e9b8bf7` |
| `obstacle-challenge-final-code.ino` | `a0bf8c1ca5330442bed69d8299892572` |
| `lab-calibration.py` | `aad32eb3ca1954ad38683ee5a004da1a` |
| `camera-settings.py` | `f02e7f9eaae329e8867e965ac4f98511` |
| `parallel_parking.py` | `9d622df00da023df403fc06669c3c014` |
| `partial_parking.py` | `34bb36075c2ab680220bd6557056992a` |
| `open-challenge.ino` | `caf69d495f3d0f259a796dfcb81e5b5c` |
| `parking_camera_calibration.json` | `d2ebad50bf7a573496625ee7d20f26da` |

`main.py` was replaced on 2026-09-23 by the version dated 2026-09-18, from a second Google Drive download (`drive-download-20260922T190246Z-1-001.zip`) whose `cmd.txt` shows the coach starting `final-code/obstacle-challenge-final-code/main.py`. The version it replaces (MD5 `238dd1af810112d09598e433b483d5f8`), which [software §1-§6](../../docs/apac_2026_software.md) describes, stays in the history ([the file before the change](https://github.com/teddriveomo/wro-2026-future-engineers/blob/a0b36254bd93709a8d6fc53e1eead0ca39324006/src/apac-2026/obstacle-challenge/main.py)). The ESP32 firmware in those downloads was byte-identical to the file then committed (MD5 `28a6cdfb`). Every `main.py` name the parking modules use also exists in the 2026-09-18 file.

Not included, because none of them runs on this vehicle: the build without LEDs, an older firmware for a smaller robot, backup copies (`.bak`), component tests and the rest of the coaching repository. The Open Challenge variant for the smaller robot is left out for the same reason.

On 2026-09-23 the Obstacle Challenge files were updated again from the coach's working copy (the `FutureEngineers` folder: commit `63c2b31` plus uncommitted changes): `main.py` now hands over to a parking module after the twelfth corner ([software §10](../../docs/apac_2026_software.md#10-what-the-current-mainpy-changes)), the parking modules and `lab-calibration.py` carry new values, and the ESP32 firmware sets the LEDs to brightness 80 (50 before). Earlier versions stay in the history. Every `main.py` name the parking modules use exists in the committed `main.py`.

Also left out from the September downloads: earlier and experimental versions of `main.py` (`main_16_Sep.py`, `mainversion2.py`, `corner_slot_filler.py`) and standalone tests (`parking_entry_only.py`, `cornering_sequence.py`). From that working copy, also left out: `main_backgroung_notworking.py` (an experiment its name marks as not working), `parking_exit_only.py` and the `basic-functions/` and `invididual-components/` tests.
