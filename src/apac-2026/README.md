# APAC 2026 competition code

The Obstacle Challenge code the vehicle runs at WRO Future Engineers APAC 2026. The vehicle is described in [`docs/apac_2026_vehicle.md`](../../docs/apac_2026_vehicle.md).

| File | Runs on | What it does |
|---|---|---|
| [`obstacle-challenge/main.py`](obstacle-challenge/main.py) | Raspberry Pi 5 | Camera, pillar and line detection, and the driving decisions; sends drive commands to the ESP32 |
| [`obstacle-challenge/esp32/obstacle-challenge-final-code/obstacle-challenge-final-code.ino`](obstacle-challenge/esp32/obstacle-challenge-final-code/obstacle-challenge-final-code.ino) | ESP32 | Drive motor, steering servo, LEDs, the three TF-Luna sensors and the BNO055 through the PCA9548A; sends telemetry to the Pi |
| [`tools/lab-calibration.py`](tools/lab-calibration.py) | Pi or laptop | Interactive Lab colour-threshold picker (trackbars), with the same image preprocessing as `main.py` |
| [`tools/camera-settings.py`](tools/camera-settings.py) | Pi or laptop | Interactive brightness, contrast and gamma preview |

## Serial link

115200 baud. `main.py` opens `/dev/ttyUSB0` on the Pi (`COM3` is its Windows bench setting).

- Pi to ESP32: `speed,direction,steer` and a newline. `speed` is the 8-bit PWM duty (0 to 255); `steer` is a correction of -60 to +60 around the servo centre of 85.
- ESP32 to Pi: telemetry lines `T,seq,ms,heading,left,center,right,validMask,hardStop,appliedSpeed`.

## Settings as committed

| Setting | Value | File |
|---|---|---|
| Drive speed | `SPEED = 210` (82 % duty) | `main.py` |
| Turning, parking, collision escape | `100`, `70`, `80` | `main.py` |
| Leave the parking area at the start | `ENABLE_PARKING_EXIT = True` | `main.py` |
| Park at the end | `ENABLE_PARKING_IN = False` | `main.py` |
| Servo centre | `SERVO_CENTER = 85` | firmware |
| Mux channels | TF-Luna left 0, centre 3, right 2; BNO055 4 | firmware |
| Front hard stop | disabled since 2026-09-10 (`forwardHardStopIsActive()` returns `false`); see [vehicle §7](../../docs/apac_2026_vehicle.md) | firmware |

On the Pi, `main.py` sets GPIO3 high once it has initialised and is waiting for the start signal, and low when it exits.

## Provenance and checksums

Copied on 2026-09-21 from the team coach's working repository (private; commit `63c2b31`). The ESP32 file was `obstacle-challenge-final-code_withLED.ino`; it is renamed here and placed in a folder of the same name, as the Arduino IDE expects. The files are stored byte-for-byte (see [`.gitattributes`](.gitattributes)), so these MD5s can be checked on any checkout:

| File | MD5 |
|---|---|
| `main.py` | `238dd1af810112d09598e433b483d5f8` |
| `obstacle-challenge-final-code.ino` | `28a6cdfba6ad9164471c5e9b56110a0c` |
| `lab-calibration.py` | `ad3b6f191bb384d708ae5894bbf4d70d` |
| `camera-settings.py` | `f02e7f9eaae329e8867e965ac4f98511` |

Not included, because none of them runs on this vehicle: the build without LEDs, an older firmware for a smaller robot, backup copies (`.bak`), component tests and the rest of the coaching repository. The Open Challenge code is not in this folder.
