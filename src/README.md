# Control software

Two rounds, two processors.

## `Round 1/` — ESP32 firmware

Compiles in the Arduino IDE or PlatformIO, board = **ESP32**. Each sketch sits in
its own folder, per Arduino IDE convention.

| Sketch | Role |
|---|---|
| `round 1/round 1.ino` | **Main driving firmware.** Captures a target heading from the BNO055, drives straight by proportionally counter-steering the servo to null heading error, and detects corners on the rising edge of a side TF-Luna crossing 150 cm (steps the target heading by 90 deg). Drives an N20 via TB6612 (append-only module): 20 kHz 10-bit PWM, observer turn counter, settle-gated 0.5 s run-on after 12 corners, then brake. Steering-sign fix + retune 2026-08-10 (`58adb1c`): center 106, limits 64/136, gain 1.0, DRIVE_SPEED 1000. |
| `motor_test/motor_test.ino` | Serial bring-up for the drive motor via `motor_control.h` (w/s speed steps, `r` reverse, `b` brake, `i` state). Pin constants matched to the main-firmware harness: PWM 33 · IN1 25 · IN2 26 · STBY 27. |
| `triple_ultrasonic_sensor/triple_ultrasonic_sensor.ino` | Standalone 3x HC-SR04 test (front / left / right), interrupt-driven and non-blocking. Alternative sensing prototype — see the LiDAR-vs-ultrasonic comparison in [`docs/2_power_and_sensors.md`](../docs/2_power_and_sensors.md#why-lidar-over-ultrasonic-for-corner-detection). |
| `GetAngle_IMU/GetAngle_IMU.ino` | Early MPU6050 read-out test. Superseded by the BNO055; retained because the reason for the switch is documented against it. |

**Libraries:** Adafruit BNO055, Adafruit Unified Sensor, Adafruit BusIO,
ESP32Servo (+ Adafruit MPU6050 for the legacy test).

**Key I/O (main sketch):** I2C `SDA = GPIO21` / `SCL = GPIO22` @ 400 kHz ·
servo signal `GPIO13` · PCA9548A mux `0x70` (ch0/1/2 = left/centre/right TF-Luna
@ `0x10`, ch4 = BNO055 @ `0x28`) ·
TB6612 AIN1 `GPIO25` / AIN2 `GPIO26` / PWMA `GPIO33` / STBY `GPIO27`.

**Drive-motor control: integrated (2026-08-03), append-only.** The original
steering / corner logic is byte-untouched; the module observes target-heading
steps rather than modifying trigger code. ~~Physical bring-up pending the build~~
**Driven on the mat 2026-08-08/09** (test footage under [`other/`](../other)) —
see [`docs/1_mobility.md`](../docs/1_mobility.md).

## `Round 2/` — perception, on the Raspberry Pi 5

| Path | Role |
|---|---|
| `round2.py` | **Pi 5 flight script.** Calibrated-Lab colour picker (venue calibration via `--calib` → `calib.json`), PyAV camera capture with v4l2 exposure/white-balance lock, temporal vote gate, and the serial command stream to the ESP32 (`RED`/`GREEN`/`CLEAR`/`REVERSE`/`POS,cx,h` on `/dev/ttyUSB0` @ 115200, refreshed every 0.5 s against the controller's 1.5 s dead-man). |
| `main.cpp` | **ESP32 obstacle controller.** Heading capture at the start button, IMUPLUS yaw, PD steering with deadband and clamp, persist-gated corner spikes, visual-servo swerve on the Pi's command stream. Center/limits aligned to the Round-1 mat tune 2026-08-10 — bench-verify before flashing. |
| `eval_picker.py` | Measurement harness — imports `round2.py`'s detection functions verbatim (serial/camera stubbed) and re-derives the picker tables in [`docs/eval_raw/`](../docs/eval_raw). |
| `history/` | Prior code lineages, retained as iteration evidence. |
| [`detector/`](Round%202/detector/README.md) | Pillar detector. Stripped NanoDet-Plus, training loop, evaluation, operating-point sweep, ONNX export, deploy weights. **Superseded 2026-08-05 — kept as iteration evidence**; see the status banner in its README. |
| `vision/` | Classical HSV + connected-components pipeline, 0.43 ms/frame. Retained as a diagnostic and zero-dependency sanity check, not as the competition detector — reasoning in [`docs/4_systems_and_decisions.md`](../docs/4_systems_and_decisions.md#d1--neural-pillar-detector-deferred-then-adopted-the-same-day). |

~~The current Round 2 stack — calibrated-Lab colour picker + the ESP32 obstacle
controller — is implemented off-repo and lands after integration fixes.~~
**Corrected 2026-08-10: the stack has been in-repo since 2026-08-06/07**
(`round2.py` + `main.cpp` above); this paragraph was stale on the judged
surface. State machine and stack rationale:
[`docs/3_software.md`](../docs/3_software.md#2-obstacle-challenge--implemented).
