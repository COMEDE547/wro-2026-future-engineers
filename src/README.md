# Control software

Two rounds, two processors.

## `Round 1/` — ESP32 firmware

Compiles in the Arduino IDE or PlatformIO, board = **ESP32**. Each sketch sits in
its own folder, per Arduino IDE convention.

| Sketch | Role |
|---|---|
| `round 1/round 1.ino` | **Main driving firmware.** Captures a target heading from the BNO055, drives straight by proportionally counter-steering the servo to null heading error, and detects corners on the rising edge of a side TF-Luna crossing 150 cm (steps the target heading by 90 deg). |
| `triple_ultrasonic_sensor/triple_ultrasonic_sensor.ino` | Standalone 3x HC-SR04 test (front / left / right), interrupt-driven and non-blocking. Alternative sensing prototype — see the LiDAR-vs-ultrasonic comparison in [`docs/2_power_and_sensors.md`](../docs/2_power_and_sensors.md#why-lidar-over-ultrasonic-for-corner-detection). |
| `GetAngle_IMU/GetAngle_IMU.ino` | Early MPU6050 read-out test. Superseded by the BNO055; retained because the reason for the switch is documented against it. |

**Libraries:** Adafruit BNO055, Adafruit Unified Sensor, Adafruit BusIO,
ESP32Servo (+ Adafruit MPU6050 for the legacy test).

**Key I/O (main sketch):** I2C `SDA = GPIO21` / `SCL = GPIO22` @ 400 kHz ·
servo signal `GPIO13` · PCA9548A mux `0x70` (ch0/1/2 = left/centre/right TF-Luna
@ `0x10`, ch4 = BNO055 @ `0x28`).

**Not implemented:** drive-motor control. The firmware holds and corrects a
heading but does not command propulsion — see [`docs/1_mobility.md`](../docs/1_mobility.md).

## `Round 2/` — perception, on the Raspberry Pi 5

| Path | Role |
|---|---|
| [`detector/`](Round%202/detector/README.md) | Pillar detector. Stripped NanoDet-Plus, training loop, evaluation, operating-point sweep, ONNX export, deploy weights. **This is the shipping perception path.** |
| `vision/` | Classical HSV + connected-components pipeline, 0.43 ms/frame. Retained as a diagnostic and zero-dependency sanity check, not as the competition detector — reasoning in [`docs/4_systems_and_decisions.md`](../docs/4_systems_and_decisions.md#d1--neural-pillar-detector-deferred-then-adopted-the-same-day). |

The Obstacle Challenge controller that consumes the detector output is
**specified but not written** — state machine in [`docs/3_software.md`](../docs/3_software.md#2-obstacle-challenge--specified-not-implemented).
