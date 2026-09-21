# APAC 2026 wiring — power and signals (as built, 2026-09-20)

Current diagram for the September rebuild. It replaces `circuit_diagram_complete_2026-08-11.jpg` (the Nationals vehicle) and adds the WS2812 LEDs on GPIO4, the power switch on the battery lead, the start button on GPIO32 and the MEX drive motor. Pins come from the Nationals firmware and the 2026-09-20 wiring draft and will be re-checked against the APAC firmware when it is committed. All grounds are common (not drawn).

```mermaid
flowchart LR
    BAT["3S LiPo<br/>11.1 V, 2200 mAh"] --> SW["Power switch<br/>(battery lead)"]
    SW --> FCM["Fast-charge module<br/>5 V USB-A out"]
    SW --> BUCK["Buck-2<br/>5 V rail"]
    SW --> DRV["TB6612FNG<br/>VM = pack"]
    FCM -->|"USB-A to USB-C"| PI["Raspberry Pi 5"]
    PI -->|"USB"| CAM["Lenovo 300 FHD camera"]
    PI -->|"USB: data + 5 V"| ESP["ESP32"]
    BUCK -->|"5 V"| TFL["3x TF-Luna"]
    BUCK -->|"5 V"| SRV["MG90 servo"]
    BUCK -->|"5 V"| LED["6x WS2812 LEDs"]
    ESP -->|"3.3 V, GPIO21 SDA, GPIO22 SCL"| MUX["PCA9548A 0x70"]
    MUX -->|"ch0 left, ch3 centre, ch2 right"| TFL
    MUX -->|"ch4"| IMU["BNO055 0x28"]
    ESP -->|"3.3 V"| IMU
    ESP -->|"GPIO13"| SRV
    ESP -->|"3.3 V, GPIO25 AIN1, GPIO26 AIN2, GPIO33 PWMA, GPIO27 STBY"| DRV
    DRV -->|"AO1, AO2"| MOT["MEX motor<br/>6 V rated, 400 rpm"]
    ESP -->|"GPIO4 data"| LED
    BTN["Start button<br/>(rear beams)"] -->|"GPIO32"| ESP
```

Loads on each rail: the fast-charge module feeds the Pi 5, which powers the camera and the ESP32 over USB (600 mA total without USB-PD, see [APAC 2026 vehicle §6](../docs/apac_2026_vehicle.md)); Buck-2 feeds the three TF-Lunas, the servo and the LEDs (up to 0.36 A for the LEDs at full white); the TB6612FNG takes the motor current straight from the pack.
