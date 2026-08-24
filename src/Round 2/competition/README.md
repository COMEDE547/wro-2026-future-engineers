# Competition stack — Nationals 2026 (flashed build)

This folder is the Round 2 configuration flashed for the Hyderabad Nationals.

| File | Role |
|---|---|
| `test.cpp` | ESP32 firmware: PID heading hold (Kp 1.5 / Ki 0.02 / Kd 0.15, deadband-gated P and I, integral clamp ±50, correction clamp ±20 on wrapped ±180 error), 14-state machine including the parking-lot **exit** S-curve at boot (PARK_WAIT → PARK_REVERSE → PARK_PHASE1 → PARK_STRAIGHT_LEG → PARK_PHASE2), front-LiDAR reverse-arc cornering, and the PAUSE → PASS → SIDE → YAW-BACK obstacle-pass sequence. |
| `detecttor.py` | Raspberry Pi 5 detector/sender: 2-class ONNX (index 0 = green, 1 = red), 5-of-7 temporal vote with spatial lock, 7-column offset LUT with closeness bands, `DRIVE` / `REVERSE` / `CLEAR` / `TRACKING` serial protocol with per-line checksums @ 115200, `ESP_READY` / `PI_HELLO` / `ESP_ACK` handshake gated on the pin-32 start button. |
| `detect.py` | Alias entry point so `python detect.py` runs the same detector. |

**Model file:** place the weights next to `detecttor.py` under the name in `ONNX_MODEL_PATH` (`best_ncnn.onnx`). The weights are deliberately not duplicated in this folder; the 3-class v2 weights in `../detector/models/` decode green/red at the same indices (0/1) if substituted — the extra magenta class is simply ignored by this 2-class map.

**Lineage:** for the run, this supersedes `../pi_sender` + `../round2_ino` and `../main.cpp` + `../round2.py`; both earlier stacks are retained as engineering history (see `docs/3_software.md` and the Engineering Journal).
