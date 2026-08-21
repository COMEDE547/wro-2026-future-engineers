# Round 2 — Pi vision sender (`pi_sender.py`)

Companion to `../round2_ino/round2_ino.ino`. Runs on the Raspberry Pi, watches the
track through the Pi camera, and tells the ESP32 which side to pass the next pillar on.

**This is a separate lineage from `../round2.py` + `../main.cpp`.** Those stay untouched.
This stack was built from the Round 1 sketch upward, with the detector wired in from
the start rather than bolted on.

## Chain

```
Picamera2 640x480
  -> letterbox 320 (same decode as detector/detect_3class.py)
  -> pillar_3class_yolo26n_320_v2.onnx   (ONNX Runtime, CPU)
  -> conf >= 0.40
  -> per-class IoU 0.5 dedup            <-- the model is NMS-free and emits duplicate
                                            boxes on ~7% of pillars; without this they
                                            are counted twice
  -> pick: red/green only, area >= 0.2% of frame, centre-x inside the middle 80%
  -> 2-frame hysteresis
  -> "R" / "G" / "C" line on serial, every frame
```

Class map is **0=green, 1=red, 2=magenta** and must not be reordered.
Magenta is detected and drawn but **never sent** — the firmware protocol has no `M`;
parking is firmware phase 2.

## Serial contract with the firmware

Newline-terminated tokens at 115200: `R`/`RED`, `G`/`GREEN`, `C`/`CLEAR`, case-insensitive.
Whole tokens are matched on the ESP32 side on purpose — a character-level parser reads the
second letter of `GREEN` as `R`. Send at least once per second; the firmware treats 1.5 s of
silence as `CLEAR` (dead-man), which degrades the run to Round-1 driving rather than to a
stale command.

## Setup (Pi OS Bookworm)

```bash
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-serial
pip3 install onnxruntime --break-system-packages
```

Do not use a bare venv — `picamera2` only exists as the apt package.
Serial permission denied: `sudo usermod -aG dialout $USER`, then log out and back in.

Put the model beside the script as `best.onnx`:

```bash
cp "../detector/models/pillar_3class_yolo26n_320_v2.onnx" best.onnx
```

## Running

| Situation | Command |
|---|---|
| **Competition run (headless)** | `python3 pi_sender.py --no-show` |
| Bench, monitor attached | `python3 pi_sender.py` |
| Bench, headless — view in a browser | `python3 pi_sender.py --no-serial --stream` then open the printed `http://<pi-ip>:8080` |
| Check the model on saved frames | `python3 pi_sender.py --source frames/ --no-serial` |

Two things that will cost a run if skipped:

- **Start `pi_sender.py` before pressing the robot's start button.** Opening the serial port
  pulses DTR and resets most ESP32 boards.
- **`--no-show` is required when nothing is plugged into the Pi's HDMI.** The preview window
  opens by default; with no display attached, OpenCV throws at startup and the script dies
  before the first frame.

**WRO §11.10 forbids wireless during scored runs.** Wi-Fi off, no `--stream` at the arena.
The stream is a bench tool.

## Live view

`--stream` serves an annotated MJPEG feed over plain HTTP — every detection with class and
confidence, the chosen pillar drawn thick, the corridor bounds, the current R/G/C state, and
a HUD line with fps, CPU temperature and `vcgencmd get_throttled`.

That HUD is also the power/thermal instrument. A Pi 5 running inference in a closed frame
browns out or throttles before anything else fails; `--threads 2` (default) caps the ONNX
intra-op threads to keep the current spike down.

## Flags

| Flag | Default | Notes |
|---|---|---|
| `--conf` | 0.40 | detection threshold |
| `--min-area` | 0.002 | ignore pillars smaller than this fraction of the frame (too far to act on) |
| `--roi` | 0.80 | central band that counts as our corridor |
| `--confirm` | 2 | frames a change must survive before it is sent |
| `--threads` | 2 | ONNX CPU threads |
| `--shutter` | 0 (auto) | fixed exposure in microseconds |
| `--rot180` | off | camera mounted inverted |
| `--port` | auto | first `ttyUSB*`/`ttyACM*`; pass explicitly on Windows |

## Caveat on the numbers

The model reports 0.980 mAP50-95 on held-out val and 41/41 on a blind gold set, but this
project's own history is the reason not to lean on that: the earlier `nanodet_lite` scored
0.941 in validation and did not transfer to real footage. Bench on mat photos and on the
robot before trusting it in a scored round.
