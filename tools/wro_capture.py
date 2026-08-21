#!/usr/bin/env python3
"""
wro_capture.py v1.2 — dataset capture rig for WRO Future Engineers pillar detection.

NEW IN v1.2: press 'p' to start a clip, 'p' again to stop, 'q' to quit. Single
keypress, no Enter needed — works over a Raspberry Pi Connect remote shell.
This is now the DEFAULT trigger. GPIO button (v1.1) still available via --button.

Backends: Picamera2 (CSI camera, Pi OS Bookworm/Trixie) or OpenCV V4L2 (USB cam).

WIRING
------
  Button : one leg -> GPIO17, other leg -> any GND pin. Nothing else.
           Internal pull-up is enabled in software; no external resistor.
  LED    : anode -> GPIO27 through a ~330 ohm resistor, cathode -> GND.
           LED off = idle. LED on = recording.

  gpiozero works on Pi 4 AND Pi 5. RPi.GPIO does NOT work on Pi 5 — do not
  substitute it. If the pin factory misbehaves:  export GPIOZERO_PIN_FACTORY=lgpio

INSTALL (Pi OS Bookworm/Trixie)
-------------------------------
    sudo apt update
    sudo apt install -y python3-picamera2 python3-gpiozero python3-lgpio ffmpeg
  A venv MUST be created with --system-site-packages or picamera2 won't import:
    python3 -m venv --system-site-packages ~/venv

USAGE
-----
    python3 wro_capture.py --list-cameras

    # DEFAULT: 'p' toggles recording, 'q' quits.
    python3 wro_capture.py --label red --lighting arena-warm

    # physical GPIO button instead (see WIRING above)
    python3 wro_capture.py --label red --lighting arena-warm --button 17 --led 27

    # fixed-length clip, no interaction
    python3 wro_capture.py --label red --lighting arena-warm --duration 45

OVER RASPBERRY PI CONNECT
-------------------------
Run it inside tmux. If the Connect session drops mid-clip the process is killed
and an in-progress .mp4 is left unfinalised — tmux keeps it alive:
    tmux new -s cap
    python3 wro_capture.py --label red --lighting arena-warm
    # detach: Ctrl-B then D    reattach: tmux attach -t cap
"""

from __future__ import annotations

import argparse
import atexit
import json
import os
import platform
import select
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

SCRIPT_VERSION = "1.2.0"

_sigint = False


def _handle_sigint(signum, frame):
    global _sigint
    _sigint = True
    print("\n[!] stop requested, finalising...")


signal.signal(signal.SIGINT, _handle_sigint)


# ----------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------

def parse_size(s: str) -> tuple[int, int]:
    w, h = s.lower().split("x")
    return int(w), int(h)


def parse_pair(s: str) -> tuple[float, float]:
    a, b = s.split(",")
    return float(a), float(b)


def jsonable(obj):
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v) for v in obj]
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return str(obj)


def next_session_dir(root: Path, lighting: str) -> Path:
    day = datetime.now().strftime("%Y-%m-%d")
    n = 1
    while True:
        d = root / f"{day}_S{n:02d}_{lighting}"
        if not d.exists():
            return d
        n += 1


# ----------------------------------------------------------------------------
# triggers — decide when a clip starts and stops
# ----------------------------------------------------------------------------

class DurationTrigger:
    """Original behaviour: one clip, fixed length."""

    def __init__(self, duration: float):
        self.duration = duration
        self._done = False

    def wait_for_start(self) -> bool:
        if self._done:
            return False
        self._done = True
        return not _sigint

    def should_stop(self, elapsed: float) -> bool:
        return _sigint or elapsed >= self.duration

    def close(self):
        pass


class ButtonTrigger:
    """
    Toggle recording with a physical GPIO button.
      tap   -> start clip / stop clip
      hold 2s -> quit the script
    """

    def __init__(self, pin: int, led_pin: int | None, hold_s: float,
                 max_clip_s: float, bounce_s: float = 0.08):
        from gpiozero import Button, LED

        self.hold_s = hold_s
        self.max_clip_s = max_clip_s
        self._tap = False
        self._quit = False
        self._press_t = 0.0
        self._lock = threading.Lock()

        # pull_up=True -> wire the other leg of the button to GND.
        self.btn = Button(pin, pull_up=True, bounce_time=bounce_s)
        self.btn.when_pressed = self._on_press
        self.btn.when_released = self._on_release

        self.led = LED(led_pin) if led_pin is not None else None
        if self.led:
            self.led.off()

    def _on_press(self):
        self._press_t = time.monotonic()

    def _on_release(self):
        dt = time.monotonic() - self._press_t
        with self._lock:
            if dt >= self.hold_s:
                self._quit = True
            else:
                self._tap = True

    def _take_tap(self) -> bool:
        with self._lock:
            t, self._tap = self._tap, False
            return t

    def wait_for_start(self) -> bool:
        print("  [idle] tap button to START  |  hold 2 s to quit")
        while True:
            if _sigint or self._quit:
                return False
            if self._take_tap():
                if self.led:
                    self.led.on()
                return True
            time.sleep(0.02)

    def should_stop(self, elapsed: float) -> bool:
        if _sigint or self._quit:
            return True
        if elapsed >= self.max_clip_s:
            print("\n[!] max clip length reached")
            return True
        if self._take_tap():
            return True
        return False

    def clip_done(self):
        if self.led:
            self.led.off()

    def should_quit(self) -> bool:
        return self._quit or _sigint

    def close(self):
        if self.led:
            self.led.off()
            self.led.close()
        self.btn.close()


class KeyTrigger:
    """
    Single-keypress toggle from the terminal. Default trigger.
        p  -> start clip / stop clip
        q  -> quit

    Uses cbreak mode, NOT raw mode. Two reasons that matters:
      * cbreak leaves ISIG on, so Ctrl-C still works as an escape hatch.
      * cbreak leaves OPOST/ONLCR alone, so print() newlines aren't mangled
        into staircase output.
    Terminal settings are restored on exit and via atexit, otherwise a crash
    leaves your shell with no echo and you have to blind-type `reset`.
    """

    def __init__(self, max_clip_s: float, rec_key: str = "p",
                 quit_key: str = "q"):
        self.max_clip_s = max_clip_s
        self.rec_key = rec_key.lower()
        self.quit_key = quit_key.lower()
        self._tap = False
        self._quit = False
        self._closed = False
        self._lock = threading.Lock()

        self.fd = sys.stdin.fileno()
        self.isatty = os.isatty(self.fd)
        self._old = None
        if self.isatty:
            import termios
            import tty
            self._termios = termios
            self._old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            atexit.register(self._restore)
        else:
            # no TTY (systemd, pipe) — fall back to line mode
            print("[!] stdin is not a terminal; falling back to "
                  "line mode (type p<Enter> / q<Enter>)")

        threading.Thread(target=self._reader, daemon=True).start()

    def _restore(self):
        if self._old is not None:
            try:
                self._termios.tcsetattr(self.fd, self._termios.TCSADRAIN,
                                        self._old)
            except Exception:
                pass
            self._old = None

    def _reader(self):
        while not self._closed:
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
            except Exception:
                return
            if not r:
                continue
            if self.isatty:
                ch = sys.stdin.read(1)
            else:
                line = sys.stdin.readline()
                if not line:
                    return
                ch = line.strip()[:1]
            if not ch:
                continue
            ch = ch.lower()
            with self._lock:
                if ch == self.quit_key:
                    self._quit = True
                elif ch == self.rec_key:
                    self._tap = True

    def _take_tap(self) -> bool:
        with self._lock:
            t, self._tap = self._tap, False
            return t

    def wait_for_start(self) -> bool:
        print(f"  [idle] press '{self.rec_key}' to START  |  "
              f"'{self.quit_key}' to quit")
        while True:
            if _sigint or self._quit:
                return False
            if self._take_tap():
                return True
            time.sleep(0.02)

    def should_stop(self, elapsed: float) -> bool:
        if _sigint or self._quit:
            return True
        if elapsed >= self.max_clip_s:
            print("\n[!] max clip length reached")
            return True
        return self._take_tap()

    def clip_done(self):
        pass

    def should_quit(self) -> bool:
        return self._quit or _sigint

    def close(self):
        self._closed = True
        self._restore()


# ----------------------------------------------------------------------------
# picamera2 backend
# ----------------------------------------------------------------------------

def have_picamera2() -> bool:
    try:
        import picamera2  # noqa: F401
        return True
    except Exception:
        return False


def list_cameras() -> None:
    if not have_picamera2():
        print("picamera2 not available. USB devices:")
        os.system("v4l2-ctl --list-devices 2>/dev/null || ls -1 /dev/video* 2>/dev/null")
        return
    from picamera2 import Picamera2
    for i, info in enumerate(Picamera2.global_camera_info()):
        print(f"[{i}] {info}")
    cam = Picamera2()
    print("\nSensor modes (pick one whose crop matches your inference FOV):")
    for m in cam.sensor_modes:
        print(f"  size={m.get('size')} fps={m.get('fps')} "
              f"crop={m.get('crop_limits')} bit_depth={m.get('bit_depth')}")
    cam.close()


def build_controls(a) -> dict:
    from libcamera import controls as lc

    ctrl: dict = {"FrameRate": float(a.fps)}

    # Colour IS the class label. Whatever you pick here, the inference pipeline
    # must use the same setting or you have an undiagnosable domain shift.
    if a.awb == "lock":
        ctrl["AwbEnable"] = False
        if a.colour_gains:
            ctrl["ColourGains"] = parse_pair(a.colour_gains)
    else:
        ctrl["AwbEnable"] = True
        ctrl["AwbMode"] = lc.AwbModeEnum.Auto

    # Short shutter = less motion blur while driving.
    if a.shutter is not None or a.gain is not None:
        ctrl["AeEnable"] = False
        if a.shutter is not None:
            ctrl["ExposureTime"] = int(a.shutter)
        if a.gain is not None:
            ctrl["AnalogueGain"] = float(a.gain)
    else:
        ctrl["AeEnable"] = True

    # Camera Module 3 only. Continuous AF hunts and smears frames on a moving
    # robot — manual locked focus is almost always correct.
    if a.focus == "continuous":
        ctrl["AfMode"] = lc.AfModeEnum.Continuous
    elif a.focus == "auto":
        ctrl["AfMode"] = lc.AfModeEnum.Auto
    elif a.focus is not None:
        try:
            ctrl["AfMode"] = lc.AfModeEnum.Manual
            ctrl["LensPosition"] = float(a.focus)   # dioptres = 1 / distance_m
        except ValueError:
            pass
    return ctrl


class PiCamBackend:
    name = "picamera2"

    def __init__(self, a):
        from picamera2 import Picamera2
        self.a = a
        self.cam = Picamera2(camera_num=a.device)
        cfg = self.cam.create_video_configuration(
            main={"size": parse_size(a.size), "format": "RGB888"},
            controls={"FrameRate": float(a.fps)},
            buffer_count=6,
        )
        self.cam.configure(cfg)
        self.cam.set_controls(build_controls(a))
        self.cam.start()
        time.sleep(a.settle)          # let AE/AWB converge
        self.encoder = None
        self.video_path = None
        self.stills_dir = None
        self.n_stills = 0
        self.next_still = 0.0

    def applied(self):
        return jsonable(self.cam.capture_metadata())

    def start_clip(self, clip_base: Path, stills_dir: Path | None):
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import FfmpegOutput, FileOutput

        self.stills_dir = stills_dir
        self.n_stills = 0
        self.next_still = time.monotonic()
        self.clip_base = clip_base
        self.video_path = None
        self.encoder = None

        if self.a.mode in ("video", "both"):
            self.video_path = clip_base.with_suffix(".mp4")
            self.encoder = H264Encoder(bitrate=self.a.bitrate)
            try:
                out = FfmpegOutput(str(self.video_path))
            except Exception:
                self.video_path = clip_base.with_suffix(".h264")
                out = FileOutput(str(self.video_path))
            # start_encoder (not start_recording) keeps the camera running
            # between clips, so clip 2 doesn't pay the warm-up cost again.
            try:
                self.cam.start_encoder(self.encoder, out)
            except Exception:
                self.cam.start_recording(self.encoder, out)

    def poll(self, elapsed: float):
        if self.stills_dir is None:
            return
        now = time.monotonic()
        if now >= self.next_still:
            p = self.stills_dir / f"{self.clip_base.name}_{self.n_stills:04d}.jpg"
            try:
                req = self.cam.capture_request()
                req.save("main", str(p))
                req.release()
            except Exception:
                self.cam.capture_file(str(p))
            self.n_stills += 1
            self.next_still += self.a.still_interval

    def stop_clip(self) -> dict:
        if self.encoder is not None:
            try:
                self.cam.stop_encoder()
            except Exception:
                self.cam.stop_recording()
                self.cam.start()
            self.encoder = None
        return {"video": str(self.video_path) if self.video_path else None,
                "stills": self.n_stills}

    def close(self):
        try:
            self.cam.stop()
        finally:
            self.cam.close()


# ----------------------------------------------------------------------------
# opencv backend (USB camera)
# ----------------------------------------------------------------------------

class CvBackend:
    name = "opencv"

    def __init__(self, a):
        import cv2
        self.cv2 = cv2
        self.a = a
        w, h = parse_size(a.size)
        self.cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open /dev/video{a.device}")
        # MJPG first: most USB cams only hit high res/fps in MJPG, not YUYV.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, a.fps)
        if a.awb == "lock":
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        if a.shutter is not None:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, a.shutter / 100.0)

        self.w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or a.fps
        if (self.w, self.h) != (w, h):
            print(f"[!] camera gave {self.w}x{self.h}, not {w}x{h}")
        for _ in range(10):
            self.cap.read()
        time.sleep(a.settle)
        self.writer = None
        self.recording = False

    def applied(self):
        return {"width": self.w, "height": self.h, "fps": self.fps}

    def start_clip(self, clip_base: Path, stills_dir: Path | None):
        self.clip_base = clip_base
        self.stills_dir = stills_dir
        self.n_stills = 0
        self.n_frames = 0
        self.next_still = time.monotonic()
        self.video_path = None
        self.writer = None
        if self.a.mode in ("video", "both"):
            self.video_path = clip_base.with_suffix(".mp4")
            self.writer = self.cv2.VideoWriter(
                str(self.video_path),
                self.cv2.VideoWriter_fourcc(*"mp4v"),
                self.fps, (self.w, self.h))
        self.recording = True

    def poll(self, elapsed: float):
        ok, frame = self.cap.read()
        if not ok:
            return
        if not self.recording:
            return
        self.n_frames += 1
        if self.writer is not None:
            self.writer.write(frame)
        now = time.monotonic()
        if self.stills_dir is not None and now >= self.next_still:
            p = self.stills_dir / f"{self.clip_base.name}_{self.n_stills:04d}.jpg"
            self.cv2.imwrite(str(p), frame,
                             [int(self.cv2.IMWRITE_JPEG_QUALITY), self.a.quality])
            self.n_stills += 1
            self.next_still += self.a.still_interval

    def stop_clip(self) -> dict:
        self.recording = False
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        return {"video": str(self.video_path) if self.video_path else None,
                "stills": self.n_stills, "frames": self.n_frames}

    def close(self):
        self.cap.release()


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(
        description="Capture WRO pillar footage on a Raspberry Pi.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    p.add_argument("--list-cameras", action="store_true")
    p.add_argument("--label", choices=["red", "green", "mixed", "negative"],
                   help="what is in frame. 'negative' = distractors only "
                        "(orange/blue lines, magenta walls, people, robots)")
    p.add_argument("--lighting", default=None,
                   help="REQUIRED. Lighting tag, e.g. arena-warm, arena-dim, "
                        "backlit. This is the train/val holdout unit.")
    p.add_argument("--notes", default="")

    p.add_argument("--outdir", default="~/wro_dataset/raw")
    p.add_argument("--session", default=None)

    # trigger
    p.add_argument("--button", type=int, default=None,
                   metavar="GPIO",
                   help="BCM pin for a toggle button (tap=start/stop, hold 2s=quit). "
                        "Wire the other leg to GND.")
    p.add_argument("--led", type=int, default=None, metavar="GPIO",
                   help="BCM pin for a recording-indicator LED (via ~330 ohm)")
    p.add_argument("--key", action="store_true",
                   help="force keyboard mode (this is already the default when "
                        "no --button and no --duration is given)")
    p.add_argument("--rec-key", default="p", help="key that toggles recording")
    p.add_argument("--quit-key", default="q", help="key that exits")
    p.add_argument("--hold-quit", type=float, default=2.0,
                   help="seconds to hold the button to exit")
    p.add_argument("--max-clip", type=float, default=300.0,
                   help="safety cap on clip length in button/key mode")

    p.add_argument("--mode", choices=["video", "stills", "both"], default="both")
    p.add_argument("--duration", type=float, default=None,
                   help="fixed clip length in seconds. If omitted and no "
                        "--button is given, the keyboard toggle is used.")
    p.add_argument("--still-interval", type=float, default=1.0,
                   help="seconds between stills (do NOT go below ~0.5)")
    p.add_argument("--size", default="1280x720")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--bitrate", type=int, default=12_000_000)
    p.add_argument("--quality", type=int, default=95)
    p.add_argument("--settle", type=float, default=2.0)

    p.add_argument("--awb", choices=["auto", "lock"], default="auto")
    p.add_argument("--colour-gains", default=None, help="R,B e.g. 1.85,1.55")
    p.add_argument("--shutter", type=int, default=None, help="microseconds")
    p.add_argument("--gain", type=float, default=None)
    p.add_argument("--focus", default=None,
                   help="'auto', 'continuous', or dioptres (1/distance_m)")

    p.add_argument("--backend", choices=["auto", "picamera2", "opencv"],
                   default="auto")
    p.add_argument("--device", type=int, default=0)

    a = p.parse_args()

    if a.list_cameras:
        list_cameras()
        return 0
    if not a.label:
        p.error("--label is required")
    if not a.lighting:
        p.error("--lighting is required. Untagged sessions cannot be held out "
                "for validation, and a random split will leak.")
    if a.still_interval < 0.5 and a.mode in ("stills", "both"):
        print(f"[!] --still-interval {a.still_interval}s produces near-duplicate "
              f"frames. You inflate the dataset without adding information.")

    root = Path(os.path.expanduser(a.outdir))
    session_dir = (root / a.session) if a.session \
        else next_session_dir(root, a.lighting)
    label_dir = session_dir / a.label
    label_dir.mkdir(parents=True, exist_ok=True)
    stills_dir = None
    if a.mode in ("stills", "both"):
        stills_dir = label_dir / "stills"
        stills_dir.mkdir(exist_ok=True)

    # trigger selection
    if a.button is not None:
        trig = ButtonTrigger(a.button, a.led, a.hold_quit, a.max_clip)
        trig_name = f"button GPIO{a.button}" + (f" + LED GPIO{a.led}" if a.led else "")
    elif a.duration is not None and not a.key:
        trig = DurationTrigger(a.duration)
        trig_name = f"fixed {a.duration:.0f}s"
    else:
        trig = KeyTrigger(a.max_clip, a.rec_key, a.quit_key)
        trig_name = f"keyboard '{a.rec_key}'"

    backend_name = a.backend
    if backend_name == "auto":
        backend_name = "picamera2" if have_picamera2() else "opencv"

    print(f"session : {session_dir}")
    print(f"label   : {a.label}   lighting: {a.lighting}")
    print(f"backend : {backend_name}   {a.size} @ {a.fps} fps   mode={a.mode}")
    print(f"trigger : {trig_name}\n")

    cam = PiCamBackend(a) if backend_name == "picamera2" else CvBackend(a)

    clips = 0
    total_stills = 0
    try:
        while trig.wait_for_start():
            stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
            clip_base = label_dir / f"{a.label}_{stamp}"
            applied = cam.applied()
            cam.start_clip(clip_base, stills_dir)
            t0 = time.monotonic()
            print(f"  [REC] {clip_base.name}")
            while True:
                elapsed = time.monotonic() - t0
                if trig.should_stop(elapsed):
                    break
                cam.poll(elapsed)
                if backend_name == "picamera2":
                    time.sleep(0.01)
                print(f"\r        t={elapsed:5.1f}s", end="", flush=True)
            res = cam.stop_clip()
            elapsed = round(time.monotonic() - t0, 2)
            if hasattr(trig, "clip_done"):
                trig.clip_done()
            print(f"\r  [ok ] {elapsed}s  stills={res.get('stills')}  "
                  f"video={res.get('video')}")

            meta = {
                "script_version": SCRIPT_VERSION,
                "captured_at": datetime.now().isoformat(timespec="seconds"),
                "hostname": platform.node(),
                "session": session_dir.name,
                "label": a.label,
                "lighting": a.lighting,
                "notes": a.notes,
                "trigger": trig_name,
                "requested": {
                    "size": a.size, "fps": a.fps, "mode": a.mode,
                    "still_interval_s": a.still_interval,
                    "awb": a.awb, "colour_gains": a.colour_gains,
                    "shutter_us": a.shutter, "analogue_gain": a.gain,
                    "focus": a.focus, "bitrate": a.bitrate,
                    "jpeg_quality": a.quality,
                },
                "result": jsonable({**res, "elapsed_s": elapsed,
                                    "backend": backend_name,
                                    "applied_controls": applied}),
            }
            clip_base.with_suffix(".meta.json").write_text(json.dumps(meta, indent=2))

            clips += 1
            total_stills += res.get("stills") or 0
            if hasattr(trig, "should_quit") and trig.should_quit():
                break
    finally:
        cam.close()
        trig.close()

    on_disk = sum(1 for _ in session_dir.rglob("*.jpg"))
    print(f"\nclips this run     : {clips}")
    print(f"stills this run    : {total_stills}")
    print(f"stills in session  : {on_disk}")
    print(f"session dir        : {session_dir}")
    return 0


RUN_AT_BOOT = r"""
# /etc/systemd/system/wro-capture.service
# sudo systemctl daemon-reload && sudo systemctl enable --now wro-capture
[Unit]
Description=WRO dataset capture (button-driven)
After=multi-user.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
ExecStart=/usr/bin/python3 /home/pi/wro_capture.py \
    --label red --lighting arena-warm --button 17 --led 27 --mode stills
Restart=on-failure

[Install]
WantedBy=multi-user.target
"""

if __name__ == "__main__":
    sys.exit(main())
