#!/usr/bin/env python3
"""WRO block detector — Raspberry Pi port (Pi 5, Raspberry Pi OS Bookworm).

Two capture backends behind one interface; --source auto probes CSI first,
falls back to USB:

  CSI (Camera Module 2/3) -> Picamera2/libcamera. The ISP scales straight to
      the processing size in hardware, so the software resize is GONE on this
      path. NOTE: on Pi 5 the old V4L2 /dev/video0 route to CSI cams is dead —
      Picamera2 is the supported stack, do not try PyAV/ffmpeg here.
  USB (UVC webcam) -> cv2.VideoCapture(CAP_V4L2), MJPG, BUFFERSIZE=1, plus a
      grab thread keeping only the newest frame (V4L2's buffer queue otherwise
      adds 100-200 ms of lag).

The pipeline is BGR-native end to end: picamera2's "RGB888" arrays are B,G,R
in memory (their documented quirk), cv2.VideoCapture is BGR, imshow is BGR —
zero channel swaps; Lab via COLOR_BGR2LAB.

CALIBRATION v2 — per-brightness discs:
  Each color is now THREE tight Lab discs (low / medium / high illumination)
  instead of one inflated disc. Chroma shifts with brightness — a shadowed
  pillar sits closer to neutral in (a,b) than a glare-lit one — so one disc
  wide enough for both is also wide enough for false positives. The runtime
  mask is the UNION of a color's discs, each with its own L floor. All six
  buckets (2 colors x 3 levels) are mandatory before N accepts.
  Legacy single-disc JSONs still load (auto-wrapped as one disc per color);
  recalibrate with the 3-level scheme when convenient.

Setup:
    sudo apt update
    sudo apt install -y python3-opencv python3-picamera2
    # optional venv MUST inherit system packages — libcamera bindings are
    # apt-only, a clean venv breaks picamera2:
    python3 -m venv --system-site-packages venv

GUI: apt's python3-opencv (GTK) shows windows fine on Bookworm/Wayland; pip's
opencv-python (Qt) often cannot — if imshow dies, use the apt build or set
QT_QPA_PLATFORM=xcb. Robot runtime: --headless (requires a saved calibration —
calibrate once over VNC/HDMI first).

High-fps once the physical camera module is confirmed:
    Module 3 (IMX708) 120 fps:  --fps 120 --sensor 1536x864 --size 320x180
    Module 2 (IMX219) ~100 fps: --fps 100 --sensor 640x480  --size 240x180
(The fast sensor modes on Module 3 are 16:9 — pair them with a 16:9 --size or
the ISP crop/scale will distort.)

Usage:
    python3 block_detector_pi.py --list
    python3 block_detector_pi.py                       # auto: CSI, else USB 0
    python3 block_detector_pi.py --source usb -c 1     # 2nd UVC cam
    python3 block_detector_pi.py -c "HD Webcam"        # USB by partial name
    python3 block_detector_pi.py --headless            # robot runtime
    python3 block_detector_pi.py --fresh               # ignore saved calib

Keys — calibration: RED 1=low 2=medium 3=high, GREEN 4=low 5=medium 6=high,
R clear last bucket, N accept all six (auto-saves), Q abort.
Live: C recalibrate, Q quit.
"""

import argparse
import json
import queue
import re
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import cv2

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

FRAME_W, FRAME_H = 240, 180          # default processing size, 4:3
USB_CAPTURE_W, USB_CAPTURE_H = 640, 480
DEFAULT_FPS = 30.0
MIN_CHROMA = 10.0
EDGE_MARGIN = 6
DISPLAY_SCALE = 3
DEFAULT_CALIB_PATH = Path(__file__).with_name("wro_block_calib.json")

LEVELS = ("low", "medium", "high")
KEY_TO_BUCKET = {
    ord("1"): ("red", "low"),
    ord("2"): ("red", "medium"),
    ord("3"): ("red", "high"),
    ord("4"): ("green", "low"),
    ord("5"): ("green", "medium"),
    ord("6"): ("green", "high"),
}

# Pi platform V4L2 nodes (ISP/codec/CSI plumbing) — not user cameras.
PLATFORM_NODE_HINTS = ("pisp", "rp1-cfe", "unicam", "bcm2835", "rpivid", "hevc")


# ---------------------------------------------------------------------------
# Color math
# ---------------------------------------------------------------------------

def bgr_to_lab(bgr_u8: np.ndarray) -> np.ndarray:
    """float32 CIELAB, D65: L in [0,100], a/b signed. Same units as the
    desktop rgb_to_lab — Lab does not care which channel order fed it."""
    bgr = bgr_u8.astype(np.float32) * (1.0 / 255.0)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)


def get_masks(lab: np.ndarray, calib: dict) -> tuple:
    """calib = {color: {level: disc}}. A pixel matches a color if it falls in
    ANY of that color's discs (each disc: squared (a,b) distance < tol^2 and
    L above that disc's own floor), and clears the global chroma gate.
    Squared comparisons throughout — no per-disc sqrt."""
    L, a, b = lab[..., 0], lab[..., 1], lab[..., 2]
    chroma_ok = (a ** 2 + b ** 2) > MIN_CHROMA ** 2

    masks = {}
    for color in ("red", "green"):
        m = np.zeros(L.shape, dtype=bool)
        for d in (calib.get(color) or {}).values():
            dist2 = (a - d["a_center"]) ** 2 + (b - d["b_center"]) ** 2
            m |= (dist2 < d["tol"] ** 2) & (L > d["l_min"])
        masks[color] = m & chroma_ok

    return masks["red"], masks["green"]


def extract_bounding_box(mask: np.ndarray, min_area: int = 60,
                         min_extent: float = 0.55, max_aspect: float = 6.0) -> dict:
    """Largest connected blob. Shape gates unchanged: aspect <= max_aspect
    either way, extent >= min_extent — rejects reflections and scatter."""
    m = mask.astype(np.uint8)
    if cv2.countNonZero(m) < min_area:
        return None

    n, _labels, stats, _cent = cv2.connectedComponentsWithStats(m, connectivity=8)
    if n <= 1:
        return None

    areas = stats[1:, cv2.CC_STAT_AREA]
    i = 1 + int(np.argmax(areas))
    area = int(stats[i, cv2.CC_STAT_AREA])
    if area < min_area:
        return None

    x = int(stats[i, cv2.CC_STAT_LEFT])
    y = int(stats[i, cv2.CC_STAT_TOP])
    w = int(stats[i, cv2.CC_STAT_WIDTH])
    h = int(stats[i, cv2.CC_STAT_HEIGHT])

    if w < 6 or h < 6 or w > max_aspect * h or h > max_aspect * w:
        return None
    if area / float(w * h) < min_extent:
        return None

    return {"x": x, "y": y, "width": w, "height": h,
            "center_x": x + w // 2, "center_y": y + h // 2}


def process_frame(frame_bgr: np.ndarray, calib: dict,
                  edge_margin: int = EDGE_MARGIN) -> tuple:
    lab = bgr_to_lab(frame_bgr)
    red_mask, green_mask = get_masks(lab, calib)

    if edge_margin > 0:
        for m in (red_mask, green_mask):
            m[:edge_margin, :] = False
            m[-edge_margin:, :] = False
            m[:, :edge_margin] = False
            m[:, -edge_margin:] = False

    return extract_bounding_box(red_mask), extract_bounding_box(green_mask)


# ---------------------------------------------------------------------------
# Display (frames are already BGR — no conversion anywhere)
# ---------------------------------------------------------------------------

def draw_boxes(frame_bgr: np.ndarray, red_box: dict, green_box: dict,
               roi: tuple = None) -> np.ndarray:
    out = frame_bgr.copy()
    if roi is not None:
        rx, ry, rw, rh = roi
        cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), (255, 255, 0), 1)
    for box, color, label in ((red_box, (0, 0, 255), "RED"),
                              (green_box, (0, 255, 0), "GREEN")):
        if box:
            x, y, w, h = box["x"], box["y"], box["width"], box["height"]
            cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
            cv2.putText(out, f"{label} {w}x{h}", (x, max(0, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    return out


def upscale_for_display(frame_bgr: np.ndarray, scale: int = DISPLAY_SCALE) -> np.ndarray:
    h, w = frame_bgr.shape[:2]
    return cv2.resize(frame_bgr, (w * scale, h * scale),
                      interpolation=cv2.INTER_NEAREST)


def show(window: str, frame_bgr: np.ndarray, red_box, green_box,
         roi=None, hud: str = None) -> int:
    disp = upscale_for_display(draw_boxes(frame_bgr, red_box, green_box, roi=roi))
    if hud:
        cv2.putText(disp, hud, (5, disp.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.imshow(window, disp)
    return cv2.waitKey(1) & 0xFF


# ---------------------------------------------------------------------------
# Calibration — v2: 6 buckets (2 colors x 3 brightness levels)
# ---------------------------------------------------------------------------

def sample_roi_lab(frame_bgr: np.ndarray, roi: tuple) -> np.ndarray:
    x, y, w, h = roi
    return bgr_to_lab(frame_bgr[y:y + h, x:x + w])


def calibrate_color(lab_patches: list, margin: float = 5.0,
                    l_percentile: float = 5.0, max_tol: float = 15.0) -> dict:
    """Median center + MAD spread over pooled patches; single circular radius,
    hard-capped so a messy sample can't degenerate into match-everything.
    Now fitted PER BRIGHTNESS BUCKET, so each disc stays tight."""
    a_vals = np.concatenate([p[..., 1].ravel() for p in lab_patches])
    b_vals = np.concatenate([p[..., 2].ravel() for p in lab_patches])
    l_vals = np.concatenate([p[..., 0].ravel() for p in lab_patches])

    a_center = float(np.median(a_vals))
    b_center = float(np.median(b_vals))
    a_mad = float(np.median(np.abs(a_vals - a_center)))
    b_mad = float(np.median(np.abs(b_vals - b_center)))

    spread = np.sqrt((a_mad * 1.4826) ** 2 + (b_mad * 1.4826) ** 2)
    tol = min(spread * 1.3 + margin, max_tol)

    return {"a_center": a_center, "b_center": b_center, "tol": tol,
            "l_min": float(np.percentile(l_vals, l_percentile))}


def save_calib(calib: dict, path: Path, frame_size: tuple):
    payload = {"version": 2,
               "frame": list(frame_size),
               "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
               "calib": calib}
    path.write_text(json.dumps(payload, indent=2))
    print(f"Calibration saved -> {path}")


def load_calib(path: Path) -> dict:
    """v2 schema: {color: {level: disc}}. Legacy v1 single-disc files are
    auto-wrapped as one 'legacy' disc per color so they still run — the union
    logic doesn't care how many discs a color has."""
    try:
        raw = json.loads(path.read_text())["calib"]
        out = {}
        for color in ("red", "green"):
            c = raw[color]
            if "a_center" in c:                      # legacy v1 single disc
                c = {"legacy": c}
            out[color] = {}
            for level, d in c.items():
                out[color][level] = {k: float(d[k])
                                     for k in ("a_center", "b_center",
                                               "tol", "l_min")}
            if not out[color]:
                return None
        if any("legacy" in out[c] for c in out):
            print("Note: legacy single-disc calibration loaded — redo with the "
                  "3-brightness scheme (C) when convenient.")
        return out
    except Exception:
        return None


def print_calib(calib: dict, indent: str = "  "):
    for color in ("red", "green"):
        for level, d in (calib.get(color) or {}).items():
            print(f"{indent}{color}/{level}: a={d['a_center']:.1f} "
                  f"b={d['b_center']:.1f} tol={d['tol']:.1f} "
                  f"l_min={d['l_min']:.1f}")


def calib_usable(calib: dict) -> bool:
    return bool(calib) and bool(calib.get("red")) and bool(calib.get("green"))


def run_calibration_session(cam, roi: tuple, window: str, frame_size: tuple,
                            fallback: dict = None, save_path: Path = None) -> dict:
    """Six buckets: RED x {low, medium, high} on keys 1/2/3, GREEN on 4/5/6.
    Each keypress ADDS a sample to that bucket (sliding window of the newest
    MAX_SAMPLES so pre-AE-settle samples don't poison the fit) and refits that
    bucket's disc; the live preview updates with every disc added so far.
    N requires ALL SIX buckets filled — a missing brightness level is a hole
    in runtime coverage, not a smaller tolerance."""
    MAX_SAMPLES = 4
    samples = {(c, l): deque(maxlen=MAX_SAMPLES)
               for c in ("red", "green") for l in LEVELS}
    calib = {"red": {}, "green": {}}

    print("\n=== CALIBRATION — 3 brightness levels per color ===")
    print("Make the brightness yourself: LOW = shadow the pillar with your")
    print("hand / turn it away from the light; MEDIUM = normal; HIGH = face")
    print("the brightest light on the field. Let AE settle before sampling.")
    print("RED   in the cyan box:  1=low  2=medium  3=high   (2-3 samples each)")
    print("GREEN in the cyan box:  4=low  5=medium  6=high   (2-3 samples each)")
    print("N = accept all six (saves), R = clear last bucket, Q = abort.\n")

    last_bucket = None
    last_sample_time = 0.0
    debounce_s = 0.6

    while True:
        frame = cam.get()
        if frame is None:
            if not cam.alive:
                print(f"\nCapture died during calibration: {cam.error}")
                return fallback
            continue

        has_any = calib["red"] or calib["green"]
        red_box, green_box = (process_frame(frame, calib) if has_any
                              else (None, None))
        counts = {c: "/".join(str(len(samples[(c, l)])) for l in LEVELS)
                  for c in ("red", "green")}
        hud = (f"R {counts['red']}  G {counts['green']}  (l/m/h) | "
               f"1-3=RED 4-6=GRN N=done R=undo Q=abort")
        key = show(window, frame, red_box, green_box, roi=roi, hud=hud)

        now = time.monotonic()
        if key in KEY_TO_BUCKET and (now - last_sample_time) > debounce_s:
            color, level = KEY_TO_BUCKET[key]
            samples[(color, level)].append(sample_roi_lab(frame, roi))
            calib[color][level] = calibrate_color(list(samples[(color, level)]))
            last_bucket, last_sample_time = (color, level), now
            d = calib[color][level]
            print(f"Sampled {color.upper()}/{level} "
                  f"({len(samples[(color, level)])}/{MAX_SAMPLES}) -> "
                  f"a={d['a_center']:.1f} b={d['b_center']:.1f} "
                  f"tol={d['tol']:.1f} l_min={d['l_min']:.1f}")
        elif key == ord("r") and last_bucket:
            color, level = last_bucket
            samples[last_bucket].clear()
            calib[color].pop(level, None)
            print(f"Cleared {color}/{level} — sample it again.")
            last_bucket = None
        elif key == ord("n"):
            missing = [f"{c}/{l}" for c in ("red", "green") for l in LEVELS
                       if l not in calib[c]]
            if not missing:
                print("Calibration accepted.\n")
                if save_path:
                    save_calib(calib, save_path, frame_size)
                return calib
            print(f"Missing buckets: {', '.join(missing)}")
        elif key == ord("q"):
            print("Calibration aborted — keeping previous calibration."
                  if fallback else "Calibration aborted.")
            return fallback


# ---------------------------------------------------------------------------
# Camera sources — the actual Pi-specific part
# ---------------------------------------------------------------------------

def csi_cameras():
    """(global_index, info) for real CSI cameras. libcamera also enumerates
    UVC cams on newer stacks — those Ids contain 'usb'; we keep UVC on the
    cv2 path where MJPG/fps are controllable. Raises ImportError if picamera2
    is missing."""
    from picamera2 import Picamera2
    out = []
    for i, inf in enumerate(Picamera2.global_camera_info()):
        if "usb" not in inf.get("Id", "").lower():
            out.append((i, inf))
    return out


class CsiSource:
    """Picamera2/libcamera. capture_array blocks until the next completed
    frame — worst case one frame of latency, so no grab thread is needed.
    ISP scales to (w, h) in hardware: no software resize on this path."""

    def __init__(self, camera_num: int, w: int, h: int, fps: float,
                 sensor: tuple = None, label: str = "?"):
        self.camera_num, self.w, self.h, self.fps = camera_num, w, h, fps
        self.sensor = sensor
        self.label = label
        self.picam = None
        self.error = None
        self._started = False

    def open(self) -> bool:
        try:
            from picamera2 import Picamera2
        except ImportError as e:
            print(f"picamera2 not importable ({e}).\n"
                  "Fix: sudo apt install python3-picamera2 ; "
                  "a venv must be created with --system-site-packages.")
            return False
        try:
            self.picam = Picamera2(self.camera_num)
            frame_us = int(round(1_000_000 / self.fps))
            kw = {
                # picamera2 "RGB888" = B,G,R in memory — exactly what cv2 wants.
                "main": {"size": (self.w, self.h), "format": "RGB888"},
                "controls": {"FrameDurationLimits": (frame_us, frame_us)},
            }
            if self.sensor:
                kw["sensor"] = {"output_size": self.sensor}
            self.picam.configure(self.picam.create_video_configuration(**kw))
            self.picam.start()
            self._started = True
            cfg = self.picam.camera_configuration() or {}
            sensor_sz = (cfg.get("sensor") or {}).get("output_size", "?")
            print(f"CSI open: {self.label} main={self.w}x{self.h} "
                  f"sensor={sensor_sz} target {self.fps:g} fps")
            return True
        except Exception as e:
            print(f"CSI open failed: {e}")
            return False

    def start(self):
        pass  # capture is pull-based

    def get(self, timeout: float = 1.0):
        try:
            return self.picam.capture_array("main")
        except Exception as e:
            self.error = e
            return None

    @property
    def alive(self) -> bool:
        return self._started and self.error is None

    def stop(self):
        if self.picam:
            try:
                self.picam.stop()
                self.picam.close()
            except Exception:
                pass


def list_v4l2_devices(include_platform: bool = False):
    """(path, name) for /dev/video* nodes, platform plumbing filtered out."""
    out = []
    nodes = sorted(Path("/dev").glob("video*"),
                   key=lambda p: int(re.sub(r"\D", "", p.name) or 0))
    for p in nodes:
        namef = Path("/sys/class/video4linux") / p.name / "name"
        try:
            name = namef.read_text().strip()
        except OSError:
            name = "?"
        if not include_platform and any(h in name.lower()
                                        for h in PLATFORM_NODE_HINTS):
            continue
        out.append((str(p), name))
    return out


def dedupe_metadata_twins(devs):
    """A UVC cam exposes capture+metadata nodes with the same name; the lower
    node is the capture one. Collapse adjacent same-name pairs."""
    out = []
    for path, name in devs:
        if out and out[-1][1] == name:
            continue
        out.append((path, name))
    return out


def resolve_usb(spec: str):
    """Index into the detected UVC list, /dev/videoN path, or partial name.
    NOTE: a bare index is NOT the /dev/video number — platform nodes make the
    raw numbering meaningless; use an explicit /dev/ path to force a node."""
    if spec.startswith("/dev/"):
        return spec
    devs = dedupe_metadata_twins(list_v4l2_devices())
    if spec.isdigit():
        i = int(spec)
        if i < len(devs):
            return devs[i][0]
        print(f"USB index {i} out of range. Found: {devs or 'none'}")
        return None
    matches = [d for d in devs if spec.lower() in d[1].lower()]
    if matches:
        return matches[0][0]
    print(f"No USB camera matching {spec!r}. Found: {devs or 'none'}")
    return None


class UsbSource:
    """cv2.VideoCapture(CAP_V4L2) + a grab thread keeping only the newest
    frame. MJPG fourcc set BEFORE size/fps (driver quirk); BUFFERSIZE=1."""

    def __init__(self, device: str, w: int, h: int, fps: float):
        self.device, self.w, self.h, self.fps = device, w, h, fps
        self.cap = None
        self._q = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = None
        self.error = None

    def open(self) -> bool:
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print(f"FAILED to open {self.device}")
            return False
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, USB_CAPTURE_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, USB_CAPTURE_H)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = self.cap.read()
        if not ok or frame is None:
            print(f"{self.device} opened but returned no frame "
                  "(metadata node? try the other /dev/video number).")
            self.cap.release()
            return False
        aw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        af = self.cap.get(cv2.CAP_PROP_FPS)
        fcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fcc_s = "".join(chr((fcc >> 8 * i) & 0xFF) for i in range(4))
        print(f"USB open: {self.device} {aw}x{ah}@{af:.0f} {fcc_s!s}")
        return True

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        try:
            while not self._stop.is_set():
                ok, frame = self.cap.read()
                if not ok or frame is None:
                    raise RuntimeError("VideoCapture.read() failed (unplugged?)")
                small = cv2.resize(frame, (self.w, self.h),
                                   interpolation=cv2.INTER_AREA)
                if self._q.full():
                    try:
                        self._q.get_nowait()
                    except queue.Empty:
                        pass
                self._q.put(small)
        except Exception as e:
            if not self._stop.is_set():
                self.error = e
        finally:
            try:
                self.cap.release()
            except Exception:
                pass

    def get(self, timeout: float = 1.0):
        try:
            return self._q.get(timeout=timeout)
        except queue.Empty:
            return None

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_wh(s: str) -> tuple:
    m = re.fullmatch(r"(\d+)\s*[xX]\s*(\d+)", s.strip())
    if not m:
        print(f"Bad size {s!r} — expected WxH, e.g. 240x180")
        sys.exit(2)
    return int(m.group(1)), int(m.group(2))


def list_all_cameras():
    try:
        cams = csi_cameras()
        if cams:
            print("CSI (libcamera):")
            for idx, (num, inf) in enumerate(cams):
                print(f"  [csi {idx}] {inf.get('Model', '?')}  {inf.get('Id', '')}")
        else:
            print("CSI: none detected (`rpicam-hello --list-cameras` to verify).")
    except ImportError:
        print("CSI: picamera2 not importable "
              "(sudo apt install python3-picamera2).")
    devs = dedupe_metadata_twins(list_v4l2_devices())
    if devs:
        print("USB (V4L2):")
        for i, (path, name) in enumerate(devs):
            print(f"  [{i}] {path}  {name}")
    else:
        print("USB: no UVC capture nodes.")


def open_source(args, w: int, h: int):
    sensor = parse_wh(args.sensor) if args.sensor else None

    if args.source in ("auto", "csi"):
        try:
            cams = csi_cameras()
        except ImportError:
            cams = []
            msg = ("picamera2 not importable — sudo apt install "
                   "python3-picamera2 (venv needs --system-site-packages).")
            if args.source == "csi":
                print(msg)
                return None
            print(msg + " Trying USB.")
        if cams:
            pick = 0
            if args.source == "csi" and args.camera.isdigit():
                pick = min(int(args.camera), len(cams) - 1)
            num, inf = cams[pick]
            csi = CsiSource(num, w, h, args.fps, sensor=sensor,
                            label=inf.get("Model", "?"))
            if csi.open():
                return csi
            if args.source == "csi":
                return None
            print("CSI open failed — trying USB.")
        elif args.source == "csi":
            print("No CSI camera detected (check ribbon seating/orientation; "
                  "`rpicam-hello --list-cameras`).")
            return None

    dev = resolve_usb(args.camera)
    if dev is None:
        return None
    usb = UsbSource(dev, w, h, args.fps)
    return usb if usb.open() else None


def main():
    ap = argparse.ArgumentParser(description="WRO red/green block detector (Pi)")
    ap.add_argument("--source", choices=("auto", "csi", "usb"), default="auto",
                    help="capture backend (default: probe CSI, fall back USB)")
    ap.add_argument("-c", "--camera", default="0",
                    help="USB: index into detected UVC list, /dev/videoN, or "
                         "partial name. CSI: index into detected CSI list.")
    ap.add_argument("--fps", type=float, default=DEFAULT_FPS)
    ap.add_argument("--size", default=f"{FRAME_W}x{FRAME_H}",
                    help="processing size WxH (CSI: delivered by ISP directly)")
    ap.add_argument("--sensor", default=None,
                    help="CSI sensor mode hint WxH — required for high-fps "
                         "modes, e.g. 1536x864 for Module 3 @120fps")
    ap.add_argument("--headless", action="store_true",
                    help="no GUI (robot runtime); requires saved calibration")
    ap.add_argument("--list", action="store_true", help="list cameras and exit")
    ap.add_argument("--calib", type=Path, default=DEFAULT_CALIB_PATH,
                    help="calibration JSON path")
    ap.add_argument("--fresh", action="store_true", help="ignore saved calibration")
    args = ap.parse_args()

    if args.list:
        list_all_cameras()
        return

    w, h = parse_wh(args.size)

    cam = open_source(args, w, h)
    if cam is None:
        print("\nNo camera opened. Checklist:\n"
              "  - --list to see what's detected\n"
              "  - CSI: ribbon seated, contacts facing the right way, "
              "`rpicam-hello` works\n"
              "  - USB: try the explicit /dev/videoN path\n"
              "  - close anything holding the camera")
        sys.exit(1)
    cam.start()

    window = "WRO Block Detector"
    if not args.headless:
        cv2.namedWindow(window)

    s = min(w, h) // 4
    roi = ((w - s) // 2, (h - s) // 2, s, s)

    calib = None
    if not args.fresh:
        calib = load_calib(args.calib)
        if calib:
            print(f"Loaded calibration from {args.calib}"
                  + ("" if args.headless else " (C = recalibrate if lighting changed)")
                  + ":")
            print_calib(calib)

    if calib is None:
        if args.headless:
            print("Headless mode needs an existing calibration file.\n"
                  "Calibrate once with a display/VNC attached, then rerun "
                  "--headless.")
            cam.stop()
            sys.exit(1)
        calib = run_calibration_session(cam, roi, window, (w, h),
                                        fallback=None, save_path=args.calib)
        if not calib_usable(calib):
            print("No usable calibration — exiting.")
            cam.stop()
            cv2.destroyAllWindows()
            return

    print("=== LIVE ===  "
          + ("Ctrl+C = quit\n" if args.headless else "C = recalibrate, Q = quit\n"))

    # Temporal confirmation: color must appear in >= `required` of the last
    # `history_len` frames — kills single-frame flicker without real lag.
    history_len, required = 5, 3
    red_hist = deque(maxlen=history_len)
    green_hist = deque(maxlen=history_len)

    frame_count, misses = 0, 0
    fps_ema = None
    t_prev = time.perf_counter()

    try:
        while True:
            frame = cam.get()
            if frame is None:
                if not cam.alive:
                    print(f"\nCapture died: {cam.error}")
                    break
                misses += 1
                if misses == 5:
                    print("\nNo frames for ~5s — camera stalled?")
                continue
            misses = 0

            t0 = time.perf_counter()
            red_box, green_box = process_frame(frame, calib)
            proc_ms = (time.perf_counter() - t0) * 1000.0

            red_hist.append(red_box)
            green_hist.append(green_box)
            display_red = (red_box if sum(b is not None for b in red_hist)
                           >= required else None)
            display_green = (green_box if sum(b is not None for b in green_hist)
                             >= required else None)

            now = time.perf_counter()
            dt = max(now - t_prev, 1e-6)
            t_prev = now
            fps_ema = (1.0 / dt) if fps_ema is None else 0.9 * fps_ema + 0.1 / dt

            key = 255
            if not args.headless:
                hud = f"{fps_ema:4.1f} fps  proc {proc_ms:.2f} ms  C=recal Q=quit"
                key = show(window, frame, display_red, display_green, hud=hud)

            frame_count += 1
            red_str = (f"R[x={display_red['x']},y={display_red['y']},"
                       f"w={display_red['width']},h={display_red['height']}]"
                       if display_red else "R:None")
            green_str = (f"G[x={display_green['x']},y={display_green['y']},"
                         f"w={display_green['width']},h={display_green['height']}]"
                         if display_green else "G:None")
            print(f"F{frame_count} {fps_ema:4.1f}fps {proc_ms:.2f}ms | "
                  f"{red_str} | {green_str}   ", end="\r")

            if key == ord("q"):
                break
            elif key == ord("c"):
                calib = run_calibration_session(cam, roi, window, (w, h),
                                                fallback=calib,
                                                save_path=args.calib)
                red_hist.clear()
                green_hist.clear()
                print("=== LIVE ===  C = recalibrate, Q = quit\n")

    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        if not args.headless:
            cv2.destroyAllWindows()
        print("\nFinal calibration:")
        print_calib(calib or {})


if __name__ == "__main__":
    main()