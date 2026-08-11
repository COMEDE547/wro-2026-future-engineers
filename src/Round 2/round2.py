#!/usr/bin/env python3

import time
import threading
import queue
import subprocess
import re
import serial
import argparse
import json
import os
import glob
import sys
import logging
import numpy as np
import cv2
import av
from collections import deque
from PIL import Image
# --- Minimum height (in pixels) before the robot starts to swerve ---
MIN_SWERVE_HEIGHT = 45   # block must be at least this tall to trigger avoidance
REVERSE_HEIGHT = 80  # if block is below this y, reverse instead of swerve
            # --- Thresholds for determining which side the block is on ---
LEFT_SIDE_MAX   = 90   # pixel x < this = left side
RIGHT_SIDE_MIN  = 150   # pixel x > this = right side
def rgb_to_lab(frame: np.ndarray) -> np.ndarray:
    """2026-08-11 (research A2): OpenCV native LAB replaces the hand-rolled float32
    pipeline — per-pixel pow/cbrt in NumPy was the suspected fps ceiling; cvtColor is
    SIMD C. Output is OpenCV 8-bit LAB: L scaled 0..255 (x255/100), a and b offset
    +128 (same units as before, only shifted). ALL calibration values now live in this
    space; legacy float-LAB calib.json files migrate automatically in load_calib().
    Verified against the old float path on synthetic frames (mask IoU, bbox parity) —
    still confirm once on a saved real mat frame before trusting (RETUNE-REQUIRED only
    if that check fails)."""
    return cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)


_AB_LUT_CACHE = {}


def _ab_lut(a_center: float, b_center: float, tol: float, min_chroma: float = 10.0) -> np.ndarray:
    """256x256 boolean lookup over (a,b): inside the calibrated chroma-distance circle
    AND outside the gray core — exactly the old circular-mask semantics (a box inRange
    would widen the corners), applied at fancy-indexing speed. Cached per calibration."""
    key = (round(a_center, 2), round(b_center, 2), round(tol, 2), round(min_chroma, 2))
    lut = _AB_LUT_CACHE.get(key)
    if lut is None:
        ax = np.arange(256, dtype=np.float32)
        A, B = np.meshgrid(ax, ax, indexing='ij')
        inside = ((A - a_center) ** 2 + (B - b_center) ** 2) < (tol * tol)
        chroma = np.sqrt((A - 128.0) ** 2 + (B - 128.0) ** 2)
        lut = inside & (chroma > min_chroma)
        _AB_LUT_CACHE[key] = lut
    return lut


def migrate_calib_to_u8(calib: dict) -> dict:
    """Deterministic legacy migration (research A2): float-LAB values carry the
    documented affine map — a,b offset +128 (tolerance units unchanged), l_min x255/100."""
    out = {"space": "cv2lab_u8"}
    for color, c in calib.items():
        if color == "space":
            continue
        out[color] = {
            'a_center': float(c['a_center']) + 128.0,
            'b_center': float(c['b_center']) + 128.0,
            'tol': float(c['tol']),
            'l_min': float(c['l_min']) * 255.0 / 100.0,
        }
    return out


def mask_for(lab: np.ndarray, calib: dict, color: str) -> np.ndarray:
    c = calib.get(color)
    if c is None:
        return np.zeros(lab.shape[:2], dtype=bool)
    L, a, b = lab[..., 0], lab[..., 1], lab[..., 2]
    return _ab_lut(c['a_center'], c['b_center'], c['tol'])[a, b] & (L > c['l_min'])


def get_masks(lab: np.ndarray, calib: dict) -> tuple:
    return mask_for(lab, calib, 'red'), mask_for(lab, calib, 'green')


def extract_bounding_box(mask: np.ndarray, min_area: int = 60, min_extent: float = 0.55) -> dict:
    """2026-08-11 (research A2): largest blob via cv2.connectedComponentsWithStats —
    native single pass with area/bbox in the stats table; replaces scipy.ndimage.label
    plus a python-side reduction (scipy dependency dropped from this file entirely).
    connectivity=4 matches scipy's default structure. Returned width/height keep the
    old span-minus-one convention so every downstream threshold (MIN_SWERVE_HEIGHT,
    REVERSE_HEIGHT, aspect, safe lines) is numerically untouched."""
    if int(mask.sum()) < min_area:
        return None
    n, _labels, stats, _cents = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=4)
    if n <= 1:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA]
    i = 1 + int(np.argmax(areas))
    area = int(stats[i, cv2.CC_STAT_AREA])
    if area < min_area:
        return None
    x = int(stats[i, cv2.CC_STAT_LEFT])
    y = int(stats[i, cv2.CC_STAT_TOP])
    tw = int(stats[i, cv2.CC_STAT_WIDTH])
    th = int(stats[i, cv2.CC_STAT_HEIGHT])
    w, h = tw - 1, th - 1
    if w < 6 or h < 6 or (w / max(h, 1)) > 6 or (h / max(w, 1)) > 6:
        return None
    extent = area / (tw * th)
    if extent < min_extent:
        return None
    return {"x": x, "y": y, "width": w, "height": h,
            "center_x": x + w // 2, "center_y": y + h // 2}


def process_frame(frame: np.ndarray, calib: dict, edge_margin: int = 6) -> tuple:
    """Detect red/green blocks and (when calibrated) the magenta parking bay.
    2026-08-11 (research B1 foundation): magenta rides the same LAB pipeline for free —
    it is DATA ONLY downstream (MAG telemetry to the ESP32, which drives nothing from
    it); the parking attempt-or-descope decision gets venue-real detectability first."""
    if frame is None or frame.size == 0:
        return None, None, None

    lab = rgb_to_lab(frame)
    red_mask, green_mask = get_masks(lab, calib)
    mag_mask = mask_for(lab, calib, 'magenta')

    if edge_margin > 0:
        for m in (red_mask, green_mask, mag_mask):
            m[:edge_margin, :] = False
            m[-edge_margin:, :] = False
            m[:, :edge_margin] = False
            m[:, -edge_margin:] = False

    red_box = extract_bounding_box(red_mask)
    green_box = extract_bounding_box(green_mask)
    mag_box = extract_bounding_box(mag_mask) if 'magenta' in calib else None
    return red_box, green_box, mag_box


def upscale_for_display(frame_bgr: np.ndarray, scale: int = 3) -> np.ndarray:
    """Upscale for display."""
    h, w = frame_bgr.shape[:2]
    return cv2.resize(frame_bgr, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)


def draw_boxes(frame_bgr: np.ndarray, red_box: dict, green_box: dict, roi: tuple = None, mag_box: dict = None) -> np.ndarray:
    out = frame_bgr.copy()

    if roi is not None:
        rx, ry, rw, rh = roi
        cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), (255, 255, 0), 1)

    if red_box:
        x, y, w, h = red_box['x'], red_box['y'], red_box['width'], red_box['height']
        cx, cy = red_box['center_x'], red_box['center_y']

        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.circle(out, (cx, cy), 3, (0, 0, 255), -1)

        label1 = f"RED {w}x{h}px"
        label2 = f"pos=({x},{y}) center=({cx},{cy})"
        cv2.putText(out, label1, (x, max(0, y - 22)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        cv2.putText(out, label2, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

    if green_box:
        x, y, w, h = green_box['x'], green_box['y'], green_box['width'], green_box['height']
        cx, cy = green_box['center_x'], green_box['center_y']

        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(out, (cx, cy), 3, (0, 255, 0), -1)

        label1 = f"GREEN {w}x{h}px"
        label2 = f"pos=({x},{y}) center=({cx},{cy})"
        cv2.putText(out, label1, (x, max(0, y - 22)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
        cv2.putText(out, label2, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

    if mag_box:
        x, y, w, h = mag_box['x'], mag_box['y'], mag_box['width'], mag_box['height']
        cv2.rectangle(out, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.putText(out, f"MAG {w}x{h}px", (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)

    return out


def sample_roi_lab(frame: np.ndarray, roi: tuple) -> np.ndarray:
    """Sample LAB values from ROI."""
    x, y, w, h = roi
    patch = frame[y:y + h, x:x + w]
    return rgb_to_lab(patch)


def calibrate_color(lab_patches: list, margin: float = 5.0,
                     l_percentile: float = 5.0, max_tol: float = 15.0) -> dict:
    """Derive thresholds from sampled patches."""
    a_vals = np.concatenate([p[..., 1].flatten() for p in lab_patches])
    b_vals = np.concatenate([p[..., 2].flatten() for p in lab_patches])
    l_vals = np.concatenate([p[..., 0].flatten() for p in lab_patches])

    a_center = float(np.median(a_vals))
    b_center = float(np.median(b_vals))
    a_mad = float(np.median(np.abs(a_vals - a_center)))
    b_mad = float(np.median(np.abs(b_vals - b_center)))

    spread = np.sqrt((a_mad * 1.4826) ** 2 + (b_mad * 1.4826) ** 2)
    tol = min(spread * 1.3 + margin, max_tol)

    return {
        'a_center': a_center,
        'b_center': b_center,
        'tol': tol,
        'l_min': float(np.percentile(l_vals, l_percentile)),
    }


def run_calibration_session(get_frame, roi: tuple, window_name: str, initial: dict = None) -> dict:
    """Interactive calibration session. `initial` (e.g. a saved calib.json) is the starting
    calibration: press Q to keep it, or resample to replace it."""
    MAX_SAMPLES = 6
    samples = {'red': deque(maxlen=MAX_SAMPLES), 'green': deque(maxlen=MAX_SAMPLES), 'magenta': deque(maxlen=MAX_SAMPLES)}
    calib = dict(initial) if initial else {}
    calib['space'] = 'cv2lab_u8'   # 2026-08-11: all values in OpenCV 8-bit LAB from here on

    print("\n=== CALIBRATION SESSION ===")
    print("Tip: let the camera's auto-exposure settle for a second before sampling.")
    print("Hold the RED block inside the cyan box, press 1 to sample (3-4x, moving it slightly).")
    print("Hold the GREEN block inside the cyan box, press 2 to sample (3-4x).")
    print("OPTIONAL: hold the MAGENTA parking marker in the box, press 3 to sample (parking research).")
    print("Press N when satisfied with both, or R to clear the last color's samples.")
    print("Press Q to abort calibration.\n")

    last_color = None
    last_sample_time = 0.0
    debounce_s = 0.6

    while True:
        frame = get_frame()
        if frame is None:
            continue

        red_box, green_box, mag_box = (None, None, None)
        if calib:
            red_box, green_box, mag_box = process_frame(frame, calib)

        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        display = draw_boxes(bgr, red_box, green_box, roi=roi, mag_box=mag_box)
        display = upscale_for_display(display, scale=3)
        status = (f"RED:{len(samples['red'])} GREEN:{len(samples['green'])} MAG:{len(samples['magenta'])} | "
                  f"1=RED 2=GREEN 3=MAG  N=done  Q=abort")
        cv2.putText(display, status, (5, display.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imshow(window_name, display)

        key = cv2.waitKey(50) & 0xFF
        now = time.monotonic()

        if key == ord('1') and (now - last_sample_time) > debounce_s:
            samples['red'].append(sample_roi_lab(frame, roi))
            calib['red'] = calibrate_color(list(samples['red']))
            last_color = 'red'
            last_sample_time = now
            print(f"Sampled RED ({len(samples['red'])}/{MAX_SAMPLES}) -> "
                  f"a_center={calib['red']['a_center']:.1f} b_center={calib['red']['b_center']:.1f} "
                  f"tol={calib['red']['tol']:.1f} l_min={calib['red']['l_min']:.1f}")
        elif key == ord('2') and (now - last_sample_time) > debounce_s:
            samples['green'].append(sample_roi_lab(frame, roi))
            calib['green'] = calibrate_color(list(samples['green']))
            last_color = 'green'
            last_sample_time = now
            print(f"Sampled GREEN ({len(samples['green'])}/{MAX_SAMPLES}) -> "
                  f"a_center={calib['green']['a_center']:.1f} b_center={calib['green']['b_center']:.1f} "
                  f"tol={calib['green']['tol']:.1f} l_min={calib['green']['l_min']:.1f}")
        elif key == ord('3') and (now - last_sample_time) > debounce_s:
            samples['magenta'].append(sample_roi_lab(frame, roi))
            calib['magenta'] = calibrate_color(list(samples['magenta']))
            last_color = 'magenta'
            last_sample_time = now
            print(f"Sampled MAGENTA ({len(samples['magenta'])}/{MAX_SAMPLES}) -> "
                  f"a_center={calib['magenta']['a_center']:.1f} b_center={calib['magenta']['b_center']:.1f} "
                  f"tol={calib['magenta']['tol']:.1f} l_min={calib['magenta']['l_min']:.1f}")
        elif key == ord('r') and last_color:
            samples[last_color].clear()
            calib.pop(last_color, None)
            print(f"Cleared samples for {last_color} — start sampling it again.")
        elif key == ord('n'):
            if 'red' in calib and 'green' in calib:
                print("Calibration accepted.\n")
                return calib
            print("Sample both RED (1) and GREEN (2) before continuing.")
        elif key == ord('q'):
            print("Calibration aborted, using previous/default calibration if any.")
            return calib

    return calib


def open_camera(camera_id: int):
    """Open camera using PyAV with improved settings."""
    try:
        container = av.open(
            f'/dev/video{camera_id}',
            format='v4l2',
            options={
                'video_size': '640x480',
                'framerate': '30',
                'input_format': 'mjpeg'
            }
        )
        stream = container.streams.video[0]
        stream.thread_type = 'AUTO'
        return container, stream
    except Exception as e:
        try:
            container = av.open(
                f'/dev/video{camera_id}',
                format='v4l2',
                options={
                    'video_size': '640x480',
                    'framerate': '30',
                    'input_format': 'yuyv422'
                }
            )
            stream = container.streams.video[0]
            stream.thread_type = 'AUTO'
            return container, stream
        except Exception as e:
            print(f"Camera error: {e}")
            return None, None


def resize_frame(frame: np.ndarray, target_w: int = 240, target_h: int = 240) -> np.ndarray:
    """Resize frame using OpenCV (faster and more reliable than PIL)."""
    if frame is None or frame.size == 0:
        return None
    return cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)


def start_capture_thread(container, stream, frame_size=240):
    """Start capture thread with improved frame handling."""
    frame_q = queue.Queue(maxsize=1)
    stop_flag = threading.Event()

    def capture_loop():
        try:
            for packet in container.demux(stream):
                if stop_flag.is_set():
                    break
                for frame in packet.decode():
                    if stop_flag.is_set():
                        break
                    try:
                        if frame.format.name != 'rgb24':
                            frame = frame.reformat(format='rgb24')
                        img = frame.to_ndarray(format='rgb24')

                        if img is not None and img.size > 0:
                            img = resize_frame(img, frame_size, frame_size)
                            # img = cv2.rotate(img, cv2.ROTATE_180)
                            if img is not None:
                                if frame_q.full():
                                    try:
                                        frame_q.get_nowait()
                                    except queue.Empty:
                                        pass
                                frame_q.put(img)
                    except Exception as e:
                        print(f"Frame processing error: {e}")
                        continue
        except Exception as e:
            print(f"\nCapture thread stopped: {e}")

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    return t, frame_q, stop_flag


def set_manual_camera_controls(camera_id: int, exposure_value: int = 500,
                                wb_temperature: int = 4500):
    """Lock auto_exposure/white_balance so LAB calibration stays stable across runs.

    exposure 500 = 50 ms = an exact multiple of both the 10 ms (50 Hz) and 8.33 ms (60 Hz)
    mains half-periods, so it is flicker-safe on either grid. Trade-off: 50 ms exposure caps
    real fps at ~20 even though we request 30 — accepted; do not "fix" by lowering exposure.
    """
    dev = f'/dev/video{camera_id}'
    cmds = [
        ['v4l2-ctl', '-d', dev, '-c', 'auto_exposure=1'],
        ['v4l2-ctl', '-d', dev, '-c', f'exposure_time_absolute={exposure_value}'],
        ['v4l2-ctl', '-d', dev, '-c', 'white_balance_automatic=0'],
        ['v4l2-ctl', '-d', dev, '-c', f'white_balance_temperature={wb_temperature}'],
        # 2026-08-11 (research A8): 1 = 50 Hz anti-flicker — Indian venues; belt-and-
        # suspenders on top of the 50 ms flicker-safe exposure already locked above.
        ['v4l2-ctl', '-d', dev, '-c', 'power_line_frequency=1'],
    ]
    for cmd in cmds:
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Warning: could not run {' '.join(cmd)} ({e})")
    print(f"Camera controls locked: exposure={exposure_value}, wb_temp={wb_temperature}")
    # Read back what the driver actually accepted — a silent clamp here would invalidate calibration.
    try:
        out = subprocess.run(['v4l2-ctl', '-d', dev, '-C', 'exposure_time_absolute'],
                             capture_output=True, text=True, timeout=5)
        print(f"Driver reports: {out.stdout.strip()}")
    except Exception as e:
        print(f"Warning: could not read back exposure ({e})")


def save_calib(calib: dict, path: str):
    """Persist calibration so race-day headless runs can load it (calibrate during check time)."""
    try:
        with open(path, 'w') as f:
            json.dump(calib, f, indent=2)
        print(f"Calibration saved to {path}")
    except Exception as e:
        print(f"Warning: could not save calibration to {path} ({e})")


def load_calib(path: str):
    """Load a previously saved calibration; returns {} when absent or unreadable."""
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            calib = json.load(f)
        if calib and calib.get('space') != 'cv2lab_u8':
            calib = migrate_calib_to_u8(calib)
            log.info("[calib] legacy float-LAB file detected — migrated to cv2lab_u8 "
                     "(a,b +128; l_min x255/100; tol unchanged)")
        print(f"Loaded calibration from {path}: {list(calib.keys())}")
        return calib
    except Exception as e:
        print(f"Warning: could not load calibration from {path} ({e})")
        return {}


def create_kalman_filter():
    """Constant-velocity Kalman filter: state=[x,y,vx,vy], measurement=[x,y]."""
    kf = cv2.KalmanFilter(4, 2, 0, type = cv2.CV_64F)
    kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0]], np.float64)
    kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                      [0, 1, 0, 1],
                                      [0, 0, 1, 0],
                                      [0, 0, 0, 1]], np.float64)
    kf.processNoiseCov = np.eye(4, dtype=np.float64) * 0.03
    kf.measurementNoiseCov = np.eye(2, dtype=np.float64) * 1.0
    return kf





def kalman_update(kf, box, initialized: bool):
    if box is not None:
        measurement = np.array([[np.float64(box['center_x'])],
                                [np.float64(box['center_y'])]])
        if not initialized:
            kf.statePre = np.array([[box['center_x']], [box['center_y']], [0], [0]], np.float64)
            kf.statePost = np.array([[box['center_x']], [box['center_y']], [0], [0]], np.float64)
            initialized = True
        else:
            kf.predict()

        kf.correct(measurement)

        smoothed = {
            'center_x': int(kf.statePost[0, 0]),
            'center_y': int(kf.statePost[1, 0]),
            'vx': float(kf.statePost[2, 0]),
            'vy': float(kf.statePost[3, 0])
        }
        return smoothed, initialized
    else:

        if not initialized:
            return None, initialized
        predicted = kf.predict()
        smoothed = {
            'center_x': int(predicted[0, 0]),
            'center_y': int(predicted[1, 0]),
            'vx': float(predicted[2, 0]),
            'vy': float(predicted[3, 0])
        }
        return smoothed, initialized

log = logging.getLogger("round2")


def setup_logging(logdir: str) -> str:
    """2026-08-11 (fix #5): every run now writes a timestamped full-stack log — the Pi's
    per-frame decisions, every command sent to the ESP32 ([TX]), and the ESP32's own
    telemetry ([ESP]), which has been narrated over this same USB serial since day one
    with nothing ever reading it. Always on: no mat or race run is ever unrecorded again."""
    os.makedirs(logdir, exist_ok=True)
    path = os.path.join(logdir, time.strftime("run_%Y%m%d_%H%M%S.log"))
    fmt = logging.Formatter("%(asctime)s.%(msecs)03d %(message)s", datefmt="%H:%M:%S")
    fh = logging.FileHandler(path)
    fh.setFormatter(fmt)
    sh = logging.StreamHandler()
    sh.setFormatter(fmt)
    log.setLevel(logging.INFO)
    log.addHandler(fh)
    log.addHandler(sh)
    log.info(f"=== round2 run log: {path} ===")
    return path


def open_serial(headless: bool, retries: int = 30, wait_s: float = 1.0):
    """2026-08-11 (fix #1): '/dev/ttyUSB0' was hardcoded, and a failed open only printed a
    warning and carried on — the race-day failure mode was a USB re-enumeration leaving
    the Pi detecting pillars and commanding nobody. Now: stable /dev/serial/by-id/ paths
    are preferred, the open retries for ~`retries` seconds, and headless (race) mode
    HARD-FAILS rather than run without the ESP32. GUI mode keeps warn-and-continue —
    bench calibration legitimately runs with no robot attached."""
    for attempt in range(1, retries + 1):
        candidates = sorted(glob.glob('/dev/serial/by-id/*')) + ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']
        for dev in candidates:
            try:
                ser = serial.Serial(dev, 115200, timeout=1)
                log.info(f"Serial open on {dev} (attempt {attempt})")
                return ser
            except Exception:
                continue
        log.warning(f"Serial: no port found (attempt {attempt}/{retries})")
        time.sleep(wait_s)
    if headless:
        log.error(f"FATAL: no serial port after {retries} attempts — refusing to run headless without the ESP32.")
        sys.exit(2)
    log.warning("Continuing WITHOUT serial — GUI/bench mode only, no commands will reach the robot.")
    return None


def make_sender(ser):
    """Wraps ser.write so every command is mirrored into the run log with a [TX] stamp —
    causality between what the Pi saw and what it commanded is reconstructable per line."""
    def send(msg: bytes):
        if ser is None:
            return
        try:
            ser.write(msg)
            log.info("[TX] " + msg.decode(errors="replace").strip())
        except Exception as e:
            log.error(f"[TX-FAIL] {msg!r}: {e}")
    return send


def start_esp_log_thread(ser):
    """Drains the ESP32's telemetry (same USB serial, previously discarded unread) into
    the run log as [ESP] lines — [turn] n/12, MODE telemetry, [stop] markers."""
    stop = threading.Event()

    def rx():
        while not stop.is_set():
            try:
                line = ser.readline()
                if line:
                    log.info("[ESP] " + line.decode(errors="replace").strip())
            except Exception:
                time.sleep(0.2)

    t = threading.Thread(target=rx, daemon=True)
    t.start()
    return stop


def main(camera_id: int = 0, frame_size: int = 240, headless: bool = False,
         calib_path: str = "calib.json", logdir: str = "logs"):
    """Main function with improved camera handling.

    headless=True is the race-day mode: no cv2 UI at all (the field has no monitor and
    rule 9.9 forbids calibrating at round start) — calibration comes from calib_path,
    written earlier by a GUI session during check time."""

    setup_logging(logdir)

    # --- Lock exposure/white balance before opening the stream ---
    set_manual_camera_controls(camera_id, exposure_value=500, wb_temperature=4500)

    # --- Serial connection to ESP32 ---
    ser = open_serial(headless)
    send = make_sender(ser)
    esp_log_stop = start_esp_log_thread(ser) if ser is not None else None

    # --- Open camera ---
    container, stream = open_camera(camera_id)
    if container is None:
        print("Cannot open webcam.")
        return

    # --- Start capture thread ---
    t, frame_q, stop_flag = start_capture_thread(container, stream, frame_size)

    def get_frame(timeout=1.0):
        try:
            return frame_q.get(timeout=timeout)
        except queue.Empty:
            return None

    # --- Test frame capture ---
    print("Testing camera...")
    test_frames = 0
    for _ in range(10):
        frame = get_frame(timeout=2.0)
        if frame is not None:
            test_frames += 1
            print(f"Got test frame {test_frames}, shape: {frame.shape}")
        time.sleep(0.1)

    if test_frames == 0:
        print("No frames received from camera!")
        stop_flag.set()
        t.join(timeout=2.0)
        container.close()
        return

    # --- Main loop ---
    window_name = "WRO Block Detector"

    roi_w, roi_h = frame_size // 5, frame_size // 5
    roi = ((frame_size - roi_w) // 2, (frame_size - roi_h) // 2, roi_w, roi_h)

    saved = load_calib(calib_path)

    if headless:
        if 'red' not in saved or 'green' not in saved:
            log.error(f"headless mode needs a calibration file with both colors at {calib_path}.")
            log.error("Run once WITHOUT --headless during check time to calibrate and save it.")
            stop_flag.set()
            t.join(timeout=2.0)
            container.close()
            if ser is not None:
                ser.close()
            return
        calib = saved
        print("=== LIVE DETECTION (headless) ===\n")
    else:
        cv2.namedWindow(window_name)
        calib = run_calibration_session(get_frame, roi, window_name, initial=saved)
        if 'red' in calib and 'green' in calib:
            save_calib(calib, calib_path)
        print("=== LIVE DETECTION ===")
        print("Press C to recalibrate, Q to quit.\n")

    log.info("[calib] " + json.dumps(calib))   # calibration provenance lands in every run log

    history_len = 7
    required = 5
    red_hist = deque(maxlen=history_len)
    green_hist = deque(maxlen=history_len)
    mag_hist = deque(maxlen=history_len)

    # --- Single Kalman filter for whichever color is currently tracked ---
    kf = create_kalman_filter()
    kf_initialized = False
    kf_color = None  # which color the filter is currently locked onto

    last_sent = None
    frame_count = 0
    clear_counter = 0      
    CLEAR_HISTORY = 10
    last_reverse_send = 0.0    # rate-limit REVERSE to 10 Hz instead of every frame
    last_mag_send = 0.0        # rate-limit MAG telemetry to 5 Hz (data only, drives nothing)
    last_state_keepalive = 0.0 # re-send RED/GREEN every 0.5 s (feeds the ESP32 1.5 s dead-man)

    perf_times = deque(maxlen=101)  # 2026-08-11 (fix #8): measure before optimizing —
    proc_ms = deque(maxlen=100)     # effective fps + frame-processing cost, logged every 100 frames

    try:
        while True:
            frame = get_frame(timeout=0.5)
            if frame is None:
                continue

            _t0 = time.monotonic()
            red_box, green_box, mag_box = process_frame(frame, calib)
            proc_ms.append((time.monotonic() - _t0) * 1000.0)
            red_hist.append(red_box)
            green_hist.append(green_box)
            mag_hist.append(mag_box)

            red_confirmed = sum(b is not None for b in red_hist) >= required
            green_confirmed = sum(b is not None for b in green_hist) >= required
            mag_confirmed = sum(b is not None for b in mag_hist) >= required


            current_detection = None
            active_box = None

            # Determine which block to act on, if any
            if red_confirmed and green_confirmed:
                if red_box is not None and green_box is not None:
                # Both visible – pick the closer one (taller box)
                    if red_box['height'] >= green_box['height']:
                        primary_box = red_box
                        primary_color = 'red'
                    else:
                        primary_box = green_box
                        primary_color = 'green'
                elif red_box is not None:
                    primary_box = red_box
                    primary_color = 'red'
                elif green_box is not None:
                    primary_box = green_box
                    primary_color = 'green'
                else:
                    primary_box = None
                    primary_color = None
            elif red_confirmed:
                primary_box = red_box
                primary_color = 'red'
            elif green_confirmed:
                primary_box = green_box
                primary_color = 'green'
            else:
                primary_box = None
                primary_color = None

            too_close = False
            if primary_box is not None:
                block_height = primary_box['height']
                
                # --- Too far away? ---
                if block_height < MIN_SWERVE_HEIGHT:
                    current_detection = None
                    active_box = None
                
                # --- Dangerously close? ---
                elif block_height > REVERSE_HEIGHT:
                    too_close = True
                    now_t = time.monotonic()
                    if ser is not None and (now_t - last_reverse_send) >= 0.1:
                        send(b'REVERSE\n')
                        last_reverse_send = now_t
                    current_detection = None   # skip normal command this frame
                else:
                    if primary_color == 'red':
                        if primary_box['center_x'] <= LEFT_SIDE_MAX:
                            # Already on the left – safe, no swerve needed
                            current_detection = None
                        else:
                            current_detection = 'red'
                            active_box = primary_box
                    else:  # green
                        if primary_box['center_x'] >= RIGHT_SIDE_MIN:
                            # Already on the right – safe
                            current_detection = None
                        else:
                            current_detection = 'green'
                            active_box = primary_box

            # --- Kalman smoothing for steering (single filter, reset on target change) ---
            if current_detection != kf_color:
                kf = create_kalman_filter()
                kf_initialized = False
                kf_color = current_detection

            smoothed = None
            if current_detection is not None:
                smoothed, kf_initialized = kalman_update(kf, active_box, kf_initialized)
            if current_detection is None:
                if not too_close:
                    clear_counter += 1
                # too close = the opposite of clear: freeze the counter so a debounced
                # CLEAR can never fire while we are still on top of the pillar
            else:
                clear_counter = 0

            # Send command only on change
            if current_detection != last_sent and ser is not None:
                if current_detection is None and clear_counter < CLEAR_HISTORY:
                    pass
                else:
                    if current_detection == 'red':
                        send(b'RED\n')
                    elif current_detection == 'green':
                        send(b'GREEN\n')
                    else:
                        send(b'CLEAR\n')
                    last_sent = current_detection
                    last_state_keepalive = time.monotonic()

            # State keepalive: re-send the active color every 0.5 s so the ESP32's 1.5 s
            # dead-man never fires during a long, otherwise-silent pass.
            if current_detection is not None and current_detection == last_sent and ser is not None:
                now_t = time.monotonic()
                if now_t - last_state_keepalive >= 0.5:
                    send(b'RED\n' if current_detection == 'red' else b'GREEN\n')
                    last_state_keepalive = now_t

            # POS stream: while tracking, send the (Kalman-smoothed) block position every
            # frame — this drives the ESP32's gradient steering and doubles as a keepalive.
            if current_detection is not None and active_box is not None and ser is not None:
                cx = smoothed['center_x'] if smoothed is not None else active_box['center_x']
                send(f"POS,{int(cx)},{int(active_box['height'])}\n".encode())

            # 2026-08-11 (research B1 foundation): magenta bay telemetry — the ESP32
            # logs [mag] sightings and drives NOTHING from them. Wednesday's mat run
            # answers "can we even see the bay reliably" before any parking code exists.
            if mag_confirmed and mag_box is not None and ser is not None:
                now_t = time.monotonic()
                if now_t - last_mag_send >= 0.2:
                    send(f"MAG,{int(mag_box['center_x'])},{int(mag_box['height'])}\n".encode())
                    last_mag_send = now_t

            # Display (GUI mode only — headless does zero cv2 UI work)
            display_red = red_box if red_confirmed else None
            display_green = green_box if green_confirmed else None
            display_mag = mag_box if mag_confirmed else None
            if not headless:
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                display = draw_boxes(bgr, display_red, display_green, mag_box=display_mag)

                cv2.line(display, (LEFT_SIDE_MAX, 0), (LEFT_SIDE_MAX, frame_size-1), (0, 165, 255), 1)
                cv2.putText(display, "RED SAFE <", (2, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 165, 255), 1)

                cv2.line(display, (RIGHT_SIDE_MIN, 0), (RIGHT_SIDE_MIN, frame_size-1), (0,255,0), 1)
                cv2.putText(display, "GREEN SAFE >", (RIGHT_SIDE_MIN+2, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

                cv2.putText(display, "DANGER", (LEFT_SIDE_MAX+5, frame_size-10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

                cv2.imshow(window_name, upscale_for_display(display, scale=3))
            frame_count += 1
            perf_times.append(time.monotonic())
            if frame_count % 100 == 0 and len(perf_times) >= 2:
                _span = perf_times[-1] - perf_times[0]
                if _span > 0:
                    log.info(f"[perf] eff_fps={(len(perf_times) - 1) / _span:.1f} "
                             f"proc_ms_avg={sum(proc_ms) / max(len(proc_ms), 1):.1f}")
            red_str = (f"RED[x={display_red['x']}px, y={display_red['y']}px, "
                        f"w={display_red['width']}px, h={display_red['height']}px, "
                        f"center=({display_red['center_x']}px, {display_red['center_y']}px)]"
                        if display_red else "RED:None")

            green_str = (f"GREEN[x={display_green['x']}px, y={display_green['y']}px, "
                        f"w={display_green['width']}px, h={display_green['height']}px, "
                        f"center=({display_green['center_x']}px, {display_green['center_y']}px)]"
                        if display_green else "GREEN:None")

            smoothed_str = (f"SMOOTHED[{kf_color},x={smoothed['center_x']},y={smoothed['center_y']},"
                             f"vx={smoothed['vx']:.1f},vy={smoothed['vy']:.1f}]"
                             if smoothed is not None else "SMOOTHED:None")

            log.info(f"Frame {frame_count} | {red_str} | {green_str} | {smoothed_str}")

            if not headless:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    calib = run_calibration_session(get_frame, roi, window_name, initial=calib)
                    if 'red' in calib and 'green' in calib:
                        save_calib(calib, calib_path)
                    red_hist.clear()
                    green_hist.clear()
                    last_sent = None
                    kf = create_kalman_filter()
                    kf_initialized = False
                    kf_color = None
                    print("=== LIVE DETECTION ===")

    except KeyboardInterrupt:
        pass
    finally:
        if esp_log_stop is not None:
            esp_log_stop.set()
        stop_flag.set()
        t.join(timeout=2.0)
        container.close()
        if ser is not None:
            ser.close()
        if not headless:
            cv2.destroyAllWindows()
        log.info("Final calibration used:")
        for color, c in calib.items():
            log.info(f"  {color}: {c}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WRO Round 2 block detector")
    parser.add_argument("--headless", action="store_true",
                        help="race-day mode: no cv2 UI, calibration loaded from --calib")
    parser.add_argument("--calib", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json"),
                        help="path to the calibration JSON (default: calib.json next to this script)")
    parser.add_argument("--logdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"),
                        help="directory for run logs (always on; default: logs/ next to this script)")
    args = parser.parse_args()
    main(camera_id=0, frame_size=240, headless=args.headless, calib_path=args.calib, logdir=args.logdir)