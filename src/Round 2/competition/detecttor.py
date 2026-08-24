#!/usr/bin/env python3
"""
WRO block detector — ONNX + image-grid drive.

Red cube → 400ms pause at 45px, then wide PASS right until off-camera, then
SIDE (straight) until the left LiDAR, then yaw back. Green is the mirror.
Green is the mirror (PASS left, right LiDAR). Nearest cube first if two
are in view. After each reverse-arc: 1 or 2 cubes only; a 3rd waits.

Handshake: after the ESP button (pin 32) the Pi waits for ESP_READY, then
sends PI_HELLO until ESP_ACK. Commands carry an XOR checksum so USB garbage
is NAK'd instead of executed.

Max 8 cubes on the mat; a second cube seen during a pass is held as pending
until the current cube is passed, then the grid takes it.
"""

import math
import time
import threading
import queue
import subprocess
import os
import numpy as np
from collections import deque

# OpenCV's Qt build on Pi often lacks the Wayland plugin. Prefer X11/XWayland.
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
import cv2
import onnxruntime as ort

ONNX_MODEL_PATH = "best_ncnn.onnx"
MODEL_INPUT_SIZE = 224
CLASS_NAMES = {0: "green", 1: "red"}
CONF_THRESHOLD = 0.45
USE_CUDA_IF_AVAILABLE = False

CAPTURE_W = 640
CAPTURE_H = 480
FRAME_SIZE = 240

CAMERA_ID = 0
CAMERA_INDEXES = (0, 2)
CAMERA_EXPOSURE = 300
CAMERA_WB_TEMP = 4500
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15
SERIAL_PORTS = ("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyAMA0")
SERIAL_BAUD = 115200
SERIAL_RETRY_S = 1.0
# Boot service usually has no desktop. Set WRO_HEADLESS=0 to show the preview.
HEADLESS = os.environ.get("WRO_HEADLESS", "1" if not os.environ.get("DISPLAY") else "0") != "0"

VOTE_HISTORY = 7
MIN_VOTES = 5
CLEAR_HISTORY = 3
LOCK_MATCH_PX = 70
MAX_BLOCKS = 8
MAX_SEGMENT_CUBES = 2          # after each reverse-arc: 1 or 2 cubes, never a 3rd

REVERSE_HEIGHT_PX = 80
DRIVE_RESEND_S = 0.12
AVOID_HEIGHT_PX = 22
SWERVE_HEIGHT_PX = 45          # start the pass only when the cube is this tall
# Keep full DIFF until the cube is off-camera — do not HOLD on the edge.
EDGE_LOCK_FRAC = 0.14          # red target x ≈ 34px; green ≈ 206px
EDGE_OK_PX = 14
EDGE_KP = 0.40                 # extra deg per pixel the cube is still in-lane
EDGE_HOLD_OFFSET = 0
EDGE_MIN_OFFSET = 0
PASSED_EDGE = 0.28
GONE_EDGE = 0.06               # cube this far off the pass edge = out of view → SIDE
HOLD_OFFSET = EDGE_HOLD_OFFSET
HOLD_MIN_PX = 32
ALIGN_PAUSE_S = 0.40           # stop 400 ms at 45px, then swerve
CANNOT_PASS_S = 2.0            # still not around the cube → reverse a bit and retry
REVERSE_BURST_S = 0.55         # one short reverse, then try PASS again (do not latch)
MAX_REVERSE_BURSTS = 3
RECOVER_DELAY_S = 0.12         # brief confirm it is gone, then SIDE (not rejoin)
MAX_STEER_DEG = 35
GREEN_MIN_OFFSET = 35          # green pass-left needs full DIFF or the body clips
RED_MIN_OFFSET = 35            # red pass-right, same full swerve
PASSED_BEHIND_Y = 0.68         # unused for rejoin; SIDE waits for side LiDAR

# Closeness bands from bbox height (pixels in 240 frame). Tune on the mat.
BAND_FAR_PX = 22
BAND_MID_PX = 32
BAND_NEAR_PX = 44
BAND_CLOSE_PX = 58

# 7 columns on center_x. Red: pass right (cube should slide left). Green is mirrored.
COL_EDGES = (30, 64, 98, 142, 176, 210)

# PWM 0–255. Lane on the ESP is STRAIGHT_SPEED 220.
# Swerve is slow so the cube can be placed on the edge; HOLD is faster along that line.
SPEED_FAR = 180
SPEED_MID = 150
SPEED_NEAR = 120
SPEED_CLOSE = 100
SPEED_SWERVE = 120             # after the 400 ms pause
SPEED_HOLD = 180
SPEED_PAUSE = 0                # motors off during the 45px pause
SPEED_SIDE = 160               # straight after the cube leaves the camera
SPEED_RECOVER = 150
SPEED_STUCK = 110

# Offset-only LUT (deg from center). Speed comes from SPEED_* by band.
# col 0 = already on the pass side … col 6 = cube on the wrong side.
# Old on-robot LUT peaked at 26/36/40/42 and never used full DIFF=35 until
# the cube was already close — that is why the body brushed. Cap at 35.
OFFSET_LUT = (
    #  0   1   2   3   4   5   6
    ( 18, 24, 28, 32, 35, 35, 35),  # FAR  — swing out early
    ( 22, 26, 30, 35, 35, 35, 35),  # MID
    ( 26, 30, 35, 35, 35, 35, 35),  # NEAR — keep body off the cube
    ( 28, 32, 35, 35, 35, 35, 35),  # CLOSE
)

RECOVER_OFFSET = 0
STUCK_PASS_S = 2.5
STUCK_REVERSE_S = 4.2
STUCK_OFFSET = 35


def parse_tel_turns(body: str):
    """turns=3/12 or turns=3 from ESP telemetry / ARC DONE."""
    for part in body.replace(",", " ").split():
        if part.startswith("turns="):
            num = part.split("=", 1)[1].split("/")[0]
            try:
                return int(num)
            except ValueError:
                return None
    return None


def xor_checksum(body: str) -> str:
    got = 0
    for b in body.encode("ascii", errors="ignore"):
        got ^= b
    return f"{got:02X}"


def format_cmd(body: str) -> str:
    body = body.strip()
    return f"{body}*{xor_checksum(body)}\n"


def box_already_passed(box: dict, color: str) -> bool:
    """True if the cube is on the pass side of the image (red left, green right)."""
    if not box:
        return True
    cx = float(box.get("center_x", FRAME_SIZE / 2.0))
    if color == "red" and cx < FRAME_SIZE * PASSED_EDGE:
        return True
    if color == "green" and cx > FRAME_SIZE * (1.0 - PASSED_EDGE):
        return True
    return False


def cube_gone(box: dict, color: str) -> bool:
    """True once the cube is off the pass edge / camera — then SIDE + side LiDAR."""
    if not box:
        return True
    h = int(box.get("height") or 0)
    if h < AVOID_HEIGHT_PX:
        return True
    cx = float(box.get("center_x", FRAME_SIZE / 2.0))
    if color == "red" and cx < FRAME_SIZE * GONE_EDGE:
        return True
    if color == "green" and cx > FRAME_SIZE * (1.0 - GONE_EDGE):
        return True
    return False


def better_pending(old, box, color, lock):
    """Keep the tallest extra cube that is not the locked one."""
    if box is None or color is None:
        return old
    if lock is not None and color == lock.get("color") and box_matches_lock(lock, box):
        return old
    if old is None or int(box.get("height") or 0) >= int(old["box"]["height"]):
        return {"box": box, "color": color}
    return old


def box_matches_lock(lock: dict, box: dict) -> bool:
    if not lock or not box:
        return False
    ob = lock.get("box") or {}
    dx = float(box.get("center_x", 0)) - float(ob.get("center_x", 0))
    dy = float(box.get("center_y", 0)) - float(ob.get("center_y", 0))
    return math.hypot(dx, dy) <= LOCK_MATCH_PX


def height_band(height: int) -> int:
    if height >= BAND_CLOSE_PX:
        return 3
    if height >= BAND_NEAR_PX:
        return 2
    if height >= BAND_MID_PX:
        return 1
    return 0


def column_index(center_x: float, color: str) -> int:
    """Column in the LUT. Green is mirrored so the last col is always 'wrong side'."""
    x = float(center_x)
    if color == "green":
        x = FRAME_SIZE - 1 - x
    for i, edge in enumerate(COL_EDGES):
        if x < edge:
            return i
    return len(COL_EDGES)


def edge_target_x(color: str) -> float:
    """Where the cube center should sit: left edge for red, right edge for green."""
    if color == "green":
        return FRAME_SIZE * (1.0 - EDGE_LOCK_FRAC)
    return FRAME_SIZE * EDGE_LOCK_FRAC


def edge_error_px(box: dict, color: str) -> float:
    """Positive = cube still in the lane (need more pass steer). 0 = parked on edge."""
    cx = float(box.get("center_x", FRAME_SIZE / 2.0))
    target = edge_target_x(color)
    if color == "green":
        return target - cx
    return cx - target


def cube_is_behind(box: dict, color: str) -> bool:
    """True after we have gone past the cube — time to curve back to center."""
    if cube_gone(box, color):
        return True
    if not box:
        return True
    if not box_already_passed(box, color):
        return False
    cy = float(box.get("center_y", 0))
    return cy >= FRAME_SIZE * PASSED_BEHIND_Y


def pass_offset_for(color: str, band: int, col: int, err: float) -> int:
    offset = int(round(max(OFFSET_LUT[band][col], EDGE_KP * err)))
    offset = max(OFFSET_LUT[band][col], min(MAX_STEER_DEG, offset))
    if color == "green":
        offset = max(offset, GREEN_MIN_OFFSET)
    else:
        offset = max(offset, RED_MIN_OFFSET)
    return min(MAX_STEER_DEG, offset)


def pass_steer_offset(box: dict, color: str) -> int:
    h = max(int(box.get("height") or 0), 1)
    cx = float(box.get("center_x", FRAME_SIZE / 2.0))
    return pass_offset_for(color, height_band(h), column_index(cx, color), edge_error_px(box, color))


def box_height(box) -> int:
    if not box:
        return 0
    return int(box.get("height") or 0)


def nearer_cube(*pairs):
    """Pick the closest cube: taller bbox in the 240px frame is nearer."""
    best_box, best_color = None, None
    best_h = -1
    for box, color in pairs:
        if box is None or not color:
            continue
        h = box_height(box)
        if h > best_h:
            best_h = h
            best_box, best_color = box, color
    return best_box, best_color


def should_retarget_lock(lock: dict, live_box: dict, other_box: dict) -> bool:
    """Always prefer the nearer cube when two are in view."""
    if other_box is None:
        return False
    other_h = box_height(other_box)
    live_h = box_height(live_box or (lock or {}).get("box"))
    return other_h > live_h


def make_pass_lock(color: str, box: dict, old=None) -> dict:
    t0 = old["t0"] if old and "t0" in old else time.time()
    gone_t0 = old.get("gone_t0") if old else None
    recovered = bool(old.get("recovered")) if old else False
    pause_t0 = old.get("pause_t0") if old else None
    pause_done = bool(old.get("pause_done")) if old else False
    swerve_t0 = old.get("swerve_t0") if old else None
    pause_steer = old.get("pause_steer") if old else None
    reverse_until = old.get("reverse_until") if old else None
    reverse_count = int(old.get("reverse_count") or 0) if old else 0
    counted = bool(old.get("counted")) if old else False
    waiting_side = bool(old.get("waiting_side")) if old else False
    if pause_steer is None and box is not None:
        pause_steer = MAX_STEER_DEG
    return {
        "color": color,
        "box": dict(box),
        "t0": t0,
        "gone_t0": gone_t0,
        "recovered": recovered,
        "pause_t0": pause_t0,
        "pause_done": pause_done,
        "swerve_t0": swerve_t0,
        "pause_steer": pause_steer,
        "reverse_until": reverse_until,
        "reverse_count": reverse_count,
        "counted": counted,
        "waiting_side": waiting_side,
    }




def hold_command(box: dict, color: str, band: int, col: int) -> dict:
    return {
        "speed": SPEED_HOLD,
        "offset": HOLD_OFFSET,
        "phase": "HOLD",
        "band": band,
        "col": col,
    }


def side_command(box: dict, color: str) -> dict:
    h = max(int((box or {}).get("height") or 0), 1)
    cx = float((box or {}).get("center_x", FRAME_SIZE / 2.0))
    return {
        "speed": SPEED_SIDE,
        "offset": 0,
        "phase": "SIDE",
        "band": height_band(h),
        "col": column_index(cx, color),
    }


def grid_command(box: dict, color: str, recover_ok: bool = False,
                  pause_ok: bool = True, freeze_offset: int = None) -> dict:
    """PAUSE 400ms at 45px, then wide PASS until off-camera, then SIDE."""
    h = max(int(box.get("height") or 0), 1)
    cx = float(box.get("center_x", FRAME_SIZE / 2.0))
    band = height_band(h)
    col = column_index(cx, color)
    if recover_ok or cube_gone(box, color):
        return side_command(box, color)
    if not pause_ok:
        offset = freeze_offset if freeze_offset is not None else MAX_STEER_DEG
        return {
            "speed": SPEED_PAUSE,
            "offset": int(offset),
            "phase": "PAUSE",
            "band": band,
            "col": col,
        }
    return {
        "speed": SPEED_SWERVE,
        "offset": MAX_STEER_DEG,
        "phase": "PASS",
        "band": band,
        "col": col,
    }


def load_onnx_session(model_path: str) -> tuple:
    providers = ["CPUExecutionProvider"]
    if USE_CUDA_IF_AVAILABLE and "CUDAExecutionProvider" in ort.get_available_providers():
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    session = ort.InferenceSession(model_path, providers=providers)
    input_name = session.get_inputs()[0].name
    output_names = [o.name for o in session.get_outputs()]
    print(f"Loaded ONNX model '{model_path}' | providers={session.get_providers()} "
          f"| input={input_name} | outputs={output_names}")
    return session, input_name, output_names


def preprocess(frame: np.ndarray, size: int) -> tuple:
    h, w = frame.shape[:2]
    scale = size / max(h, w)
    nh, nw = int(h * scale), int(w * scale)
    resized = cv2.resize(frame, (nw, nh))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    top = (size - nh) // 2
    left = (size - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized
    tensor = canvas.astype(np.float32) / 255.0
    tensor = tensor.transpose(2, 0, 1)
    tensor = np.expand_dims(tensor, axis=0)
    return np.ascontiguousarray(tensor), scale, left, top


def _pred_to_box(conf, x1, y1, x2, y2, scale: float, left: int, top: int) -> dict:
    ox1 = (x1 - left) / scale
    oy1 = (y1 - top) / scale
    ox2 = (x2 - left) / scale
    oy2 = (y2 - top) / scale
    ow = ox2 - ox1
    oh = oy2 - oy1
    return {
        "x": int(round(ox1)), "y": int(round(oy1)),
        "width": int(round(ow)), "height": int(round(oh)),
        "center_x": int(round(ox1 + ow / 2)), "center_y": int(round(oy1 + oh / 2)),
        "confidence": float(conf),
    }


def decode_onnx_output(raw_output: np.ndarray, scale: float, left: int, top: int,
                        conf_thresh: float) -> dict:
    preds = raw_output[0]
    ranked = {0: [], 1: []}
    for pred in preds:
        x1, y1, x2, y2, conf, cls_id = pred
        if conf < conf_thresh:
            continue
        cls_id = int(cls_id)
        if cls_id not in ranked:
            continue
        ranked[cls_id].append((float(conf), x1, y1, x2, y2))
    results = {}
    for cls_id, items in ranked.items():
        color = CLASS_NAMES.get(cls_id)
        if color is None or not items:
            continue
        items.sort(key=lambda t: t[0], reverse=True)
        boxes = []
        for conf, x1, y1, x2, y2 in items[:2]:
            box = _pred_to_box(conf, x1, y1, x2, y2, scale, left, top)
            boxes.append(box)
        if boxes:
            results[color] = boxes
    return results


def detect_blocks_onnx(frame: np.ndarray, session, input_name: str) -> tuple:
    tensor, scale, left, top = preprocess(frame, MODEL_INPUT_SIZE)
    outputs = session.run(None, {input_name: tensor})
    decoded = decode_onnx_output(outputs[0], scale, left, top, CONF_THRESHOLD)
    reds = decoded.get("red") or []
    greens = decoded.get("green") or []
    reds.sort(key=lambda b: int(b.get("height") or 0), reverse=True)
    greens.sort(key=lambda b: int(b.get("height") or 0), reverse=True)
    red = reds[0] if reds else None
    green = greens[0] if greens else None
    red_alt = reds[1] if len(reds) > 1 else None
    green_alt = greens[1] if len(greens) > 1 else None
    return red, green, red_alt, green_alt


def upscale_for_display(frame_bgr: np.ndarray, scale: int = 3) -> np.ndarray:
    h, w = frame_bgr.shape[:2]
    return cv2.resize(frame_bgr, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)


def draw_grid(frame_bgr: np.ndarray) -> np.ndarray:
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    for x in COL_EDGES:
        cv2.line(out, (x, 0), (x, h), (80, 80, 80), 1)
    left = int(round(edge_target_x("red")))
    right = int(round(edge_target_x("green")))
    cv2.line(out, (left, 0), (left, h), (0, 80, 255), 1)
    cv2.line(out, (right, 0), (right, h), (0, 200, 0), 1)
    return out


def draw_boxes(frame_bgr: np.ndarray, red_box: dict, green_box: dict) -> np.ndarray:
    out = draw_grid(frame_bgr)
    for box, bgr_color, label in ((red_box, (0, 0, 255), "RED"), (green_box, (0, 255, 0), "GREEN")):
        if not box:
            continue
        x, y, w, h = box['x'], box['y'], box['width'], box['height']
        cx, cy = box['center_x'], box['center_y']
        cv2.rectangle(out, (x, y), (x + w, y + h), bgr_color, 2)
        cv2.circle(out, (cx, cy), 3, bgr_color, -1)
        conf = box.get('confidence')
        conf_str = f" conf={conf:.2f}" if conf is not None else ""
        cv2.putText(out, f"{label} {w}x{h}px{conf_str}", (x, max(0, y - 22)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, bgr_color, 1)
        cv2.putText(out, f"pos=({x},{y}) center=({cx},{cy})", (x, max(0, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, bgr_color, 1)
    return out


def resize_frame(frame: np.ndarray, target_w: int = 240, target_h: int = 240) -> np.ndarray:
    if frame is None or frame.size == 0:
        return None
    return cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)


def open_opencv_camera(index: int):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    print(f"Camera opened: index {index}, frame {frame.shape[1]}x{frame.shape[0]}")
    return cap


def set_manual_camera_controls(camera_id: int, exposure_value: int, wb_temperature: int):
    dev = f'/dev/video{camera_id}'
    cmds = [
        ['v4l2-ctl', '-d', dev, '-c', 'auto_exposure=1'],
        ['v4l2-ctl', '-d', dev, '-c', f'exposure_time_absolute={exposure_value}'],
        ['v4l2-ctl', '-d', dev, '-c', 'white_balance_automatic=0'],
        ['v4l2-ctl', '-d', dev, '-c', f'white_balance_temperature={wb_temperature}'],
    ]
    for cmd in cmds:
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Warning: could not run {' '.join(cmd)} ({e})")
    print(f"Camera controls locked: exposure={exposure_value}, wb_temp={wb_temperature}")


def start_capture_thread(camera_id: int, frame_size=240):
    frame_q = queue.Queue(maxsize=1)
    stop_flag = threading.Event()
    holder = {"cap": None}

    def enqueue(img):
        if img is None:
            return
        if frame_q.full():
            try:
                frame_q.get_nowait()
            except queue.Empty:
                pass
        frame_q.put(img)

    def capture_loop():
        indexes = []
        for i in (camera_id,) + CAMERA_INDEXES:
            if i not in indexes:
                indexes.append(i)
        while not stop_flag.is_set():
            cap = None
            for idx in indexes:
                if stop_flag.is_set():
                    break
                cap = open_opencv_camera(idx)
                if cap is not None:
                    holder["cap"] = cap
                    set_manual_camera_controls(idx, CAMERA_EXPOSURE, CAMERA_WB_TEMP)
                    break
            if cap is None:
                print("No camera frames. USB 2 port, then: pkill -f detect.py && python detect.py")
                time.sleep(2.0)
                continue
            try:
                while not stop_flag.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        print("Camera read failed; reopening...")
                        break
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    enqueue(resize_frame(rgb, frame_size, frame_size))
            finally:
                try:
                    cap.release()
                except Exception:
                    pass
                holder["cap"] = None
            if not stop_flag.is_set():
                time.sleep(0.4)

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    return t, frame_q, stop_flag, holder


def open_serial():
    try:
        import serial
    except Exception as e:
        print(f"Could not import serial: {e}")
        return None
    last_err = None
    for port in SERIAL_PORTS:
        try:
            ser = serial.Serial(port, SERIAL_BAUD, timeout=0.15)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print(f"Serial port opened: {port}")
            return ser
        except Exception as e:
            last_err = e
    print(f"Could not open serial port: {last_err}")
    return None


def start_serial_manager(link_ok: threading.Event, cubes_ok: threading.Event,
                          stop_flag: threading.Event, holder: dict,
                          arc_hold: dict = None):
    """Open USB when it appears, handshake on ESP_READY, enable cubes after first arc."""
    if arc_hold is None:
        arc_hold = {"turns": 0, "passes": 0, "yaw_done": False}
    def loop():
        last_hello = 0.0
        saw_ready = False
        print("Serial: retry until ESP USB is up, then wait for pin 32 ...", flush=True)
        while not stop_flag.is_set():
            ser = holder.get("ser")
            if ser is None:
                ser = open_serial()
                holder["ser"] = ser
                if ser is None:
                    time.sleep(SERIAL_RETRY_S)
                    continue
                saw_ready = False
                link_ok.clear()
                cubes_ok.clear()
                arc_hold["turns"] = 0
                arc_hold["passes"] = 0
                print("Serial: waiting for pin-32 press (ESP_READY) ...", flush=True)
            try:
                raw = ser.readline()
            except Exception as e:
                msg = str(e).lower()
                if "returned no data" in msg or "readiness to read" in msg:
                    continue
                print(f"Serial read failed ({e}); retrying", flush=True)
                try:
                    ser.close()
                except Exception:
                    pass
                holder["ser"] = None
                link_ok.clear()
                cubes_ok.clear()
                arc_hold["turns"] = 0
                arc_hold["passes"] = 0
                saw_ready = False
                time.sleep(SERIAL_RETRY_S)
                continue
            if not raw:
                now = time.time()
                if saw_ready and not link_ok.is_set() and (now - last_hello >= 0.4):
                    try:
                        ser.write(format_cmd("PI_HELLO").encode("ascii"))
                        ser.flush()
                    except Exception:
                        pass
                    last_hello = now
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            body = line.split("*", 1)[0].strip() if "*" in line else line
            if body.startswith("TEL"):
                if "CUBES ON" in body or " cubes=1" in body:
                    if not cubes_ok.is_set():
                        print("First reverse-arc done — cube pass enabled", flush=True)
                    cubes_ok.set()
                n = parse_tel_turns(body)
                if n is not None and n > int(arc_hold.get("turns") or 0):
                    arc_hold["turns"] = n
                    arc_hold["passes"] = 0
                    print(
                        f"Reverse-arc {n} done — this straight: 1–2 cubes "
                        f"(3rd waits until next arc)",
                        flush=True,
                    )
                    cubes_ok.set()
                if "YAW DONE" in body:
                    arc_hold["yaw_done"] = True
                continue
            if body.startswith("ESP_READY"):
                if not saw_ready:
                    print("Serial handshake: ESP_READY (button pressed)", flush=True)
                saw_ready = True
                try:
                    ser.write(format_cmd("PI_HELLO").encode("ascii"))
                    ser.flush()
                except Exception:
                    pass
                last_hello = time.time()
            elif body.startswith("ESP_ACK"):
                if not link_ok.is_set():
                    print("Serial handshake OK (ESP_ACK)", flush=True)
                link_ok.set()
            elif body.startswith("ESP_NAK"):
                if saw_ready:
                    print("<<< ESP_NAK — resending PI_HELLO", flush=True)
                    try:
                        ser.write(format_cmd("PI_HELLO").encode("ascii"))
                        ser.flush()
                    except Exception:
                        pass
                    last_hello = time.time()
    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t


def main(camera_id: int = CAMERA_ID, frame_size: int = FRAME_SIZE):
    set_manual_camera_controls(camera_id, CAMERA_EXPOSURE, CAMERA_WB_TEMP)
    ser_hold = {"ser": None}
    link_ok = threading.Event()
    cubes_ok = threading.Event()
    rx_stop = threading.Event()
    arc_hold = {"turns": 0, "passes": 0, "yaw_done": False}
    start_serial_manager(link_ok, cubes_ok, rx_stop, ser_hold, arc_hold)

    session, input_name, _ = load_onnx_session(ONNX_MODEL_PATH)
    t, frame_q, stop_flag, cam = start_capture_thread(camera_id, frame_size)

    def get_frame(timeout=1.0):
        try:
            return frame_q.get(timeout=timeout)
        except queue.Empty:
            return None

    print("Testing camera...")
    test_frames = 0
    for _ in range(15):
        frame = get_frame(timeout=1.0)
        if frame is not None:
            test_frames += 1
            print(f"Got test frame {test_frames}, shape: {frame.shape}")
            break
    if test_frames == 0:
        print("No frames received from camera!")
        stop_flag.set()
        rx_stop.set()
        t.join(timeout=2.0)
        if cam.get("cap") is not None:
            cam["cap"].release()
        return

    window_name = "WRO Block Detector (ONNX grid)"
    show_preview = not HEADLESS
    if show_preview:
        try:
            cv2.namedWindow(window_name)
        except Exception as e:
            print(f"No display ({e}) — running headless", flush=True)
            show_preview = False
    red_hist = deque(maxlen=VOTE_HISTORY)
    green_hist = deque(maxlen=VOTE_HISTORY)
    last_sent = None
    last_send_time = 0.0
    frame_count = 0
    clear_counter = 0
    pass_lock = None
    pending_block = None

    def latest_box(hist):
        for b in reversed(hist):
            if b is not None:
                return b
        return None

    def nearest_box(hist):
        boxes = [b for b in hist if b is not None]
        if not boxes:
            return None
        return max(boxes, key=lambda b: int(b.get("height") or 0))

    def hist_max_height(hist):
        hs = [b["height"] for b in hist if b is not None]
        return max(hs) if hs else 0

    def serial_write(body: str) -> None:
        ser = ser_hold.get("ser")
        if ser is None or not link_ok.is_set():
            return
        kind = body.split(",", 1)[0].upper()
        if kind in ("DRIVE", "REVERSE", "RED", "GREEN") and not cubes_ok.is_set():
            return
        if kind == "TRACKING" and body.startswith("TRACKING,1") and not cubes_ok.is_set():
            return
        line = format_cmd(body)
        try:
            ser.write(line.encode("ascii", errors="ignore"))
            ser.flush()
            print(f">>> Sent {line.strip()}", flush=True)
        except Exception as e:
            print(f"Serial write failed: {e}", flush=True)
            link_ok.clear()
            try:
                ser.close()
            except Exception:
                pass
            ser_hold["ser"] = None

    try:
        while True:
            frame = get_frame(timeout=0.5)
            if frame is None:
                continue

            red_box, green_box, red_alt, green_alt = detect_blocks_onnx(frame, session, input_name)
            red_hist.append(red_box)
            green_hist.append(green_box)

            def confirmed(hist):
                return sum(1 for b in hist if b is not None) >= MIN_VOTES

            red_confirmed = confirmed(red_hist)
            green_confirmed = confirmed(green_hist)
            red_stable = nearest_box(red_hist) if red_confirmed else None
            green_stable = nearest_box(green_hist) if green_confirmed else None

            now = time.time()
            primary_box, primary_color = nearer_cube(
                (red_stable, "red") if red_confirmed else (None, None),
                (green_stable, "green") if green_confirmed else (None, None),
                (red_alt, "red") if red_confirmed else (None, None),
                (green_alt, "green") if green_confirmed else (None, None),
            )

            decision = "CLEAR"
            active_box = None
            if primary_box is not None:
                color_hist = red_hist if primary_color == "red" else green_hist
                block_height = max(primary_box["height"], hist_max_height(color_hist))
                if block_height > REVERSE_HEIGHT_PX:
                    decision = "REVERSE"
                    active_box = primary_box
                elif block_height >= SWERVE_HEIGHT_PX:
                    decision = "DRIVE"
                    active_box = primary_box

            if arc_hold.get("yaw_done"):
                arc_hold["yaw_done"] = False
                if pass_lock is not None and pass_lock.get("waiting_side"):
                    pass_lock = None
                    if pending_block is not None and int(arc_hold.get("passes") or 0) < MAX_SEGMENT_CUBES:
                        primary_box = pending_block["box"]
                        primary_color = pending_block["color"]
                        pending_block = None
                        if primary_box["height"] > REVERSE_HEIGHT_PX:
                            decision = "REVERSE"
                            active_box = primary_box
                        elif primary_box["height"] >= SWERVE_HEIGHT_PX:
                            decision = "DRIVE"
                            active_box = primary_box
                    else:
                        pending_block = None

            if decision == "REVERSE":
                pass_lock = make_pass_lock(primary_color, active_box, pass_lock)
            elif pass_lock is not None:
                lock_color = pass_lock["color"]
                live_box = red_stable if lock_color == "red" else green_stable
                if live_box is not None and not box_matches_lock(pass_lock, live_box):
                    pending_block = better_pending(
                        pending_block, live_box, lock_color, pass_lock
                    )
                    live_box = None
                if pass_lock.get("waiting_side") and live_box is not None:
                    pending_block = better_pending(
                        pending_block, live_box, lock_color, pass_lock
                    )
                    live_box = None
                other_color = "green" if lock_color == "red" else "red"
                other_box, other_color = nearer_cube(
                    (green_stable, "green") if lock_color == "red" else (red_stable, "red"),
                    (green_alt, "green") if lock_color == "red" else (red_alt, "red"),
                )
                pending_block = better_pending(
                    pending_block, other_box, other_color, pass_lock
                )
                same_alt = red_alt if lock_color == "red" else green_alt
                nearer_rival, rival_color = nearer_cube(
                    (other_box, other_color),
                    (same_alt, lock_color),
                )
                if should_retarget_lock(pass_lock, live_box, nearer_rival) and not pass_lock.get(
                    "waiting_side"
                ):
                    pass_lock = make_pass_lock(rival_color, nearer_rival)
                    lock_color = rival_color
                    live_box = nearer_rival
                alt_box = red_alt if lock_color == "red" else green_alt
                pending_block = better_pending(
                    pending_block, alt_box, lock_color, pass_lock
                )

                if live_box is not None:
                    if cube_gone(live_box, lock_color):
                        if pass_lock.get("gone_t0") is None:
                            pass_lock["gone_t0"] = now
                        pass_lock["recovered"] = (
                            (now - float(pass_lock["gone_t0"])) >= RECOVER_DELAY_S
                        )
                        if pass_lock["recovered"]:
                            pass_lock["waiting_side"] = True
                    else:
                        pass_lock["gone_t0"] = None
                        pass_lock["recovered"] = False
                    already_passing = pass_lock.get("swerve_t0") is not None
                    if decision != "REVERSE" and (
                        int(live_box.get("height") or 0) >= SWERVE_HEIGHT_PX
                        or already_passing
                        or pass_lock.get("waiting_side")
                    ):
                        decision = "DRIVE"
                    pass_lock = make_pass_lock(lock_color, live_box, pass_lock)
                    active_box = live_box
                    primary_box = live_box
                    primary_color = lock_color
                else:
                    last_box = pass_lock.get("box")
                    already_passing = pass_lock.get("swerve_t0") is not None
                    if already_passing or pass_lock.get("waiting_side"):
                        if pass_lock.get("gone_t0") is None:
                            pass_lock["gone_t0"] = now
                        pass_lock["recovered"] = (
                            (now - float(pass_lock["gone_t0"])) >= RECOVER_DELAY_S
                        )
                        if pass_lock["recovered"]:
                            pass_lock["waiting_side"] = True
                        decision = "DRIVE"
                        active_box = last_box
                        primary_box = last_box
                        primary_color = lock_color
                    elif pending_block is not None:
                        if int(arc_hold.get("passes") or 0) >= MAX_SEGMENT_CUBES:
                            pending_block = None
                            pass_lock = None
                            decision = "CLEAR"
                            active_box = None
                        else:
                            pass_lock = None
                            primary_box = pending_block["box"]
                            primary_color = pending_block["color"]
                            pending_block = None
                            if primary_box["height"] > REVERSE_HEIGHT_PX:
                                decision = "REVERSE"
                                active_box = primary_box
                            elif primary_box["height"] >= SWERVE_HEIGHT_PX:
                                decision = "DRIVE"
                                active_box = primary_box
                                pass_lock = make_pass_lock(primary_color, primary_box)
                    else:
                        pass_lock = None

            if decision == "DRIVE" and pass_lock is None and active_box is not None:
                if int(arc_hold.get("passes") or 0) >= MAX_SEGMENT_CUBES:
                    decision = "CLEAR"
                    active_box = None
                    pending_block = None
                elif int(active_box.get("height") or 0) >= SWERVE_HEIGHT_PX:
                    pass_lock = make_pass_lock(primary_color, active_box)
                else:
                    pass_lock = None

            if decision in ("DRIVE", "REVERSE") and pass_lock is None:
                if int(arc_hold.get("passes") or 0) >= MAX_SEGMENT_CUBES:
                    decision = "CLEAR"
                    active_box = None
                    pending_block = None

            if (
                pass_lock is not None
                and pass_lock.get("waiting_side")
                and not arc_hold.get("yaw_done")
            ):
                decision = "DRIVE"
                if active_box is None:
                    active_box = pass_lock.get("box")
                    primary_box = active_box
                    primary_color = pass_lock["color"]

            clear_counter = clear_counter + 1 if decision == "CLEAR" else 0
            if decision == "CLEAR" and clear_counter >= CLEAR_HISTORY and pending_block is None:
                if not (pass_lock and pass_lock.get("waiting_side")):
                    pass_lock = None

            ser_now = ser_hold.get("ser")
            if ser_now is None and frame_count % 30 == 0:
                print("Serial is NOT open — ESP will never DRIVE. Check /dev/ttyUSB*", flush=True)
            if ser_now is not None and not link_ok.is_set() and frame_count % 30 == 0:
                print("Serial handshake not complete — not sending drive cmds", flush=True)
            if link_ok.is_set() and not cubes_ok.is_set() and frame_count % 30 == 0:
                print("Waiting for first reverse-arc corner before cube pass", flush=True)

            try:
                due = (now - last_send_time) >= DRIVE_RESEND_S
                if decision == "REVERSE" and active_box is not None and primary_color:
                    cmd = (
                        f"REVERSE,{active_box['center_x']},{active_box['center_y']},"
                        f"{active_box['width']},{active_box['height']}"
                    )
                    if cmd != last_sent or due:
                        serial_write(cmd)
                        serial_write(
                            f"TRACKING,1,{active_box['height']},{primary_color}"
                        )
                        last_sent = cmd
                        last_send_time = now
                elif decision == "DRIVE" and active_box is not None and primary_color:
                    if pass_lock is None:
                        pass_lock = make_pass_lock(primary_color, active_box)
                    recover_ok = bool(
                        pass_lock.get("recovered") or pass_lock.get("waiting_side")
                    )
                    if recover_ok:
                        pause_ok = True
                    else:
                        if int(active_box.get("height") or 0) >= SWERVE_HEIGHT_PX:
                            if pass_lock.get("pause_t0") is None:
                                pass_lock["pause_t0"] = now
                            if pass_lock.get("pause_steer") is None:
                                pass_lock["pause_steer"] = MAX_STEER_DEG
                        pause_ok = (
                            pass_lock.get("pause_t0") is not None
                            and (now - float(pass_lock["pause_t0"])) >= ALIGN_PAUSE_S
                        )
                        if pause_ok and not pass_lock.get("pause_done"):
                            pass_lock["pause_done"] = True
                            pass_lock["swerve_t0"] = now
                    cmd_info = grid_command(
                        active_box, primary_color,
                        recover_ok=recover_ok, pause_ok=pause_ok,
                        freeze_offset=pass_lock.get("pause_steer"),
                    )
                    if (
                        cmd_info["phase"] == "SIDE"
                        and pass_lock is not None
                        and not pass_lock.get("counted")
                    ):
                        pass_lock["counted"] = True
                        pass_lock["waiting_side"] = True
                        arc_hold["passes"] = int(arc_hold.get("passes") or 0) + 1
                        print(
                            f"Segment cube {arc_hold['passes']}/{MAX_SEGMENT_CUBES} "
                            f"off-camera — straight until side LiDAR, then yaw back",
                            flush=True,
                        )
                    swerve_s = 0.0
                    if pass_lock.get("swerve_t0"):
                        swerve_s = now - float(pass_lock["swerve_t0"])
                    live_now = (
                        (primary_color == "red" and red_stable is not None)
                        or (primary_color == "green" and green_stable is not None)
                    )
                    h_now = int(active_box.get("height") or 0)
                    cannot_go = (
                        cmd_info["phase"] == "PASS"
                        and live_now
                        and h_now >= SWERVE_HEIGHT_PX
                        and swerve_s >= CANNOT_PASS_S
                    )
                    if cannot_go:
                        until = pass_lock.get("reverse_until")
                        if until is None:
                            count = int(pass_lock.get("reverse_count") or 0)
                            if count >= MAX_REVERSE_BURSTS:
                                cannot_go = False
                                pass_lock["swerve_t0"] = now
                            else:
                                pass_lock["reverse_count"] = count + 1
                                pass_lock["reverse_until"] = now + REVERSE_BURST_S
                                until = pass_lock["reverse_until"]
                        if cannot_go and until is not None and now < float(until):
                            decision = "REVERSE"
                            cmd = (
                                f"REVERSE,{active_box['center_x']},{active_box['center_y']},"
                                f"{active_box['width']},{active_box['height']}"
                            )
                            if cmd != last_sent or due:
                                serial_write(cmd)
                                last_sent = cmd
                                last_send_time = now
                        else:
                            pass_lock["reverse_until"] = None
                            pass_lock["swerve_t0"] = now
                            cannot_go = False
                    if decision == "DRIVE":
                        cmd = (
                            f"DRIVE,{primary_color},{cmd_info['speed']},{cmd_info['offset']},"
                            f"{cmd_info['phase']},{active_box['height']}"
                        )
                        if cmd != last_sent or due:
                            serial_write(cmd)
                            last_sent = cmd
                            last_send_time = now
                elif decision == "CLEAR" and clear_counter >= CLEAR_HISTORY:
                    if pending_block is None and (last_sent != "CLEAR" or due):
                        serial_write("CLEAR")
                        serial_write("TRACKING,0,0,none")
                        last_sent = "CLEAR"
                        last_send_time = now
            except Exception as e:
                print(f"Serial write failed: {e}", flush=True)

            display_red = red_box if red_confirmed else None
            display_green = green_box if green_confirmed else None
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            display = draw_boxes(bgr, display_red, display_green)
            h_now = primary_box["height"] if primary_box else 0
            ser_now = ser_hold.get("ser")
            ser_txt = ser_now.port if ser_now is not None else "NO-SERIAL"
            link_txt = "LINK" if link_ok.is_set() else "NO-LINK"
            info = f"{decision} h={h_now} seg={arc_hold.get('passes', 0)}/{MAX_SEGMENT_CUBES} t={arc_hold.get('turns', 0)} {ser_txt} {link_txt}"
            if decision == "DRIVE" and active_box is not None and primary_color:
                gi = grid_command(
                    active_box, primary_color,
                    recover_ok=bool(pass_lock and pass_lock.get("recovered")),
                    pause_ok=bool(pass_lock and pass_lock.get("pause_done")),
                    freeze_offset=(pass_lock or {}).get("pause_steer"),
                )
                info += f" {gi['phase']} spd={gi['speed']} off={gi['offset']} c{gi['col']}b{gi['band']}"
            cv2.putText(
                display, info, (2, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                (0, 255, 255) if decision != "CLEAR" else (255, 255, 255), 1,
            )
            if pending_block is not None:
                cv2.putText(
                    display,
                    f"PENDING {pending_block['color']} h={pending_block['box']['height']}",
                    (2, frame_size - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 255), 1,
                )
            display = upscale_for_display(display, scale=3)
            if show_preview:
                try:
                    cv2.imshow(window_name, display)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                except Exception as e:
                    print(f"Preview disabled ({e})", flush=True)
                    show_preview = False
            frame_count += 1
            print(f"Frame {frame_count} | {decision} | RED:{red_box} | GREEN:{green_box}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag.set()
        rx_stop.set()
        t.join(timeout=2.0)
        if cam.get("cap") is not None:
            try:
                cam["cap"].release()
            except Exception:
                pass
        ser = ser_hold.get("ser")
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if show_preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
