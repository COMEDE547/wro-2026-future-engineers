"""Future Engineers Round 2 controller.

Active path: one unified pillar detector plus a non-blocking navigation state
machine. The older detector and lookup-table functions remain below only as
inactive reference code; the main entry point never calls them.
"""

# Importing Modules
import os
import cv2
import numpy as np
import serial
import time
import threading
import _thread
import math
from collections import deque
from dataclasses import dataclass, replace
#from gpiozero import LED
if os.name == "nt":
    class LED:
        def __init__(self, pin):
            self.pin = pin

        def on(self):
            pass

        def off(self):
            pass
else:
    from gpiozero import LED
from time import sleep

cv2.setNumThreads(2)

SPEED_K = 1.0
SPEED_K_PARKING = 1.0
PARKING_SPEED = 70
TURNING_SPEED = 100
SPEED = 210
PARKING_SPEED = int(PARKING_SPEED*SPEED_K_PARKING)
TURNING_SPEED = int(TURNING_SPEED*SPEED_K)
SPEED = int(SPEED*SPEED_K)
SPEED_NO_AURA_FARM = 90
PARKING_EXIT_FINAL_LAUNCH_SPEED = 70
PARKING_EXIT_FINAL_LAUNCH_SECONDS = 0.35

# Debugging
DEBUG = False
SHOW_LIVE_UI = True

# Round 2 feature switches. Parking exit/in run as isolated non-blocking states
# inside the unified controller.
ENABLE_COLOR_DETECTION = True
ENABLE_PARKING_EXIT = True
ENABLE_PARKING_IN = False

# Serial Values
PORT = "COM3" if os.name == "nt" else "/dev/ttyUSB0"
BAUDRATE = 115200
SERIAL_TIMEOUT = 0.05
TELEMETRY_HOLD_SECONDS = 0.25
TELEMETRY_STALE_SECONDS = 0.30

# ESP32 telemetry validity bits (protocol v2).
VALID_LEFT = 0x01
VALID_CENTER = 0x02
VALID_RIGHT = 0x04
VALID_HEADING = 0x08
INVALID_DISTANCE = 100000

LIDAR_PORT = None

# Movement
DIRECTION = None
KP = 1.0
COUNTER = 0
COUNTER_MAX = 12
target_angle = 0
BLOCK_MULITPLIER_GREEN_ANTI = 1.0

# Navigation-only state-machine tuning.  The corner threshold deliberately
# stays above the ESP32's 18 cm local hard-stop threshold.
COMMAND_REFRESH_SECONDS = 0.05
NAVIGATION_LOOP_SECONDS = 0.01
DIRECTION_SAMPLE_COUNT = 9
DIRECTION_MIN_DIFFERENCE_CM = 15
DIRECTION_LOCK_TIMEOUT_SECONDS = 5.0
CORNER_TRIGGER_CM = 38
CORNER_TRIGGER_RELEASE_CM = 45
CORNER_SIDE_OPEN_CM = 40
CORNER_CONFIRMATION_SAMPLES = 3
CORNER_APPROACH_HEADING_TOLERANCE = 20
# Round 2: the labelled CW corner-wall recording has right opening 250-254 cm.
# Use a long opening, not L+R: the opposite sensor can be inside its blind zone.
# 150 cm is an initial test threshold, mirrored for CCW (not field-validated).
OBSTACLE_CORNER_SIDE_OPEN_CM = 150
OBSTACLE_CORNER_HEADING_TOLERANCE = 8.0
OBSTACLE_CORNER_HEADING_SPREAD = 1.0
OBSTACLE_CORNER_CONFIRMATION_SAMPLES = 5
OBSTACLE_CORNER_CONFIRMATION_SECONDS = 0.30
OBSTACLE_CORNER_MAX_SAMPLE_GAP_SECONDS = 0.20
POST_RELEASE_RECHECK_SECONDS = 1.0
# Recovery used only after repeated front-interlock/corner retry loops.  It
# abandons the incomplete turn, steers back to the previous straight heading,
# and creates enough camera distance for the normal detector to decide again.
DEEP_RECOVERY_SPEED = 55
DEEP_RECOVERY_FRONT_CLEARANCE_CM = 70
DEEP_RECOVERY_MIN_SECONDS = 0.60
DEEP_RECOVERY_MAX_SECONDS = 0.80
DEEP_RECOVERY_HEADING_TOLERANCE = 10
DEEP_RECOVERY_CONFIRMATION_SAMPLES = 3
# Historical parking geometry, expressed as offsets from the measured starting
# BNO heading instead of assuming that the sensor always starts at exactly 0.
PARKING_HEADING_TOLERANCE = 4
PARKING_PHASE_NOTICE_SECONDS = 5.0
PARKING_EXIT_SCAN_FRAMES = 20
PARKING_ENTRY_DELAY_SECONDS = 7.5
PARKING_ENTRY_HEADING_TOLERANCE = 10
PARKING_ENTRY_CONFIRMATION_SAMPLES = 5
PARKING_ENTRY_CONFIRMATION_SECONDS = 0.30
PARKING_ANTI_LEFT_OPEN_CM = 100
PARKING_CLOCK_RIGHT_NEAR_CM = 80
PARKING_ANTI_REVERSE_ALIGN_TIMEOUT = 5.0
# Minimal side-wall safeguard requested for active pillar passing only.
# It changes steering only; it never stops, reverses, or changes navigation state.
SIDE_NUDGE_TRIGGER_CM = 3
SIDE_NUDGE_STEER = 10
TURN_SIDE_CLEARANCE_CM = 501 #20
# Adaptive initial corner arc after confirmed side-clearance evidence.
CORNER_REVERSE_CLEARANCE_CM = 60
CORNER_REVERSE_SPEED = 70
CORNER_REVERSE_STEER = 48
CORNER_REVERSE_MAX_SECONDS = 0.80
# Pillar-specific short recovery before deeper recovery is needed.
PILLAR_RELEASE_FRONT_CM = 45
PILLAR_RELEASE_SIDE_CM = 12
PILLAR_RELEASE_ANGLE = 12
PILLAR_RELEASE_SPEED = 50

PRE_TURN_BACKUP_SECONDS = 0.0 #0.1
POST_TURN_BACKUP_SECONDS = 2.8
RECENTER_SECONDS = 0.25 #0.30
TURN_TIMEOUT_SECONDS = 5.0
HARD_STOP_RELEASE_CM = 30
PYTHON_EMERGENCY_RELEASE_CM = 10
EMERGENCY_ESCAPE_SPEED = 80
EMERGENCY_ESCAPE_SECONDS = 0.30
EMERGENCY_ESCAPE_MIN_PROGRESS_CM = 3
EMERGENCY_ESCAPE_MAX_ATTEMPTS = 2
EMERGENCY_ESCAPE_SIDE_AVOID_CM = 10
HARD_STOP_RELEASE_SAMPLES = 3
HARD_STOP_MAX_REVERSE_SECONDS = 0.80
HARD_STOP_APPLY_GRACE_SECONDS = 0.35
HARD_STOP_MIN_PROGRESS_CM = 4
MAX_RECOVERY_ATTEMPTS = 3
HEADING_MAX_STEER = 30
TURN_STEER = 48
TURN_HEADING_TOLERANCE = 5

# Logic
BACK_AFTER_TURN_TIME = 0.8
BACK_BEFORE_TURN_TIME = 1.5
last_turn = time.time()
last_cooldown = time.time()
last_block_pass = time.time()
COOLDOWN = 3
InnerStuckFailsafeFlag = False
OuterStuckFailsafeFlag = False

# Data
data = None
angle, left, front, right, ir = None, None, None, None, None
last_valid_telemetry = None
last_valid_telemetry_time = 0.0
last_telemetry_status = {
    "protocol": None,
    "sequence": None,
    "esp_millis": None,
    "valid_mask": 0,
    "hard_stop": False,
    "applied_speed": None,
    "received_at": 0.0,
}
last_sensor_valid_time = {
    "heading": 0.0,
    "left": 0.0,
    "center": 0.0,
    "right": 0.0,
}
legacy_telemetry_sequence = 0

ui_running = True
ui_display_frame = None
ui_display_time = 0.0
ui_display_lock = threading.Lock()

# Camera Settings
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30
CAMERA_STALE_SECONDS = 0.30
BRIGHTNESS = 0
CONTRAST = 3.0
GAMMA = 0.7
inv_gamma = 1.0 / GAMMA
table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(256)]).astype("uint8")

# Image Processing Values
red_lower   = np.array([0, 167, 120])
red_upper   = np.array([255, 255, 255])
green_lower = np.array([0, 0, 0])
green_upper = np.array([255, 87, 255])
blue_lower  = np.array([0, 137, 0])
blue_upper  = np.array([255, 177, 92])
magenta_lower = np.array([0, 160, 0])
magenta_upper = np.array([255, 255, 130])

led = LED(3)
led.off()

colors = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "orange": (255, 165, 0)
}

masks = {
    "red": (red_lower, red_upper),
    "green": (green_lower, green_upper),
    "blue": (blue_lower, blue_upper)
}

COLOR_RANGES = {
    "red":    ([0, 184, 108], [255, 255, 255]),
    "green":  ([0, 0, 0],     [255, 101, 255]),
    "blue":   ([0, 153, 0],   [255, 186, 90]),
    "orange": ([0, 130, 150], [255, 255, 255])
}

MIN_AREA = {
    "red": 500,
    "green": 500,
    "blue": 300
}

# Unified detector thresholds. Areas and dimensions are fractions of the fixed
# camera frame, so the same calibration is used on the laptop and Raspberry Pi.
DETECTOR_MIN_AREA_RATIO = {
    "red": 0.00045,
    "green": 0.00045,
    "blue": 0.00030,
    "orange": 0.00030,
}
DETECTOR_MIN_HEIGHT_RATIO = {
    "red": 0.025,
    "green": 0.025,
    "blue": 0.010,
    "orange": 0.010,
}
DETECTOR_MIN_CONFIDENCE = 0.45
DETECTOR_MIN_RECTANGULARITY = 0.35
DETECTOR_MIN_SOLIDITY = 0.60
DETECTOR_TRACK_HISTORY = 5
DETECTOR_CONFIRMATION_HITS = 3
DETECTOR_UNCONFIRMED_MISSES = 2
DETECTOR_TRACK_MATCH_DISTANCE = 0.25
DETECTOR_ROI_TOP_RATIO = 1.0 / 3.0
DETECTOR_GRID_COLUMNS = 5
DETECTOR_GRID_ROWS = 5

# The mat's long blue and orange tape lines identify the approach to a corner.
# HSV is used only for these course landmarks; LAB remains the pillar detector.
COURSE_LINE_HSV_RANGES = {
    "blue": ((95, 60, 25), (135, 255, 255)),
    "orange": ((3, 55, 35), (22, 255, 255)),
}
COURSE_LINE_MIN_AREA_RATIO = 0.00025
COURSE_LINE_MIN_WIDTH_RATIO = 0.10
COURSE_LINE_MIN_ELONGATION = 2.0
CORNER_CONTEXT_CONFIRMATION_FRAMES = 3
VISION_CORNER_APPROACH_TIMEOUT_SECONDS = 6.0
PILLAR_CORNER_VETO_SECONDS = 1.0
POST_TURN_TAPE_IGNORE_GRACE_SECONDS = 3.0
CORNER_FOREGROUND_AREA_ADVANTAGE = 1.25

# Continuous pillar-control tuning for the installed servo orientation.
PILLAR_RED_TARGET_X = 0.25
RED_PRE_PASS_REVERSE_SECONDS = 0.8
RED_PRE_PASS_REVERSE_SPEED = 80
RED_PRE_PASS_REVERSE_STEER = -60
PILLAR_RED_STEERING_MULTIPLIER = 1.50
PILLAR_GREEN_TARGET_X = 0.62
PILLAR_PROXIMITY_START = 0.38
PILLAR_PROXIMITY_FULL = 0.90
PILLAR_NEAR_BOTTOM = 0.78
PILLAR_LATERAL_KP = 200.0
PILLAR_MAX_CORRECTION = 40
PILLAR_STEERING_SMOOTHING = 0.65
PILLAR_HEADING_WEIGHT = 0.25
PILLAR_MIN_CONTROL_WEIGHT = 0.55
PILLAR_GREEN_STEERING_MULTIPLIER = 1.50
PILLAR_ACQUIRE_TIMEOUT_SECONDS = 0.8
PILLAR_PASS_TIMEOUT_SECONDS = 4.0
PILLAR_CONFIRM_PASSED_SECONDS = 0.35
PILLAR_REQUIRED_LOST_FRAMES = 3
PILLAR_REACQUIRE_COOLDOWN_SECONDS = 0.60
PILLAR_APPROACH_SPEED = 105
PILLAR_PASS_SPEED = 100
PILLAR_MIN_PASS_SPEED = 80
PILLAR_MIN_STEERING_HOLD_SECONDS = 0.45
PILLAR_CLEARANCE_HOLD_SECONDS = 0.25
ALPHA_POSITION_WINDOW_SECONDS = 5.0

# The previous 1280x720 controller used contour areas of 4,000, 7,000 and
# 20,000 pixels to distinguish when a pillar needed progressively stronger
# action. Ratios preserve those same physical breakpoints at any resolution.
PILLAR_AREA_NEAR_RATIO = 4000.0 / (CAMERA_WIDTH * CAMERA_HEIGHT)
PILLAR_AREA_CLOSE_RATIO = 7000.0 / (CAMERA_WIDTH * CAMERA_HEIGHT)
PILLAR_AREA_TOO_CLOSE_RATIO = 20000.0 / (CAMERA_WIDTH * CAMERA_HEIGHT)
PILLAR_DISTANCE_PROFILES = {
    "far": {
        "speed": 100,
        "gain": 0.70,
        "min_steer": 5.0,
        "max_steer": 18.0,
        "min_weight": 0.55,
        "smoothing": 0.55,
    },
    "near": {
        "speed": 95,
        "gain": 1.00,
        "min_steer": 22.0,
        "max_steer": 38.0,
        "min_weight": 0.70,
        "smoothing": 0.68,
    },
    "close": {
        "speed": 90,
        "gain": 1.25,
        "min_steer": 30.0,
        "max_steer": 48.0,
        "min_weight": 0.85,
        "smoothing": 0.80,
    },
    "too_close": {
        "speed": 80,
        "gain": 1.50,
        "min_steer": 32.0,
        "max_steer": 50.0,
        "min_weight": 1.00,
        "smoothing": 0.90,
    },
}

APPROACH_CORNER_TIMEOUT_SECONDS = 3.0

class ThreadedVideoCapture:
    """
    Drop-in replacement for cv2.VideoCapture that:
    - sets V4L2 MJPG (lighter on Pi CPU)
    - shrinks internal buffer to 1 (no multi-frame lag)
    - runs a grab thread keeping only the most recent frame
    - exposes read()/isOpened()/release() just like OpenCV
    """
    def __init__(self, device="/dev/video1"):
        # The detector always receives one verified geometry. If the camera
        # driver negotiates another mode, read() resizes it before processing.
        w = CAMERA_WIDTH
        h = CAMERA_HEIGHT
        fps = CAMERA_FPS
        self.output_width = w
        self.output_height = h

        if os.name == "nt":
            self.cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print("Error: Cannot open camera")
            raise SystemExit(1)

        # Use MJPG; many USB webcams support it and it slashes CPU usage on Pi
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # Keep buffer tiny to avoid latency
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # Optional exposure/brightness tweaks (preserve your BRIGHTNESS via postprocess)
        # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)

        self.lock = threading.Lock()
        self.frame = None
        self.frame_sequence = 0
        self.frame_time = 0.0
        self.running = True
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

        # Warmup: wait until we have a frame
        t0 = time.time()
        while self.frame is None and (time.time() - t0) < 2.0:
            time.sleep(0.01)

        if self.frame is not None:
            actual_h, actual_w = self.frame.shape[:2]
            print(
                f"CAMERA: source={actual_w}x{actual_h}, "
                f"detector={self.output_width}x{self.output_height}@{fps}"
            )

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if not ok:
                continue
            with self.lock:
                self.frame = f
                self.frame_sequence += 1
                self.frame_time = time.monotonic()

    def read(self):
        # returns (ret, frame) like cv2.VideoCapture
        with self.lock:
            if self.frame is None:
                return False, None
            frame = self.frame.copy()
        if frame.shape[1] != self.output_width or frame.shape[0] != self.output_height:
            frame = cv2.resize(
                frame,
                (self.output_width, self.output_height),
                interpolation=cv2.INTER_AREA,
            )
        return True, frame

    def read_with_metadata(self):
        with self.lock:
            if self.frame is None:
                return False, None, self.frame_sequence, self.frame_time
            frame = self.frame.copy()
            sequence = self.frame_sequence
            captured_at = self.frame_time
        if frame.shape[1] != self.output_width or frame.shape[0] != self.output_height:
            frame = cv2.resize(
                frame,
                (self.output_width, self.output_height),
                interpolation=cv2.INTER_AREA,
            )
        return True, frame, sequence, captured_at

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.running = False
        try:
            self.t.join(timeout=0.5)
        except Exception:
            pass
        self.cap.release()

camera_device = 1 if os.name == "nt" else "/dev/video0"
cap = ThreadedVideoCapture(camera_device)

# Logic Constants
program_start_time = time.time()
red_turning_values = [0, 0, 0, 0, 0, 0, 0, 5, 10, 15, 0, 10, 15, 25, 30, 5, 10, 15, 25, 35, 0, 0, -1, -1, -1]
red_turning_values_4th_turn_anti = [-5, 0, 0, 0, 0, -5, 0, 5, 10, 15, -5, 10, 15, 25, 30, -5, 10, 15, 25, 35, -5, 0, -1, -1, -1]
red_turning_values_less_than_3000 = [0, 0, 0, 0, 0, 0, 0, 5, 20, 25, -10, 10, 15, 25, 30, -10, 10, 15, 25, 35, 0, 0, -1, -1, -1]
green_turning_values = [0, 0, 0, 0, 0, -15, -10, -5, 0, 0, -30, -25, -15, -10, 0, -35, -25, -15, -10, -5, -1, -1, -1, 0, 0]
green_turning_values_4th_turn_clock = [0, 0, 0, 0, 5, -15, -10, -5, 0, 5, -30, -25, -15, -10, 5, -35, -25, -15, -10, -5, -1, -1, -1, 0, 0] ## TODO: NEED TO BE UPDATED
green_turning_values_less_than_3000 = [0, 0, 0, 0, 0, -25, -20, -5, 0, 0, -35, -30, -15, -10, 0, -35, -25, -15, -10, -5, -1, -1, -1, 0, 0]

# Setting Up Serial
ser = serial.Serial(PORT, BAUDRATE, timeout=SERIAL_TIMEOUT)
time.sleep(2) # Let ESP32 Initialize

# Functions
# Logic Functions
def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle

def angle_diff(target, current):
    diff = target - current
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return diff

# Communication Functions
def get_front(port=None, baud=115200, timeout=0.1):
    global DIRECTION
    if DIRECTION == "anticlocwise":
        port = "/dev/ttyAMA3"
    else:
        port = "/dev/ttyAMA0"
    if not port:
        return None
    try:
        with serial.Serial(port, baud, timeout=timeout) as ser:
            # Read 9 bytes (full TF-LUNA frame)
            frame = ser.read(9)
            if len(frame) != 9:
                return None

            # Check header (0x59 0x59)
            if frame[0] != 0x59 or frame[1] != 0x59:
                return None

            # Extract distance (low byte + high byte)
            dist = frame[2] | (frame[3] << 8)
            return dist if dist >= 0 else None
    except serial.SerialException as e:
        print(f"[ERROR] Failed to read from {port}: {e}")
        return None

def _distance_is_valid(value):
    return value is not None and 0 < value < INVALID_DISTANCE


def _parse_telemetry_line(line):
    """Parse ESP32 v2 telemetry while retaining legacy four-field support.

    v2: T,seq,ms,heading,left,center,right,validMask,hardStop,appliedSpeed
    legacy: heading,left,center,right
    """
    global last_valid_telemetry, last_valid_telemetry_time
    global last_telemetry_status, last_sensor_valid_time
    global legacy_telemetry_sequence

    parts = [part.strip() for part in line.split(",")]
    now = time.monotonic()

    try:
        if len(parts) >= 10 and parts[0] == "T":
            sequence = int(parts[1])
            esp_millis = int(parts[2])
            heading_raw = float(parts[3])
            left_raw = int(parts[4])
            center_raw = int(parts[5])
            right_raw = int(parts[6])
            valid_mask = int(parts[7], 0)
            hard_stop = bool(int(parts[8]))
            applied_speed = int(parts[9])
            protocol = "v2"
        elif len(parts) == 4:
            legacy_telemetry_sequence += 1
            sequence = legacy_telemetry_sequence
            esp_millis = None
            heading_raw = float(parts[0])
            left_raw = int(parts[1])
            center_raw = int(parts[2])
            right_raw = int(parts[3])
            valid_mask = 0
            if -180 <= normalize_angle(heading_raw) <= 180:
                valid_mask |= VALID_HEADING
            if _distance_is_valid(left_raw):
                valid_mask |= VALID_LEFT
            if _distance_is_valid(center_raw):
                valid_mask |= VALID_CENTER
            if _distance_is_valid(right_raw):
                valid_mask |= VALID_RIGHT
            hard_stop = False
            applied_speed = None
            protocol = "legacy"
        else:
            return None
    except (TypeError, ValueError):
        return None

    # The v2 mask is authoritative, with range checks as a second guard.
    heading_ok = bool(valid_mask & VALID_HEADING) and -180 <= heading_raw <= 180
    left_ok = bool(valid_mask & VALID_LEFT) and _distance_is_valid(left_raw)
    center_ok = bool(valid_mask & VALID_CENTER) and _distance_is_valid(center_raw)
    right_ok = bool(valid_mask & VALID_RIGHT) and _distance_is_valid(right_raw)

    checked_mask = 0
    if left_ok:
        checked_mask |= VALID_LEFT
        last_sensor_valid_time["left"] = now
    if center_ok:
        checked_mask |= VALID_CENTER
        last_sensor_valid_time["center"] = now
    if right_ok:
        checked_mask |= VALID_RIGHT
        last_sensor_valid_time["right"] = now
    if heading_ok:
        checked_mask |= VALID_HEADING
        last_sensor_valid_time["heading"] = now

    values = (
        normalize_angle(heading_raw) if heading_ok else None,
        left_raw if left_ok else None,
        center_raw if center_ok else None,
        right_raw if right_ok else None,
        0,
    )
    last_valid_telemetry = values
    last_valid_telemetry_time = now
    last_telemetry_status = {
        "protocol": protocol,
        "sequence": sequence,
        "esp_millis": esp_millis,
        "valid_mask": checked_mask,
        "hard_stop": hard_stop,
        "applied_speed": applied_speed,
        "received_at": now,
    }
    return values


def telemetry_is_fresh():
    return (
        last_telemetry_status["received_at"] > 0
        and time.monotonic() - last_telemetry_status["received_at"]
        <= TELEMETRY_STALE_SECONDS
    )


def read_data():
    """Return the newest available sensor tuple without throwing telemetry away."""
    latest = None
    try:
        # Bound draining so a continuously streaming ESP32 cannot starve control.
        for _ in range(20):
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                break
            parsed = _parse_telemetry_line(line)
            if parsed is not None:
                latest = parsed
            if ser.in_waiting == 0:
                break
    except serial.SerialException as exc:
        print(f"[SERIAL ERROR] {exc}")

    if latest is not None:
        return latest
    if (
        last_valid_telemetry is not None
        and time.monotonic() - last_valid_telemetry_time
        <= TELEMETRY_HOLD_SECONDS
    ):
        return last_valid_telemetry
    return [None, None, None, None, None]

def wait_until_and_read_data():
    d = [None, None, None, None, None]
    while d[2] is None:
        d = read_data()
    return d

def read_latest():
    return read_data()

def send_data(speed, direction, steer):
    speed = max(0, min(255, int(round(speed))))
    direction = 1 if int(direction) == 1 else 0
    steer = max(-60, min(60, int(round(steer))))
    cmd = f"{speed},{direction},{steer}\n"
    ser.write(cmd.encode())

def flush_serial():
    """Consume buffered serial lines by parsing them, never by discarding them."""
    latest = None
    for _ in range(100):
        if ser.in_waiting == 0:
            break
        line = ser.readline().decode(errors="ignore").strip()
        parsed = _parse_telemetry_line(line)
        if parsed is not None:
            latest = parsed
    return latest

# Movement Functions
def steer_until_angle(current_angle, new_target, speed, direction, steer):
    if new_target > current_angle:
        sign = ">"
    else:
        sign = "<"

    data = [None, None, None, None, None]
    while not data[0]:
        data = read_data()
        flush_serial()
    angle, _, _, _, _ = data

    if sign == "<":
        while angle > new_target:
            send_data(speed, direction, steer)
            time.sleep(0.05)
            data = None
            while not data:
                data = read_latest()
            angle, _, _, _, _ = data
    elif sign == ">":
        while angle < new_target:
            send_data(speed, direction, steer)
            time.sleep(0.05)
            data = None
            while not data:
                data = read_latest()
            angle, _, _, _, _ = data

    send_data(0, 0, 0)

# Image Processing Functions
def show_camera(frame):
    cv2.imshow("USB Camera - Block Detection", frame)
    key = cv2.waitKey(1) & 0xFF
    if key in (ord("q"), 27):
        send_data(0, 0, 0)
        _thread.interrupt_main()

def publish_detection_view(frame):
    global ui_display_frame, ui_display_time
    with ui_display_lock:
        ui_display_frame = frame.copy()
        ui_display_time = time.time()

def live_ui_loop():
    """Keep the camera window responsive while showing detector annotations."""
    while ui_running:
        frame = None
        with ui_display_lock:
            if ui_display_frame is not None and time.time() - ui_display_time < 0.5:
                frame = ui_display_frame.copy()
        if frame is None:
            ret, raw_frame = cap.read()
            if ret:
                frame = raw_frame
        if frame is not None:
            show_camera(frame)
        else:
            time.sleep(0.01)

def start_live_ui():
    if SHOW_LIVE_UI:
        threading.Thread(target=live_ui_loop, daemon=True, name="LiveRobotUI").start()

def get_frame(cap):
    ret, frame = cap.read()
    while not ret:
        ret, frame = cap.read()
    return frame

def adjust_frame(frame):
    adjusted = cv2.convertScaleAbs(frame, alpha=CONTRAST, beta=BRIGHTNESS)

    inv_gamma = 1.0 / GAMMA
    table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)

    return adjusted

def process_frame(frame, masks, colors, DIR, SHOW=False):
    results = []

    frame = adjust_frame(frame)

    height = frame.shape[0]

    if DIR == "clockwise":
        third_line = height // 3
    else:
        third_line = height // 2

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    for name, (lower, upper) in masks.items():
        mask = cv2.inRange(lab, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                if y + h > third_line:
                    results.append((name, area))

                    if SHOW:
                        # Draw bounding rectangle
                        cv2.rectangle(frame, (x, y), (x + w, y + h), colors[name], 2)
                        # Put label
                        cv2.putText(frame, f"{name} ({area})", (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[name], 2)

    results.sort(key=lambda t: t[1], reverse=True)

    if SHOW:
        cv2.line(frame, (0, third_line), (frame.shape[1], third_line), (0, 255, 255), 2)
        publish_detection_view(frame)


    return results

def detect_biggest_block(frame, SHOW=False):
    global last_turn, DIRECTION, last_cooldown
    full_h, full_w, _ = frame.shape
    roi_start = int(full_h * 0.2)
    frame = frame[roi_start:full_h, 0:full_w]

    adjusted = cv2.convertScaleAbs(frame, alpha=CONTRAST, beta=BRIGHTNESS)
    adjusted = cv2.LUT(adjusted, table)

    lab = cv2.cvtColor(adjusted, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l_clahe = clahe.apply(l)
    lab_clahe = cv2.merge((l_clahe, a, b))
    adjusted_frame = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

    lab_frame = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2LAB)
    lab_raw = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    detections = {"red": [], "green": [], "blue": [], "orange": []}
    blue_y_values, orange_y_values = [], []

    for color, (lower, upper) in COLOR_RANGES.items():
        lower_bound = np.array(lower, dtype=np.uint8)
        upper_bound = np.array(upper, dtype=np.uint8)
        source_lab = lab_raw if color == "orange" else lab_frame
        mask = cv2.inRange(source_lab, lower_bound, upper_bound)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA.get(color, 0):
                continue
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            cx, cy = x + w_box // 2, y + h_box // 2
            detections[color].append((cx, cy, area, color))
            if color == "blue":
                blue_y_values.append(cy)
            elif color == "orange":
                orange_y_values.append(cy)
            if SHOW:
                cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), colors[color], 2)
                cv2.circle(frame, (cx, cy), 4, colors[color], -1)
                cv2.putText(frame, f"{color} ({area})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[color], 2)

    max_blue_y = max(blue_y_values) if blue_y_values else None
    max_orange_y = max(orange_y_values) if orange_y_values else None

    valid_blocks = []
    for color in ["red", "green"]:
        for (cx, cy, area, col) in detections[color]:
            if time.time() - last_cooldown > COOLDOWN:
                if max_blue_y and cy <= max_blue_y:
                    continue
                if max_orange_y and cy <= max_orange_y:
                    continue

            ### CODENAME PENDING FAILSAFE
            cy_full = cy + roi_start
            cell_w = full_w // 5
            cell_h = full_h // 5
            col_idx = cx // cell_w
            row_idx = cy_full // cell_h
            cell_num = row_idx * 5 + col_idx + 1

            if DIRECTION == "anticlockwise" and cell_num in [1, 6, 11, 16, 21]:
                if time.time() - last_turn <= 1:
                    print("CODENAMING PENDING FAILSAFE")
                    continue

            valid_blocks.append((col, cy, cx, area))

    closest_block = None
    if valid_blocks:
        closest_block = max(valid_blocks, key=lambda b: b[1])
        color, cy, cx, area = closest_block
        cy_full = cy + roi_start
        x_percent = (cx / full_w) * 100
        y_percent = ((full_h - cy_full) / full_h) * 100
        cell_w = full_w // 5
        cell_h = full_h // 5
        col_idx = cx // cell_w
        row_idx = cy_full // cell_h
        cell_number = row_idx * 5 + col_idx + 1
    else:
        color, cy, cx, area, cell_number, x_percent, y_percent = (None, None, None, None, None, None, None)

    special_case = 0
    if DIRECTION == "anticlockwise" and detections["green"]:
        cell_w = full_w // 5
        cell_h = full_h // 5
        green_in_right = False
        green_in_left_front = False
        for (cx_g, cy_g, area_g, _) in detections["green"]:
            cy_full_g = cy_g + roi_start
            col_idx = cx_g // cell_w
            row_idx = cy_full_g // cell_h
            cell_num = row_idx * 5 + col_idx + 1
            if cell_num in [5, 10, 15, 20, 25] and area_g > 30000:
                green_in_right = True
            if cell_num % 5 in [1, 2, 3, 4]:
                if ((max_blue_y and cy_g > max_blue_y) or (max_orange_y and cy_g > max_orange_y)):
                    if area_g > 7000:
                        green_in_left_front = True
        if green_in_right and green_in_left_front:
            special_case = 1
    elif DIRECTION == "clockwise" and detections["red"]:
        cell_w = full_w // 5
        cell_h = full_h // 5
        red_in_left = False
        red_in_right_front = False
        for (cx_r, cy_r, area_r, _) in detections["red"]:
            cy_full_r = cy_r + roi_start
            col_idx = cx_r // cell_w
            row_idx = cy_full_r // cell_h
            cell_num = row_idx * 5 + col_idx + 1
            if cell_num in [1, 6, 11, 16, 21] and area_r > 20000:
                red_in_left = True
            if cell_num % 5 in [4, 0]:
                if ((max_blue_y and cy_r > max_blue_y) or (max_orange_y and cy_r > max_orange_y)):
                    if area_r > 7000:
                        red_in_right_front = True
        if red_in_left and red_in_right_front:
            special_case = 1

    if SHOW:
        cell_w = full_w // 5
        cell_h = full_h // 5
        display = frame.copy()
        for i in range(1, 5):
            cv2.line(display, (i * cell_w, 0), (i * cell_w, frame.shape[0]), (200, 200, 200), 1)
        for j in range(1, 5):
            y_line = j * cell_h - roi_start
            if 0 <= y_line < frame.shape[0]:
                cv2.line(display, (0, y_line), (frame.shape[1], y_line), (200, 200, 200), 1)
        num = 1
        for r in range(5):
            for c in range(5):
                y_pos = r * cell_h + 20 - roi_start
                if 0 <= y_pos < frame.shape[0]:
                    x_pos = c * cell_w + 10
                    cv2.putText(display, str(num), (x_pos, y_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                num += 1
        if closest_block:
            cv2.rectangle(display, (cx - 20, cy - 20), (cx + 20, cy + 20), (0, 255, 255), 3)
            cv2.putText(display, f"Closest: {color} Cell:{cell_number}",
                        (cx, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        publish_detection_view(display)

    return (color if closest_block else None,
            y_percent if closest_block else None,
            x_percent if closest_block else None,
            area if closest_block else None,
            cell_number if closest_block else None,
            special_case)

# Running Functions
def exit_parking_lot():
    global DIRECTION
    data = [None, None, None, None, None]
    while not data[2]:
        data = read_data()
        flush_serial()
    angle, left, front, right, ir = data
    if left > right:
        DIRECTION = "anticlockwise"
    else:
        DIRECTION = "clockwise"
    print(f"DIRECTION: {DIRECTION}")

    if DIRECTION == "clockwise":
        steer_until_angle(0, 10, PARKING_SPEED, 0, 50)
        steer_until_angle(10, 25, PARKING_SPEED, 1, -50)
        steer_until_angle(25, 45, PARKING_SPEED, 0, 50)

    elif DIRECTION == "anticlockwise":
        steer_until_angle(0, -10, PARKING_SPEED, 0, -50)
        steer_until_angle(-10, -25, PARKING_SPEED, 1, 50)
        steer_until_angle(-25, -45, PARKING_SPEED, 0, -50)

def first_block_sequence():
    global DIRECTION
    detected_blocks = []
    itr = 0
    Flag = True
    while Flag:
        itr += 1

        if itr >= 20:
            break

        # The previous exit manoeuvre ends with speed 0. Keep moving while
        # checking the first block and keep the ESP32 command watchdog alive.
        send_data(PARKING_SPEED, 0, 0)
        detected_blocks = process_frame(get_frame(cap), masks, colors, SHOW=True, DIR=DIRECTION)

        Flag2 = True
        while len(detected_blocks) > 0 and Flag2:
            for object in detected_blocks:
                if object[0] == "blue":
                    detected_blocks.remove(object)
                    continue
                else:
                    Flag = False
                    Flag2 = False

    if detected_blocks:
        if detected_blocks[0][0] == "red" and DIRECTION == "anticlockwise":
            steer_until_angle(-45, -30, PARKING_SPEED, 0, 10)
            steer_until_angle(-30, -0, PARKING_SPEED, 0, 20)

        elif detected_blocks[0][0] == "green" and DIRECTION == "anticlockwise":
            steer_until_angle(-45, -90, PARKING_SPEED, 0, -35)
            send_data(PARKING_SPEED, 0, 0)
            distance = 100
            while distance >= 25:
                send_data(PARKING_SPEED, 0, 0)
                _, _, distance, _, _ = read_data()
                while not distance:
                    _, _, distance, _, _ = read_data()
            steer_until_angle(-90, 0, PARKING_SPEED, 0, 50)

        elif detected_blocks[0][0] == "green" and DIRECTION == "clockwise":
            steer_until_angle(45, 30, PARKING_SPEED, 0, -20)
            steer_until_angle(30, 0, PARKING_SPEED, 0, -30)

        elif detected_blocks[0][0] == "red" and DIRECTION == "clockwise":
            steer_until_angle(45, 70, PARKING_SPEED, 0, 35)
            send_data(PARKING_SPEED, 0, 0)
            distance = 100
            while distance >= 20:
                send_data(PARKING_SPEED, 0, 0)
                _, _, distance, _, _ = read_data()
                while not distance:
                    _, _, distance, _, _ = read_data()
            steer_until_angle(90, 0, PARKING_SPEED, 0, -50)

    elif DIRECTION == "anticlockwise":
        steer_until_angle(-45, -30, PARKING_SPEED, 0, 10)
        steer_until_angle(-30, -0, PARKING_SPEED, 0, 20)

    elif DIRECTION == "clockwise":
        steer_until_angle(45, 30, PARKING_SPEED, 0, -20)
        steer_until_angle(30, 0, PARKING_SPEED, 0, -30)

def main_logic():
    global DIRECTION, KP, SPEED, COUNTER
    global last_turn, target_angle, last_block_pass, last_cooldown
    global ir, left, right, front
    global InnerStuckFailsafeFlag, OuterStuckFailsafeFlag
    left = 1000
    right = 1000
    ir = 0

    if DIRECTION == "anticlockwise":
        angle = 0
        while COUNTER < COUNTER_MAX:
            ret, frame = cap.read()
            if not ret:
                continue

            block = detect_biggest_block(frame, SHOW=True)

            data = read_data()
            print(data)
            last_left = left
            if data and data[2]:
                angle, left, front, right, ir = data
                # print(data)

            # print(angle_diff(target_angle, angle))
            if (angle_diff(target_angle, angle) > 50 and time.time() - last_turn < 5) or (angle_diff(target_angle, angle) > 60) or (angle_diff(target_angle, angle) > 50 and time.time() - last_turn > 10):
                print("GOING WRONG DIRECTION FAILSAFE")
                send_data(SPEED, 0, 40)
                time.sleep(0.5)
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                angle, left, front, right, ir = data
                continue

            # print(block)

            if not block[0] and time.time() - last_turn > 5 and time.time() - last_block_pass > 2: ### FAILSAFE --> It doesn't speed up after a turn for 5s
                SPEED = SPEED
            else:
                SPEED = SPEED_NO_AURA_FARM

            # REVERSE PRECEDENCE
            if block[1] and block[1] < 20 and block[2] > 40 and block[2] < 60:
                send_data(0, 0, 0)
                time.sleep(0.2)
                send_data(80, 1, 0)
                time.sleep(2)
                send_data(0, 0, 0)
                flush_serial()
                ret = None
                while not ret:
                    ret, frame = cap.read()
                ret = None
                while not ret:
                    ret, frame = cap.read()
                continue

            if ir == 1:
                ### IR REVERSE FAILSAFE
                print("IR REVERSE FAILSAFE")
                send_data(80, 1, 0)
                time.sleep(1)
                if COUNTER % 4 == 0:
                    time.sleep(1)
                    steer_until_angle(0, -30, 100, 0, -50)
                    steer_until_angle(-30, 0, 100, 0, 50)
                send_data(0, 0, 0)
                flush_serial()
                ir = 1
                while ir:
                    data = None
                    while not data[2]:
                        data = read_data()
                    angle, left, front, right, ir = data
                ret = None
                while not ret:
                    ret, frame = cap.read()
                continue

            if ir == 1:
                send_data(80, 1, 0)
                time.sleep(1)
                send_data(0, 0, 0)
                flush_serial()

            ### INNER STUCK FAILSAFE
            if left < 5 and right > 60:
                if not InnerStuckFailsafeFlag:
                    InnerStuckFailsafeFlag = True
                    last_inner_stuck = time.time()
                elif InnerStuckFailsafeFlag and time.time() - last_inner_stuck > 5:
                    send_data(80, 1, -10)
                    time.sleep(1)
                    send_data(80, 0, 10)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    data = wait_until_and_read_data()
                    InnerStuckFailsafeFlag = False
                    continue

            ### OUTER STUCK FAILSAFE
            if right < 5 and left > 60:
                if not OuterStuckFailsafeFlag:
                    OuterStuckFailsafeFlag = True
                    last_outer_stuck = time.time()
                elif OuterStuckFailsafeFlag and time.time() - last_outer_stuck > 5:
                    send_data(80, 1, 10)
                    time.sleep(1)
                    send_data(80, 0, -10)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    data = wait_until_and_read_data()
                    OuterStuckFailsafeFlag = False
                    continue

            # BLOCK AVOID PRECEDENCE

            # Matrix
            ### DOUBLE TROUBLE FAILSAFE
            if block[5]:
                print("DOUBLE TROUBLE FAILSAFE ACTIVATED")
                send_data(80, 0, 0)
                time.sleep(2)
                send_data(0, 0, 0)
                flush_serial()

            if block[0] and block[3] > 4000:
                # if block[4] in [] ### AFTERTURN EXTREME FAILSAFE
                color, y, x, area, cell, dt_failsafe_flag = block

                if color == "red" or color == "green":
                    last_block_pass = time.time()
                if color == "red":
                    if red_turning_values[cell-1] != -1:
                        send_data(SPEED, 0, int(red_turning_values[cell-1]))
                        if COUNTER % 4 == 0:
                            send_data(SPEED, 0, int(red_turning_values_4th_turn_anti[cell-1]))
                        elif area < 3000:
                            send_data(SPEED, 0, red_turning_values_less_than_3000[cell - 1])
                        continue
                    elif red_turning_values[cell-1] == -1:
                        send_data(80, 1, 0)
                        time.sleep(1)
                        send_data(0, 0, 0)
                        flush_serial()
                        continue
                    else:
                        error = angle_diff(target_angle, angle)
                        send_data(SPEED, 0, error)
                elif color == "green":
                    if green_turning_values[cell-1] != -1: ### DOUBLE TROUBLE KINDA FAILSAFE
                        if abs(angle_diff(target_angle, angle)) < 5 and left > 100 and time.time() - last_turn > 5:
                            print("DOUBLE TROUBLE KINDA FAILSWAFE")
                            send_data(80, 0, 0)
                            time.sleep(1)
                            send_data(0, 0, 0)
                            continue
                        send_data(SPEED, 0, int(green_turning_values[cell-1] * BLOCK_MULITPLIER_GREEN_ANTI))
                        continue
                    elif green_turning_values[cell-1] == -1:
                        send_data(0, 0, 0)
                        data = wait_until_and_read_data()
                        angle, left, front, right, ir = data
                        if abs(angle_diff(target_angle, angle)) < 5:
                            if left > 100:
                                send_data(80, 0, 0)
                                time.sleep(1)
                                send_data(0, 0, 0)
                        send_data(80, 1, 0)
                        time.sleep(1)
                        send_data(0, 0, 0)
                        flush_serial()
                        continue
                    # else:
                    #     error = angle_diff(target_angle, angle)
                    #     send_data(SPEED, 0, error)
            # Other
            # if block[0]:
            #     if area > 4000 and y < 85:
            #         if color == "red":
            #             send_data(SPEED, 0, 25)
            #             continue
            #         elif color == "green":
            #             send_data(SPEED, 0, -25)
            #             continue

            # TURN PRECEDENCE

            data = read_data()
            last_left = left
            if data[2]:
                angle, left, front, right, ir = data
                # print(data)

            # PARKING PRECEDENCE
            if data[2] == 0 and COUNTER == COUNTER_MAX - 1 and time.time() - last_turn > 7.5 and left > 100 and abs(angle_diff(0, angle)) < 15:
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                if abs(angle_diff(0, data[0])) > 10:
                    continue
                if data[1] < 100:
                    continue
                print("MAIN LOOP COMPLETE")
                send_data(0, 0, 0)
                return

            # print(left, front)
            # if COUNTER == COUNTER_MAX - 1 and front < 15 and not block[0]:
            #     print("PARKING")
            #     parking()
            #     exit()

            if front < 15 and block[0] == None and (left > 100 or ((COUNTER + 1) % 4 == 0 and left > 50)) and COUNTER != COUNTER_MAX - 1:
                ### FALSE TURN FAILSAFE
                data = None
                while not data:
                    data = read_data()
                    if data[2]:
                        angle, left, front, right, ir = data
                    else:
                        continue
                if abs(angle_diff(normalize_angle(target_angle+5), angle)) > 20:
                    send_data(80, 1, 0)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    flush_serial()
                    continue
                target_angle = normalize_angle(target_angle - 90)
                COUNTER += 1
                print(f"COUNTER = {COUNTER}, TARGET={target_angle}")
                if right >= 25:
                    while True:
                        send_data(TURNING_SPEED, 1, 40)

                        data = read_data()
                        if not data[2]:
                            continue
                        angle, left, front, right, ir = data

                        err = angle_diff(normalize_angle(target_angle+10), angle)
                        if err >= -10:
                            send_data(80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            send_data(0, 0, 0)
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                elif right <= 25:
                    send_data(80, 1, 0)
                    time.sleep(BACK_BEFORE_TURN_TIME)
                    send_data(0, 0, 0)
                    while True:
                        send_data(TURNING_SPEED, 0, -40)
                        data = read_data()
                        if not data[2]:
                            continue
                        angle, left, front, right, ir = data

                        err = angle_diff(normalize_angle(target_angle+10), angle)

                        if err >= -10:
                            send_data(80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            send_data(0, 0, 0)
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                flush_serial()
            # elif front < 15 and COUNTER % 4 == 0:
            #     print("PARKING LOT COLLISION FAILSAFE")
            #     send_data(80, 1, 0)
            #     time.sleep(1)
            #     send_data(0, 0, 0)
            #     steer_until_angle(0, -10, 80, 0, -50)
            #     steer_until_angle(-10, -25, 80, 1, 50)
            #     steer_until_angle(-25, -45, 80, 0, -50)


            # PID PRECEDENCE
            if COUNTER % 4 == 0:
                error = angle_diff(normalize_angle(target_angle - 4), angle)
            else:
                error = angle_diff(target_angle, angle)
            send_data(SPEED, 0, error)
            if COUNTER == COUNTER_MAX - 1:
                print("COUNTER MAX - 1")
            if COUNTER == COUNTER_MAX - 1 and time.time() - last_turn > 7.5 and left > 100 and abs(angle_diff(0, angle)) < 15:
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                if abs(angle_diff(0, data[0])) > 10:
                    continue
                if data[1] < 100:
                    continue
                print("MAIN LOOP COMPLETE")
                send_data(0, 0, 0)
                return
    if DIRECTION == "clockwise":
        angle = 0
        while COUNTER < COUNTER_MAX:
            ret, frame = cap.read()
            if not ret:
                continue

            block = detect_biggest_block(frame, SHOW=True)

            data = read_data()
            print(data)
            last_left = left
            if data and data[2]:
                angle, left, front, right, ir = data
                # print(data)

            # print(angle_diff(target_angle, angle))
            if (angle_diff(target_angle, angle) < -50 and time.time() - last_turn < 5) or (angle_diff(target_angle, angle) < -60) or (angle_diff(target_angle, angle) < -50 and time.time() - last_turn > 10):
                print("GOING WRONG DIRECTION FAILSAFE")
                send_data(SPEED, 0, -40)
                time.sleep(0.5)
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                angle, left, front, right, ir = data
                continue

            # print(block)

            if not block[0] and time.time() - last_turn > 5 and time.time() - last_block_pass > 2: ### FAILSAFE --> It doesn't speed up after a turn for 5s
                SPEED = SPEED
            else:
                SPEED = SPEED_NO_AURA_FARM

            # REVERSE PRECEDENCE
            if block[1] and block[1] < 20 and block[2] > 40 and block[2] < 60:
                send_data(0, 0, 0)
                time.sleep(0.2)
                send_data(80, 1, 0)
                time.sleep(1)
                send_data(0, 0, 0)
                flush_serial()
                ret = None
                while not ret:
                    ret, frame = cap.read()
                ret = None
                while not ret:
                    ret, frame = cap.read()
                continue

            if ir == 1:
                ### IR REVERSE FAILSAFE
                print("IR REVERSE FAILSAFE")
                send_data(80, 1, 0)
                time.sleep(1)
                if COUNTER % 4 == 0:
                    time.sleep(1)
                    steer_until_angle(0, -30, 100, 0, -50)
                    steer_until_angle(-30, 0, 100, 0, 50)
                send_data(0, 0, 0)
                flush_serial()
                ir = 1
                while ir:
                    data = None
                    while not data[2]:
                        data = read_data()
                    angle, left, front, right, ir = data
                ret = None
                while not ret:
                    ret, frame = cap.read()
                continue

            if ir == 1:
                send_data(80, 1, 0)
                time.sleep(1)
                send_data(0, 0, 0)
                flush_serial()

            ### LEFT STUCK FAILSAFE
            if left < 5 and right > 60:
                if not InnerStuckFailsafeFlag:
                    InnerStuckFailsafeFlag = True
                    last_inner_stuck = time.time()
                elif InnerStuckFailsafeFlag and time.time() - last_inner_stuck > 5:
                    send_data(80, 1, -10)
                    time.sleep(1)
                    send_data(80, 0, 10)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    data = wait_until_and_read_data()
                    InnerStuckFailsafeFlag = False
                    continue

            ### RIGHT STUCK FAILSAFE
            if right < 5 and left > 60:
                if not OuterStuckFailsafeFlag:
                    OuterStuckFailsafeFlag = True
                    last_outer_stuck = time.time()
                elif OuterStuckFailsafeFlag and time.time() - last_outer_stuck > 5:
                    send_data(80, 1, 10)
                    time.sleep(1)
                    send_data(80, 0, -10)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    data = wait_until_and_read_data()
                    OuterStuckFailsafeFlag = False
                    continue

            # PARKING PRECEDENCE FRFR

            if COUNTER == COUNTER_MAX - 1 and right < 80 and abs(angle_diff(0, angle)) < 15:
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                angle, left, front, right, ir = data
                if right > 80:
                    continue
                if abs(angle_diff(0, data[0])) > 10:
                    continue
                print("MAIN LOOP COMPLETE -- 24")
                send_data(0, 0, 0)
                return

            # BLOCK AVOID PRECEDENCE

            # Matrix
            ### DOUBLE TROUBLE FAILSAFE
            if block[5]:
                print("DOUBLE TROUBLE FAILSAFE ACTIVATED")
                send_data(80, 0, 0)
                time.sleep(2)
                send_data(0, 0, 0)
                flush_serial()

            if block[0] and block[3] > 4000:
                # if block[4] in [] ### AFTERTURN EXTREME FAILSAFE
                color, y, x, area, cell, dt_failsafe_flag = block

                if color == "red" or color == "green":
                    last_block_pass = time.time()
                if color == "red":
                    if red_turning_values[cell-1] != -1:
                        if abs(angle_diff(target_angle, angle)) < 5 and right > 100 and time.time() - last_turn > 5: ### DOUBLE TROUBLE KINDA FAILSAFE
                            print("DOUBLE TROUBLE KINDA FAILSWAFE")
                            send_data(80, 0, 0)
                            time.sleep(1)
                            send_data(0, 0, 0)
                            continue
                        send_data(SPEED, 0, int(red_turning_values[cell-1]))
                        continue
                    elif red_turning_values[cell-1] == -1:
                        send_data(80, 1, 0)
                        time.sleep(1)
                        send_data(0, 0, 0)
                        flush_serial()
                        continue
                    else:
                        error = angle_diff(target_angle, angle)
                        send_data(SPEED, 0, error)
                elif color == "green":
                    if green_turning_values[cell-1] != -1: ### DOUBLE TROUBLE KINDA FAILSAFE
                        send_data(SPEED, 0, int(green_turning_values[cell-1] * BLOCK_MULITPLIER_GREEN_ANTI))
                        if COUNTER % 4 == 0:
                            send_data(SPEED, 0, int(green_turning_values_4th_turn_clock[cell-1])) ##### PENDING:
                        continue
                    elif green_turning_values[cell-1] == -1:
                        send_data(0, 0, 0)
                        data = wait_until_and_read_data()
                        angle, left, front, right, ir = data
                        send_data(80, 1, 0)
                        time.sleep(1)
                        send_data(0, 0, 0)
                        flush_serial()
                        continue
                    # else:
                    #     error = angle_diff(target_angle, angle)
                    #     send_data(SPEED, 0, error)
            # Other
            # if block[0]:
            #     if area > 4000 and y < 85:
            #         if color == "red":
            #             send_data(SPEED, 0, 25)
            #             continue
            #         elif color == "green":
            #             send_data(SPEED, 0, -25)
            #             continue

            # TURN PRECEDENCE

            data = read_data()
            last_left = left
            if data[2]:
                angle, left, front, right, ir = data
                # print(data)

            # PARKING PRECEDENCE
            if data[2] == 0 and COUNTER == COUNTER_MAX - 1 and right < 80 and abs(angle_diff(0, angle)) < 15:
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                if abs(angle_diff(0, data[0])) > 10:
                    continue
                if data[1] < 100:
                    continue
                send_data(80, 0, 0)
                time.sleep(0.5)
                send_data(0, 0, 0)
                if right > 80:
                    continue
                print("MAIN LOOP COMPLETE")
                send_data(0, 0, 0)
                return

            # print(left, front)
            # if COUNTER == COUNTER_MAX - 1 and front < 15 and not block[0]:
            #     print("PARKING")
            #     parking()
            #     exit()

            if front < 15 and block[0] == None and (right > 100 or ((COUNTER + 1) % 4 == 0)) and COUNTER != COUNTER_MAX - 1:
                ### FALSE TURN FAILSAFE
                data = None
                while not data:
                    data = read_data()
                    if data[2]:
                        angle, left, front, right, ir = data
                    else:
                        continue
                if abs(angle_diff(normalize_angle(target_angle-5), angle)) > 20:
                    send_data(80, 1, 0)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    flush_serial()
                    continue
                target_angle = normalize_angle(target_angle + 90)
                COUNTER += 1
                print(f"COUNTER = {COUNTER}, TARGET={target_angle}")
                if left >= 25:
                    while True:
                        send_data(TURNING_SPEED, 1, -40)

                        data = read_data()
                        if not data[2]:
                            continue
                        angle, left, front, right, ir = data

                        err = angle_diff(normalize_angle(target_angle-10), angle)
                        if err <= 10:
                            send_data(80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            send_data(0, 0, 0)
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                elif left <= 25:
                    send_data(80, 1, 0)
                    time.sleep(BACK_BEFORE_TURN_TIME)
                    send_data(0, 0, 0)
                    while True:
                        send_data(TURNING_SPEED, 0, 40)
                        data = read_data()
                        if not data[2]:
                            continue
                        angle, left, front, right, ir = data

                        err = angle_diff(normalize_angle(target_angle-10), angle)

                        if err <= 10:
                            send_data(80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            send_data(0, 0, 0)
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                flush_serial()
            # elif front < 15 and COUNTER % 4 == 0:
            #     print("PARKING LOT COLLISION FAILSAFE")
            #     send_data(80, 1, 0)
            #     time.sleep(1)
            #     send_data(0, 0, 0)
            #     steer_until_angle(0, -10, 80, 0, -50)
            #     steer_until_angle(-10, -25, 80, 1, 50)
            #     steer_until_angle(-25, -45, 80, 0, -50)


            # PID PRECEDENCE
            if COUNTER % 4 == 0:
                error = angle_diff(normalize_angle(target_angle + 4), angle)
            else:
                error = angle_diff(target_angle, angle)
            send_data(SPEED, 0, error)
            if COUNTER == COUNTER_MAX - 1:
                print("COUNTER MAX - 1")

@dataclass
class PillarDetection:
    color: str
    bbox: tuple
    center: tuple
    bottom: int
    normalized_area: float
    height_ratio: float
    aspect_ratio: float
    rectangularity: float
    solidity: float
    confidence: float
    center_x_norm: float
    bottom_norm: float
    observed_at: float
    grid_column: int = 0
    blocked_by_corner: bool = False
    foreground_priority: bool = False
    track_id: int = 0
    confirmed: bool = False
    seen_this_frame: bool = True
    hit_count: int = 1
    miss_count: int = 0

    def distance_band(self):
        if self.normalized_area < PILLAR_AREA_NEAR_RATIO:
            return "far"
        if self.normalized_area < PILLAR_AREA_CLOSE_RATIO:
            return "near"
        if self.normalized_area < PILLAR_AREA_TOO_CLOSE_RATIO:
            return "close"
        return "too_close"

    def proximity(self):
        span = max(0.01, PILLAR_PROXIMITY_FULL - PILLAR_PROXIMITY_START)
        return max(0.0, min(1.0, (self.bottom_norm - PILLAR_PROXIMITY_START) / span))


class UnifiedPillarDetector:
    """One normalized detector and persistent tracker for the whole run."""

    DRAW_COLORS = {
        "red": (0, 0, 255),
        "green": (0, 255, 0),
        "blue": (255, 0, 0),
        "orange": (0, 165, 255),
    }

    def __init__(self):
        self.active = None
        self.passing_track_id = None
        self.hit_history = deque(maxlen=DETECTOR_TRACK_HISTORY)
        self.next_track_id = 1
        self.last_candidates = []
        self.course_line_depths = {"blue": None, "orange": None}
        self.course_line_boxes = {"blue": [], "orange": []}
        self.corner_reference_y = None
        self.corner_context_current = False
        self.corner_context_confirmed = False
        self.corner_context_history = deque(maxlen=CORNER_CONTEXT_CONFIRMATION_FRAMES)
        self.ignored_corner_pillars = []
        self.corner_trigger_pillars = []
        self.actionable_pillar_visible = False
        self.last_actionable_pillar_at = 0.0
        self.corner_filter_suppressed_until = 0.0

    def _detect_course_line_depths(self, frame, roi_top):
        frame_h, frame_w = frame.shape[:2]
        frame_area = float(frame_h * frame_w)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), np.uint8)
        depths = {"blue": None, "orange": None}
        boxes = {"blue": [], "orange": []}

        for color, (lower, upper) in COURSE_LINE_HSV_RANGES.items():
            mask = cv2.inRange(
                hsv,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            )
            mask[:roi_top, :] = 0
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            line_depths = []
            for contour in contours:
                area = cv2.contourArea(contour)
                x, y, width, height = cv2.boundingRect(contour)
                horizontal_elongation = width / float(max(1, height))
                if area < frame_area * COURSE_LINE_MIN_AREA_RATIO:
                    continue
                if width < frame_w * COURSE_LINE_MIN_WIDTH_RATIO:
                    continue
                if horizontal_elongation < COURSE_LINE_MIN_ELONGATION:
                    continue
                boxes[color].append((x, y, width, height))
                line_depths.append(y + height // 2)

            if line_depths:
                depths[color] = max(line_depths)

        self.course_line_depths = depths
        self.course_line_boxes = boxes

    def _apply_corner_context(self, candidates, frame_h, frame_w):
        blue_depth = self.course_line_depths["blue"]
        orange_depth = self.course_line_depths["orange"]
        both_lines_visible = blue_depth is not None and orange_depth is not None
        corner_filter_enabled = not self.corner_filter_suppressed()
        self.corner_reference_y = (
            max(blue_depth, orange_depth) if both_lines_visible else None
        )
        # Acquisition alone is not a commitment to pass. Protect only the best
        # match to a track for which the controller has started avoidance.
        passing_candidate = (
            self._matching_candidate(candidates)
            if self.active is not None
            and self.passing_track_id == self.active.track_id
            else None
        )

        classified = []
        for candidate in candidates:
            if candidate.color not in ("red", "green"):
                classified.append(candidate)
                continue

            grid_column = min(
                DETECTOR_GRID_COLUMNS - 1,
                max(0, int(candidate.center_x_norm * DETECTOR_GRID_COLUMNS)),
            )
            blocked = False
            if both_lines_visible and corner_filter_enabled:
                center_behind_lines = candidate.center[1] <= self.corner_reference_y
                fully_behind_lines = candidate.bottom <= self.corner_reference_y
                in_outer_grid_column = grid_column in (0, DETECTOR_GRID_COLUMNS - 1)
                blocked = center_behind_lines and (
                    fully_behind_lines or in_outer_grid_column
                )

            # Orange/blue tape may move in front of a pillar that the robot is
            # already committed to passing. Keep only that matching active
            # track actionable; new side/background pillars remain filtered.
            active_track_priority = (
                blocked
                and candidate is passing_candidate
            )
            if active_track_priority:
                blocked = False

            candidate = replace(
                candidate,
                grid_column=grid_column,
                blocked_by_corner=blocked,
                foreground_priority=active_track_priority,
            )
            classified.append(candidate)

        # Restore the previous controller's "closest valid block" priority.
        # When a large, lower interior pillar and a smaller outer-column pillar
        # are both behind the tape reference, the interior pillar is the obstacle
        # directly in front of the robot. Only the smaller background pillar is
        # ignored. Relative size is used because the unified mask often measures
        # a visible foreground pillar below the old 4,000-pixel contour cutoff.
        blocked_inner = [
            (index, item)
            for index, item in enumerate(classified)
            if item.color in ("red", "green")
            and item.blocked_by_corner
            and item.grid_column not in (0, DETECTOR_GRID_COLUMNS - 1)
        ]
        blocked_outer = [
            item
            for item in classified
            if item.color in ("red", "green")
            and item.blocked_by_corner
            and item.grid_column in (0, DETECTOR_GRID_COLUMNS - 1)
        ]
        if blocked_inner and blocked_outer:
            foreground_index, foreground = max(
                blocked_inner,
                key=lambda pair: (pair[1].bottom_norm, pair[1].normalized_area),
            )
            largest_background = max(
                blocked_outer,
                key=lambda item: item.normalized_area,
            )
            foreground_is_dominant = (
                foreground.normalized_area
                >= largest_background.normalized_area * CORNER_FOREGROUND_AREA_ADVANTAGE
                and foreground.bottom_norm >= largest_background.bottom_norm
            )
            if foreground_is_dominant:
                classified[foreground_index] = replace(
                    foreground,
                    blocked_by_corner=False,
                    foreground_priority=True,
                )

        ignored = [
            item
            for item in classified
            if item.color in ("red", "green") and item.blocked_by_corner
        ]
        actionable = [
            item
            for item in classified
            if item.color in ("red", "green") and not item.blocked_by_corner
        ]
        corner_triggers = [
            item
            for item in ignored
            if item.grid_column in (0, DETECTOR_GRID_COLUMNS - 1)
        ]
        # A foreground obstacle always has precedence over a possible corner
        # hint from a smaller background block.
        if actionable:
            corner_triggers = []

        self.ignored_corner_pillars = ignored
        self.corner_trigger_pillars = corner_triggers
        self.actionable_pillar_visible = bool(actionable)
        if self.actionable_pillar_visible:
            self.last_actionable_pillar_at = time.monotonic()
        self.corner_context_current = bool(corner_triggers)
        self.corner_context_history.append(self.corner_context_current)
        self.corner_context_confirmed = (
            len(self.corner_context_history) == CORNER_CONTEXT_CONFIRMATION_FRAMES
            and all(self.corner_context_history)
        )
        return classified

    def actionable_pillar_recently_seen(self):
        return (
            self.actionable_pillar_visible
            or time.monotonic() - self.last_actionable_pillar_at
            <= PILLAR_CORNER_VETO_SECONDS
        )

    def suppress_corner_filter(self, duration):
        self.corner_filter_suppressed_until = time.monotonic() + max(0.0, duration)

    def corner_filter_suppressed(self):
        return time.monotonic() < self.corner_filter_suppressed_until

    @staticmethod
    def _iou(first_bbox, second_bbox):
        ax, ay, aw, ah = first_bbox
        bx, by, bw, bh = second_bbox
        x1 = max(ax, bx)
        y1 = max(ay, by)
        x2 = min(ax + aw, bx + bw)
        y2 = min(ay + ah, by + bh)
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        union = aw * ah + bw * bh - intersection
        return intersection / union if union > 0 else 0.0

    @staticmethod
    def _center_distance(first, second):
        dx = first.center_x_norm - second.center_x_norm
        dy = first.bottom_norm - second.bottom_norm
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def _candidate_score(candidate):
        area_score = min(1.0, candidate.normalized_area / 0.02)
        return 0.55 * candidate.bottom_norm + 0.35 * candidate.confidence + 0.10 * area_score

    def _extract_candidates(self, frame):
        frame_h, frame_w = frame.shape[:2]
        frame_area = float(frame_h * frame_w)
        roi_top = max(0, min(frame_h, int(frame_h * DETECTOR_ROI_TOP_RATIO)))
        adjusted = adjust_frame(frame)
        lab = cv2.cvtColor(adjusted, cv2.COLOR_BGR2LAB)
        lightness, channel_a, channel_b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        lab = cv2.merge((clahe.apply(lightness), channel_a, channel_b))
        kernel = np.ones((3, 3), np.uint8)
        candidates = []

        for color, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(
                lab,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask[:roi_top, :] = 0
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area <= 0:
                    continue
                x, y, width, height = cv2.boundingRect(contour)
                normalized_area = area / frame_area
                height_ratio = height / float(frame_h)
                if normalized_area < DETECTOR_MIN_AREA_RATIO[color]:
                    continue
                if height_ratio < DETECTOR_MIN_HEIGHT_RATIO[color]:
                    continue

                aspect_ratio = height / float(max(1, width))
                rectangularity = area / float(max(1, width * height))
                hull_area = cv2.contourArea(cv2.convexHull(contour))
                solidity = area / hull_area if hull_area > 0 else 0.0

                if color in ("red", "green"):
                    if not 0.65 <= aspect_ratio <= 8.0:
                        continue
                    if rectangularity < DETECTOR_MIN_RECTANGULARITY:
                        continue
                    if solidity < DETECTOR_MIN_SOLIDITY:
                        continue
                else:
                    if not 0.20 <= aspect_ratio <= 10.0:
                        continue
                    if rectangularity < 0.25 or solidity < 0.50:
                        continue

                area_quality = min(
                    1.0,
                    normalized_area / max(0.00001, DETECTOR_MIN_AREA_RATIO[color] * 5.0),
                )
                height_quality = min(1.0, height_ratio / 0.25)
                confidence = max(
                    0.0,
                    min(
                        1.0,
                        0.20 * area_quality
                        + 0.20 * height_quality
                        + 0.25 * rectangularity
                        + 0.35 * solidity,
                    ),
                )
                if confidence < DETECTOR_MIN_CONFIDENCE:
                    continue

                center_x = x + width // 2
                center_y = y + height // 2
                bottom = y + height
                candidates.append(
                    PillarDetection(
                        color=color,
                        bbox=(x, y, width, height),
                        center=(center_x, center_y),
                        bottom=bottom,
                        normalized_area=normalized_area,
                        height_ratio=height_ratio,
                        aspect_ratio=aspect_ratio,
                        rectangularity=rectangularity,
                        solidity=solidity,
                        confidence=confidence,
                        center_x_norm=center_x / float(frame_w),
                        bottom_norm=bottom / float(frame_h),
                        observed_at=time.monotonic(),
                    )
                )

        self._detect_course_line_depths(frame, roi_top)
        return self._apply_corner_context(candidates, frame_h, frame_w)

    def _start_track(self, candidates):
        pillar_candidates = [
            item
            for item in candidates
            if item.color in ("red", "green") and not item.blocked_by_corner
        ]
        if not pillar_candidates:
            return None
        selected = max(pillar_candidates, key=(
            (lambda item: (item.bottom_norm, item.normalized_area))
            if getattr(self, "parking_first_scan", False) else self._candidate_score
        ))
        self.hit_history.clear()
        self.hit_history.append(True)
        self.active = replace(
            selected,
            track_id=self.next_track_id,
            confirmed=False,
            hit_count=1,
            miss_count=0,
        )
        self.next_track_id += 1
        return self.active

    def _matching_candidate(self, candidates, include_blocked=False):
        matches = []
        for candidate in candidates:
            if candidate.color != self.active.color or (
                candidate.blocked_by_corner and not include_blocked
            ):
                continue
            distance = self._center_distance(self.active, candidate)
            overlap = self._iou(self.active.bbox, candidate.bbox)
            if distance <= DETECTOR_TRACK_MATCH_DISTANCE or overlap >= 0.05:
                score = 1.5 * overlap - distance + 0.20 * candidate.bottom_norm
                matches.append((score, candidate))
        return max(matches, key=lambda item: item[0])[1] if matches else None

    def update(self, frame):
        candidates = self._extract_candidates(frame)
        self.last_candidates = candidates

        # Drop all tracking only when every visible pillar is side-on/background.
        # An ignored small pillar must never suppress a valid foreground pillar.
        if self.ignored_corner_pillars and not self.actionable_pillar_visible:
            self.release_active()
            return None

        if self.active is not None and self.actionable_pillar_visible:
            # A nearby ignored block of the same color must not release the
            # foreground track. Check its best association, not every neighbor.
            active_match = self._matching_candidate(candidates, include_blocked=True)
            if active_match is not None and active_match.blocked_by_corner:
                self.release_active()

        if self.active is None:
            return self._start_track(candidates)

        match = self._matching_candidate(candidates)
        if match is not None:
            self.hit_history.append(True)
            hit_count = sum(self.hit_history)
            confirmed = self.active.confirmed or hit_count >= DETECTOR_CONFIRMATION_HITS
            alpha = 0.65
            smooth_x = int(alpha * match.center[0] + (1.0 - alpha) * self.active.center[0])
            smooth_y = int(alpha * match.center[1] + (1.0 - alpha) * self.active.center[1])
            smooth_bottom = int(alpha * match.bottom + (1.0 - alpha) * self.active.bottom)
            temporal_quality = hit_count / float(DETECTOR_TRACK_HISTORY)
            self.active = replace(
                match,
                center=(smooth_x, smooth_y),
                bottom=smooth_bottom,
                center_x_norm=smooth_x / float(frame.shape[1]),
                bottom_norm=smooth_bottom / float(frame.shape[0]),
                confidence=min(1.0, 0.75 * match.confidence + 0.25 * temporal_quality),
                track_id=self.active.track_id,
                confirmed=confirmed,
                seen_this_frame=True,
                hit_count=hit_count,
                miss_count=0,
            )
            return self.active

        self.hit_history.append(False)
        misses = self.active.miss_count + 1
        self.active = replace(
            self.active,
            confidence=max(0.0, self.active.confidence * 0.90),
            seen_this_frame=False,
            hit_count=sum(self.hit_history),
            miss_count=misses,
        )

        # An unconfirmed candidate may be replaced; a confirmed pillar remains
        # locked until the controller explicitly confirms that it was passed.
        if not self.active.confirmed and misses >= DETECTOR_UNCONFIRMED_MISSES:
            self.active = None
            return self._start_track(candidates)
        return self.active

    def commit_active_pass(self, track_id):
        if self.active is not None and self.active.track_id == track_id:
            self.passing_track_id = track_id

    def release_active(self):
        self.active = None
        self.passing_track_id = None
        self.hit_history.clear()

    def reset_navigation_context(self):
        """Discard every detection collected before a corner phase boundary."""
        self.release_active()
        self.last_candidates = []
        self.course_line_depths = {"blue": None, "orange": None}
        self.course_line_boxes = {"blue": [], "orange": []}
        self.corner_reference_y = None
        self.corner_context_current = False
        self.corner_context_confirmed = False
        self.corner_context_history.clear()
        self.ignored_corner_pillars = []
        self.corner_trigger_pillars = []
        self.actionable_pillar_visible = False
        self.last_actionable_pillar_at = 0.0

    def annotate(self, frame, state, steering, heading, sensor_data):
        display = frame.copy()
        roi_top = int(display.shape[0] * DETECTOR_ROI_TOP_RATIO)
        cv2.line(display, (0, roi_top), (display.shape[1], roi_top), (255, 255, 0), 2)
        roi_height = display.shape[0] - roi_top
        for column in range(1, DETECTOR_GRID_COLUMNS):
            x_line = column * display.shape[1] // DETECTOR_GRID_COLUMNS
            cv2.line(display, (x_line, roi_top), (x_line, display.shape[0]), (110, 110, 110), 1)
        for row in range(1, DETECTOR_GRID_ROWS):
            y_line = roi_top + row * roi_height // DETECTOR_GRID_ROWS
            cv2.line(display, (0, y_line), (display.shape[1], y_line), (110, 110, 110), 1)

        for color, boxes in self.course_line_boxes.items():
            draw_color = self.DRAW_COLORS[color]
            for x, y, width, height in boxes:
                cv2.rectangle(display, (x, y), (x + width, y + height), draw_color, 2)
        if self.corner_reference_y is not None:
            cv2.line(
                display,
                (0, self.corner_reference_y),
                (display.shape[1], self.corner_reference_y),
                (255, 0, 255),
                2,
            )

        for candidate in self.last_candidates:
            x, y, width, height = candidate.bbox
            color = self.DRAW_COLORS[candidate.color]
            cv2.rectangle(display, (x, y), (x + width, y + height), color, 1)
            if candidate.blocked_by_corner:
                cv2.rectangle(display, (x, y), (x + width, y + height), (255, 0, 255), 3)
                cv2.putText(
                    display,
                    f"IGNORE CORNER C{candidate.grid_column + 1}",
                    (max(5, x), max(25, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 0, 255),
                    2,
                )
            elif candidate.foreground_priority:
                cv2.rectangle(display, (x, y), (x + width, y + height), (0, 255, 255), 3)
                cv2.putText(
                    display,
                    f"FRONT PRIORITY C{candidate.grid_column + 1}",
                    (max(5, x), max(25, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )

        if self.corner_context_confirmed:
            cv2.putText(
                display,
                "CORNER LINES CONFIRMED - SIDE PILLAR IGNORED",
                (20, roi_top + 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 0, 255),
                2,
            )

        if self.corner_filter_suppressed():
            remaining = max(0.0, self.corner_filter_suppressed_until - time.monotonic())
            cv2.putText(
                display,
                f"POST-TURN BLOCK PRIORITY {remaining:.1f}s",
                (20, roi_top + 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )

        if self.active is not None:
            x, y, width, height = self.active.bbox
            draw_color = (0, 255, 255) if self.active.confirmed else (255, 255, 0)
            cv2.rectangle(display, (x, y), (x + width, y + height), draw_color, 3)
            label = (
                f"TRACK {self.active.track_id} {self.active.color.upper()} "
                f"conf={self.active.confidence:.2f} hits={self.active.hit_count}/5 "
                f"bottom={self.active.bottom_norm:.2f} "
                f"area={self.active.normalized_area:.4f} "
                f"{self.active.distance_band().upper()}"
            )
            cv2.putText(display, label, (max(5, x), max(25, y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, draw_color, 2)

            target_x = PILLAR_RED_TARGET_X if self.active.color == "red" else PILLAR_GREEN_TARGET_X
            target_pixel = int(target_x * display.shape[1])
            cv2.line(display, (target_pixel, 0), (target_pixel, display.shape[0]), draw_color, 2)

        heading_text = "--" if heading is None else f"{heading:.1f}"
        sensor_text = "L=-- C=-- R=--"
        if sensor_data is not None:
            _, left_value, center_value, right_value, _ = sensor_data
            sensor_text = f"L={left_value} C={center_value} R={right_value}"
        cv2.putText(display, f"STATE: {state}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
        cv2.putText(display, f"heading={heading_text} steer={steering:+.1f}", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(display, sensor_text, (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        return display


class NavigationOnlyController:
    """Non-blocking chassis controller used while vision and parking are disabled.

    Direction is decided once from several starting samples and is never changed
    during the run.  Every manoeuvre is a timed state, so the ESP32 continues to
    receive a command at least every COMMAND_REFRESH_SECONDS.
    """

    def __init__(self):
        self.state = "DIRECTION_LOCK"
        self.state_started = time.monotonic()
        self.last_command_sent = 0.0
        self.command = (0, 0, 0)
        self.last_processed_sequence = None
        self.direction_samples = []
        self.heading_samples = []
        self.corner_confirmation = 0
        self.corner_armed = True
        self.pending_target = None
        self.turn_mode = None
        self.turn_attempts = 0
        self.release_confirmation = 0
        self.resume_state = "STRAIGHT"
        self.recovery_start_center = None
        self.recovery_best_center = None
        self.recovery_reverse_applied = False
        self.sensor_hold = False
        self.sensor_hold_started = None
        self.last_notice = 0.0

    def transition(self, new_state, reason=""):
        if new_state != self.state:
            suffix = f": {reason}" if reason else ""
            print(f"[NAV] {self.state} -> {new_state}{suffix}")
        self.state = new_state
        self.state_started = time.monotonic()
        self.last_command_sent = 0.0

    def command_robot(self, speed, direction, steering, force=False):
        requested = (
            max(0, min(255, int(round(speed)))),
            1 if int(direction) == 1 else 0,
            max(-60, min(60, int(round(steering)))),
        )
        now = time.monotonic()
        if (
            force
            or requested != self.command
            or now - self.last_command_sent >= COMMAND_REFRESH_SECONDS
        ):
            send_data(*requested)
            self.last_command_sent = now
        self.command = requested

    def brake(self):
        # ESP32 v2 maps speed zero to the L293D active electrical brake.
        self.command_robot(0, 0, 0)

    def notice(self, message, period=1.0):
        now = time.monotonic()
        if now - self.last_notice >= period:
            print(message)
            self.last_notice = now

    @staticmethod
    def circular_mean_headings(headings):
        if not headings:
            return None
        sin_sum = sum(math.sin(math.radians(value)) for value in headings)
        cos_sum = sum(math.cos(math.radians(value)) for value in headings)
        return normalize_angle(math.degrees(math.atan2(sin_sum, cos_sum)))

    def heading_correction(self, heading, target=None, reverse=False):
        if target is None:
            target = target_angle
        correction = angle_diff(target, heading) * KP
        if reverse:
            correction = -correction
        return max(-HEADING_MAX_STEER, min(HEADING_MAX_STEER, correction))

    def is_new_sample(self):
        sequence = last_telemetry_status["sequence"]
        if sequence is None or sequence == self.last_processed_sequence:
            return False
        self.last_processed_sequence = sequence
        return True

    def required_navigation_sensors_valid(self, data):
        mask = last_telemetry_status["valid_mask"]
        required = VALID_HEADING | VALID_CENTER
        if self.state == "STRAIGHT":
            required |= VALID_LEFT if DIRECTION == "anticlockwise" else VALID_RIGHT
        return (
            telemetry_is_fresh()
            and (mask & required) == required
            and data[0] is not None
            and data[2] is not None
        )

    def side_is_open(self, data):
        mask = last_telemetry_status["valid_mask"]
        if DIRECTION == "anticlockwise":
            return bool(mask & VALID_LEFT) and data[1] is not None and data[1] >= CORNER_SIDE_OPEN_CM
        if DIRECTION == "clockwise":
            return bool(mask & VALID_RIGHT) and data[3] is not None and data[3] >= CORNER_SIDE_OPEN_CM
        return False

    def turn_clearance(self, data):
        # This is the opposite-side clearance used by the proven turn logic.
        mask = last_telemetry_status["valid_mask"]
        if DIRECTION == "anticlockwise":
            return data[3] if (mask & VALID_RIGHT) else None
        if DIRECTION == "clockwise":
            return data[1] if (mask & VALID_LEFT) else None
        return None

    def turn_steering(self):
        if DIRECTION == "anticlockwise":
            return TURN_STEER if self.turn_mode == "reverse" else -TURN_STEER
        return -TURN_STEER if self.turn_mode == "reverse" else TURN_STEER

    def lock_direction(self):
        global DIRECTION, target_angle

        clockwise_votes = sum(1 for difference, _ in self.direction_samples if difference < 0)
        anticlockwise_votes = sum(1 for difference, _ in self.direction_samples if difference > 0)
        if len(self.direction_samples) < DIRECTION_SAMPLE_COUNT:
            return False
        if max(clockwise_votes, anticlockwise_votes) < 7:
            return False

        DIRECTION = "anticlockwise" if anticlockwise_votes > clockwise_votes else "clockwise"
        target_angle = self.circular_mean_headings(self.heading_samples)
        print(
            f"[NAV] DIRECTION LOCKED: {DIRECTION} "
            f"({anticlockwise_votes} anticlockwise / {clockwise_votes} clockwise votes), "
            f"start heading={target_angle:.1f}"
        )
        self.transition("STRAIGHT", "direction cannot change until restart")
        return True

    def update_direction_lock(self, data, new_sample):
        global target_angle

        self.brake()
        mask = last_telemetry_status["valid_mask"]
        required = VALID_LEFT | VALID_RIGHT | VALID_HEADING
        if not telemetry_is_fresh() or (mask & required) != required:
            self.notice(
                f"[NAV] Waiting for fresh start sensors; validMask=0x{mask:02X}"
            )
            return

        if DIRECTION in ("clockwise", "anticlockwise"):
            target_angle = data[0]
            self.transition("STRAIGHT", "using direction already locked by parking exit")
            return

        if new_sample:
            difference = data[1] - data[3]
            vote = difference if abs(difference) >= DIRECTION_MIN_DIFFERENCE_CM else 0
            self.direction_samples.append((vote, data[0]))
            self.heading_samples.append(data[0])
            # Sliding window: direction must remain convincing as new samples
            # replace old ones; an early noisy vote cannot lock the whole run.
            if len(self.direction_samples) > DIRECTION_SAMPLE_COUNT:
                self.direction_samples.pop(0)
                self.heading_samples.pop(0)

        if self.lock_direction():
            return

        elapsed = time.monotonic() - self.state_started
        if elapsed >= DIRECTION_LOCK_TIMEOUT_SECONDS:
            self.notice(
                "[NAV] Need at least 7 agreeing votes in the latest 9; "
                "robot remains electrically braked",
                period=2.0,
            )

    def begin_corner(self, data):
        global target_angle

        if not self.corner_armed:
            return False
        self.corner_armed = False
        turn_delta = -90 if DIRECTION == "anticlockwise" else 90
        self.pending_target = normalize_angle(target_angle + turn_delta)
        self.corner_confirmation = 0
        self.turn_attempts = 0
        clearance = self.turn_clearance(data)

        if clearance is not None and clearance >= TURN_SIDE_CLEARANCE_CM:
            # Preserve the old manoeuvre: reverse arc where room permits.  This
            # also backs the camera away and increases its view of the new lane.
            self.turn_mode = "reverse"
            self.transition(
                "TURN",
                f"corner confirmed, reverse arc, target={self.pending_target:.1f}",
            )
        else:
            # When the opposite side is tight, first back straight, then take a
            # forward arc exactly as the previous controller intended.
            self.turn_mode = "forward"
            self.start_hard_stop_recovery(
    data,
    "TURN",
    f"single controlled pre-turn reverse, target={self.pending_target:.1f}",
)
        return True

    def start_hard_stop_recovery(self, data, resume_state, reason):
        self.resume_state = resume_state
        self.release_confirmation = 0
        self.recovery_start_center = data[2]
        self.recovery_best_center = data[2]
        self.recovery_reverse_applied = False
        self.transition("HARD_STOP_RELEASE", reason)

    def update_straight(self, data, new_sample):
        heading, _, center, _, _ = data
        hard_stop = last_telemetry_status["hard_stop"]

        if new_sample:
            if not self.corner_armed and center >= CORNER_TRIGGER_RELEASE_CM:
                self.corner_armed = True
                self.corner_confirmation = 0
                print("[NAV] Corner detector re-armed")

            if (
                self.corner_armed
                and center <= CORNER_TRIGGER_CM
                and self.side_is_open(data)
            ):
                self.corner_confirmation += 1
            else:
                # Confirmation samples must be consecutive telemetry packets.
                self.corner_confirmation = 0

        if self.corner_confirmation >= CORNER_CONFIRMATION_SAMPLES:
            self.begin_corner(data)
            return

        local_emergency = center <= PYTHON_EMERGENCY_RELEASE_CM
        if hard_stop or local_emergency:
            applied = last_telemetry_status["applied_speed"]
            print(
                f"[HARD STOP RECOVERY] center={center} cm, "
                f"ESP hardStop={int(hard_stop)}, appliedSpeed={applied}; "
                "starting reverse-and-release"
            )
            self.start_hard_stop_recovery(data, "STRAIGHT", "release front interlock")
            return

        # Slow down inside the approach band, while still triggering a corner
        # well before the ESP32's 18 cm independent safety interlock.
        drive_speed = SPEED
        if center <= CORNER_TRIGGER_RELEASE_CM:
            drive_speed = min(SPEED, 65)
        steering = self.heading_correction(heading)
        self.command_robot(drive_speed, 0, steering)

    def update_pre_turn_backup(self, data):
        heading = data[0]
        steering = self.heading_correction(heading, reverse=True)
        self.command_robot(TURNING_SPEED, 1, steering)
        if time.monotonic() - self.state_started >= PRE_TURN_BACKUP_SECONDS:
            self.transition("TURN", "backup complete")

    def finish_turn(self):
        global COUNTER, target_angle, last_turn, last_cooldown

        target_angle = self.pending_target
        COUNTER += 1
        last_turn = time.time()
        last_cooldown = time.time()
        self.pending_target = None
        self.turn_attempts = 0
        print(f"[NAV] CORNER {COUNTER}/{COUNTER_MAX} COMPLETE; target={target_angle:.1f}")
        if COUNTER >= COUNTER_MAX:
            self.transition("COMPLETE", "three laps / 12 corners complete")
            self.command_robot(0, 0, 0, force=True)
        else:
            self.transition("POST_TURN_BACKUP", "backing up to maximize camera view")

    def update_turn(self, data):
        heading = data[0]
        error = angle_diff(self.pending_target, heading)
        direction = 1 if self.turn_mode == "reverse" else 0

        if (
            last_telemetry_status["hard_stop"]
            and direction == 0
        ):
            print("[ESP HARD STOP] Forward turn suppressed; releasing before retry")
            self.start_hard_stop_recovery(data, "TURN", "release before forward-turn retry")
            return

        self.command_robot(TURNING_SPEED, direction, self.turn_steering())

        if (
            abs(error) <= TURN_HEADING_TOLERANCE
            and time.monotonic() - self.state_started >= 0.20
        ):
            self.finish_turn()
            return

        if time.monotonic() - self.state_started >= TURN_TIMEOUT_SECONDS:
            self.turn_attempts += 1
            if self.turn_attempts > MAX_RECOVERY_ATTEMPTS:
                self.transition("FAULT", "turn could not reach target after retries")
                return
            self.turn_mode = "forward" if self.turn_mode == "reverse" else "reverse"
            self.transition(
                "TURN_RETRY_PAUSE",
                f"turn timeout; retry {self.turn_attempts}/{MAX_RECOVERY_ATTEMPTS} "
                f"using {self.turn_mode} arc",
            )

    def update_post_turn_backup(self, data):
                if POST_TURN_BACKUP_SECONDS <= 0:
                    self.transition("RECENTER")
                    return

                heading = data[0]
                steering = self.heading_correction(heading, reverse=True)
                self.command_robot(TURNING_SPEED, 1, steering)

                if time.monotonic() - self.state_started >= POST_TURN_BACKUP_SECONDS:
                    self.transition("RECENTER")

    def update_recenter(self, data):
        heading = data[0]
        if last_telemetry_status["hard_stop"]:
            self.start_hard_stop_recovery(data, "RECENTER", "hard stop while recentering")
            return
        self.command_robot(min(SPEED, 75), 0, self.heading_correction(heading))
        if time.monotonic() - self.state_started >= RECENTER_SECONDS:
            self.transition("STRAIGHT")

    def update_hard_stop_release(self, data, new_sample):
        heading, _, center, _, _ = data
        elapsed = time.monotonic() - self.state_started
        self.command_robot(
            min(TURNING_SPEED, 70),
            1,
            self.heading_correction(heading, reverse=True),
        )

        if new_sample:
            self.recovery_best_center = max(self.recovery_best_center, center)
            applied_speed = last_telemetry_status["applied_speed"]
            if applied_speed is not None and applied_speed < 0:
                self.recovery_reverse_applied = True

        protocol = last_telemetry_status["protocol"]
        progress = self.recovery_best_center - self.recovery_start_center
        reverse_proven = self.recovery_reverse_applied if protocol == "v2" else progress >= HARD_STOP_MIN_PROGRESS_CM
        released = (
            not last_telemetry_status["hard_stop"]
            and center >= HARD_STOP_RELEASE_CM
            and progress >= HARD_STOP_MIN_PROGRESS_CM
            and reverse_proven
        )
        if new_sample:
            self.release_confirmation = self.release_confirmation + 1 if released else 0

        if self.release_confirmation >= HARD_STOP_RELEASE_SAMPLES:
            self.release_confirmation = 0
            if self.resume_state == "TURN" and self.pending_target is not None:
                self.turn_mode = "forward"
                self.transition("TURN", "hard-stop latch released")
            elif (
                self.resume_state == "STRAIGHT"
                and
                self.corner_armed
                and self.side_is_open(data)
                and center <= CORNER_TRIGGER_RELEASE_CM
            ):
                self.begin_corner(data)
            else:
                self.transition(self.resume_state, "hard-stop latch released")
            return

        if (
            protocol == "v2"
            and elapsed >= HARD_STOP_APPLY_GRACE_SECONDS
            and not self.recovery_reverse_applied
        ):
            self.transition("FAULT", "ESP32 did not report an applied reverse command")
            self.command_robot(0, 0, 0, force=True)
            return

        if elapsed >= HARD_STOP_MAX_REVERSE_SECONDS:
            reason = (
                f"bounded reverse ended: center improvement={progress:.1f} cm, "
                f"hardStop={int(last_telemetry_status['hard_stop'])}"
            )
            self.transition("FAULT", reason)
            self.command_robot(0, 0, 0, force=True)

    def update(self, data):
        new_sample = self.is_new_sample()

        if self.state == "DIRECTION_LOCK":
            self.update_direction_lock(data, new_sample)
            return

        if self.state in ("COMPLETE", "FAULT"):
            self.brake()
            return

        if not self.required_navigation_sensors_valid(data):
            mask = last_telemetry_status["valid_mask"]
            self.brake()
            if not self.sensor_hold:
                age = time.monotonic() - last_telemetry_status["received_at"]
                print(
                    f"[SENSOR HOLD] Braking: fresh={telemetry_is_fresh()}, "
                    f"validMask=0x{mask:02X}, telemetryAge={age:.3f}s"
                )
                self.sensor_hold = True
                self.sensor_hold_started = time.monotonic()
            return

        if self.sensor_hold:
            # No movement was commanded during the hold, so exclude that time
            # from manoeuvre timeouts and timed movement durations.
            if self.sensor_hold_started is not None:
                self.state_started += time.monotonic() - self.sensor_hold_started
            print("[SENSOR HOLD] Fresh heading and center restored; resuming state")
            self.sensor_hold = False
            self.sensor_hold_started = None

        if self.state == "STRAIGHT":
            self.update_straight(data, new_sample)
        elif self.state == "PRE_TURN_BACKUP":
            self.update_pre_turn_backup(data)
        elif self.state == "TURN":
            self.update_turn(data)
        elif self.state == "POST_TURN_BACKUP":
            self.update_post_turn_backup(data)
        elif self.state == "RECENTER":
            self.update_recenter(data)
        elif self.state == "HARD_STOP_RELEASE":
            self.update_hard_stop_release(data, new_sample)
        elif self.state == "TURN_RETRY_PAUSE":
            self.brake()
            if time.monotonic() - self.state_started >= 0.25:
                self.transition("TURN")
        else:
            self.transition("FAULT", f"unknown state {self.state}")


class ObstacleChallengeController(NavigationOnlyController):
    """Round 2 state machine using one persistent pillar track."""

    def __init__(self, detector):
        super().__init__()
        self.detector = detector
        self.state = "PARKING_EXIT_DIRECTION" if ENABLE_PARKING_EXIT else "WAIT"
        self.state_started = time.monotonic()
        self.locked_track_id = None
        self.locked_pillar_color = None
        self.pillar_was_near = False
        self.filtered_pillar_correction = 0.0
        self.last_pillar_steering = 0.0
        self.last_pillar_speed = PILLAR_MIN_PASS_SPEED
        self.pillar_lost_started_at = None
        self.last_pillar_passed_at = 0.0
        self.recenter_next_state = "FOLLOW_STRAIGHT"
        self.last_steering = 0.0
        self.vision_corner_armed = True
        self.corner_approach_from_lines = False
        self.alpha_corner_locked = False
        self.alpha_corner_reference_y = None
        self.corner_after_pillar_pending = False
        self.block_detection_suspended = bool(ENABLE_PARKING_EXIT)
        self.corner_evidence_started_at = None
        self.corner_evidence_last_at = None
        self.corner_evidence_sequence = None
        self.corner_heading_min_error = None
        self.corner_heading_max_error = None

        self.navigation_release_count = 0
        self.navigation_release_corner = COUNTER
        self.last_detection_capture_at = 0.0
        self.current_round2_sensor_data = None
        self.deep_recovery_target_heading = None
        self.deep_recovery_confirmation = 0
        self.recovery_camera_paused = False
        self.parking_reference_heading = None
        self.parking_exit_scan_count = 0
        self.parking_exit_last_capture_at = 0.0
        self.parking_exit_first_color = None
        self.parking_entry_search_active = False
        self.parking_entry_confirmation = 0
        self.parking_entry_evidence_started_at = None

        self.corner_opposite_min_cm = None
        self.initial_corner_reverse_started_at = None
        self.pillar_reverse_heading = None
        self.pillar_release_side = None
        self.pillar_resume_waiting_frame = False
        self.emergency_escape_resume_state = None
        self.emergency_escape_started = None
        self.emergency_escape_attempt = 0
        self.emergency_escape_start_center = None
        self.emergency_escape_resume_elapsed = 0.0

    def command_robot(self, speed, direction, steering, force=False):
        if (
            speed > 0
            and direction == 0
            and self.state in ("PASS_RED_RIGHT", "PASS_GREEN_LEFT", "CONFIRM_PASSED")
            and self.current_round2_sensor_data is not None
        ):
            left = self.current_round2_sensor_data[1]
            right = self.current_round2_sensor_data[3]
            left_extremely_close = _distance_is_valid(left) and left <= SIDE_NUDGE_TRIGGER_CM
            right_extremely_close = _distance_is_valid(right) and right <= SIDE_NUDGE_TRIGGER_CM
            if left_extremely_close and not right_extremely_close:
                steering = max(float(steering), SIDE_NUDGE_STEER)
            elif right_extremely_close and not left_extremely_close:
                steering = min(float(steering), -SIDE_NUDGE_STEER)
        self.last_steering = float(steering)
        super().command_robot(speed, direction, steering, force=force)

    def heading_correction(self, heading, target=None, reverse=False):
        # A clearance reverse must preserve the avoidance angle already gained.
        if reverse and self.state == "HARD_STOP_RELEASE" and self.pillar_reverse_heading is not None:
            target = self.pillar_reverse_heading
        return super().heading_correction(heading, target=target, reverse=reverse)

    def emergency_escape_steering(self, data=None):
        """Reverse-wheel command which preserves the committed forward intent."""
        state = self.emergency_escape_resume_state or self.state
        color = self.locked_pillar_color
        if data is not None:
            left, right = data[1], data[3]
            left_close = _distance_is_valid(left) and left <= EMERGENCY_ESCAPE_SIDE_AVOID_CM
            right_close = _distance_is_valid(right) and right <= EMERGENCY_ESCAPE_SIDE_AVOID_CM
            if right_close and (not left_close or right < left):
                # Reverse geometry: this sign moves the robot away from the
                # right boundary. The committed pass direction is retained.
                return 60
            if left_close and (not right_close or left < right):
                return -60
        if color == "red" or "RED" in state:
            return -60
        if color == "green" or "GREEN" in state:
            return 60
        if abs(self.last_steering) >= 5:
            return -max(-60, min(60, int(round(self.last_steering))))
        return -45 if DIRECTION == "clockwise" else 45

    def begin_emergency_escape(self, data, resume_state=None):
        now = time.monotonic()
        self.emergency_escape_resume_state = resume_state or self.state
        self.emergency_escape_resume_elapsed = max(0.0, now - self.state_started)
        self.emergency_escape_attempt = 1
        self.emergency_escape_start_center = data[2]
        self.emergency_escape_started = now
        self.state = "EMERGENCY_ESCAPE"
        self.state_started = now
        steering = self.emergency_escape_steering(data)
        print(
            f"[EMERGENCY ESCAPE] front={data[2]}cm; attempt=1/2 "
            f"reverse speed={EMERGENCY_ESCAPE_SPEED} steer={steering:+d}; "
            f"resume={self.emergency_escape_resume_state}"
        )
        self.command_robot(EMERGENCY_ESCAPE_SPEED, 1, steering, force=True)

    def finish_emergency_escape(self, reason):
        resume = self.emergency_escape_resume_state
        elapsed_before_escape = self.emergency_escape_resume_elapsed
        self.brake()
        self.state = resume
        self.state_started = time.monotonic() - elapsed_before_escape
        self.last_command_sent = 0.0
        self.emergency_escape_resume_state = None
        self.emergency_escape_started = None
        self.emergency_escape_attempt = 0
        self.emergency_escape_start_center = None
        print(f"[EMERGENCY ESCAPE] {reason}; resuming {resume}")

    def update_emergency_escape(self, data):
        now = time.monotonic()
        left, center, right = data[1], data[2], data[3]
        if not all(_distance_is_valid(value) for value in (left, center, right)):
            self.brake()
            return
        steering = self.emergency_escape_steering(data)
        if now - self.emergency_escape_started < EMERGENCY_ESCAPE_SECONDS:
            self.command_robot(EMERGENCY_ESCAPE_SPEED, 1, steering)
            return
        progress = center - self.emergency_escape_start_center
        if progress >= EMERGENCY_ESCAPE_MIN_PROGRESS_CM and center > PYTHON_EMERGENCY_RELEASE_CM:
            self.finish_emergency_escape(f"front improved by {progress:.1f}cm")
            return
        if self.emergency_escape_attempt >= EMERGENCY_ESCAPE_MAX_ATTEMPTS:
            self.brake()
            self.transition("FAULT", f"two escape pulses made only {progress:.1f}cm progress")
            return
        self.emergency_escape_attempt += 1
        self.emergency_escape_start_center = center
        self.emergency_escape_started = now
        stronger = -60 if steering < 0 else 60
        print(
            f"[EMERGENCY ESCAPE] insufficient progress={progress:.1f}cm; "
            f"attempt=2/2 reverse speed={EMERGENCY_ESCAPE_SPEED} steer={stronger:+d}"
        )
        self.command_robot(EMERGENCY_ESCAPE_SPEED, 1, stronger, force=True)


    def update_pillar_release(self, data, new_sample):
        heading, _, center, _, _ = data
        elapsed = time.monotonic() - self.state_started
        side_index = self.pillar_release_side
        if side_index is not None:
            side_bit = VALID_LEFT if side_index == 1 else VALID_RIGHT
            if not (last_telemetry_status["valid_mask"] & side_bit) or not _distance_is_valid(data[side_index]):
                self.release_confirmation = 0
                self.brake()
                return
        error = angle_diff(self.pillar_reverse_heading, heading)
        self.command_robot(PILLAR_RELEASE_SPEED, 1, self.heading_correction(heading, reverse=True))
        if new_sample:
            self.recovery_best_center = max(self.recovery_best_center, center)
            applied = last_telemetry_status["applied_speed"]
            if applied is not None and applied < 0:
                self.recovery_reverse_applied = True
            progress = center - self.recovery_start_center
            applied_ok = (self.recovery_reverse_applied if last_telemetry_status["protocol"] == "v2"
                          else progress >= HARD_STOP_MIN_PROGRESS_CM)
            ready = (not last_telemetry_status["hard_stop"] and applied_ok
                     and center >= PILLAR_RELEASE_FRONT_CM and progress >= HARD_STOP_MIN_PROGRESS_CM
                     and abs(error) <= TURN_HEADING_TOLERANCE
                     and (side_index is None or data[side_index] >= PILLAR_RELEASE_SIDE_CM))
            self.release_confirmation = self.release_confirmation + 1 if ready else 0
        if self.release_confirmation >= HARD_STOP_RELEASE_SAMPLES:
            self.recovery_camera_paused = False
            self.pillar_reverse_heading = None
            self.pillar_resume_waiting_frame = True
            self.pillar_lost_started_at = None
            self.transition(self.resume_state, "pillar recovery: front, side and heading ready")
            self.brake()
            return
        if (last_telemetry_status["protocol"] == "v2" and elapsed >= HARD_STOP_APPLY_GRACE_SECONDS
                and not self.recovery_reverse_applied):
            self.transition("FAULT", "ESP32 did not report an applied reverse command")
            self.command_robot(0, 0, 0, force=True)
        elif elapsed >= HARD_STOP_MAX_REVERSE_SECONDS:
            # Preserve the existing bounded recovery escalation; do not claim
            # success merely because the short reverse timer expired.
            self.start_deep_recovery(data, "pillar release incomplete; existing deeper recovery")


    def pause_camera_for_recovery(self):
        if self.recovery_camera_paused:
            return
        self.recovery_camera_paused = True
        if not self.block_detection_suspended:
            print("[ROUND2] CAMERA PAUSED: recovery has priority over block detection")

    def resume_camera_after_recovery(self):
        # Discard every track/history item observed before the recovery. The
        # next state accepts only a frame captured after its own transition.
        self.detector.reset_navigation_context()
        if self.block_detection_suspended:
            self.block_detection_suspended = False
            self.vision_corner_armed = True
        self.recovery_camera_paused = False
        print("[ROUND2] CAMERA RESUMED: recovery complete; waiting for a fresh frame")

    def release_locked_pillar_for_recovery(self):
        self.detector.release_active()
        self.locked_track_id = None
        self.locked_pillar_color = None
        self.pillar_lost_started_at = None
        self.pillar_was_near = False
        self.filtered_pillar_correction = 0.0

    def parking_target(self, offset):
        return normalize_angle(self.parking_reference_heading + offset)

    def parking_arc(self, data, offset, speed, direction, steering, next_state, label):
        if self.state in ("PARKING_EXIT_CW_RED_OUT", "PARKING_EXIT_CW_RED_RETURN",
                          "PARKING_EXIT_ANTI_GREEN_OUT", "PARKING_EXIT_ANTI_GREEN_RETURN"):
            if last_telemetry_status["hard_stop"] or data[2] <= PYTHON_EMERGENCY_RELEASE_CM:
                self.begin_emergency_escape(data)
                return
            if time.monotonic() - self.state_started >= 5.0:
                self.brake()
                self.transition("FAULT", "first-block arc timeout")
                return
        heading = data[0]
        target = self.parking_target(offset)
        if (
            abs(angle_diff(target, heading)) <= PARKING_HEADING_TOLERANCE
            and time.monotonic() - self.state_started >= 0.12
        ):
            self.brake()
            self.transition(next_state, f"{label} reached {target:.1f} deg")
            return
        self.command_robot(speed, direction, steering)
        if time.monotonic() - self.state_started >= PARKING_PHASE_NOTICE_SECONDS:
            self.notice(
                f"[PARKING] {label} still aligning: target={target:.1f} "
                f"heading={heading:.1f} state={self.state} "
                f"requestedPWM={speed} direction={direction} "
                f"appliedPWM={last_telemetry_status['applied_speed']} "
                f"hardStop={int(last_telemetry_status['hard_stop'])} "
                f"validMask=0x{last_telemetry_status['valid_mask']:02X} "
                f"front={data[2]}cm protocol={last_telemetry_status['protocol']}",
                period=2.0,
            )

    def resume_detection_for_parking_exit(self):
        self.detector.reset_navigation_context()
        self.detector.parking_first_scan = True
        self.detector.suppress_corner_filter(3.0)
        self.block_detection_suspended = False
        self.vision_corner_armed = False
        self.parking_exit_scan_count = 0
        self.parking_exit_last_capture_at = self.last_detection_capture_at
        print("[PARKING EXIT] CAMERA DETECTION ON: checking first pillar")

    def suspend_detection_for_parking_in(self):
        if not self.block_detection_suspended:
            self.detector.reset_navigation_context()
        self.block_detection_suspended = True
        self.locked_track_id = None
        self.locked_pillar_color = None
        self.pillar_lost_started_at = None
        print("[PARKING IN] CAMERA DETECTION OFF: parking manoeuvre locked")

    def update_parking_exit_direction(self, data, new_sample):
        global DIRECTION, target_angle

        self.brake()
        if new_sample:
            difference = data[1] - data[3]
            vote = difference if abs(difference) >= DIRECTION_MIN_DIFFERENCE_CM else 0
            self.direction_samples.append((vote, data[0]))
            self.heading_samples.append(data[0])
            if len(self.direction_samples) > DIRECTION_SAMPLE_COUNT:
                self.direction_samples.pop(0)
                self.heading_samples.pop(0)

        if len(self.direction_samples) < DIRECTION_SAMPLE_COUNT:
            return
        clockwise_votes = sum(1 for difference, _ in self.direction_samples if difference < 0)
        anticlockwise_votes = sum(1 for difference, _ in self.direction_samples if difference > 0)
        if max(clockwise_votes, anticlockwise_votes) < 7:
            self.notice("[PARKING EXIT] Need 7 agreeing direction votes; holding brake", period=2.0)
            return

        DIRECTION = "anticlockwise" if anticlockwise_votes > clockwise_votes else "clockwise"
        self.parking_reference_heading = self.circular_mean_headings(self.heading_samples)
        target_angle = self.parking_reference_heading
        print(
            f"[PARKING EXIT] DIRECTION LOCKED: {DIRECTION}; "
            f"reference heading={self.parking_reference_heading:.1f}"
        )
        self.transition("PARKING_EXIT_ARC_1", "starting historical S-curve")

    def choose_parking_exit_first_block(self, color):
        self.detector.parking_first_scan = False
        self.parking_exit_first_color = color
        chosen = color if color in ("red", "green") else "none"
        print(f"[PARKING EXIT] FIRST BLOCK: {chosen}")
        if DIRECTION == "clockwise" and color == "red":
            self.parking_curve_sign = 1
            self.parking_sequence_tick = None
            self.parking_sequence_elapsed = 0.0
            self.parking_red_reverse_started = None
            self.transition("PARKING_EXIT_TIMED_CURVE", "first red: forward curve, then reverse, then sharp left return")
        elif DIRECTION == "anticlockwise" and color == "green":
            self.parking_curve_sign = -1
            self.parking_sequence_tick = None
            self.parking_sequence_elapsed = 0.0
            self.transition("PARKING_EXIT_TIMED_CURVE", "green: -30 steering for 1s, then right 90 degrees")
        elif DIRECTION == "clockwise":
            self.transition("PARKING_EXIT_CW_RETURN_1", "green/no-block lane handoff")
        else:
            self.transition("PARKING_EXIT_ANTI_RETURN_1", "red/no-block lane handoff")

    def update_parking_exit_scan(self, detection):
        self.brake()
        self.detector.suppress_corner_filter(1.0)
        if self.last_detection_capture_at <= self.parking_exit_last_capture_at:
            return
        self.parking_exit_last_capture_at = self.last_detection_capture_at
        self.parking_exit_scan_count += 1
        if (
            detection is not None
            and detection.seen_this_frame
            and detection.color in ("red", "green")
            and getattr(detection, "confirmed", False)
        ):
            self.choose_parking_exit_first_block(detection.color)
            return
        if self.parking_exit_scan_count >= PARKING_EXIT_SCAN_FRAMES:
            self.choose_parking_exit_first_block(None)

    def complete_parking_exit(self):
        global target_angle

        target_angle = getattr(self, "parking_exit_final_heading", self.parking_reference_heading)
        self.detector.reset_navigation_context()
        self.block_detection_suspended = False
        self.corner_armed = True
        self.vision_corner_armed = True
        self.corner_confirmation = 0
        self.navigation_release_count = 0
        self.recenter_next_state = "FOLLOW_STRAIGHT"
        self.transition(
            "FOLLOW_STRAIGHT",
            f"parking exit complete; lane heading={target_angle:.1f}",
        )

    def update_parking_exit(self, data, detection, new_sample):
        state = self.state
        if state == "PARKING_EXIT_RED_FORWARD":
            now = time.monotonic()
            if last_telemetry_status["hard_stop"] or data[2] <= PYTHON_EMERGENCY_RELEASE_CM:
                self.brake()
                self.parking_sequence_tick = None
                return
            if self.parking_sequence_tick is not None:
                self.parking_sequence_elapsed += min(max(0.0, now - self.parking_sequence_tick), NAVIGATION_LOOP_SECONDS * 3)
            self.parking_sequence_tick = now
            if self.parking_sequence_elapsed >= 0.9:
                self.brake()
                self.parking_sequence_tick = None
                self.parking_sequence_elapsed = 0.0
                self.transition("PARKING_EXIT_SHARP_RETURN", "0.5s forward complete; begin sharp left return")
                return
            self.command_robot(PARKING_SPEED, 0, 0)
        elif state == "PARKING_EXIT_RED_REVERSE":
            now = time.monotonic()
            mask = last_telemetry_status["valid_mask"]
            if ((mask & (VALID_LEFT | VALID_RIGHT)) != (VALID_LEFT | VALID_RIGHT)
                    or not all(_distance_is_valid(v) and v > 5 for v in (data[1], data[3]))):
                self.brake()
                return
            if self.parking_red_reverse_started is None:
                self.parking_red_reverse_started = now
                print(f"[PARKING RED REVERSE] speed={RED_PRE_PASS_REVERSE_SPEED} reverse steer={RED_PRE_PASS_REVERSE_STEER:+d} duration={RED_PRE_PASS_REVERSE_SECONDS}s")
            if now - self.parking_red_reverse_started >= RED_PRE_PASS_REVERSE_SECONDS:
                self.brake()
                self.parking_sequence_tick = None
                self.parking_sequence_elapsed = 0.0
                self.transition("PARKING_EXIT_RED_FORWARD", "first-red reverse complete; forward straight for 0.5s")
                return
            self.command_robot(RED_PRE_PASS_REVERSE_SPEED, 1, RED_PRE_PASS_REVERSE_STEER)
        elif state in ("PARKING_EXIT_TIMED_CURVE", "PARKING_EXIT_SHARP_RETURN"):
            self.update_timed_parking_pass(data)
        elif state == "PARKING_EXIT_DIRECTION":
            self.update_parking_exit_direction(data, new_sample)
        elif state == "PARKING_EXIT_ARC_1":
            sign = 1 if DIRECTION == "clockwise" else -1
            self.parking_arc(data, 10 * sign, PARKING_SPEED, 0, 50 * sign,
                             "PARKING_EXIT_ARC_2", "exit arc 1")
        elif state == "PARKING_EXIT_ARC_2":
            sign = 1 if DIRECTION == "clockwise" else -1
            self.parking_arc(data, 25 * sign, PARKING_SPEED, 1, -50 * sign,
                             "PARKING_EXIT_ARC_3", "exit reverse arc")
        elif state == "PARKING_EXIT_ARC_3":
            sign = 1 if DIRECTION == "clockwise" else -1
            scan_offset = 35 if DIRECTION == "clockwise" else -45
            self.parking_arc(data, scan_offset, PARKING_SPEED, 0, 50 * sign,
                             "PARKING_EXIT_SCAN", "exit arc 3")
            if self.state == "PARKING_EXIT_SCAN":
                self.resume_detection_for_parking_exit()
        elif state == "PARKING_EXIT_SCAN":
            self.update_parking_exit_scan(detection)
        elif state == "PARKING_EXIT_CW_RETURN_1":
            self.parking_arc(data, 30, PARKING_SPEED, 0, -20,
                             "PARKING_EXIT_CW_RETURN_2", "clockwise lane return 1")
        elif state == "PARKING_EXIT_CW_RETURN_2":
            before = self.state
            launch_speed = (
                PARKING_EXIT_FINAL_LAUNCH_SPEED
                if time.monotonic() - self.state_started < PARKING_EXIT_FINAL_LAUNCH_SECONDS
                else PARKING_SPEED
            )
            self.parking_arc(data, 0, launch_speed, 0, -30,
                             "PARKING_EXIT_COMPLETE", "clockwise lane return 2")
            if self.state == "PARKING_EXIT_COMPLETE":
                self.complete_parking_exit()
        elif state == "PARKING_EXIT_ANTI_RETURN_1":
            self.parking_arc(data, -30, PARKING_SPEED, 0, 10,
                             "PARKING_EXIT_ANTI_RETURN_2", "anticlockwise lane return 1")
        elif state == "PARKING_EXIT_ANTI_RETURN_2":
            before = self.state
            launch_speed = (
                PARKING_EXIT_FINAL_LAUNCH_SPEED
                if time.monotonic() - self.state_started < PARKING_EXIT_FINAL_LAUNCH_SECONDS
                else PARKING_SPEED
            )
            self.parking_arc(data, 0, launch_speed, 0, 20,
                             "PARKING_EXIT_COMPLETE", "anticlockwise lane return 2")
            if self.state == "PARKING_EXIT_COMPLETE":
                self.complete_parking_exit()
        elif state == "PARKING_EXIT_CW_RED_OUT":
            self.parking_arc(data, 70, PARKING_SPEED, 0, 55,
                             "PARKING_EXIT_CW_RED_APPROACH", "clockwise red outward arc")
        elif state == "PARKING_EXIT_CW_RED_APPROACH":
            self.update_first_block_approach(data, "PARKING_EXIT_CW_RED_RETURN")
        elif state == "PARKING_EXIT_CW_RED_RETURN":
            before = self.state
            self.parking_arc(data, 0, PARKING_SPEED, 0, -50,
                             "PARKING_EXIT_COMPLETE", "clockwise red lane return")
            if self.state == "PARKING_EXIT_COMPLETE":
                self.complete_parking_exit()
        elif state == "PARKING_EXIT_ANTI_GREEN_OUT":
            self.parking_arc(data, -90, PARKING_SPEED, 0, -35,
                             "PARKING_EXIT_ANTI_GREEN_APPROACH", "anticlockwise green outward arc")
        elif state == "PARKING_EXIT_ANTI_GREEN_APPROACH":
            self.update_first_block_approach(data, "PARKING_EXIT_ANTI_GREEN_RETURN")
        elif state == "PARKING_EXIT_ANTI_GREEN_RETURN":
            before = self.state
            self.parking_arc(data, 0, PARKING_SPEED, 0, 50,
                             "PARKING_EXIT_COMPLETE", "anticlockwise green lane return")
            if self.state == "PARKING_EXIT_COMPLETE":
                self.complete_parking_exit()

    def update_timed_parking_pass(self, data):
        now = time.monotonic()
        first_red_approach = (self.state == "PARKING_EXIT_TIMED_CURVE"
                              and DIRECTION == "clockwise"
                              and self.parking_exit_first_color == "red")
        # Catch the approach before the generic forward interlock can hold it.
        if first_red_approach and (data[2] <= 45 or last_telemetry_status["hard_stop"]):
            self.brake()
            # Finish aligned with the course heading measured before parking exit.
            # Computing current_heading - 90 accumulated manoeuvre error (for
            # example 73 - 90 = -17 degrees) into all later corner targets.
            self.parking_exit_final_heading = self.parking_reference_heading
            self.parking_red_reverse_started = None
            self.parking_sequence_tick = None
            self.parking_sequence_elapsed = 0.0
            self.transition("PARKING_EXIT_RED_REVERSE", f"red approach front={data[2]}cm; reverse full right, then forward full left")
            return
        blocked = last_telemetry_status["hard_stop"] or data[2] <= PYTHON_EMERGENCY_RELEASE_CM
        if blocked:
            self.parking_sequence_tick = None
            self.begin_emergency_escape(data)
            return
        tick = self.parking_sequence_tick
        if tick is not None:
            # Do not count a sensor/camera interruption as commanded travel.
            self.parking_sequence_elapsed += min(max(0.0, now - tick), NAVIGATION_LOOP_SECONDS * 3)
        self.parking_sequence_tick = now
        sign = self.parking_curve_sign
        if first_red_approach:
            if self.parking_sequence_elapsed >= 4.0:
                self.brake()
                self.transition("FAULT", "red approach exceeded 4s without reaching distance trigger")
                return
            self.command_robot(PARKING_SPEED, 0, 30)
            return
        if self.state == "PARKING_EXIT_TIMED_CURVE":
            if self.parking_sequence_elapsed < 2.0:
                self.command_robot(PARKING_SPEED, 0, 30 * sign)
                return
            self.parking_exit_final_heading = normalize_angle(data[0] - 90 * sign)
            self.parking_sequence_elapsed = 0.0
            self.transition("PARKING_EXIT_SHARP_RETURN", f"curve complete; 90-degree return target={self.parking_exit_final_heading:.1f}")
        error = angle_diff(self.parking_exit_final_heading, data[0])
        if abs(error) <= PARKING_HEADING_TOLERANCE:
            self.brake()
            self.complete_parking_exit()
            return
        if self.parking_sequence_elapsed >= 5.0:
            self.brake()
            self.transition("FAULT", "timed parking return did not reach heading within 5s of motion")
            return
        self.command_robot(PARKING_SPEED, 0, 45 if error > 0 else -45)

    def update_first_block_approach(self, data, next_state):
        mask = last_telemetry_status["valid_mask"]
        if (mask & (VALID_LEFT | VALID_RIGHT | VALID_CENTER)) != (VALID_LEFT | VALID_RIGHT | VALID_CENTER):
            self.brake()
            return
        if last_telemetry_status["hard_stop"] or data[2] <= PYTHON_EMERGENCY_RELEASE_CM:
            self.begin_emergency_escape(data)
            return
        # Leave room for the return arc before the emergency threshold.
        if data[2] <= 40 or min(data[1], data[3]) <= 15:
            self.brake()
            self.transition(next_state, "first-block clearance boundary; start lane return")
            return
        if time.monotonic() - self.state_started >= 2.0:
            self.brake()
            self.transition("FAULT", "first-block approach timed out without clearance boundary")
            return
        self.command_robot(min(PARKING_SPEED, 70), 0, 0)

    def parking_entry_signature(self, data):
        heading_error = abs(angle_diff(self.parking_reference_heading, data[0]))
        if heading_error > PARKING_ENTRY_HEADING_TOLERANCE:
            return False
        if DIRECTION == "anticlockwise":
            return data[1] >= PARKING_ANTI_LEFT_OPEN_CM
        return data[3] <= PARKING_CLOCK_RIGHT_NEAR_CM

    def begin_parking_in(self):
        self.parking_entry_search_active = False
        self.parking_entry_confirmation = 0
        self.corner_armed = False
        self.suspend_detection_for_parking_in()
        self.brake()
        if DIRECTION == "anticlockwise":
            self.transition("PARKING_IN_ANTI_POSITION", "parking entrance confirmed")
        else:
            self.transition("PARKING_IN_CLOCK_ARC", "parking entrance confirmed")

    def update_parking_entry_search(self, data, detection, new_sample):
        actionable_detection = (
            detection is not None
            and detection.seen_this_frame
            and not detection.blocked_by_corner
        )

        if actionable_detection:
            self.parking_entry_confirmation = 0
            self.parking_entry_evidence_started_at = None
            self.enter_acquire(detection)
            return

        if self.detector.actionable_pillar_visible:
            self.parking_entry_confirmation = 0
            self.parking_entry_evidence_started_at = None
            self.brake()
            return

        if last_telemetry_status["hard_stop"] or data[2] <= PYTHON_EMERGENCY_RELEASE_CM:
            self.parking_entry_confirmation = 0
            self.parking_entry_evidence_started_at = None
            self.start_hard_stop_recovery(data, "PARKING_ENTRY_SEARCH", "release while searching for parking")
            return

        eligible = (
            time.time() - last_turn >= PARKING_ENTRY_DELAY_SECONDS
            and self.parking_entry_signature(data)
        )
        if new_sample:
            if eligible:
                if self.parking_entry_confirmation == 0:
                    self.parking_entry_evidence_started_at = time.monotonic()
                self.parking_entry_confirmation += 1
            else:
                self.parking_entry_confirmation = 0
                self.parking_entry_evidence_started_at = None

        confirmed_for = (
            time.monotonic() - self.parking_entry_evidence_started_at
            if self.parking_entry_evidence_started_at is not None else 0.0
        )
        if (
            self.parking_entry_confirmation >= PARKING_ENTRY_CONFIRMATION_SAMPLES
            and confirmed_for >= PARKING_ENTRY_CONFIRMATION_SECONDS
        ):
            print(
                f"[PARKING IN] ENTRY CONFIRMED: direction={DIRECTION} "
                f"L={data[1]} C={data[2]} R={data[3]} "
                f"headingError={angle_diff(self.parking_reference_heading, data[0]):+.1f}"
            )
            self.begin_parking_in()
            return

        self.command_robot(
            PARKING_SPEED,
            0,
            self.heading_correction(data[0], target=self.parking_reference_heading),
        )

    def complete_parking_in(self):
        self.command_robot(0, 0, 0, force=True)
        self.transition("COMPLETE", "parking manoeuvre complete")
        print("[PARKING IN] COMPLETE; electrical brake applied")

    def update_parking_in(self, data):
        state = self.state
        elapsed = time.monotonic() - self.state_started
        if state == "PARKING_IN_ANTI_POSITION":
            self.brake()
            if data[2] < 25:
                self.transition("PARKING_IN_ANTI_POSITION_BACK", "front below 25 cm")
            elif data[2] > 70:
                self.transition("PARKING_IN_ANTI_POSITION_FORWARD", "front above 70 cm")
            else:
                self.transition("PARKING_IN_ANTI_SETTLE", "front within 25-70 cm")
        elif state == "PARKING_IN_ANTI_POSITION_BACK":
            self.command_robot(75, 1, 0)
            if elapsed >= 1.5:
                self.brake()
                self.transition("PARKING_IN_ANTI_SETTLE", "1.5 s reverse adjustment complete")
        elif state == "PARKING_IN_ANTI_POSITION_FORWARD":
            self.command_robot(75, 0, 0)
            if elapsed >= 1.0:
                self.brake()
                self.transition("PARKING_IN_ANTI_SETTLE", "1.0 s forward adjustment complete")
        elif state == "PARKING_IN_ANTI_SETTLE":
            self.brake()
            if elapsed >= 1.0:
                self.transition("PARKING_IN_ANTI_REVERSE_ALIGN", "starting sensor-guided reverse")
        elif state == "PARKING_IN_ANTI_REVERSE_ALIGN":
            positioned = data[1] <= 80 and data[2] >= 80
            if positioned or elapsed >= PARKING_ANTI_REVERSE_ALIGN_TIMEOUT:
                reason = "left/front position reached" if positioned else "bounded sensor-guided reverse complete"
                self.brake()
                self.transition("PARKING_IN_ANTI_ARC", reason)
            else:
                steering = max(
                    -60,
                    min(60, -angle_diff(self.parking_reference_heading, data[0])),
                )
                self.command_robot(50, 1, steering)
        elif state == "PARKING_IN_ANTI_ARC":
            self.parking_arc(data, -80, 75, 1, 55,
                             "PARKING_IN_ANTI_STRAIGHT", "anticlockwise parking arc")
        elif state == "PARKING_IN_ANTI_STRAIGHT":
            self.command_robot(60, 1, 0)
            if elapsed >= 3.0:
                self.complete_parking_in()
        elif state == "PARKING_IN_CLOCK_ARC":
            self.parking_arc(data, -90, 80, 0, -30,
                             "PARKING_IN_CLOCK_STRAIGHT", "clockwise parking arc")
        elif state == "PARKING_IN_CLOCK_STRAIGHT":
            self.command_robot(80, 0, 0)
            if elapsed >= 2.0:
                self.complete_parking_in()

    def start_hard_stop_recovery(self, data, resume_state, reason):
        self.pillar_reverse_heading = (
            data[0] if resume_state in (
                "ACQUIRE_PILLAR", "PASS_RED_RIGHT", "PASS_GREEN_LEFT", "CONFIRM_PASSED"
            ) else None
        )
        self.pillar_release_side = None
        if self.pillar_reverse_heading is not None:
            color = self.locked_pillar_color
            if color is None:
                color = {"PASS_RED_RIGHT": "red", "PASS_GREEN_LEFT": "green"}.get(resume_state)
            side_index = 1 if color == "green" else 3 if color == "red" else None
            side_bit = VALID_LEFT if side_index == 1 else VALID_RIGHT
            if (side_index is not None and last_telemetry_status["valid_mask"] & side_bit
                    and _distance_is_valid(data[side_index]) and data[side_index] < PILLAR_RELEASE_SIDE_CM):
                self.pillar_release_side = side_index
                # Correct away from the cramped passing-side wall, by at most
                # 12 degrees towards the lane. Never overwrite the course target.
                away = 1 if side_index == 1 else -1
                lane_error = angle_diff(target_angle, data[0])
                correction = min(PILLAR_RELEASE_ANGLE, max(0, away * lane_error))
                self.pillar_reverse_heading = normalize_angle(data[0] + away * correction)
            print(f"[PILLAR RECOVERY] heading={data[0]:.1f} -> {self.pillar_reverse_heading:.1f}; "
                  f"front goal={PILLAR_RELEASE_FRONT_CM} cm, crampedSide={self.pillar_release_side}")
        self.pause_camera_for_recovery()
        # Count repeated navigation releases across state changes. Successful
        # release is not proof of forward progress. Pillar/turn retries keep
        # their existing behaviour and are not part of this budget.
        if resume_state in ("FOLLOW_STRAIGHT", "APPROACH_CORNER", "RECENTER"):
            if self.navigation_release_corner != COUNTER:
                self.navigation_release_corner = COUNTER
                self.navigation_release_count = 0
            self.navigation_release_count += 1
            if self.navigation_release_count > MAX_RECOVERY_ATTEMPTS:
                self.start_deep_recovery(
                    data,
                    "repeated navigation interlocks without completing a corner",
                )
                return
        super().start_hard_stop_recovery(
            data, resume_state,
            f"{reason}; L={data[1]} C={data[2]} R={data[3]} "
            f"headingError={angle_diff(target_angle, data[0]):+.1f} "
            f"ESPstop={int(last_telemetry_status['hard_stop'])}",
        )

    def update_hard_stop_release(self, data, new_sample):
        resume = self.resume_state
        if (any(not _distance_is_valid(v) or v <= PILLAR_RELEASE_SIDE_CM for v in (data[1], data[3]))
                or (time.monotonic() - self.state_started >= 0.4
                    and data[2] - self.recovery_start_center < HARD_STOP_MIN_PROGRESS_CM)):
            self.brake()
            self.resume_camera_after_recovery()
            self.transition("REASSESS_FRONT", "short reverse clearance/progress limit")
            return
        if resume in ("ACQUIRE_PILLAR", "PASS_RED_RIGHT", "PASS_GREEN_LEFT", "CONFIRM_PASSED"):
            self.update_pillar_release(data, new_sample)
            return
        # A reverse command which was applied but could not satisfy the short
        # release window means the robot needs room, not a terminal FAULT.
        if (
            time.monotonic() - self.state_started >= HARD_STOP_MAX_REVERSE_SECONDS
            and self.recovery_reverse_applied
        ):
            self.start_deep_recovery(data, "short hard-stop release exhausted; backing farther")
            return
        super().update_hard_stop_release(data, new_sample)
        if resume in ("FOLLOW_STRAIGHT", "APPROACH_CORNER", "RECENTER") and self.state == resume:
            self.resume_camera_after_recovery()
            self.reset_corner_evidence()
            self.transition("REASSESS_FRONT", "release complete; brake and recheck wall/block before moving")
            self.brake()
            return
        if self.state == resume:
            self.recovery_camera_paused = False

    def update_reassess_front(self, data, detection):
        self.brake()
        # Wait for the next fresh camera assessment; the update gate checks
        # camera/telemetry freshness before this method is called.
        if not last_telemetry_status["hard_stop"] and self.last_detection_capture_at > self.state_started:
            if detection is not None and detection.seen_this_frame and not detection.blocked_by_corner:
                self.deep_recovery_target_heading = None
                self.enter_acquire(detection)
                return
            if self.corner_armed and self.corner_sensors_agree(data, CORNER_TRIGGER_RELEASE_CM) and not self.detector.actionable_pillar_visible:
                self.corner_approach_from_lines = False
                self.reset_corner_evidence()
                self.deep_recovery_target_heading = None
                self.transition("APPROACH_CORNER", "post-release wall/opening agree; confirming corner")
                return
            if data[2] >= HARD_STOP_RELEASE_CM:
                self.deep_recovery_target_heading = None
                self.deep_recovery_attempts = 0
                next_state = (
                    "PARKING_ENTRY_SEARCH"
                    if self.parking_entry_search_active
                    else "FOLLOW_STRAIGHT"
                )
                self.transition(next_state, "post-release front clearance restored")
                return
        if time.monotonic() - self.state_started >= POST_RELEASE_RECHECK_SECONDS:
            if last_telemetry_status["hard_stop"] or data[2] <= PYTHON_EMERGENCY_RELEASE_CM:
                committed_state = self.resume_state or "FOLLOW_STRAIGHT"
                self.begin_emergency_escape(data, resume_state=committed_state)
                return
            self.notice("[RECOVERY HOLD] Reassessment unresolved; braked, awaiting clearance and fresh vision", period=1.0)

    def start_deep_recovery(self, data, reason):
        """Back away on the pre-corner heading instead of entering FAULT."""
        if self.state in ("HARD_STOP_RELEASE", "REASSESS_FRONT"):
            self.brake()
            self.resume_camera_after_recovery()
            self.transition("REASSESS_FRONT", "reverse already attempted; reassess without extending reverse")
            return
        # Reassessment must not reset the total reverse-attempt budget.
        if self.state == "REASSESS_FRONT":
            self.deep_recovery_attempts = getattr(self, "deep_recovery_attempts", 0) + 1
        else:
            self.deep_recovery_attempts = 1
        if self.deep_recovery_attempts > MAX_RECOVERY_ATTEMPTS:
            self.transition("FAULT", "recovery retry limit reached; inspect clearance and applied reverse")
            self.command_robot(0, 0, 0, force=True)
            return
        self.initial_corner_reverse_started_at = None
        self.pillar_reverse_heading = None
        self.pillar_resume_waiting_frame = False
        self.release_locked_pillar_for_recovery()
        self.pause_camera_for_recovery()
        self.deep_recovery_target_heading = target_angle
        self.deep_recovery_confirmation = 0
        self.deep_recovery_start_center = data[2]
        self.pending_target = None
        self.turn_mode = None
        self.turn_attempts = 0
        self.corner_armed = not self.parking_entry_search_active
        self.corner_confirmation = 0
        self.corner_approach_from_lines = False
        self.navigation_release_count = 0
        self.reset_corner_evidence()
        self.transition(
            "DEEP_REVERSE_RECOVERY",
            f"{reason}; align={self.deep_recovery_target_heading:.1f} "
            f"L={data[1]} C={data[2]} R={data[3]}",
        )
        self.brake()

    def update_deep_recovery(self, data, detection, new_sample):
        heading = data[0]
        center = data[2]
        elapsed = time.monotonic() - self.state_started
        heading_error = angle_diff(self.deep_recovery_target_heading, heading)
        aligned = abs(heading_error) <= DEEP_RECOVERY_HEADING_TOLERANCE
        front_clear = center >= DEEP_RECOVERY_FRONT_CLEARANCE_CM

        # One short reverse only; side measurements do not prove rear clearance.
        side_clear = all(_distance_is_valid(v) and v > PILLAR_RELEASE_SIDE_CM for v in (data[1], data[3]))
        progress = center - self.deep_recovery_start_center
        if center >= HARD_STOP_RELEASE_CM and not last_telemetry_status["hard_stop"]:
            self.brake()
            self.deep_recovery_target_heading = None
            self.deep_recovery_attempts = 0
            self.resume_camera_after_recovery()
            self.transition("REASSESS_FRONT", "clearance restored; no heading-chasing reverse")
            return
        if (not side_clear or elapsed >= DEEP_RECOVERY_MAX_SECONDS
                or (elapsed >= 0.4 and progress < HARD_STOP_MIN_PROGRESS_CM)):
            self.brake()
            self.resume_camera_after_recovery()
            self.transition("REASSESS_FRONT", "reverse budget/clearance/progress limit; hold for fresh assessment")
            return

        steering = self.heading_correction(
            heading, target=self.deep_recovery_target_heading, reverse=True
        )
        self.command_robot(DEEP_RECOVERY_SPEED, 1, steering)

        if new_sample:
            ready = aligned and front_clear
            self.deep_recovery_confirmation = (
                self.deep_recovery_confirmation + 1 if ready else 0
            )

        if (
            elapsed >= DEEP_RECOVERY_MIN_SECONDS
            and self.deep_recovery_confirmation >= DEEP_RECOVERY_CONFIRMATION_SAMPLES
        ):
            self.brake()
            self.resume_camera_after_recovery()
            self.transition(
                "REASSESS_FRONT",
                f"deep reverse clear and aligned; C={center} headingError={heading_error:+.1f}",
            )
            return

        if elapsed >= DEEP_RECOVERY_MAX_SECONDS:
            # Do not terminate the run.  Brake only for the fresh assessment;
            # REASSESS_FRONT can request another deeper reverse if unresolved.
            self.brake()
            self.resume_camera_after_recovery()
            self.transition(
                "REASSESS_FRONT",
                f"deep reverse timed reassessment; C={center} headingError={heading_error:+.1f}",
            )

    def suspend_block_detection(self):
        if self.block_detection_suspended:
            return
        self.block_detection_suspended = True
        self.locked_track_id = None
        self.locked_pillar_color = None
        self.pillar_lost_started_at = None
        self.filtered_pillar_correction = 0.0
        self.detector.reset_navigation_context()
        print("[ROUND2] CAMERA DETECTION OFF: corner turn and post-turn backup")

    def resume_block_detection(self):
        if not self.block_detection_suspended:
            return
        # Reset again so no track or line history from before the turn can be
        # reused when the first forward-facing frame is processed.
        self.detector.reset_navigation_context()
        self.detector.suppress_corner_filter(POST_TURN_TAPE_IGNORE_GRACE_SECONDS)
        self.block_detection_suspended = False
        self.vision_corner_armed = True
        print(
            "[ROUND2] CAMERA DETECTION ON: post-turn backup complete; "
            f"blocks override corner tapes for {POST_TURN_TAPE_IGNORE_GRACE_SECONDS:.1f}s"
        )

    def lock_direction(self):
        global DIRECTION, target_angle

        clockwise_votes = sum(1 for difference, _ in self.direction_samples if difference < 0)
        anticlockwise_votes = sum(1 for difference, _ in self.direction_samples if difference > 0)
        if len(self.direction_samples) < DIRECTION_SAMPLE_COUNT:
            return False
        if max(clockwise_votes, anticlockwise_votes) < 7:
            return False

        DIRECTION = "anticlockwise" if anticlockwise_votes > clockwise_votes else "clockwise"
        target_angle = self.circular_mean_headings(self.heading_samples)
        if self.parking_reference_heading is None:
            self.parking_reference_heading = target_angle
        print(
            f"[ROUND2] DIRECTION LOCKED: {DIRECTION} "
            f"({anticlockwise_votes} anticlockwise / {clockwise_votes} clockwise), "
            f"start heading={target_angle:.1f}"
        )
        self.transition("FOLLOW_STRAIGHT", "direction locked until restart")
        return True

    def update_detect_direction(self, data, new_sample):
        global target_angle

        self.brake()
        mask = last_telemetry_status["valid_mask"]
        required = VALID_LEFT | VALID_CENTER | VALID_RIGHT | VALID_HEADING
        if not telemetry_is_fresh() or (mask & required) != required:
            self.notice(f"[ROUND2] Waiting for start sensors; validMask=0x{mask:02X}")
            return

        if DIRECTION in ("clockwise", "anticlockwise"):
            target_angle = data[0]
            if self.parking_reference_heading is None:
                self.parking_reference_heading = target_angle
            self.transition("FOLLOW_STRAIGHT", "using direction set by parking exit")
            return

        if new_sample:
            difference = data[1] - data[3]
            vote = difference if abs(difference) >= DIRECTION_MIN_DIFFERENCE_CM else 0
            self.direction_samples.append((vote, data[0]))
            self.heading_samples.append(data[0])
            if len(self.direction_samples) > DIRECTION_SAMPLE_COUNT:
                self.direction_samples.pop(0)
                self.heading_samples.pop(0)

        if not self.lock_direction() and time.monotonic() - self.state_started >= DIRECTION_LOCK_TIMEOUT_SECONDS:
            self.notice(
                "[ROUND2] Need 7 agreeing direction votes in the latest 9; holding brake",
                period=2.0,
            )

    def corner_sensors_agree(self, data, front_limit=CORNER_TRIGGER_CM):
        # Require the end wall AND a long opening into the next lane, while
        # aligned with this straight's heading. Neither tape nor a short front
        # distance alone proves a corner. Do not use a blind-zone side value
        # as a precise component of a corridor-width calculation.
        side = {"anticlockwise": (1, VALID_LEFT), "clockwise": (3, VALID_RIGHT)}.get(DIRECTION)
        if side is None or not telemetry_is_fresh():
            return False
        side_index, _ = side
        required = VALID_CENTER | VALID_LEFT | VALID_RIGHT | VALID_HEADING
        mask = last_telemetry_status["valid_mask"]
        if (mask & required) != required:
            return False
        heading, front, opening = data[0], data[2], data[side_index]
        return (
            heading is not None
            and _distance_is_valid(front)
            and _distance_is_valid(data[1])
            and _distance_is_valid(data[3])
            and front <= front_limit
            and opening >= OBSTACLE_CORNER_SIDE_OPEN_CM
            and abs(angle_diff(target_angle, heading)) <= OBSTACLE_CORNER_HEADING_TOLERANCE
        )

    def reset_corner_evidence(self):
        self.corner_confirmation = 0
        self.corner_evidence_started_at = None
        self.corner_evidence_last_at = None
        self.corner_evidence_sequence = None
        self.corner_heading_min_error = None
        self.corner_heading_max_error = None
        self.corner_opposite_min_cm = None

    def collect_corner_evidence(self, data, new_sample):
        if (
            not self.corner_armed
            or last_telemetry_status["hard_stop"]
            or not self.corner_sensors_agree(data)
        ):
            self.reset_corner_evidence()
            return
        sequence = last_telemetry_status["sequence"]
        if not new_sample or sequence is None or sequence == self.corner_evidence_sequence:
            return
        now = time.monotonic()
        heading_error = angle_diff(target_angle, data[0])
        if (
            self.corner_confirmation == 0
            or self.corner_evidence_last_at is None
            or now - self.corner_evidence_last_at > OBSTACLE_CORNER_MAX_SAMPLE_GAP_SECONDS
        ):
            self.corner_confirmation = 0
            self.corner_evidence_started_at = now
            self.corner_heading_min_error = heading_error
            self.corner_heading_max_error = heading_error
            self.corner_opposite_min_cm = None
        min_error = min(self.corner_heading_min_error, heading_error)
        max_error = max(self.corner_heading_max_error, heading_error)
        if max_error - min_error > OBSTACLE_CORNER_HEADING_SPREAD:
            # Still inside the alignment band, but rotating: start a new proof.
            self.corner_confirmation = 0
            self.corner_evidence_started_at = now
            min_error = max_error = heading_error
            self.corner_opposite_min_cm = None
        self.corner_heading_min_error = min_error
        self.corner_heading_max_error = max_error
        self.corner_confirmation += 1
        self.corner_evidence_last_at = now
        self.corner_evidence_sequence = sequence
        # These are distinct, fresh samples which already passed both side
        # validity checks. Require clearance throughout the confirmation run.
        clearance = self.turn_clearance(data)
        self.corner_opposite_min_cm = (
            clearance if self.corner_opposite_min_cm is None
            else min(self.corner_opposite_min_cm, clearance)
        )

    def corner_evidence_confirmed(self):
        return (
            self.corner_confirmation >= OBSTACLE_CORNER_CONFIRMATION_SAMPLES
            and self.corner_evidence_started_at is not None
            and self.corner_evidence_last_at is not None
            and self.corner_evidence_last_at - self.corner_evidence_started_at
            >= OBSTACLE_CORNER_CONFIRMATION_SECONDS
            and time.monotonic() - self.corner_evidence_last_at
            <= OBSTACLE_CORNER_MAX_SAMPLE_GAP_SECONDS
        )

    def required_round2_sensors_valid(self, data):
        mask = last_telemetry_status["valid_mask"]
        required = VALID_HEADING | VALID_CENTER
        if self.state in (
            "FOLLOW_STRAIGHT",
            "APPROACH_CORNER",
            "REASSESS_FRONT",
            "PARKING_EXIT_DIRECTION",
            "PARKING_ENTRY_SEARCH",
            "PARKING_IN_ANTI_POSITION",
            "PARKING_IN_ANTI_REVERSE_ALIGN",
        ):
            required |= VALID_LEFT | VALID_RIGHT
        return (
            telemetry_is_fresh()
            and (mask & required) == required
            and data[0] is not None
            and data[2] is not None
        )

    @staticmethod
    def target_for_pillar(color):
        return PILLAR_RED_TARGET_X if color == "red" else PILLAR_GREEN_TARGET_X

    def pillar_steering(self, heading, detection):
        # Heading hold must not cancel the deliberate pillar-avoidance turn.
        heading_part = self.heading_correction(heading) * PILLAR_HEADING_WEIGHT
        if detection is None:
            self.filtered_pillar_correction *= 0.80
            return max(-60, min(60, heading_part + self.filtered_pillar_correction))

        target_x = self.target_for_pillar(detection.color)
        profile = PILLAR_DISTANCE_PROFILES[detection.distance_band()]
        if detection.color == "red":
            profile = dict(profile)
            profile["min_steer"] = {"far": 15.0, "near": 40.0, "close": 52.0, "too_close": 60.0}[detection.distance_band()]
            profile["max_steer"] = {"far": 35.0, "near": 50.0, "close": 60.0, "too_close": 60.0}[detection.distance_band()]
        # Installed servo direction: positive passes a red pillar on its right;
        # negative passes a green pillar on its left.
        half_width = detection.bbox[2] / (2.0 * CAMERA_WIDTH)
        pass_edge = detection.center_x_norm + (half_width if detection.color == "red" else -half_width)
        lateral_error = pass_edge - target_x
        raw_lateral = lateral_error * PILLAR_LATERAL_KP * profile["gain"]
        if detection.color == "red":
            raw_lateral *= PILLAR_RED_STEERING_MULTIPLIER
        if detection.color == "green":
            # Preserve the previous working controller's stronger green-block
            # correction, which compensated for the installed steering geometry.
            raw_lateral *= PILLAR_GREEN_STEERING_MULTIPLIER
        unsafe_side = (
            detection.color == "red" and lateral_error > 0.0
        ) or (
            detection.color == "green" and lateral_error < 0.0
        )
        # Never let image error reverse the required pass side. Once a red
        # pillar is already left of its target (or green is right of its
        # target), heading hold may straighten the robot but must not steer it
        # back toward the pillar.
        if detection.color == "red":
            raw_lateral = max(0.0, raw_lateral)
            if unsafe_side:
                raw_lateral = max(profile["min_steer"], raw_lateral)
        else:
            raw_lateral = min(0.0, raw_lateral)
            if unsafe_side:
                raw_lateral = min(-profile["min_steer"], raw_lateral)
        raw_lateral = max(
            -profile["max_steer"],
            min(profile["max_steer"], raw_lateral),
        )
        proximity = detection.proximity()
        bottom_weight = PILLAR_MIN_CONTROL_WEIGHT + (
            1.0 - PILLAR_MIN_CONTROL_WEIGHT
        ) * proximity
        control_weight = max(profile["min_weight"], bottom_weight)
        weighted_lateral = raw_lateral * control_weight
        if not detection.seen_this_frame:
            weighted_lateral = self.filtered_pillar_correction * 0.90
        alpha = profile["smoothing"]
        self.filtered_pillar_correction = (
            alpha * weighted_lateral + (1.0 - alpha) * self.filtered_pillar_correction
        )
        steering = heading_part + self.filtered_pillar_correction
        # Preserve avoidance authority after filtering and heading hold.
        if detection.seen_this_frame and unsafe_side:
            if detection.color == "red":
                steering = max(profile["min_steer"], steering)
            else:
                steering = min(-profile["min_steer"], steering)
        # During an active pass, heading hold must never reverse the required
        # avoidance direction, even when the block is already on the safe side.
        steering = max(0.0, steering) if detection.color == "red" else min(0.0, steering)
        return max(-60, min(60, steering))

    @staticmethod
    def pillar_speed(detection):
        proximity = detection.proximity() if detection is not None else 0.0
        speed = PILLAR_APPROACH_SPEED - (
            PILLAR_APPROACH_SPEED - PILLAR_MIN_PASS_SPEED
        ) * proximity
        if detection is not None:
            area_speed_limit = PILLAR_DISTANCE_PROFILES[detection.distance_band()]["speed"]
            speed = min(speed, area_speed_limit)
        return max(PILLAR_MIN_PASS_SPEED, min(PILLAR_PASS_SPEED, int(round(speed))))

    def command_pillar_pass(self, heading, detection, force=False):
        self.detector.commit_active_pass(detection.track_id)
        steering = self.pillar_steering(heading, detection)
        speed = self.pillar_speed(detection)
        self.locked_pillar_color = detection.color
        self.last_pillar_steering = steering
        self.last_pillar_speed = speed
        self.command_robot(speed, 0, steering, force=force)
        return speed, steering

    def enter_acquire(self, detection):
        if self.state == "APPROACH_CORNER":
            self.corner_after_pillar_pending = True
        self.red_pre_reverse_started = None
        self.red_pre_reverse_done = False
        self.detector.commit_active_pass(detection.track_id)
        self.locked_track_id = detection.track_id
        self.locked_pillar_color = detection.color
        self.pillar_was_near = detection.bottom_norm >= PILLAR_NEAR_BOTTOM
        self.pillar_lost_started_at = None
        self.filtered_pillar_correction = 0.0
        self.last_pillar_steering = 0.0
        self.last_pillar_speed = PILLAR_MIN_PASS_SPEED
        self.transition(
            "ACQUIRE_PILLAR",
            f"track={detection.track_id} color={detection.color} confidence={detection.confidence:.2f}",
        )


    def alpha_position_detected(self, detection):
        if self.last_pillar_passed_at <= 0.0:
            return False
        if time.monotonic() - self.last_pillar_passed_at > ALPHA_POSITION_WINDOW_SECONDS:
            return False

        both_tapes_visible = (
            self.detector.course_line_depths["blue"] is not None
            and self.detector.course_line_depths["orange"] is not None
        )
        if not both_tapes_visible:
            return False

        # Match the previous working controller's 4,000-pixel precedence:
        # Alpha is only valid while no near, actionable foreground pillar needs
        # to be passed before the corner.
        actionable_near_pillar = any(
            item.color in ("red", "green")
            and not item.blocked_by_corner
            and item.normalized_area >= PILLAR_AREA_NEAR_RATIO
            and (
                self.detector.corner_reference_y is None
                or item.center[1] > self.detector.corner_reference_y
            )
            for item in self.detector.last_candidates
        )
        if actionable_near_pillar:
            return False

        tracked_pillar_visible = (
            detection is not None
            and detection.seen_this_frame
            and detection.color in ("red", "green")
        )
        raw_pillar_visible = any(
            item.color in ("red", "green")
            for item in self.detector.last_candidates
        )
        return tracked_pillar_visible or raw_pillar_visible

    def enter_line_corner_approach(self, alpha_position=False):
        ignored_columns = sorted(
            {item.grid_column + 1 for item in self.detector.corner_trigger_pillars}
        )
        blue_depth = self.detector.course_line_depths["blue"]
        orange_depth = self.detector.course_line_depths["orange"]
        self.vision_corner_armed = False
        self.corner_approach_from_lines = True
        if alpha_position:
            self.alpha_corner_locked = True
            if self.detector.corner_reference_y is not None:
                self.alpha_corner_reference_y = self.detector.corner_reference_y
            print("[ROUND2] ALPHA CORNER LOCKED: background pillars deferred until recenter")
        self.corner_confirmation = 0
        self.locked_track_id = None
        self.filtered_pillar_correction = 0.0
        self.detector.release_active()
        reason_prefix = "ALPHA POSITION DETECTED; corner sequence first; " if alpha_position else ""
        self.transition(
            "APPROACH_CORNER",
            f"{reason_prefix}blue={blue_depth} orange={orange_depth} "
            f"ignoredGridColumns={ignored_columns}",
        )

    def update_follow_straight(self, data, detection, new_sample):
        heading, _, center, _, _ = data
        hard_stop = last_telemetry_status["hard_stop"]

        if new_sample and not self.corner_armed and center >= CORNER_TRIGGER_RELEASE_CM:
            self.corner_armed = True
            self.corner_confirmation = 0
            print("[ROUND2] Corner detector re-armed")

        if detection is not None and not detection.seen_this_frame and detection.miss_count >= PILLAR_REQUIRED_LOST_FRAMES:
            self.detector.release_active()
            detection = None

        # Immediately after clearing a foreground pillar, remembered corner
        # evidence owns the next decision. A pillar beyond the tapes belongs
        # to the new lane and must be acquired only after the 90-degree turn.
        recently_passed_pillar = (
            self.last_pillar_passed_at > 0.0
            and time.monotonic() - self.last_pillar_passed_at <= ALPHA_POSITION_WINDOW_SECONDS
        )
        both_tapes_visible = (
            self.detector.course_line_depths["blue"] is not None
            and self.detector.course_line_depths["orange"] is not None
        )
        if (
            self.corner_armed
            and recently_passed_pillar
            and (self.corner_after_pillar_pending or both_tapes_visible)
        ):
            self.corner_after_pillar_pending = False
            self.enter_line_corner_approach(alpha_position=True)
            return

        if (
            detection is not None
            and detection.seen_this_frame
            and not detection.blocked_by_corner
            and time.monotonic() - self.last_pillar_passed_at >= PILLAR_REACQUIRE_COOLDOWN_SECONDS
        ):
            self.enter_acquire(detection)
            return

        if hard_stop or center <= PYTHON_EMERGENCY_RELEASE_CM:
            self.start_hard_stop_recovery(data, "FOLLOW_STRAIGHT", "release front interlock")
            return

        # Match the classic controller's `block[0] == None` turn veto. A brief
        # memory covers camera-frame gaps so a pillar cannot become a wall turn.
        if (
            center <= CORNER_TRIGGER_RELEASE_CM
            and self.detector.actionable_pillar_recently_seen()
            and (
                self.detector.actionable_pillar_visible
                or not self.corner_sensors_agree(data, CORNER_TRIGGER_RELEASE_CM)
            )
        ):
            self.corner_confirmation = 0
            if recently_passed_pillar:
                self.corner_after_pillar_pending = True
            self.brake()
            self.notice(
                f"[ROUND2] Corner vetoed by recent pillar; center={center} cm",
                period=0.5,
            )
            return

        if self.corner_armed and self.corner_sensors_agree(data, CORNER_TRIGGER_RELEASE_CM):
            self.corner_approach_from_lines = False
            self.corner_confirmation = 0
            self.transition("APPROACH_CORNER", f"front + turning-side TF-Luna agree; center={center} cm")
            return

        # Keep the existing approach speed near a wall even if it is not a corner.
        drive_speed = min(SPEED, 65) if center <= CORNER_TRIGGER_RELEASE_CM else SPEED
        self.command_robot(drive_speed, 0, self.heading_correction(heading))

    def update_acquire_pillar(self, data, detection):
        heading = data[0]
        elapsed = time.monotonic() - self.state_started

        if (
            detection is None
            or detection.track_id != self.locked_track_id
            or not detection.seen_this_frame
        ):
            self.command_robot(min(SPEED, PILLAR_APPROACH_SPEED), 0, self.heading_correction(heading))
            if elapsed >= PILLAR_ACQUIRE_TIMEOUT_SECONDS:
                self.detector.release_active()
                self.transition("FOLLOW_STRAIGHT", "candidate did not confirm")
            return

        if detection.confirmed:
            # Detection confirmation means the block is real, not yet close
            # enough to turn around. The classic block controller started
            # avoidance at 4,000 pixels; use its resolution-normalized threshold.
            if detection.normalized_area < PILLAR_AREA_NEAR_RATIO * 0.60:
                self.command_robot(
                    min(SPEED, PILLAR_PASS_SPEED), 0, self.heading_correction(heading)
                )
                self.notice(
                    f"[ROUND2] APPROACH PILLAR: {detection.color} "
                    f"track={detection.track_id} area={detection.normalized_area:.4f}; "
                    "holding course until near",
                    period=1.0,
                )
                return
            next_state = "PASS_RED_RIGHT" if detection.color == "red" else "PASS_GREEN_LEFT"
            self.transition(next_state, f"track={detection.track_id} confirmed {detection.hit_count}/5")
            speed, steering = self.command_pillar_pass(heading, detection, force=True)
            print(
                f"[ROUND2] PASS COMMAND: {detection.color} "
                f"distance={detection.distance_band()} "
                f"area={detection.normalized_area:.4f} "
                f"speed={speed} steer={steering:+.1f}"
            )
            return

        # A candidate cannot influence steering until it has appeared in at
        # least three of the latest five distinct camera frames.
        self.command_robot(
            min(SPEED, PILLAR_APPROACH_SPEED),
            0,
            self.heading_correction(heading),
        )

        if elapsed >= PILLAR_ACQUIRE_TIMEOUT_SECONDS:
            self.detector.release_active()
            self.transition("FOLLOW_STRAIGHT", "three-of-five confirmation timeout")

    def update_pass_pillar(self, data, detection):
        heading, _, center, _, _ = data
        now = time.monotonic()
        elapsed = now - self.state_started

        same_track = detection is not None and detection.track_id == self.locked_track_id
        if not same_track or not detection.seen_this_frame:
            if self.pillar_lost_started_at is None:
                self.pillar_lost_started_at = now
            self.transition("CONFIRM_PASSED", "pillar left view; short clearance movement")
            self.update_confirm_passed(data, detection)
            return

        self.pillar_lost_started_at = None

        if detection.bottom_norm >= PILLAR_NEAR_BOTTOM:
            self.pillar_was_near = True

        self.command_pillar_pass(heading, detection)

        # Keep using the four bands for this pillar while it is visible.

    def update_confirm_passed(self, data, detection):
        heading, _, center, _, _ = data
        elapsed = time.monotonic() - self.state_started
        same_track = detection is not None and detection.track_id == self.locked_track_id

        # A visible tracked pillar is still being negotiated. Neither a front
        # wall reading nor a timeout proves that the chassis has cleared it.
        if same_track and detection.seen_this_frame:
            return_state = "PASS_RED_RIGHT" if detection.color == "red" else "PASS_GREEN_LEFT"
            self.transition(return_state, "pillar still visible; continue clearance")
            self.pillar_lost_started_at = None
            self.command_pillar_pass(heading, detection, force=True)
            return

        if elapsed < PILLAR_CLEARANCE_HOLD_SECONDS:
            self.command_robot(self.last_pillar_speed, 0, self.last_pillar_steering)
            return

        if elapsed >= PILLAR_CLEARANCE_HOLD_SECONDS:
            color = self.locked_pillar_color
            print(f"[ROUND2] PILLAR PASSED: {color}, track={self.locked_track_id}")
            self.detector.release_active()
            self.locked_track_id = None
            self.locked_pillar_color = None
            self.pillar_lost_started_at = None
            self.last_pillar_passed_at = time.monotonic()
            self.filtered_pillar_correction = 0.0
            self.navigation_release_count = 0
            self.deep_recovery_attempts = 0
            if self.corner_after_pillar_pending and self.corner_armed:
                self.corner_after_pillar_pending = False
                self.enter_line_corner_approach(alpha_position=True)
                self.command_robot(self.last_pillar_speed, 0, 0, force=True)
                return
            self.transition("FOLLOW_STRAIGHT", "pillar clearance complete; continue navigation")
            self.command_robot(self.last_pillar_speed, 0, 0, force=True)
            return

    def update_approach_corner(self, data, detection, new_sample):
        heading, _, center, _, _ = data
        elapsed = time.monotonic() - self.state_started

        # The old main loop reached TURN PRECEDENCE only when there was no
        # actionable block above 4,000 pixels. Preserve that safety rule during
        # non-blocking line/TF-Luna approach: a near foreground pillar must be
        # passed before continuing toward the corner.
        near_foreground_pillar = (
            detection is not None
            and detection.seen_this_frame
            and not detection.blocked_by_corner
            and detection.normalized_area >= PILLAR_AREA_NEAR_RATIO
        )
        if near_foreground_pillar and self.alpha_corner_locked:
            # Keep using the tape depth captured when alpha was locked. The
            # live tape mask can briefly disappear during approach; a pillar
            # whose centre is below that line is physically before the corner,
            # while a pillar above it remains deferred to the new lane.
            near_foreground_pillar = (
                (
                    self.alpha_corner_reference_y is not None
                    and detection.center[1] > self.alpha_corner_reference_y
                )
                or detection.foreground_priority
                or detection.bottom_norm >= PILLAR_NEAR_BOTTOM
            )
        if near_foreground_pillar:
            self.corner_after_pillar_pending = True
            self.corner_approach_from_lines = False
            self.corner_confirmation = 0
            self.vision_corner_armed = False
            self.enter_acquire(detection)
            print(
                f"[ROUND2] CORNER PAUSED FOR FOREGROUND PILLAR: "
                f"track={detection.track_id} color={detection.color} "
                f"area={detection.normalized_area:.4f}"
            )
            return

        if (
            not self.corner_approach_from_lines
            and detection is not None
            and detection.seen_this_frame
            and time.monotonic() - self.last_pillar_passed_at >= PILLAR_REACQUIRE_COOLDOWN_SECONDS
        ):
            self.enter_acquire(detection)
            return

        if (
            not self.corner_approach_from_lines
            and self.detector.actionable_pillar_recently_seen()
            and (
                self.detector.actionable_pillar_visible
                or not self.corner_sensors_agree(data, CORNER_TRIGGER_RELEASE_CM)
            )
        ):
            self.corner_confirmation = 0
            self.brake()
            self.transition("FOLLOW_STRAIGHT", "recent pillar vetoed corner approach")
            return

        self.collect_corner_evidence(data, new_sample)

        if self.corner_evidence_confirmed():
            self.begin_corner(data)
            return

        if last_telemetry_status["hard_stop"] or center <= PYTHON_EMERGENCY_RELEASE_CM:
            self.reset_corner_evidence()
            self.start_hard_stop_recovery(data, "APPROACH_CORNER", "release during corner approach")
            return

        if (
            not self.corner_approach_from_lines
            and not self.corner_sensors_agree(data, CORNER_TRIGGER_RELEASE_CM + 15)
        ):
            self.corner_confirmation = 0
            self.transition("FOLLOW_STRAIGHT", "two-sensor corner evidence cleared")
            return

        approach_timeout = (
            VISION_CORNER_APPROACH_TIMEOUT_SECONDS
            if self.corner_approach_from_lines
            else APPROACH_CORNER_TIMEOUT_SECONDS
        )
        if elapsed >= approach_timeout:
            if self.corner_approach_from_lines:
                if self.alpha_corner_locked:
                    # Both tapes plus the just-passed foreground pillar already
                    # committed this cycle to the corner. Do not let a temporary
                    # TF-Luna confirmation delay hand control to the background
                    # pillar; keep approaching and collect a fresh evidence run.
                    self.corner_confirmation = 0
                    self.state_started = time.monotonic()
                    self.notice(
                        "[ROUND2] ALPHA CORNER RETAINED: awaiting TF-Luna confirmation",
                        period=1.0,
                    )
                    self.command_robot(min(SPEED, 80), 0, self.heading_correction(heading))
                    return
                self.corner_approach_from_lines = False
                self.corner_confirmation = 0
                self.transition(
                    "FOLLOW_STRAIGHT",
                    "vision corner not confirmed; continuing sensor navigation",
                )
                return
            self.transition("FAULT", "corner opening not confirmed before approach timeout")
            self.command_robot(0, 0, 0, force=True)
            return

        if self.corner_confirmation > 0:
            # Do not creep into the wall while collecting the confirmation run.
            self.brake()
            return
        self.command_robot(min(SPEED, 65), 0, self.heading_correction(heading))

    def begin_corner(self, data):
        global target_angle

        if (
            not self.corner_armed
            or not self.corner_evidence_confirmed()
            or last_telemetry_status["hard_stop"]
            or not self.corner_sensors_agree(data)
        ):
            return False
        print(
            f"[ROUND2] CORNER CONFIRMED: L={data[1]} C={data[2]} R={data[3]} cm; "
            f"headingError={angle_diff(target_angle, data[0]):+.1f} deg; "
            f"samples={self.corner_confirmation}"
        )
        clearance = self.turn_clearance(data)
        use_reverse = (
            CORNER_REVERSE_MAX_SECONDS > 0
            and self.corner_opposite_min_cm is not None
            and self.corner_opposite_min_cm >= CORNER_REVERSE_CLEARANCE_CM
            and _distance_is_valid(clearance)
            and clearance >= CORNER_REVERSE_CLEARANCE_CM
        )
        self.suspend_block_detection()
        self.corner_armed = False
        self.corner_confirmation = 0
        self.corner_approach_from_lines = False
        self.pending_target = normalize_angle(
            target_angle + (-90 if DIRECTION == "anticlockwise" else 90)
        )
        self.turn_attempts = 0
        self.recenter_next_state = "FOLLOW_STRAIGHT"
        self.turn_mode = "reverse" if use_reverse else "forward"
        self.initial_corner_reverse_started_at = time.monotonic() if use_reverse else None
        self.transition("TURN_90", f"initial {self.turn_mode} turn, "
                        f"oppositeMin={self.corner_opposite_min_cm} cm, "
                        f"target={self.pending_target:.1f}")
        return True

    def finish_turn(self):
        global COUNTER, target_angle, last_turn, last_cooldown

        self.initial_corner_reverse_started_at = None
        target_angle = self.pending_target
        COUNTER += 1
        last_turn = time.time()
        last_cooldown = time.time()
        self.pending_target = None
        self.turn_attempts = 0
        print(f"[NAV] CORNER {COUNTER}/{COUNTER_MAX} COMPLETE; target={target_angle:.1f}")
        if COUNTER >= COUNTER_MAX:
            if ENABLE_PARKING_IN:
                self.parking_entry_search_active = True
                self.corner_armed = False
                self.vision_corner_armed = False
                self.recenter_next_state = "PARKING_ENTRY_SEARCH"
                self.transition(
                    "POST_TURN_BACKUP",
                    "final corner complete; backing up before parking-entry search",
                )
            else:
                self.transition("COMPLETE", "three laps / 12 corners complete")
                self.command_robot(0, 0, 0, force=True)
        else:
            self.transition("POST_TURN_BACKUP", "backing up to maximize camera view")

    def update_turn_90(self, data):
        heading = data[0]
        error = angle_diff(self.pending_target, heading)
        now = time.monotonic()
        if abs(error) <= TURN_HEADING_TOLERANCE and now - self.state_started >= 0.20:
            self.finish_turn()
            return

        if (
            self.initial_corner_reverse_started_at is not None
            and now - self.initial_corner_reverse_started_at >= CORNER_REVERSE_MAX_SECONDS
        ):
            self.initial_corner_reverse_started_at = None
            self.turn_mode = "forward"
            # No brake, retry pause or new target: issue the forward arc below.
            print(f"[ROUND2] REVERSE -> FORWARD: time limit reached; "
                  f"heading={heading:.1f}, remaining={error:+.1f}, "
                  f"target={self.pending_target:.1f}")
            self.transition("TURN_90")
        direction = 1 if self.turn_mode == "reverse" else 0

        if last_telemetry_status["hard_stop"] and direction == 0:
            # Interlock releases used to reset state_started indefinitely, so
            # the existing turn timeout/retry limit never caught this loop.
            self.turn_attempts += 1
            if self.turn_attempts > MAX_RECOVERY_ATTEMPTS:
                self.start_deep_recovery(
                    data,
                    "corner blocked by repeated front interlocks; abandoning incomplete turn",
                )
                return
            self.start_hard_stop_recovery(data, "TURN_90", "release before forward turn")
            return

        if self.initial_corner_reverse_started_at is not None:
            steering = (CORNER_REVERSE_STEER if DIRECTION == "anticlockwise"
                        else -CORNER_REVERSE_STEER)
            self.command_robot(CORNER_REVERSE_SPEED, 1, steering)
            return

        self.command_robot(TURNING_SPEED, direction, self.turn_steering())

        if time.monotonic() - self.state_started >= TURN_TIMEOUT_SECONDS:
            self.turn_attempts += 1
            if self.turn_attempts > MAX_RECOVERY_ATTEMPTS:
                self.start_deep_recovery(
                    data,
                    "90-degree turn retries exhausted; abandoning incomplete turn",
                )
                return
            self.turn_mode = "forward" if self.turn_mode == "reverse" else "reverse"
            self.transition(
                "TURN_RETRY_PAUSE",
                f"turn retry {self.turn_attempts}/{MAX_RECOVERY_ATTEMPTS} using {self.turn_mode}",
            )

    def update_post_turn_backup(self, data):
        if POST_TURN_BACKUP_SECONDS <= 0:
            self.transition("RECENTER")
            self.resume_block_detection()
            return

        heading = data[0]
        steering = self.heading_correction(heading, reverse=True)
        self.command_robot(TURNING_SPEED, 1, steering)

        if time.monotonic() - self.state_started >= POST_TURN_BACKUP_SECONDS:
            self.transition("RECENTER")
            self.resume_block_detection()

    def update_recenter_round2(self, data):
        heading = data[0]
        if last_telemetry_status["hard_stop"]:
            self.start_hard_stop_recovery(data, "RECENTER", "front interlock while recentering")
            return
        self.command_robot(min(SPEED, 75), 0, self.heading_correction(heading))
        if time.monotonic() - self.state_started >= RECENTER_SECONDS:
            next_state = (
                "PARKING_ENTRY_SEARCH"
                if self.parking_entry_search_active
                else self.recenter_next_state
            )
            if self.alpha_corner_locked:
                self.alpha_corner_locked = False
                self.alpha_corner_reference_y = None
                self.corner_after_pillar_pending = False
                print("[ROUND2] ALPHA CORNER RELEASED: turn and recenter complete")
            self.transition(next_state)

    def update(self, data, detection, camera_fresh):
        new_sample = self.is_new_sample()
        self.current_round2_sensor_data = data

        if self.state == "WAIT":
            self.brake()
            required = VALID_LEFT | VALID_CENTER | VALID_RIGHT | VALID_HEADING
            mask = last_telemetry_status["valid_mask"]
            if camera_fresh and telemetry_is_fresh() and (mask & required) == required:
                self.transition("DETECT_DIRECTION", "camera and sensors ready")
            return

        if self.state == "DETECT_DIRECTION":
            if not camera_fresh:
                self.brake()
                self.notice("[ROUND2] Waiting for fresh camera frames")
                return
            self.update_detect_direction(data, new_sample)
            return

        if self.state in ("COMPLETE", "FAULT"):
            self.brake()
            return

        camera_required = (
            not self.block_detection_suspended
            and not self.recovery_camera_paused
            and self.state not in ("HARD_STOP_RELEASE", "POST_TURN_BACKUP")
        )
        if not self.required_round2_sensors_valid(data) or (camera_required and not camera_fresh):
            if self.state == "APPROACH_CORNER":
                # Missing data breaks the consecutive corner-confirmation run.
                self.corner_confirmation = 0
            mask = last_telemetry_status["valid_mask"]
            self.brake()
            if not self.sensor_hold:
                print(
                    f"[ROUND2 SENSOR HOLD] state={self.state} cameraFresh={camera_fresh} "
                    f"validMask=0x{mask:02X}"
                )
                self.sensor_hold = True
                self.sensor_hold_started = time.monotonic()
            return

        if self.sensor_hold:
            if self.sensor_hold_started is not None:
                self.state_started += time.monotonic() - self.sensor_hold_started
            print(f"[ROUND2 SENSOR HOLD] restored; resuming {self.state}")
            self.sensor_hold = False
            self.sensor_hold_started = None

        # Continue a previously requested bounded escape. New escapes are
        # started only inside branches that would otherwise hold or fault.
        if self.state == "EMERGENCY_ESCAPE":
            self.update_emergency_escape(data)
            return

        if self.state.startswith("PARKING_EXIT_"):
            self.update_parking_exit(data, detection, new_sample)
            return

        if self.state == "PARKING_ENTRY_SEARCH":
            self.update_parking_entry_search(data, detection, new_sample)
            return

        if self.state.startswith("PARKING_IN_"):
            self.update_parking_in(data)
            return

        if self.state == "REASSESS_FRONT":
            self.update_reassess_front(data, detection)
            return

        if self.pillar_resume_waiting_frame:
            if self.last_detection_capture_at <= self.state_started:
                self.brake()
                return
            self.pillar_resume_waiting_frame = False

        if self.state == "DEEP_REVERSE_RECOVERY":
            self.update_deep_recovery(data, detection, new_sample)
            return

        # A selected pillar owns control through clearance. Emergency braking
        # retains its identity and never triggers an automatic reverse.
        if self.state in ("ACQUIRE_PILLAR", "PASS_RED_RIGHT", "PASS_GREEN_LEFT", "CONFIRM_PASSED"):
            # A track that is geometrically beyond both course tapes belongs
            # to the lane after the corner. This check remains valid even while
            # the detector's short post-turn corner filter is suppressed.
            if self.alpha_position_detected(detection):
                self.corner_after_pillar_pending = False
                self.enter_line_corner_approach(alpha_position=True)
                return
            if last_telemetry_status["hard_stop"] or data[2] <= PYTHON_EMERGENCY_RELEASE_CM:
                self.begin_emergency_escape(data)
                return
            if getattr(self, "pillar_front_hold", False):
                if data[2] < HARD_STOP_RELEASE_CM or self.last_detection_capture_at <= self.pillar_hold_capture_at:
                    self.brake()
                    return
                self.pillar_front_hold = False
                self.pillar_lost_started_at = None
                self.state_started = time.monotonic()
            if self.state == "ACQUIRE_PILLAR":
                self.update_acquire_pillar(data, detection)
            elif self.state == "CONFIRM_PASSED":
                self.update_confirm_passed(data, detection)
            else:
                self.update_pass_pillar(data, detection)
            return

        if self.state == "FOLLOW_STRAIGHT" and not self.detector.corner_context_current:
            self.vision_corner_armed = True

        # Alpha position: immediately after passing one pillar, a second pillar
        # together with both tape lines belongs beyond the corner. Drop the new
        # pillar acquisition and let the existing TF-Luna-confirmed corner
        # approach/turn run first. Detection resumes after turn and backup.
        if (
            self.corner_armed
            and self.state
            in (
                "FOLLOW_STRAIGHT",
                "ACQUIRE_PILLAR",
                "PASS_RED_RIGHT",
                "PASS_GREEN_LEFT",
                "CONFIRM_PASSED",
            )
            and self.alpha_position_detected(detection)
        ):
            self.enter_line_corner_approach(alpha_position=True)
            return

        if (
            self.vision_corner_armed
            and self.corner_armed
            and self.detector.corner_context_confirmed
            and self.state
            in (
                "FOLLOW_STRAIGHT",
                "ACQUIRE_PILLAR",
            )
        ):
            self.enter_line_corner_approach()
            return

        # Any forward-driving state can encounter an obstacle. The ESP32
        # independently brakes at 18 cm; Python immediately acknowledges that
        # suppression and performs the same bounded reverse-release recovery.
        if (
            self.state
            in (
                "FOLLOW_STRAIGHT",
                "ACQUIRE_PILLAR",
                "PASS_RED_RIGHT",
                "PASS_GREEN_LEFT",
                "CONFIRM_PASSED",
                "RECENTER",
                "APPROACH_CORNER",
            )
            and (
                last_telemetry_status["hard_stop"]
                or data[2] <= PYTHON_EMERGENCY_RELEASE_CM
            )
        ):
            if self.state == "APPROACH_CORNER":
                self.reset_corner_evidence()
            self.start_hard_stop_recovery(
                data,
                self.state,
                f"front interlock while in {self.state}",
            )
            return

        if self.state == "FOLLOW_STRAIGHT":
            self.update_follow_straight(data, detection, new_sample)
        elif self.state == "ACQUIRE_PILLAR":
            self.update_acquire_pillar(data, detection)
        elif self.state in ("PASS_RED_RIGHT", "PASS_GREEN_LEFT"):
            self.update_pass_pillar(data, detection)
        elif self.state == "CONFIRM_PASSED":
            self.update_confirm_passed(data, detection)
        elif self.state == "RECENTER":
            self.update_recenter_round2(data)
        elif self.state == "APPROACH_CORNER":
            self.update_approach_corner(data, detection, new_sample)
        elif self.state == "TURN_90":
            self.update_turn_90(data)
        elif self.state == "POST_TURN_BACKUP":
            self.update_post_turn_backup(data)
        elif self.state == "HARD_STOP_RELEASE":
            self.update_hard_stop_release(data, new_sample)
        elif self.state == "TURN_RETRY_PAUSE":
            self.brake()
            if time.monotonic() - self.state_started >= 0.25:
                self.transition("TURN_90")
        else:
            self.transition("FAULT", f"unknown Round 2 state {self.state}")


def round2_obstacle_main():
    detector = UnifiedPillarDetector()
    controller = ObstacleChallengeController(detector)
    last_camera_sequence = -1
    detection = None
    display_frame = None
    print(
        f"[ROUND2] Unified detector ON; parking exit "
        f"{'ON' if ENABLE_PARKING_EXIT else 'OFF'}; parking in "
        f"{'ON' if ENABLE_PARKING_IN else 'OFF'}; "
        "red passes right, green passes left"
    )

    while True:
        telemetry = read_data()
        ret, frame, camera_sequence, captured_at = cap.read_with_metadata()
        camera_fresh = (
            ret
            and captured_at > 0
            and time.monotonic() - captured_at <= CAMERA_STALE_SECONDS
        )
        if ret:
            display_frame = frame
            if controller.block_detection_suspended or controller.recovery_camera_paused:
                detection = None
                # Consume sequence numbers without processing their frames. This
                # prevents frames captured during a turn or recovery movement
                # from being reused after normal detection resumes.
                last_camera_sequence = camera_sequence
            elif camera_sequence != last_camera_sequence:
                detection = detector.update(frame)
                controller.last_detection_capture_at = captured_at
                last_camera_sequence = camera_sequence

        controller.update(telemetry, detection, camera_fresh)

        if display_frame is not None:
            annotated = detector.annotate(
                display_frame,
                controller.state,
                controller.last_steering,
                telemetry[0],
                telemetry,
            )
            publish_detection_view(annotated)

        if controller.state == "COMPLETE" and ENABLE_PARKING_IN:
            controller.brake()
            return

        time.sleep(NAVIGATION_LOOP_SECONDS)


def navigation_only_main():
    """Run three laps without invoking colour or parking behaviour."""
    controller = NavigationOnlyController()
    print(
        "[NAV] Navigation-only mode: colour detection OFF, "
        "parking exit OFF, parking in OFF"
    )

    while True:
        telemetry = read_data()
        controller.update(telemetry)

        if controller.state == "COMPLETE" and ENABLE_PARKING_IN:
            controller.brake()
            return

        time.sleep(NAVIGATION_LOOP_SECONDS)


def parking(DEBUG = False):
    global DIRECTION
    send_data(0, 0, 0)
    if DIRECTION == "anticlockwise":
        data = wait_until_and_read_data()
        angle, left, front, right, ir = data
        if front < 25:
            send_data(75, 1, 0)
            time.sleep(1.5)
            send_data(0, 0, 0)
        elif front > 70:
            send_data(75, 0, 0)
            time.sleep(1)
            send_data(0, 0, 0)
        send_data(0, 0, 0)
        print("SET POSITION; STARTING TURN IN 1S")
        time.sleep(1)
        data = wait_until_and_read_data()
        data = wait_until_and_read_data()
        data = wait_until_and_read_data()
        angle, left, front, right, ir = data
        send_data(50, 1, 0)
        while left > 80 or (front and front < 80):
            data = read_data()
            if data:
                angle, left, front, right, ir = data
                send_data(50, 1, angle)
                print(data)
            flush_serial()
        send_data(0, 0, 0)
        steer_until_angle(0, -80, 75, 1, 55)
        send_data(60, 1, 0)
        time.sleep(3)
        send_data(0, 0, 0)
    elif DIRECTION == "clockwise":
        steer_until_angle(0, -90, 80, 0, -30)
        send_data(80, 0, 0)
        time.sleep(2)
        send_data(0, 0, 0)

def waitForOk():
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print("ESP32:", line)
            if line.upper() == "OK":
                print("Sending back OK...")
                ser.write(b"OK\n")
                break

if __name__ == "__main__":
    # MAIN
    try:
        print("Initialized")
        start_live_ui()
        led.on()
        waitForOk()
        print("STARTING")
        flush_serial()

        if ENABLE_COLOR_DETECTION:
            round2_obstacle_main()
        else:
            navigation_only_main()
            if ENABLE_PARKING_IN:
                parking()
    except KeyboardInterrupt:
        print("[STOP] Keyboard interrupt; applying electrical brake")
    finally:
        ui_running = False
        try:
            send_data(0, 0, 0)
        except Exception as brake_error:
            print(f"[STOP] Could not send final brake command: {brake_error}")
        try:
            cap.release()
        finally:
            cv2.destroyAllWindows()
            led.off()
            ser.close()

    # DEBUG
    # while True:
    #     print(wait_until_and_read_data())
    #     flush_serial()
