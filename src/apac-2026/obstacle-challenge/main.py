"""Future Engineers Round 2 controller.

Active path: one unified pillar detector and navigation state machine. After
corner 12, control passes to parallel_parking.py for the parking manoeuvre.
"""

# Importing Modules
import os
import csv
import json
from datetime import datetime
import cv2
import numpy as np
import serial
import time
import threading
import _thread
import math
from collections import deque,defaultdict
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
TURNING_SPEED = 110
SPEED = 215
FOLLOW_STRAIGHT_SPEED = 125
POST_CORNER_FOLLOW_SPEED = 120  
POST_CORNER_FOLLOW_SECONDS = 2
PARKING_SPEED = int(PARKING_SPEED*SPEED_K_PARKING)
TURNING_SPEED = int(TURNING_SPEED*SPEED_K)
SPEED = int(SPEED*SPEED_K)
FOLLOW_STRAIGHT_SPEED = int(FOLLOW_STRAIGHT_SPEED*SPEED_K)
SPEED_NO_AURA_FARM = 90
PARKING_EXIT_FINAL_LAUNCH_SPEED = 70
PARKING_EXIT_FINAL_LAUNCH_SECONDS = 0.35

# Debugging
DEBUG = True
SHOW_LIVE_UI = True

# Round 2 feature switches. Parking exit/in run as isolated non-blocking states
# inside the unified controller.
ENABLE_COLOR_DETECTION = True
ENABLE_PARKING_EXIT = True
ENABLE_PARKING_IN = True
PARTIAL_PARKING = True
PARALLEL_PARKING = False
ENABLE_RUN_CSV_LOGGING = False
RUN_CSV_FLUSH_SECONDS = 1.0

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

# Navigation-only state-machine tuning. The ESP32 forwards close-wall data;
# Python decides when to turn, reverse, or electrically brake the motor.
COMMAND_REFRESH_SECONDS = 0.05
NAVIGATION_LOOP_SECONDS = 0.01
DIRECTION_SAMPLE_COUNT = 9
DIRECTION_MIN_DIFFERENCE_CM = 15
DIRECTION_LOCK_TIMEOUT_SECONDS = 5.0
CORNER_TRIGGER_CM = 15
OBSTACLE_CORNER_TRIGGER_CM = 35
CORNER_TRIGGER_RELEASE_CM = 45
CORNER_SIDE_OPEN_CM = 150
CORNER_CONFIRMATION_SAMPLES = 3
CORNER_APPROACH_HEADING_TOLERANCE = 20
# Round 2: the labelled CW corner-wall recording has right opening 250-254 cm.
# Use a long opening, not L+R: the opposite sensor can be inside its blind zone.
# 150 cm is an initial test threshold, mirrored for CCW (not field-validated).
OBSTACLE_CORNER_SIDE_OPEN_CM = 150
OBSTACLE_CORNER_HEADING_TOLERANCE = 12.0
OBSTACLE_CORNER_HEADING_SPREAD = 1.0
CORNER_NEXT_HEADING_SPREAD = 3.0
OBSTACLE_CORNER_CONFIRMATION_SAMPLES = 3
OBSTACLE_CORNER_CONFIRMATION_SECONDS = 0.25
CORNER_NEXT_CONFIRMATION_SAMPLES = 3
CORNER_NEXT_CONFIRMATION_SECONDS = 0.15
OBSTACLE_CORNER_MAX_SAMPLE_GAP_SECONDS = 0.20
CORNER_ZONE_LOCK_CM = 105
CORNER_ZONE_LOCK_SAMPLES = 3
CORNER_ZONE_LOCK_MAX_SAMPLE_GAP_SECONDS = 0.20
CORNER_ZONE_REARM_CM = 130
CORNER_ZONE_REARM_SAMPLES = 3
NEXT_CORNER_MIN_FORWARD_SECONDS = 0.85
CORNER_PILLAR_FRONT_DROP_CM = 80
CORNER_PILLAR_FRONT_DROP_MAX_GAP_SECONDS = 0.25
CORNER_PILLAR_FRONT_NEAR_CM = 60
CORNER_PILLAR_MIN_AREA_RATIO = 0.006
CORNER_PENDING_SPEED = 120
CORNER_FINAL_APPROACH_CM = 55
CORNER_FINAL_APPROACH_SPEED = 120
CORNER_TWO_STEP_SPLIT_DEGREES = 65
TURN_DIRECT_REVERSE_FRONT_CM = 30
TURN_FORWARD_FALLBACK_MIN_FRONT_CM = 45
TURN_FORWARD_FALLBACK_SPEED = 75
TURN_NO_PROGRESS_SECONDS = 0.65
TURN_PROGRESS_DEGREES = 2.0
POST_RELEASE_RECHECK_SECONDS = 1.0
# Recovery used only after repeated front-interlock/corner retry loops.  It
# abandons the incomplete turn, steers back to the previous straight heading,
# and creates enough camera distance for the normal detector to decide again.
DEEP_RECOVERY_SPEED = 75
DEEP_RECOVERY_STEER_UPDATE_SECONDS = 0.25
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
PARKING_EXIT_REVERSE_VIEW_SECONDS = 4.5
PARKING_EXIT_FORWARD_ALIGN_SECONDS = 0.10
PARKING_EXIT_FORWARD_ALIGN_TIMEOUT_SECONDS = 4.5
PARKING_EXIT_FORWARD_ALIGN_TOLERANCE = 8
PARKING_EXIT_FORWARD_ALIGN_STEER_GAIN = 1.0
PARKING_EXIT_FORWARD_ALIGN_MAX_STEER = 30
PARKING_EXIT_FIRST_PILLAR_STEER = 40
# Graduated side-wall envelope during active pillar acquisition/passing.
PILLAR_WALL_WARNING_CM = 18
PILLAR_WALL_CRITICAL_CM = 10
PILLAR_WALL_EMERGENCY_CM = 6
PILLAR_WALL_MAX_CORRECTION = 35
PILLAR_WALL_CRITICAL_STEER = 28
PILLAR_WALL_EMERGENCY_STEER = 42
PILLAR_WALL_EMERGENCY_SPEED = 65
PILLAR_HEADING_SOFT_LIMIT_DEGREES = 25
PILLAR_HEADING_HARD_LIMIT_DEGREES = 55
TURN_SIDE_CLEARANCE_CM = 20
# Adaptive initial corner arc after confirmed side-clearance evidence.
CORNER_REVERSE_CLEARANCE_CM = 60
CORNER_REVERSE_SPEED = 200
CORNER_REVERSE_STEER = 48
CORNER_REVERSE_MAX_SECONDS = 0.50
# Pillar-specific short recovery before deeper recovery is needed.
PILLAR_RELEASE_FRONT_CM = 45
PILLAR_RELEASE_SIDE_CM = 12
PILLAR_RELEASE_ANGLE = 12
PILLAR_RELEASE_SPEED = 100 #80

PRE_TURN_BACKUP_SPEED = 88  # 70 * 1.25
PRE_TURN_BACKUP_SECONDS = 0.28  # 0.35 / 1.25
POST_TURN_BACKUP_SPEED = 125  # 100 * 1.25
POST_TURN_BACKUP_SECONDS = 0.35
POST_TURN_SCAN_HOLD_SECONDS = 0.20
RECENTER_SECONDS = 0.25 #0.30
# After a turn, use both side walls to reduce the steering needed for the
# next pillar. Ignore a side reading that is still seeing the open corner.
ROUND2_RECENTER_SPEED = 160
ROUND2_RECENTER_MIN_SECONDS = 0.55
ROUND2_RECENTER_MAX_SECONDS = 1.60
ROUND2_RECENTER_SIDE_MIN_CM = 8
ROUND2_RECENTER_SIDE_MAX_CM = 110
ROUND2_RECENTER_SIDE_TOLERANCE_CM = 10
ROUND2_RECENTER_SIDE_KP = 0.45
ROUND2_RECENTER_SIDE_MAX_STEER = 14
ROUND2_RECENTER_HEADING_GATE_DEGREES = 8
ROUND2_RECENTER_CONFIRMATION_SAMPLES = 3

# After a pillar pass, steering was held hard to one side for the whole
# avoidance manoeuvre; heading alone does not undo the resulting lateral
# offset. Recenter using both side walls, same principle as post-corner
# RECENTER, but shorter since the offset here is smaller than after a turn.
# After a pillar pass, steering was held hard to one side for the whole
# avoidance manoeuvre, so the chassis is physically offset toward the
# opposite wall from where the pillar was. Two-phase recovery:
#   phase 1 "counter" -- steer the OPPOSITE direction from the pass for a
#     short, bounded burst to actively cancel that lateral drift (an S-curve,
#     not just a heading hold).
#   phase 2 "settle" -- blend heading-hold with side-wall balance (same
#     principle as post-corner RECENTER) until centered.
PILLAR_RECENTER_SPEED = 160 #120
PILLAR_RECENTER_COUNTER_STEER_DEGREES = 22.0
PILLAR_RECENTER_COUNTER_MIN_SECONDS = 0.18
PILLAR_RECENTER_COUNTER_MAX_SECONDS = 0.45
PILLAR_RECENTER_SETTLE_MIN_SECONDS = 0.35
PILLAR_RECENTER_SETTLE_MAX_SECONDS = 1.0
PILLAR_RECENTER_SIDE_MIN_CM = 8
PILLAR_RECENTER_SIDE_MAX_CM = 110
PILLAR_RECENTER_SIDE_TOLERANCE_CM = 8
PILLAR_RECENTER_SIDE_KP = 0.5
PILLAR_RECENTER_SIDE_MAX_STEER = 18
PILLAR_RECENTER_HEADING_GATE_DEGREES = 10
PILLAR_RECENTER_CONFIRMATION_SAMPLES = 3

TURN_TIMEOUT_SECONDS = 5.0
HARD_STOP_RELEASE_CM = 30
CORNER_CLEARANCE_SIDE_NEAR_CM = 22
CORNER_CLEARANCE_REVERSE_STEER = 30
CORNER_CLEARANCE_REVERSE_SPEED = 150
CORNER_CLEARANCE_REVERSE_MAX_SECONDS = 0.40
PYTHON_EMERGENCY_RELEASE_CM = 5
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
TURN_REVERSE_STEER = 70
TURN_REVERSE_SPEED = 150
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
active_run_csv_logger = None

ui_running = True
ui_display_frame = None
ui_display_time = 0.0
ui_display_lock = threading.Lock()

# Camera Settings
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30
CAMERA_STALE_SECONDS = 0.30
CAMERA_SIDE_MASK_FRACTION = 0.16
POST_CORNER_NARROW_VIEW_SECONDS = 5.0
POST_CORNER_VIEW_SIDE_FRACTION = 0.20
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

led = LED(17)
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
BLACK_WALL_HSV_LOWER = (0, 0, 0)
BLACK_WALL_HSV_UPPER = (179, 255, 70)
BLACK_CORNER_ROI_TOP_RATIO = 0.18
BLACK_CORNER_ROI_BOTTOM_RATIO = 0.78
BLACK_CORNER_MIN_CONTOUR_AREA_RATIO = 0.055
BLACK_CORNER_MIN_WIDTH_RATIO = 0.35
BLACK_CORNER_MIN_HEIGHT_RATIO = 0.18
BLACK_CORNER_MIN_CENTER_COVERAGE = 0.32
BLACK_CORNER_MAX_OPEN_SIDE_COVERAGE = 0.26
BLACK_CORNER_OPEN_SIDE_RELATIVE_COVERAGE = 0.65
BLACK_CORNER_CONFIRMATION_FRAMES = 3
BLACK_CORNER_HISTORY = 4
CORNER_REFERENCE_HOLD_SECONDS = 0.35
CORNER_CONTEXT_CONFIRMATION_FRAMES = 3
VISION_CORNER_APPROACH_TIMEOUT_SECONDS = 6.0
PILLAR_CORNER_VETO_SECONDS = 1.0
POST_TURN_TAPE_IGNORE_GRACE_SECONDS = 3.0
CORNER_FOREGROUND_AREA_ADVANTAGE = 1.25
BACKGROUND_EARLY_SIDE_OPEN_CM = 100
BACKGROUND_FOREGROUND_GROWTH_RATIO = 1.50
BACKGROUND_FOREGROUND_BOTTOM_GROWTH = 0.08

# Colour-independent parking-block geometry. A physical parking block is wide
# and thick; the painted course tapes are wide but too thin to pass this test.
PARKING_BLOCK_MIN_WIDTH_RATIO = 0.10
PARKING_BLOCK_MAX_WIDTH_RATIO = 0.80
PARKING_BLOCK_MIN_HEIGHT_RATIO = 0.035
PARKING_BLOCK_MAX_HEIGHT_RATIO = 0.30
PARKING_BLOCK_MIN_WIDTH_OVER_HEIGHT = 2.0
PARKING_BLOCK_MIN_RECTANGULARITY = 0.45
PARKING_BLOCK_MIN_BOTTOM_RATIO = 0.55
PARKING_BLOCK_CONFIRMATION_HITS = 3
PARKING_BLOCK_HISTORY = 5
PARKING_BLOCK_MAX_MISSES = 3
PARKING_BLOCK_AVOID_SPEED = 60
PARKING_BLOCK_AVOID_STEER = 15
PARKING_BLOCK_PATH_LEFT_RATIO = 0.35
PARKING_BLOCK_PATH_RIGHT_RATIO = 0.65

# Continuous pillar-control tuning for the installed servo orientation.
PILLAR_RED_TARGET_X = 0.38
RED_PRE_PASS_REVERSE_SECONDS = 0.8
RED_PRE_PASS_REVERSE_SPEED = 120 #80
RED_PRE_PASS_REVERSE_STEER = -60
PILLAR_RED_STEERING_MULTIPLIER = 1.50
PILLAR_GREEN_TARGET_X = 0.62
PILLAR_PROXIMITY_START = 0.38
PILLAR_PROXIMITY_FULL = 0.90
PILLAR_GREEN_VISIBLE_HOLD_STEER = 25
PILLAR_NEAR_BOTTOM = 0.78
PILLAR_LATERAL_KP = 200.0
PILLAR_MAX_CORRECTION = 40
PILLAR_STEERING_SMOOTHING = 0.65
PILLAR_HEADING_WEIGHT = 0.25
PILLAR_MIN_CONTROL_WEIGHT = 0.55
PILLAR_GREEN_STEERING_MULTIPLIER = 1.50
PILLAR_AVOIDANCE_STEER_BOOST_DEGREES = 8.0
PILLAR_ACQUIRE_TIMEOUT_SECONDS = 1.2
PILLAR_PASS_TIMEOUT_SECONDS = 4.0
PILLAR_CONFIRM_PASSED_SECONDS = 0.35
PILLAR_REQUIRED_LOST_FRAMES = 3
MAX_PILLARS_PER_STRAIGHT = 2
SECOND_PILLAR_SLOT_TIMEOUT_SECONDS = 3.6
POST_CORNER_DIRECT_SECOND_SLOT_DELAY_SECONDS = 2.5
PILLAR_APPROACH_SPEED = 120 #105
PILLAR_PASS_SPEED = 120 #80
PILLAR_MIN_PASS_SPEED = 80
PILLAR_MIN_STEERING_HOLD_SECONDS = 0.45
PILLAR_CLEARANCE_HOLD_SECONDS = 0.25
PILLAR_RED_CLEARANCE_MAX_STEER = 15
ALPHA_POSITION_WINDOW_SECONDS = 5.0

# With the camera mounted slightly lower, start avoidance at a smaller visible
# pillar area while retaining confirmed-track and background checks.
PILLAR_AREA_NEAR_RATIO = 3200.0 / (CAMERA_WIDTH * CAMERA_HEIGHT)
PILLAR_AREA_CLOSE_RATIO = 7000.0 / (CAMERA_WIDTH * CAMERA_HEIGHT)
PILLAR_AREA_TOO_CLOSE_RATIO = 20000.0 / (CAMERA_WIDTH * CAMERA_HEIGHT)
PILLAR_DISTANCE_PROFILES = {
    "far": {
        "speed": 100,
        "gain": 0.70,
        "min_steer": 3.0,
        "max_steer": 12.0,
        "min_weight": 0.55,
        "smoothing": 0.55,
    },
    "near": {
        "speed": 95,
        "gain": 1.00,
        "min_steer": 26.0,
        "max_steer": 42.0,
        "min_weight": 0.70,
        "smoothing": 0.68,
    },
    "close": {
        "speed": 90,
        "gain": 1.25,
        "min_steer": 34.0,
        "max_steer": 52.0,
        "min_weight": 0.85,
        "smoothing": 0.80,
    },
    "too_close": {
        "speed": 80,
        "gain": 1.50,
        "min_steer": 38.0,
        "max_steer": 52.0,
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


def adjust_frame(frame):
    adjusted = cv2.convertScaleAbs(frame, alpha=CONTRAST, beta=BRIGHTNESS)

    inv_gamma = 1.0 / GAMMA
    table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)

    return adjusted


def camera_view_side_fraction(corner_count, now, post_corner_until):
    """Use the full view for parking and one combined side mask before it."""
    if corner_count >= COUNTER_MAX:
        return 0.0
    post_corner_fraction = (
        POST_CORNER_VIEW_SIDE_FRACTION
        if now < post_corner_until else 0.0
    )
    return max(0.0, min(0.49, max(CAMERA_SIDE_MASK_FRACTION, post_corner_fraction)))




def asymmetric_camera_view(frame, left_fraction, right_fraction):
    """Mask independent side margins without changing detector coordinates."""
    left_margin = int(round(frame.shape[1] * left_fraction))
    right_margin = int(round(frame.shape[1] * right_fraction))
    if left_margin <= 0 and right_margin <= 0:
        return frame
    narrowed = frame.copy()
    if left_margin > 0:
        narrowed[:, :left_margin] = 128
    if right_margin > 0:
        narrowed[:, frame.shape[1] - right_margin:] = 128
    return narrowed



# Running Functions



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


@dataclass
class ParkingBlockDetection:
    bbox: tuple
    center_x_norm: float
    bottom_norm: float
    normalized_area: float
    width_over_height: float
    rectangularity: float
    confirmed: bool = False
    seen_this_frame: bool = True
    hit_count: int = 1
    miss_count: int = 0
    profile: str = ""
    geometry_reason: str = ""


@dataclass
class PillarPassCommitment:
    track_id: int
    color: str
    pass_side: str
    pass_state: str
    forward_steering_sign: int
    entry_heading: float
    target_heading: float
    committed_at: float
    last_steering: float
    last_speed: int


class RunCsvLogger:
    """Permanent, buffered Round 2 telemetry and decision recorder."""

    FIELDNAMES = (
        "row_type", "event", "detail", "run_id", "wall_time", "elapsed_s",
        "telemetry_sequence", "esp_millis", "protocol", "valid_mask",
        "lap", "corner_counter", "segment", "pillars_this_straight",
        "corner_next", "state", "direction",
        "target_heading", "heading", "heading_error", "left_cm", "center_cm",
        "right_cm", "ir", "hard_stop", "requested_speed",
        "requested_direction", "requested_steering", "applied_speed",
        "camera_fresh", "track_id", "pillar_color", "pillar_confirmed",
        "pillar_seen", "pillar_bbox", "pillar_area", "pillar_bottom",
        "pillar_x", "pillar_band", "background_candidate", "foreground_priority",
        "deferred_track_id", "all_candidates", "blue_tape_depth",
        "orange_tape_depth", "parking_block", "corner_zone_locked",
        "corner_zone_count", "corner_close_count", "recovery_attempt",
        "pass_committed", "committed_track", "committed_color",
        "committed_side", "committed_state", "committed_steering",
        "committed_speed",
    )

    def __init__(self):
        self.enabled = False
        self.file = None
        self.writer = None
        self.started_at = time.monotonic()
        self.last_flush_at = self.started_at
        self.last_sequence = None
        self.run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path = None
        try:
            log_directory = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "run_logs",
            )
            os.makedirs(log_directory, exist_ok=True)
            self.path = os.path.join(
                log_directory,
                f"round2_run_{self.run_id}.csv",
            )
            self.file = open(self.path, "x", newline="", encoding="utf-8")
            self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDNAMES)
            self.writer.writeheader()
            self.file.flush()
            self.enabled = True
            print(f"[CSV] Run logging ON: {self.path}")
        except Exception as error:
            print(f"[CSV] Logging disabled; could not create run file: {error}")
            self.close()

    @staticmethod
    def _candidate_payload(detector):
        return json.dumps(
            [
                {
                    "color": item.color,
                    "bbox": list(item.bbox),
                    "area": round(item.normalized_area, 6),
                    "bottom": round(item.bottom_norm, 4),
                    "x": round(item.center_x_norm, 4),
                    "blocked": bool(item.blocked_by_corner),
                    "foreground": bool(item.foreground_priority),
                }
                for item in detector.last_candidates
                if item.color in ("red", "green")
            ],
            separators=(",", ":"),
        )

    def _build_row(self, controller, telemetry, detection, camera_fresh):
        row = {name: "" for name in self.FIELDNAMES}
        now = time.monotonic()
        telemetry = telemetry or getattr(controller, "current_round2_sensor_data", None) or (None,) * 5
        heading, left, center, right, ir = telemetry
        requested_speed, requested_direction, requested_steering = controller.command
        commitment = getattr(controller, "pillar_pass_commitment", None)
        parking_block = controller.detector.parking_block
        row.update({
            "run_id": self.run_id,
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "elapsed_s": f"{now - self.started_at:.3f}",
            "telemetry_sequence": last_telemetry_status["sequence"],
            "esp_millis": last_telemetry_status["esp_millis"],
            "protocol": last_telemetry_status["protocol"],
            "valid_mask": f"0x{last_telemetry_status['valid_mask']:02X}",
            "lap": min(3, COUNTER // 4 + 1),
            "corner_counter": COUNTER,
            "segment": COUNTER % 4 + 1,
            "pillars_this_straight": controller.pillar_slots_filled,
            "corner_next": int(controller.corner_next_after_pillars),
            "state": controller.state,
            "direction": DIRECTION,
            "target_heading": target_angle,
            "heading": heading,
            "heading_error": (
                "" if heading is None else f"{angle_diff(target_angle, heading):.2f}"
            ),
            "left_cm": left,
            "center_cm": center,
            "right_cm": right,
            "ir": ir,
            "hard_stop": int(last_telemetry_status["hard_stop"]),
            "requested_speed": requested_speed,
            "requested_direction": requested_direction,
            "requested_steering": requested_steering,
            "applied_speed": last_telemetry_status["applied_speed"],
            "camera_fresh": int(bool(camera_fresh)),
            "deferred_track_id": controller.deferred_corner_track_id,
            "all_candidates": self._candidate_payload(controller.detector),
            "blue_tape_depth": controller.detector.course_line_depths["blue"],
            "orange_tape_depth": controller.detector.course_line_depths["orange"],
            "parking_block": (
                "" if parking_block is None else json.dumps({
                    "bbox": list(parking_block.bbox),
                    "confirmed": bool(parking_block.confirmed),
                    "bottom": round(parking_block.bottom_norm, 4),
                    "ratio": round(parking_block.width_over_height, 3),
                }, separators=(",", ":"))
            ),
            "corner_zone_locked": int(controller.corner_zone_locked),
            "corner_zone_count": controller.corner_zone_lock_count,
            "corner_close_count": controller.corner_confirmation,
            "recovery_attempt": max(
                getattr(controller, "emergency_escape_attempt", 0),
                getattr(controller, "deep_recovery_attempts", 0),
            ),
            "pass_committed": int(commitment is not None),
        })
        if detection is not None:
            row.update({
                "track_id": detection.track_id,
                "pillar_color": detection.color,
                "pillar_confirmed": int(detection.confirmed),
                "pillar_seen": int(detection.seen_this_frame),
                "pillar_bbox": json.dumps(list(detection.bbox)),
                "pillar_area": f"{detection.normalized_area:.6f}",
                "pillar_bottom": f"{detection.bottom_norm:.4f}",
                "pillar_x": f"{detection.center_x_norm:.4f}",
                "pillar_band": detection.distance_band(),
                "background_candidate": int(detection.blocked_by_corner),
                "foreground_priority": int(detection.foreground_priority),
            })
        if commitment is not None:
            row.update({
                "committed_track": commitment.track_id,
                "committed_color": commitment.color,
                "committed_side": commitment.pass_side,
                "committed_state": commitment.pass_state,
                "committed_steering": f"{commitment.last_steering:.2f}",
                "committed_speed": commitment.last_speed,
            })
        return row

    def _write(self, row, flush=False):
        if not self.enabled:
            return
        try:
            self.writer.writerow(row)
            now = time.monotonic()
            if flush or now - self.last_flush_at >= RUN_CSV_FLUSH_SECONDS:
                self.file.flush()
                self.last_flush_at = now
        except Exception as error:
            print(f"[CSV] Logging disabled after write failure: {error}")
            self.close()

    def log_sample(self, controller, telemetry, detection, camera_fresh):
        sequence = last_telemetry_status["sequence"]
        if not self.enabled or sequence is None or sequence == self.last_sequence:
            return
        try:
            self.last_sequence = sequence
            row = self._build_row(controller, telemetry, detection, camera_fresh)
            row["row_type"] = "sample"
            self._write(row)
        except Exception as error:
            print(f"[CSV] Logging disabled after sample failure: {error}")
            self.close()

    def log_event(self, controller, event, detail=""):
        if not self.enabled:
            return
        try:
            telemetry = getattr(controller, "current_round2_sensor_data", None) or (None, None, None, None, None)
            row = self._build_row(controller, telemetry, None, False)
            row["row_type"] = "event"
            row["event"] = event
            row["detail"] = detail
            self._write(row, flush=True)
        except Exception as error:
            print(f"[CSV] Logging disabled after event failure: {error}")
            self.close()

    def close(self):
        if self.file is not None:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
        self.file = None
        self.writer = None
        self.enabled = False


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
        self.remembered_corner_reference_y = None
        self.corner_reference_last_seen_at = 0.0
        self.corner_context_current = False
        self.corner_context_confirmed = False
        self.corner_context_history = deque(maxlen=CORNER_CONTEXT_CONFIRMATION_FRAMES)
        self.ignored_corner_pillars = []
        self.corner_trigger_pillars = []
        self.actionable_pillar_visible = False
        self.last_actionable_pillar_at = 0.0
        self.corner_filter_suppressed_until = 0.0
        self.deferred_track_id = None
        self.parking_block = None
        self.parking_block_history = deque(maxlen=PARKING_BLOCK_HISTORY)
        self.black_corner_history = deque(maxlen=BLACK_CORNER_HISTORY)
        self.black_corner_candidate = False
        self.black_corner_confirmed = False
        self.black_corner_expected_side = None
        self.black_corner_coverage = 0.0
        self.black_corner_side_coverage = 0.0
        self.black_corner_box = None

    def _reset_black_corner(self):
        self.black_corner_history.clear()
        self.black_corner_candidate = False
        self.black_corner_confirmed = False
        self.black_corner_expected_side = None
        self.black_corner_coverage = 0.0
        self.black_corner_side_coverage = 0.0
        self.black_corner_box = None

    def _detect_black_corner_geometry(self, frame, expected_side):
        """Confirm a broad black end wall with an opening on the turn side."""
        frame_h, frame_w = frame.shape[:2]
        roi_top = int(frame_h * BLACK_CORNER_ROI_TOP_RATIO)
        roi_bottom = int(frame_h * BLACK_CORNER_ROI_BOTTOM_RATIO)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array(BLACK_WALL_HSV_LOWER, dtype=np.uint8),
            np.array(BLACK_WALL_HSV_UPPER, dtype=np.uint8),
        )
        mask[:roi_top, :] = 0
        mask[roi_bottom:, :] = 0
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_area = float(max(1, (roi_bottom - roi_top) * frame_w))
        valid_boxes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)
            if area < roi_area * BLACK_CORNER_MIN_CONTOUR_AREA_RATIO:
                continue
            if width < frame_w * BLACK_CORNER_MIN_WIDTH_RATIO:
                continue
            if height < frame_h * BLACK_CORNER_MIN_HEIGHT_RATIO:
                continue
            valid_boxes.append((area, x, y, width, height))

        center_left = frame_w // 4
        center_right = frame_w - center_left
        center_mask = mask[roi_top:roi_bottom, center_left:center_right]
        center_coverage = cv2.countNonZero(center_mask) / float(max(1, center_mask.size))
        if expected_side == "right":
            side_mask = mask[roi_top:roi_bottom, center_right:frame_w]
        else:
            side_mask = mask[roi_top:roi_bottom, 0:center_left]
        side_coverage = cv2.countNonZero(side_mask) / float(max(1, side_mask.size))

        broad_wall = bool(valid_boxes) and center_coverage >= BLACK_CORNER_MIN_CENTER_COVERAGE
        expected_opening = (
            side_coverage <= BLACK_CORNER_MAX_OPEN_SIDE_COVERAGE
            and side_coverage <= center_coverage * BLACK_CORNER_OPEN_SIDE_RELATIVE_COVERAGE
        )
        candidate = broad_wall and expected_side in ("left", "right") and expected_opening
        self.black_corner_history.append(candidate)
        self.black_corner_candidate = candidate
        self.black_corner_confirmed = (
            len(self.black_corner_history) >= BLACK_CORNER_CONFIRMATION_FRAMES
            and sum(self.black_corner_history) >= BLACK_CORNER_CONFIRMATION_FRAMES
        )
        self.black_corner_expected_side = expected_side
        self.black_corner_coverage = center_coverage
        self.black_corner_side_coverage = side_coverage
        if valid_boxes:
            _, x, y, width, height = max(valid_boxes, key=lambda item: item[0])
            self.black_corner_box = (x, y, width, height)
        else:
            self.black_corner_box = None

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

            line_boxes = []
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
                # A wide object at the camera's bottom can share the tape's
                # colour. It must not become the corner boundary.
                parking = self.parking_block
                if parking is not None and parking.seen_this_frame:
                    px, py, pw, ph = parking.bbox
                    overlap_w = max(0, min(x + width, px + pw) - max(x, px))
                    overlap_h = max(0, min(y + height, py + ph) - max(y, py))
                    if overlap_w * overlap_h >= 0.5 * width * height:
                        continue
                boxes[color].append((x, y, width, height))
                line_boxes.append((x, y, width, height))

            if line_boxes:
                # The course tape is the longest visible line, not whichever
                # orange or blue fragment happens to be nearest the camera.
                _, y, _, height = max(
                    line_boxes, key=lambda box: (box[2], box[1] + box[3] // 2)
                )
                depths[color] = y + height // 2

        self.course_line_depths = depths
        self.course_line_boxes = boxes

    def _apply_corner_context(self, candidates, frame_h, frame_w):
        blue_depth = self.course_line_depths["blue"]
        orange_depth = self.course_line_depths["orange"]
        corner_filter_enabled = not self.corner_filter_suppressed()
        visible_depths = [
            depth for depth in (blue_depth, orange_depth) if depth is not None
        ]
        current_reference_y = max(visible_depths) if visible_depths else None
        if corner_filter_enabled:
            if current_reference_y is not None:
                self.remembered_corner_reference_y = current_reference_y
                self.corner_reference_last_seen_at = time.monotonic()
            elif (
                time.monotonic() - self.corner_reference_last_seen_at
                > CORNER_REFERENCE_HOLD_SECONDS
            ):
                self.remembered_corner_reference_y = None
        self.corner_reference_y = (
            self.remembered_corner_reference_y if corner_filter_enabled else None
        )
        # Tape remains available for corner recognition, but never decides
        # which pillar is foreground. Protect a committed pass first.
        passing_candidate = (
            self._matching_candidate(candidates)
            if self.active is not None
            and self.passing_track_id == self.active.track_id
            else None
        )

        pillars = [item for item in candidates if item.color in ("red", "green")]
        nearest = passing_candidate if passing_candidate is not None else (
            max(pillars, key=lambda item: (item.bottom_norm, item.normalized_area))
            if pillars else None
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
            blocked = candidate is not nearest
            active_track_priority = candidate is passing_candidate

            candidate = replace(
                candidate,
                grid_column=grid_column,
                blocked_by_corner=blocked,
                foreground_priority=active_track_priority,
            )
            classified.append(candidate)

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



    def _detect_parking_block(self, frame, roi_top):
        """Track a wide physical obstacle without using its colour."""
        frame_h, frame_w = frame.shape[:2]
        frame_area = float(frame_h * frame_w)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 50, 150)
        edges[:max(roi_top, int(frame_h * 0.40)), :] = 0
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 5))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            if width <= 0 or height <= 0:
                continue
            width_ratio = width / float(frame_w)
            height_ratio = height / float(frame_h)
            bottom_norm = (y + height) / float(frame_h)
            width_over_height = width / float(height)
            area = cv2.contourArea(contour)
            rectangularity = area / float(width * height)
            if not PARKING_BLOCK_MIN_WIDTH_RATIO <= width_ratio <= PARKING_BLOCK_MAX_WIDTH_RATIO:
                continue
            if not PARKING_BLOCK_MIN_HEIGHT_RATIO <= height_ratio <= PARKING_BLOCK_MAX_HEIGHT_RATIO:
                continue
            if width_over_height < PARKING_BLOCK_MIN_WIDTH_OVER_HEIGHT:
                continue
            if rectangularity < PARKING_BLOCK_MIN_RECTANGULARITY:
                continue
            if bottom_norm < PARKING_BLOCK_MIN_BOTTOM_RATIO:
                continue
            candidates.append(
                ParkingBlockDetection(
                    bbox=(x, y, width, height),
                    center_x_norm=(x + width / 2.0) / frame_w,
                    bottom_norm=bottom_norm,
                    normalized_area=area / frame_area,
                    width_over_height=width_over_height,
                    rectangularity=rectangularity,
                )
            )

        candidate = max(
            candidates,
            key=lambda item: (item.bottom_norm, item.normalized_area),
            default=None,
        )
        if candidate is None:
            self.parking_block_history.append(False)
            if self.parking_block is not None:
                misses = self.parking_block.miss_count + 1
                if misses >= PARKING_BLOCK_MAX_MISSES:
                    self.parking_block = None
                    self.parking_block_history.clear()
                else:
                    self.parking_block = replace(
                        self.parking_block,
                        seen_this_frame=False,
                        miss_count=misses,
                        hit_count=sum(self.parking_block_history),
                    )
            return

        same_object = (
            self.parking_block is not None
            and abs(candidate.center_x_norm - self.parking_block.center_x_norm) <= 0.20
        )
        if not same_object:
            self.parking_block_history.clear()
        self.parking_block_history.append(True)
        hits = sum(self.parking_block_history)
        self.parking_block = replace(
            candidate,
            confirmed=hits >= PARKING_BLOCK_CONFIRMATION_HITS,
            seen_this_frame=True,
            hit_count=hits,
            miss_count=0,
        )

    def _extract_candidates(self, frame):
        frame_h, frame_w = frame.shape[:2]
        frame_area = float(frame_h * frame_w)
        roi_top = max(0, min(frame_h, int(frame_h * DETECTOR_ROI_TOP_RATIO)))
        adjusted = adjust_frame(frame)
        self._detect_parking_block(adjusted, roi_top)
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
        # The lower candidate is physically closer. This is the proven
        # double-trouble rule: when two same-colour pillars are visible, pass
        # the foreground one instead of choosing the larger next-lane pillar.
        selected = max(
            pillar_candidates,
            key=lambda item: (
                item.bottom_norm,
                item.normalized_area,
                item.confidence,
            ),
        )
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

    def update(self, frame, black_corner_search=False, black_corner_side=None):
        if black_corner_search:
            self._detect_black_corner_geometry(adjust_frame(frame), black_corner_side)
        else:
            self._reset_black_corner()
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

    def reset_navigation_context(self, preserve_active=False):
        """Clear corner/tape context; optionally retain a newly seen pillar."""
        if not preserve_active:
            self.release_active()
        self.last_candidates = []
        self.course_line_depths = {"blue": None, "orange": None}
        self.course_line_boxes = {"blue": [], "orange": []}
        self.corner_reference_y = None
        self.remembered_corner_reference_y = None
        self.corner_reference_last_seen_at = 0.0
        self.corner_context_current = False
        self.corner_context_confirmed = False
        self.corner_context_history.clear()
        self.ignored_corner_pillars = []
        self.corner_trigger_pillars = []
        self.actionable_pillar_visible = False
        self.last_actionable_pillar_at = 0.0
        self.deferred_track_id = None
        self.parking_block = None
        self.parking_block_history.clear()
        self._reset_black_corner()

    def annotate(self, frame, state, steering, heading, sensor_data,
                 pillar_count=0, corner_next=False, second_slot_grace=None):
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

        if self.black_corner_box is not None:
            x, y, width, height = self.black_corner_box
            black_status_color = (
                (0, 255, 0) if self.black_corner_confirmed
                else (0, 255, 255) if self.black_corner_candidate
                else (0, 128, 255)
            )
            cv2.rectangle(
                display, (x, y), (x + width, y + height), black_status_color, 3
            )
            cv2.putText(
                display,
                f"BLACK CORNER {'CONFIRMED' if self.black_corner_confirmed else 'CHECK'} "
                f"center={self.black_corner_coverage:.2f} "
                f"{self.black_corner_expected_side}={self.black_corner_side_coverage:.2f}",
                (20, roi_top + 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                black_status_color,
                2,
            )

        if self.parking_block is not None:
            x, y, width, height = self.parking_block.bbox
            draw_color = (
                (0, 165, 255) if self.parking_block.confirmed else (0, 255, 255)
            )
            cv2.rectangle(
                display,
                (x, y),
                (x + width, y + height),
                draw_color,
                3 if self.parking_block.confirmed else 1,
            )
            status = "CONFIRMED" if self.parking_block.confirmed else "CHECK"
            cv2.putText(
                display,
                f"PARKING BLOCK {status} "
                f"hits={self.parking_block.hit_count}/{PARKING_BLOCK_CONFIRMATION_HITS} "
                f"W/H={self.parking_block.width_over_height:.1f}",
                (max(5, x), max(25, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                draw_color,
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
            deferred = self.active.track_id == self.deferred_track_id
            draw_color = (
                (255, 0, 255)
                if deferred
                else (0, 255, 255) if self.active.confirmed else (255, 255, 0)
            )
            cv2.rectangle(display, (x, y), (x + width, y + height), draw_color, 3)
            label = (
                f"TRACK {self.active.track_id} {self.active.color.upper()} "
                f"conf={self.active.confidence:.2f} hits={self.active.hit_count}/5 "
                f"bottom={self.active.bottom_norm:.2f} "
                f"area={self.active.normalized_area:.4f} "
                f"{self.active.distance_band().upper()}"
            )
            if deferred:
                label += " BACKGROUND DEFERRED"
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
        cv2.putText(
            display,
            f"PILLAR SLOTS: {pillar_count}/{MAX_PILLARS_PER_STRAIGHT}",
            (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2,
        )
        if corner_next:
            cv2.putText(
                display, "CORNER NEXT - NO NEW PILLAR", (20, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2,
            )
        elif second_slot_grace is not None and second_slot_grace > 0:
            cv2.putText(
                display, f"SLOT 2 TIMEOUT IN {second_slot_grace:.1f}s", (20, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2,
            )
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
            return TURN_REVERSE_STEER if self.turn_mode == "reverse" else -TURN_STEER
        return -TURN_REVERSE_STEER if self.turn_mode == "reverse" else TURN_STEER

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

        if last_telemetry_status["hard_stop"] or data[2] <= PYTHON_EMERGENCY_RELEASE_CM:
            self.turn_mode = "forward"
            self.start_hard_stop_recovery(data, "TURN", "actual front interlock before turn")
            return True

        if clearance is not None and clearance >= TURN_SIDE_CLEARANCE_CM:
            # Preserve the old manoeuvre: reverse arc where room permits.  This
            # also backs the camera away and increases its view of the new lane.
            self.turn_mode = "reverse"
            self.transition(
                "TURN",
                f"corner confirmed, reverse arc, target={self.pending_target:.1f}",
            )
        else:
            # Planned positioning is not a hard-stop release: it must not
            # require a centre-distance gain or fault when open space is clear.
            self.turn_mode = "forward"
            self.transition(
                "PRE_TURN_BACKUP",
                f"short positioning reverse; opposite clearance={clearance} cm; "
                f"target={self.pending_target:.1f}",
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
        # before the wall closes to Python's emergency-release distance.
        drive_speed = SPEED
        if center <= CORNER_TRIGGER_RELEASE_CM:
            drive_speed = min(SPEED, 65)
        steering = self.heading_correction(heading)
        self.command_robot(drive_speed, 0, steering)

    def update_pre_turn_backup(self, data):
        if time.monotonic() - self.state_started >= PRE_TURN_BACKUP_SECONDS:
            self.transition("TURN", "planned pre-turn backup complete")
            return
        heading = data[0]
        steering = self.heading_correction(heading, reverse=True)
        self.command_robot(PRE_TURN_BACKUP_SPEED, 1, steering)

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

        turn_speed = TURN_REVERSE_SPEED if direction == 1 else TURNING_SPEED
        self.command_robot(turn_speed, direction, self.turn_steering())

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
                self.command_robot(POST_TURN_BACKUP_SPEED, 1, steering)

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

    def __init__(self, detector, run_logger=None):
        super().__init__()
        self.detector = detector
        self.run_logger = run_logger
        self.vision_enabled = ENABLE_COLOR_DETECTION
        self.state = "PARKING_EXIT_DIRECTION" if self.vision_enabled and ENABLE_PARKING_EXIT else "WAIT"
        self.state_started = time.monotonic()
        self.locked_track_id = None
        self.locked_pillar_color = None
        self.pillar_was_near = False
        self.filtered_pillar_correction = 0.0
        self.last_pillar_steering = 0.0
        self.last_pillar_speed = PILLAR_MIN_PASS_SPEED
        self.pillar_lost_started_at = None
        self.last_pillar_passed_at = 0.0
        self.committed_pillar_tracks_this_straight = set()
        self.pillar_slots_filled = 0
        self.first_pillar_slot_filled_at = None
        self.parking_exit_first_pillar_counted = False
        self.last_completed_corner_at = None
        self.narrow_camera_view_until = 0.0
        self.corner_next_after_pillars = False
        self.corner_next_ignored_tracks = set()
        self.deferred_corner_track_id = None
        self.deferred_corner_initial_area = None
        self.deferred_corner_initial_bottom = None
        self.deferred_track_promoted = False
       
        self.recenter_next_state = "FOLLOW_STRAIGHT"
        self.recenter_centered_samples = 0
        self.pillar_recenter_next_state = None
        self.pillar_recenter_centered_samples = 0
        self.pillar_recenter_phase = None
        self.pillar_recenter_phase_started = None
        self.last_passed_pillar_color = None
        self.last_steering = 0.0
       
        self.vision_corner_armed = True
        self.corner_approach_from_lines = False
        self.alpha_corner_locked = False
        self.alpha_corner_reference_y = None
        self.corner_after_pillar_pending = False
        self.block_detection_suspended = bool(self.vision_enabled and ENABLE_PARKING_EXIT)
        self.corner_evidence_started_at = None
        self.corner_evidence_last_at = None
        self.corner_evidence_sequence = None
        self.corner_heading_min_error = None
        self.corner_heading_max_error = None
        self.next_corner_eligible_at = 0.0
        self.next_corner_forward_seconds = None
        self.next_corner_forward_checked_at = None
        self.corner_previous_front_cm = None
        self.corner_previous_front_at = None
        self.turn_progress_error = None
        self.turn_progress_at = None

        self.navigation_release_count = 0
        self.navigation_release_corner = COUNTER
        self.last_detection_capture_at = 0.0
        self.post_corner_slow_until = 0.0
        self.post_turn_block_priority_until = 0.0
        self.current_round2_sensor_data = None
        self.deep_recovery_target_heading = None
        self.deep_recovery_confirmation = 0
        self.deep_recovery_steering = None
        self.deep_recovery_steering_updated_at = None
        self.corner_recovery_evidence_count = 0
        self.corner_recovery_evidence_started_at = None
        self.corner_recovery_evidence_last_at = None
        self.corner_recovery_evidence_sequence = None
        self.corner_clearance_attempts = 0
        self.corner_clearance_start_center = None
        self.corner_clearance_best_center = None
        self.corner_clearance_ready_samples = 0
        self.corner_clearance_reverse_applied = False
        self.corner_reapproach_from_recovery = False
        self.recovery_camera_paused = False
        self.parking_reference_heading = None
        self.parking_exit_scan_count = 0
        self.parking_exit_last_capture_at = 0.0
        self.parking_exit_first_color = None
        self.corner_opposite_min_cm = None
        self.initial_corner_reverse_started_at = None
        self.corner_turn_mid_target = None
        self.corner_zone_locked = False
        self.corner_zone_lock_armed = True
        self.corner_zone_lock_count = 0
        self.corner_zone_lock_last_sequence = None
        self.corner_zone_lock_last_time = None
        self.pillar_reverse_heading = None
        self.pillar_release_side = None
        self.pillar_resume_waiting_frame = False
        self.emergency_escape_resume_state = None
        self.emergency_escape_started = None
        self.emergency_escape_attempt = 0
        self.emergency_escape_start_center = None
        self.emergency_escape_resume_elapsed = 0.0
        self.pillar_pass_commitment = None
        self.committed_recovery_cycles = 0
        self.post_turn_asymmetric_view_active = False
        self.post_turn_asymmetric_view_started = None

    def post_turn_view_fractions(self, now):
        """Return direction-aware (left, right) detector masks after a corner."""
        if self.post_turn_asymmetric_view_active:
            forward_seconds = self.next_corner_forward_seconds or 0.0
            elapsed = (
                0.0 if self.post_turn_asymmetric_view_started is None
                else now - self.post_turn_asymmetric_view_started
            )
            if (
                self.state == "FOLLOW_STRAIGHT"
                and (forward_seconds >= 1.50 or elapsed >= 12.0)
            ):
                self.release_post_turn_view(
                    f"new straight established; forward={forward_seconds:.2f}s elapsed={elapsed:.2f}s"
                )

        if self.post_turn_asymmetric_view_active:
            if DIRECTION == "clockwise":
                return 0.08, 0.35
            if DIRECTION == "anticlockwise":
                return 0.35, 0.08

        symmetric = camera_view_side_fraction(
            COUNTER, now, self.narrow_camera_view_until
        )
        return symmetric, symmetric

    def activate_post_turn_view(self):
        self.post_turn_asymmetric_view_active = True
        self.post_turn_asymmetric_view_started = time.monotonic()
        self.detector.reset_navigation_context()
        left, right = self.post_turn_view_fractions(time.monotonic())
        print(
            f"[ROUND2] POST-TURN VIEW PROTECTED: "
            f"left={left * 100:.0f}% right={right * 100:.0f}% direction={DIRECTION}"
        )

    def release_post_turn_view(self, reason, reset_detector=True):
        if not self.post_turn_asymmetric_view_active:
            return
        self.post_turn_asymmetric_view_active = False
        self.post_turn_asymmetric_view_started = None
        if reset_detector:
            self.detector.reset_navigation_context()
        print(f"[ROUND2] POST-TURN VIEW RELEASED: {reason}")

    def transition(self, new_state, reason=""):
        old_state = self.state
        super().transition(new_state, reason)
        if old_state == "APPROACH_CORNER" and new_state != "APPROACH_CORNER":
            self.corner_reapproach_from_recovery = False
        if self.run_logger is not None and (new_state != old_state or reason):
            self.run_logger.log_event(
                self,
                "state_transition",
                f"{old_state}->{new_state}: {reason}",
            )

    def record_run_event(self, event, detail=""):
        if self.run_logger is not None:
            self.run_logger.log_event(self, event, detail)

    def count_pillar_pass(self, detection):
        """Consume the pillar slot after physical pass completion."""
        if detection.track_id not in self.committed_pillar_tracks_this_straight:
            self.committed_pillar_tracks_this_straight.add(detection.track_id)
            now = time.monotonic()
            direct_second_slot = (
                self.pillar_slots_filled == 0
                and self.last_completed_corner_at is not None
                and now - self.last_completed_corner_at
                >= POST_CORNER_DIRECT_SECOND_SLOT_DELAY_SECONDS
            )
            self.assign_pillar_slots(
                detection.track_id, detection.color, now, direct_second_slot
            )

    def assign_pillar_slots(self, track_id, color, now, direct_second_slot=False):
        previous_slots = self.pillar_slots_filled
        self.pillar_slots_filled = (
            MAX_PILLARS_PER_STRAIGHT
            if direct_second_slot
            else min(MAX_PILLARS_PER_STRAIGHT, previous_slots + 1)
        )
        count = self.pillar_slots_filled
        if previous_slots == 0 and count == 1:
            self.first_pillar_slot_filled_at = now
        elif count >= MAX_PILLARS_PER_STRAIGHT:
            self.first_pillar_slot_filled_at = None
        reason = (
            f"post-corner block after {POST_CORNER_DIRECT_SECOND_SLOT_DELAY_SECONDS:.1f}s"
            if direct_second_slot else "ordinary pillar"
        )
        self.record_run_event(
            "pillar_count_for_straight",
            f"pillar {count}/{MAX_PILLARS_PER_STRAIGHT} reason={reason} "
            f"track={track_id} color={color}",
        )
        print(f"[ROUND2] PILLAR {count}/{MAX_PILLARS_PER_STRAIGHT}: "
              f"{reason}; track={track_id} color={color}")

    def enforce_second_slot_timeout(self):
        """Fill Slot 2 after the configured timeout and prioritize cornering."""
        if self.pillar_slots_filled != 1:
            return
        now = time.monotonic()
        if self.first_pillar_slot_filled_at is None:
            self.first_pillar_slot_filled_at = now
            return
        if now - self.first_pillar_slot_filled_at < SECOND_PILLAR_SLOT_TIMEOUT_SECONDS:
            return

        self.pillar_slots_filled = MAX_PILLARS_PER_STRAIGHT
        self.first_pillar_slot_filled_at = None
        self.corner_next_after_pillars = True
        self.corner_armed = True
        self.corner_confirmation = 0
        self.reset_corner_zone_lock(armed=True)
        self.reset_deferred_corner_track()
        if self.pillar_pass_commitment is None:
            self.detector.release_active()
        self.record_run_event(
            "second_slot_timeout",
            f"Slot 2 auto-filled after {SECOND_PILLAR_SLOT_TIMEOUT_SECONDS:.1f}s; "
            "pillar detection locked until corner",
        )
        print(
            f"[ROUND2] SLOT 2 AUTO-FILLED: {SECOND_PILLAR_SLOT_TIMEOUT_SECONDS:.1f}s "
            "elapsed; CORNER NEXT, ignoring all pillars"
        )
    def commit_pillar_pass(self, detection, heading, pass_state):
        """Latch the selected colour and manoeuvre until physical clearance."""
        self.count_pillar_pass(detection)
        pass_side = "right" if detection.color == "red" else "left"
        steering_sign = 1 if detection.color == "red" else -1
        self.pillar_pass_commitment = PillarPassCommitment(
            track_id=detection.track_id,
            color=detection.color,
            pass_side=pass_side,
            pass_state=pass_state,
            forward_steering_sign=steering_sign,
            entry_heading=heading,
            target_heading=target_angle,
            committed_at=time.monotonic(),
            last_steering=float(self.last_pillar_steering),
            last_speed=int(self.last_pillar_speed),
        )
        self.committed_recovery_cycles = 0
        self.record_run_event(
            "pillar_pass_committed",
            f"track={detection.track_id} color={detection.color} side={pass_side}",
        )
        print(
            f"[ROUND2] PILLAR PASS COMMITTED: track={detection.track_id} "
            f"color={detection.color} side={pass_side}; camera loss cannot change it"
        )

    def clear_pillar_pass_commitment(self, reason):
        commitment = self.pillar_pass_commitment
        if commitment is None:
            return
        self.record_run_event(
            "pillar_pass_released",
            f"track={commitment.track_id} color={commitment.color}; {reason}",
        )
        print(
            f"[ROUND2] PILLAR PASS RELEASED: track={commitment.track_id} "
            f"color={commitment.color}; {reason}"
        )
        self.pillar_pass_commitment = None
        self.committed_recovery_cycles = 0

    def resume_committed_pillar_pass(self, data, reason):
        commitment = self.pillar_pass_commitment
        if commitment is None:
            return False
        self.resume_camera_after_recovery()
        self.pillar_resume_waiting_frame = False
        self.deep_recovery_target_heading = None
        self.locked_track_id = commitment.track_id
        self.locked_pillar_color = commitment.color
        steering = float(commitment.last_steering)
        if commitment.forward_steering_sign > 0:
            steering = max(0.0, steering)
        else:
            steering = min(0.0, steering)
        speed = max(PILLAR_MIN_PASS_SPEED, int(commitment.last_speed))
        self.transition(commitment.pass_state, reason)
        self.command_robot(speed, 0, steering, force=True)
        self.record_run_event(
            "pillar_pass_resumed",
            f"state={commitment.pass_state} speed={speed} steer={steering:+.1f}",
        )
        return True

    def command_robot(self, speed, direction, steering, force=False):
        if (
            speed > 0
            and direction == 0
            and self.state in (
                "ACQUIRE_PILLAR", "PASS_RED_RIGHT", "PASS_GREEN_LEFT", "CONFIRM_PASSED"
            )
            and self.current_round2_sensor_data is not None
        ):
            left = self.current_round2_sensor_data[1]
            right = self.current_round2_sensor_data[3]
            mask = last_telemetry_status["valid_mask"]
            left_valid = bool(mask & VALID_LEFT) and _distance_is_valid(left)
            right_valid = bool(mask & VALID_RIGHT) and _distance_is_valid(right)
            left_pressure = (
                max(0.0, min(1.0, (PILLAR_WALL_WARNING_CM - left)
                                 / (PILLAR_WALL_WARNING_CM - PILLAR_WALL_EMERGENCY_CM)))
                if left_valid else 0.0
            )
            right_pressure = (
                max(0.0, min(1.0, (PILLAR_WALL_WARNING_CM - right)
                                 / (PILLAR_WALL_WARNING_CM - PILLAR_WALL_EMERGENCY_CM)))
                if right_valid else 0.0
            )

            # Installed steering sign: positive moves away from the left wall;
            # negative moves away from the right wall. Blend both sides so the
            # same rule protects inner and outer walls in either direction.
            wall_correction = (
                left_pressure - right_pressure
            ) * PILLAR_WALL_MAX_CORRECTION
            steering = float(steering) + wall_correction

            left_critical = left_valid and left <= PILLAR_WALL_CRITICAL_CM
            right_critical = right_valid and right <= PILLAR_WALL_CRITICAL_CM
            if left_critical and not right_critical:
                steering = max(steering, PILLAR_WALL_CRITICAL_STEER)
            elif right_critical and not left_critical:
                steering = min(steering, -PILLAR_WALL_CRITICAL_STEER)

            left_emergency = left_valid and left <= PILLAR_WALL_EMERGENCY_CM
            right_emergency = right_valid and right <= PILLAR_WALL_EMERGENCY_CM
            if left_emergency or right_emergency:
                speed = min(speed, PILLAR_WALL_EMERGENCY_SPEED)
            if left_emergency and not right_emergency:
                steering = max(steering, PILLAR_WALL_EMERGENCY_STEER)
            elif right_emergency and not left_emergency:
                steering = min(steering, -PILLAR_WALL_EMERGENCY_STEER)

            steering = max(-60.0, min(60.0, steering))
            if left_pressure > 0.0 or right_pressure > 0.0:
                self.notice(
                    f"[ROUND2 WALL GUARD] L={left} R={right} "
                    f"steer={steering:+.1f} speed={speed}",
                    period=0.25,
                )
        self.last_steering = float(steering)
        super().command_robot(speed, direction, steering, force=force)

    def heading_correction(self, heading, target=None, reverse=False):
        # A clearance reverse must preserve the avoidance angle already gained.
        if reverse and self.state == "HARD_STOP_RELEASE" and self.pillar_reverse_heading is not None:
            target = self.pillar_reverse_heading
        return super().heading_correction(heading, target=target, reverse=reverse)

    def emergency_escape_steering(self, data=None):
        """Reverse-wheel command which preserves the committed forward intent."""
        commitment = self.pillar_pass_commitment
        state = (
            commitment.pass_state if commitment is not None
            else self.emergency_escape_resume_state or self.state
        )
        color = commitment.color if commitment is not None else self.locked_pillar_color
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
        commitment = self.pillar_pass_commitment
        self.emergency_escape_resume_state = (
            commitment.pass_state if commitment is not None
            else resume_state or self.state
        )
        if commitment is not None:
            self.pause_camera_for_recovery()
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
        self.record_run_event(
            "emergency_escape_started",
            f"front={data[2]} resume={self.emergency_escape_resume_state} steer={steering:+d}",
        )
        self.command_robot(EMERGENCY_ESCAPE_SPEED, 1, steering, force=True)

    def finish_emergency_escape(self, reason):
        resume = self.emergency_escape_resume_state
        elapsed_before_escape = self.emergency_escape_resume_elapsed
        self.brake()
        if self.pillar_pass_commitment is not None:
            self.emergency_escape_resume_state = None
            self.emergency_escape_started = None
            self.emergency_escape_attempt = 0
            self.emergency_escape_start_center = None
            self.resume_committed_pillar_pass(
                self.current_round2_sensor_data,
                f"emergency reverse complete; {reason}",
            )
            return
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
        if (
            progress >= EMERGENCY_ESCAPE_MIN_PROGRESS_CM
            and center >= HARD_STOP_RELEASE_CM
            and not last_telemetry_status["hard_stop"]
        ):
            self.finish_emergency_escape(f"front improved by {progress:.1f}cm")
            return
        if self.emergency_escape_attempt >= EMERGENCY_ESCAPE_MAX_ATTEMPTS:
            self.record_run_event(
                "emergency_escape_extended",
                f"two pulses progress={progress:.1f}cm; starting bounded recovery",
            )
            self.start_deep_recovery(
                data,
                f"escape pulses made only {progress:.1f}cm progress",
            )
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
                     and center >= HARD_STOP_RELEASE_CM and progress >= HARD_STOP_MIN_PROGRESS_CM
                     and abs(error) <= TURN_HEADING_TOLERANCE
                     and (side_index is None or data[side_index] >= PILLAR_RELEASE_SIDE_CM))
            self.release_confirmation = self.release_confirmation + 1 if ready else 0
        if self.release_confirmation >= HARD_STOP_RELEASE_SAMPLES:
            self.recovery_camera_paused = False
            self.pillar_reverse_heading = None
            self.pillar_resume_waiting_frame = False
            self.pillar_lost_started_at = None
            if self.resume_committed_pillar_pass(
                data,
                "pillar recovery: front, side and heading ready",
            ):
                return
            self.transition(self.resume_state, "pillar recovery: front, side and heading ready")
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
        self.parking_exit_first_pillar_counted = self.parking_exit_first_color in ("red", "green")
        if self.parking_exit_first_pillar_counted:
            self.pillar_slots_filled = 1
            self.first_pillar_slot_filled_at = None
            self.corner_next_after_pillars = True
            self.record_run_event(
                "parking_exit_first_pillar_counted",
                f"color={self.parking_exit_first_color}; ignore further pillars until corner 1",
            )
            print(
                f"[PARKING EXIT] FIRST PILLAR COUNTED: {self.parking_exit_first_color}; "
                "follow straight to corner 1"
            )
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
            scan_offset = 45 if DIRECTION == "clockwise" else -45
            self.parking_arc(data, scan_offset, PARKING_SPEED, 0, 50 * sign,
                             "PARKING_EXIT_FORWARD_ALIGN",
                             "exit arc 3; short forward heading alignment")
            if self.state == "PARKING_EXIT_FORWARD_ALIGN":
                print(
                    "[PARKING EXIT] Moving forward with moderate steering to "
                    "bring the heading within +/-8 degrees"
                )
        elif state == "PARKING_EXIT_FORWARD_ALIGN":
            elapsed = time.monotonic() - self.state_started
            target = self.parking_target(0)
            heading_error = angle_diff(target, data[0])
            if (
                elapsed >= PARKING_EXIT_FORWARD_ALIGN_SECONDS
                and abs(heading_error) <= PARKING_EXIT_FORWARD_ALIGN_TOLERANCE
            ):
                self.brake()
                self.transition(
                    "PARKING_EXIT_REVERSE_VIEW",
                    f"forward alignment complete; heading error={heading_error:+.1f} deg",
                )
                print(
                    "[PARKING EXIT] Heading aligned within +/-8 degrees; "
                    "reversing with heading correction for exactly 5.0s"
                )
            elif elapsed >= PARKING_EXIT_FORWARD_ALIGN_TIMEOUT_SECONDS:
                self.brake()
                self.transition("FAULT", "parking-exit forward heading alignment timeout")
            else:
                steering = max(
                    -PARKING_EXIT_FORWARD_ALIGN_MAX_STEER,
                    min(
                        PARKING_EXIT_FORWARD_ALIGN_MAX_STEER,
                        heading_error * PARKING_EXIT_FORWARD_ALIGN_STEER_GAIN,
                    ),
                )
                self.command_robot(PARKING_SPEED, 0, steering)
        elif state == "PARKING_EXIT_REVERSE_VIEW":
            elapsed = time.monotonic() - self.state_started
            target = self.parking_target(0)
            heading_error = angle_diff(target, data[0])
            if elapsed >= PARKING_EXIT_REVERSE_VIEW_SECONDS:
                self.brake()
                self.record_run_event(
                    "parking_exit_reverse_complete",
                    f"5.0s reverse complete; accepting heading error={heading_error:+.1f} deg",
                )
                print(
                    "[PARKING EXIT] 5.0s reverse complete; parking exit finished; "
                    f"accepting heading error={heading_error:+.1f} deg"
                )
                self.complete_parking_exit()
            else:
                steering = self.heading_correction(data[0], target=target, reverse=True)
                self.command_robot(PARKING_SPEED, 1, steering)
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
            self.command_robot(PARKING_SPEED, 0, PARKING_EXIT_FIRST_PILLAR_STEER)
            return
        if self.state == "PARKING_EXIT_TIMED_CURVE":
            if self.parking_sequence_elapsed < 2.0:
                self.command_robot(
                    PARKING_SPEED,
                    0,
                    PARKING_EXIT_FIRST_PILLAR_STEER * sign,
                )
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












    def start_hard_stop_recovery(self, data, resume_state, reason):
        commitment = self.pillar_pass_commitment
        if commitment is not None:
            resume_state = commitment.pass_state
        self.pillar_reverse_heading = (
            data[0] if resume_state in (
                "ACQUIRE_PILLAR", "PASS_RED_RIGHT", "PASS_GREEN_LEFT", "CONFIRM_PASSED"
            ) else None
        )
        self.pillar_release_side = None
        if self.pillar_reverse_heading is not None:
            color = commitment.color if commitment is not None else self.locked_pillar_color
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
        self.record_run_event(
            "hard_stop_recovery_started",
            f"resume={resume_state} C={data[2]}; {reason}",
        )
        if self.corner_next_after_pillars and commitment is None:
            self.record_run_event(
                "corner_next_recovery",
                f"C={data[2]} L={data[1]} R={data[3]} headingError={angle_diff(target_angle, data[0]):+.1f}; {reason}",
            )
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
        if (self.pillar_pass_commitment is None
                and (any(not _distance_is_valid(v) or v <= PILLAR_RELEASE_SIDE_CM for v in (data[1], data[3]))
                or (time.monotonic() - self.state_started >= 0.4
                    and data[2] - self.recovery_start_center < HARD_STOP_MIN_PROGRESS_CM))):
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
            and (
                last_telemetry_status["protocol"] != "v2"
                or self.recovery_reverse_applied
            )
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

    def reset_corner_recovery_evidence(self):
        self.corner_recovery_evidence_count = 0
        self.corner_recovery_evidence_started_at = None
        self.corner_recovery_evidence_last_at = None
        self.corner_recovery_evidence_sequence = None

    def collect_corner_recovery_evidence(self, data, new_sample):
        # This proof remains available when the post-turn forward-distance gate
        # did not re-arm the normal corner-zone recognizer.
        if (
            not self.corner_next_after_pillars
            or last_telemetry_status["hard_stop"]
            or data[2] <= PYTHON_EMERGENCY_RELEASE_CM
            or not self.corner_sensors_agree(data, CORNER_TRIGGER_RELEASE_CM)
        ):
            self.reset_corner_recovery_evidence()
            return False
        sequence = last_telemetry_status["sequence"]
        if not new_sample or sequence is None or sequence == self.corner_recovery_evidence_sequence:
            return False
        now = time.monotonic()
        if (
            self.corner_recovery_evidence_last_at is None
            or now - self.corner_recovery_evidence_last_at > OBSTACLE_CORNER_MAX_SAMPLE_GAP_SECONDS
        ):
            self.corner_recovery_evidence_count = 0
            self.corner_recovery_evidence_started_at = now
        self.corner_recovery_evidence_count += 1
        self.corner_recovery_evidence_last_at = now
        self.corner_recovery_evidence_sequence = sequence
        return (
            self.corner_recovery_evidence_count >= CORNER_NEXT_CONFIRMATION_SAMPLES
            and now - self.corner_recovery_evidence_started_at >= CORNER_NEXT_CONFIRMATION_SECONDS
        )

    def start_corner_clearance_recovery(self, data):
        self.corner_clearance_attempts += 1
        if self.corner_clearance_attempts > MAX_RECOVERY_ATTEMPTS:
            self.brake()
            self.transition("FAULT", "corner clearance reverse made no reliable progress")
            return
        self.corner_clearance_start_center = data[2]
        self.corner_clearance_best_center = data[2]
        self.corner_clearance_ready_samples = 0
        self.corner_clearance_reverse_applied = False
        self.reset_corner_recovery_evidence()
        self.pause_camera_for_recovery()
        self.transition(
            "CORNER_CLEARANCE_RECOVERY",
            f"attempt={self.corner_clearance_attempts}/{MAX_RECOVERY_ATTEMPTS}; "
            f"L={data[1]} C={data[2]} R={data[3]}; target C={HARD_STOP_RELEASE_CM}",
        )
        self.brake()

    def update_corner_clearance_recovery(self, data, new_sample):
        left, center, right = data[1], data[2], data[3]
        elapsed = time.monotonic() - self.state_started
        if new_sample:
            self.corner_clearance_best_center = max(self.corner_clearance_best_center, center)
            applied_speed = last_telemetry_status["applied_speed"]
            if applied_speed is not None and applied_speed < 0:
                self.corner_clearance_reverse_applied = True
            ready = center >= HARD_STOP_RELEASE_CM and not last_telemetry_status["hard_stop"]
            self.corner_clearance_ready_samples = (
                self.corner_clearance_ready_samples + 1 if ready else 0
            )
        if self.corner_clearance_ready_samples >= HARD_STOP_RELEASE_SAMPLES:
            self.brake()
            self.resume_camera_after_recovery()
            self.reset_corner_recovery_evidence()
            self.transition("REASSESS_FRONT", "30 cm front clearance reached; recheck corner while stopped")
            return

        progress = self.corner_clearance_best_center - self.corner_clearance_start_center
        if left <= PILLAR_RELEASE_SIDE_CM and right <= PILLAR_RELEASE_SIDE_CM:
            self.brake()
            self.transition("FAULT", f"both side walls too close for corner reverse: L={left} R={right}")
            return
        if (
            last_telemetry_status["protocol"] == "v2"
            and elapsed >= HARD_STOP_APPLY_GRACE_SECONDS
            and not self.corner_clearance_reverse_applied
        ):
            self.brake()
            self.transition("FAULT", "ESP32 did not report an applied corner reverse")
            return
        if (
            elapsed >= CORNER_CLEARANCE_REVERSE_MAX_SECONDS
            or (elapsed >= 0.4 and progress < HARD_STOP_MIN_PROGRESS_CM)
        ):
            self.brake()
            self.resume_camera_after_recovery()
            self.reset_corner_recovery_evidence()
            self.transition(
                "REASSESS_FRONT",
                f"bounded corner reverse ended: C={center} progress={progress:.1f} cm",
            )
            return

        steering = self.heading_correction(data[0], reverse=True)
        if min(left, right) < CORNER_CLEARANCE_SIDE_NEAR_CM and abs(left - right) >= 3:
            # Reverse steering sign is opposite the nearer side wall.
            steering = -CORNER_CLEARANCE_REVERSE_STEER if left < right else CORNER_CLEARANCE_REVERSE_STEER
        self.command_robot(CORNER_CLEARANCE_REVERSE_SPEED, 1, steering)

    def update_reassess_front(self, data, detection, new_sample):
        self.brake()
        center = data[2]

        # A committed pillar pass is a physical manoeuvre, not a camera guess.
        # Once reverse clearance reaches 30 cm, resume the same colour/side even
        # if the pillar has disappeared and only the wall is visible.
        if self.pillar_pass_commitment is not None:
            if last_telemetry_status["hard_stop"] or center < HARD_STOP_RELEASE_CM:
                self.start_hard_stop_recovery(
                    data,
                    self.pillar_pass_commitment.pass_state,
                    "committed pass still inside 11-29 cm release range",
                )
                return
            self.resume_committed_pillar_pass(
                data,
                "front clearance restored; continuing committed pillar pass",
            )
            return

        # A confirmed side opening after slot 2 is a corner candidate even if
        # the normal post-turn distance gate missed its re-arm window. Rebuild
        # front clearance before allowing the corner approach to move again.
        if self.collect_corner_recovery_evidence(data, new_sample):
            if center < HARD_STOP_RELEASE_CM:
                self.start_corner_clearance_recovery(data)
                return
            self.corner_armed = True
            self.corner_zone_locked = True
            self.corner_reapproach_from_recovery = True
            self.reset_corner_evidence()
            self.reset_corner_recovery_evidence()
            self.record_run_event(
                "corner_reapproach_after_recovery",
                f"C={center} L={data[1]} R={data[3]} "
                f"headingError={angle_diff(target_angle, data[0]):+.1f}",
            )
            self.transition("APPROACH_CORNER", "30 cm clearance and corner opening confirmed")
            return

        # Camera fallback is deliberately recovery-only. Normal TF-Luna corner
        # proof above always gets the first chance. A confirmed broad black end
        # wall plus the expected-side opening can recover a corner when heading
        # error or a temporary side-sensor miss prevented that normal proof.
        if (
            self.last_detection_capture_at > self.state_started
            and self.detector.black_corner_confirmed
        ):
            self.corner_armed = True
            self.corner_zone_locked = True
            self.corner_reapproach_from_recovery = True
            self.reset_corner_evidence()
            self.reset_corner_recovery_evidence()
            detail = (
                f"side={self.detector.black_corner_expected_side} "
                f"centerCoverage={self.detector.black_corner_coverage:.2f} "
                f"sideCoverage={self.detector.black_corner_side_coverage:.2f} "
                f"C={center} headingError={angle_diff(target_angle, data[0]):+.1f}"
            )
            self.record_run_event("black_camera_corner_confirmed", detail)
            print(f"[BLACK CORNER] CONFIRMED during reassessment: {detail}")
            if last_telemetry_status["hard_stop"] or center < HARD_STOP_RELEASE_CM:
                self.start_corner_clearance_recovery(data)
            else:
                self.transition(
                    "APPROACH_CORNER",
                    "black-wall geometry confirmed after TF-Luna corner proof failed",
                )
            return

        # Wait for the next fresh camera assessment; the update gate checks
        # camera/telemetry freshness before this method is called.
        if not last_telemetry_status["hard_stop"] and self.last_detection_capture_at > self.state_started:
            if self.corner_zone_locked:
                if center >= HARD_STOP_RELEASE_CM and self.corner_sensors_agree(
                    data,
                    CORNER_TRIGGER_RELEASE_CM + 15,
                ):
                    self.reset_corner_evidence()
                    self.deep_recovery_target_heading = None
                    self.corner_reapproach_from_recovery = True
                    self.transition(
                        "APPROACH_CORNER",
                        "locked corner geometry restored after recovery",
                    )
                    return
                if data[2] >= HARD_STOP_RELEASE_CM:
                    self.reset_corner_zone_lock(armed=True)
                    self.reset_corner_evidence()
                    self.reset_deferred_corner_track()
                    self.detector.reset_navigation_context()
                    self.transition(
                        "FOLLOW_STRAIGHT",
                        "corner lock explicitly abandoned after recovery",
                    )
                    return
            if (
                center >= HARD_STOP_RELEASE_CM
                and detection is not None
                and detection.seen_this_frame
                and not detection.blocked_by_corner
            ):
                self.deep_recovery_target_heading = None
                self.enter_acquire(detection)
                return
            if data[2] >= HARD_STOP_RELEASE_CM:
                self.deep_recovery_target_heading = None
                self.deep_recovery_attempts = 0
                self.transition("FOLLOW_STRAIGHT", "post-release front clearance restored")
                return
        if time.monotonic() - self.state_started >= POST_RELEASE_RECHECK_SECONDS:
            if last_telemetry_status["hard_stop"] or center <= 10:
                committed_state = self.resume_state or "FOLLOW_STRAIGHT"
                self.begin_emergency_escape(data, resume_state=committed_state)
                return
            if center < HARD_STOP_RELEASE_CM:
                self.start_hard_stop_recovery(
                    data,
                    self.resume_state or "FOLLOW_STRAIGHT",
                    f"reassessment center={center}cm; continue bounded reverse release",
                )
                return
            self.deep_recovery_target_heading = None
            self.deep_recovery_attempts = 0
            self.resume_camera_after_recovery()
            self.transition("FOLLOW_STRAIGHT", "reassessment clear; resume navigation")

    def start_deep_recovery(self, data, reason):
        """Back away on the pre-corner heading instead of entering FAULT."""
        commitment = self.pillar_pass_commitment
        if self.corner_zone_locked and commitment is None:
            self.reset_corner_zone_lock()
            print("[ROUND2] TF-LUNA CORNER ZONE RELEASED: deep recovery abandoned corner")
        if commitment is None and self.state in ("HARD_STOP_RELEASE", "REASSESS_FRONT"):
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
            self.deep_recovery_attempts = 1
            self.record_run_event(
                "recovery_cycle_restarted",
                "recoverable retry budget reached; restarting bounded motion",
            )
        self.initial_corner_reverse_started_at = None
        self.pillar_reverse_heading = None
        self.pillar_resume_waiting_frame = False
        self.release_locked_pillar_for_recovery()
        self.pause_camera_for_recovery()
        self.deep_recovery_target_heading = data[0] if commitment is not None else target_angle
        self.deep_recovery_confirmation = 0
        self.deep_recovery_steering = None
        self.deep_recovery_steering_updated_at = None
        self.deep_recovery_start_center = data[2]
        self.pending_target = None
        self.turn_mode = None
        self.turn_attempts = 0
        self.corner_armed = True
        self.corner_confirmation = 0
        self.corner_approach_from_lines = False
        self.navigation_release_count = 0
        self.reset_corner_evidence()
        self.transition(
            "DEEP_REVERSE_RECOVERY",
            f"{reason}; align={self.deep_recovery_target_heading:.1f} "
            f"L={data[1]} C={data[2]} R={data[3]}",
        )
        self.record_run_event("deep_recovery_started", reason)
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
            if self.resume_committed_pillar_pass(
                data,
                "deep recovery reached 30 cm; continuing committed pillar pass",
            ):
                return
            self.brake()
            self.deep_recovery_target_heading = None
            self.deep_recovery_attempts = 0
            self.resume_camera_after_recovery()
            self.transition("REASSESS_FRONT", "clearance restored; no heading-chasing reverse")
            return
        if (not side_clear or elapsed >= DEEP_RECOVERY_MAX_SECONDS
                or (elapsed >= 0.4 and progress < HARD_STOP_MIN_PROGRESS_CM)):
            if self.pillar_pass_commitment is not None:
                self.committed_recovery_cycles += 1
                self.start_hard_stop_recovery(
                    data,
                    self.pillar_pass_commitment.pass_state,
                    f"committed recovery cycle {self.committed_recovery_cycles}; "
                    f"C={center} progress={progress:.1f}",
                )
                return
            self.brake()
            self.resume_camera_after_recovery()
            self.transition("REASSESS_FRONT", "reverse budget/clearance/progress limit; hold for fresh assessment")
            return

        # Hold each wheel angle long enough to alter the reverse path before
        # reacting to the next heading sample. The loop runs about every 10 ms.
        now = time.monotonic()
        if (
            self.deep_recovery_steering is None
            or now - self.deep_recovery_steering_updated_at >= DEEP_RECOVERY_STEER_UPDATE_SECONDS
        ):
            self.deep_recovery_steering = self.heading_correction(
                heading, target=self.deep_recovery_target_heading, reverse=True
            )
            self.deep_recovery_steering_updated_at = now
        self.command_robot(DEEP_RECOVERY_SPEED, 1, self.deep_recovery_steering)

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
        if self.last_completed_corner_at is not None:
            self.narrow_camera_view_until = time.monotonic() + POST_CORNER_NARROW_VIEW_SECONDS
        # Blocks are actionable immediately. Tape/corner recognition is armed
        # later, after the centre TF-Luna proves that the new straight is open.
        self.vision_corner_armed = False
        print(
            "[ROUND2] CAMERA DETECTION ON: scanning pillars during post-turn backup; "
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

    def corner_sensors_agree(self, data, front_limit=OBSTACLE_CORNER_TRIGGER_CM):
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

    def reset_corner_zone_lock(self, armed=False):
        self.corner_zone_locked = False
        self.corner_zone_lock_armed = armed
        self.corner_zone_lock_count = 0
        self.corner_zone_lock_last_sequence = None
        self.corner_zone_lock_last_time = None

    def update_corner_zone_lock(self, data, detection, new_sample):
        """Enter the real corner approach after three fresh geometry samples."""
        now = time.monotonic()
        if new_sample and _distance_is_valid(data[2]):
            previous_front = self.corner_previous_front_cm
            previous_at = self.corner_previous_front_at
            self.corner_previous_front_cm = data[2]
            self.corner_previous_front_at = now
            pillar_in_front = (
                self.vision_enabled
                and detection is not None
                and detection.seen_this_frame
                and not detection.blocked_by_corner
                and detection.normalized_area >= CORNER_PILLAR_MIN_AREA_RATIO
                and detection.bbox[0] <= CAMERA_WIDTH * 0.58
                and detection.bbox[0] + detection.bbox[2] >= CAMERA_WIDTH * 0.42
            )
            if (
                self.state == "FOLLOW_STRAIGHT"
                and not self.corner_zone_locked
                and previous_front is not None
                and previous_at is not None
                and now - previous_at <= CORNER_PILLAR_FRONT_DROP_MAX_GAP_SECONDS
                and previous_front - data[2] >= CORNER_PILLAR_FRONT_DROP_CM
                and data[2] <= CORNER_PILLAR_FRONT_NEAR_CM
                and pillar_in_front
                and not self.pillar_limit_reached()
            ):
                # Keep the recognizer armed so the real end wall can still be
                # counted after this pillar is physically cleared.
                self.reset_corner_zone_lock(armed=True)
                self.enter_acquire(detection)
                self.notice(
                    f"[ROUND2] SUDDEN FRONT DROP: pillar {detection.color} "
                    f"at C={data[2]}cm; keep driving and confirm pillar before corner",
                    period=0.5,
                )
                return True
        if self.corner_zone_locked:
            if self.state == "FOLLOW_STRAIGHT":
                self.transition(
                    "APPROACH_CORNER",
                    "restored locked-corner state invariant",
                )
            return True

        # After a completed corner, keep only the next-corner recognizers
        # disarmed until the centre sensor sees a clearly open new straight.
        # Camera block detection remains active throughout this re-arm period.
        if not self.corner_zone_lock_armed:
            if not new_sample:
                return False
            sequence = last_telemetry_status["sequence"]
            center = data[2]
            fresh_open_straight = (
                sequence is not None
                and sequence != self.corner_zone_lock_last_sequence
                and telemetry_is_fresh()
                and bool(last_telemetry_status["valid_mask"] & VALID_CENTER)
                and _distance_is_valid(center)
                and center >= CORNER_ZONE_REARM_CM
            )
            if (
                not fresh_open_straight
                or self.corner_zone_lock_last_time is None
                or now - self.corner_zone_lock_last_time
                > CORNER_ZONE_LOCK_MAX_SAMPLE_GAP_SECONDS
            ):
                self.corner_zone_lock_count = 0
            if fresh_open_straight:
                self.corner_zone_lock_count += 1
            self.corner_zone_lock_last_sequence = sequence
            self.corner_zone_lock_last_time = now
            if (
                self.corner_zone_lock_count >= CORNER_ZONE_REARM_SAMPLES
                and now >= self.next_corner_eligible_at
                and (
                    self.next_corner_forward_seconds is None
                    or self.next_corner_forward_seconds >= NEXT_CORNER_MIN_FORWARD_SECONDS
                )
            ):
                self.corner_zone_lock_armed = True
                self.corner_zone_lock_count = 0
                self.vision_corner_armed = True
                print(
                    f"[ROUND2] NEXT CORNER RE-ARMED: C={center} cm for "
                    f"{CORNER_ZONE_REARM_SAMPLES} fresh samples"
                )
            return False

        eligible_states = {"FOLLOW_STRAIGHT"}
        if self.corner_next_after_pillars:
            eligible_states.add("PILLAR_RECENTER")
        if self.state not in eligible_states or not self.corner_armed:
            self.corner_zone_lock_count = 0
            self.corner_zone_lock_last_sequence = None
            self.corner_zone_lock_last_time = None
            return False

        if not new_sample:
            return False

        if (
            now < self.next_corner_eligible_at
            or (
                self.next_corner_forward_seconds is not None
                and self.next_corner_forward_seconds < NEXT_CORNER_MIN_FORWARD_SECONDS
            )
        ):
            self.corner_zone_lock_count = 0
            return False
        sequence = last_telemetry_status["sequence"]
        heading, _, center, _, _ = data
        qualifies = (
            sequence is not None
            and sequence != self.corner_zone_lock_last_sequence
            and not last_telemetry_status["hard_stop"]
            and self.corner_sensors_agree(data, CORNER_ZONE_LOCK_CM)
            and center > PYTHON_EMERGENCY_RELEASE_CM
        )

        if not qualifies:
            self.corner_zone_lock_count = 0
            self.corner_zone_lock_last_sequence = sequence
            self.corner_zone_lock_last_time = now
            return False

        if (
            self.corner_zone_lock_last_time is None
            or now - self.corner_zone_lock_last_time > CORNER_ZONE_LOCK_MAX_SAMPLE_GAP_SECONDS
        ):
            self.corner_zone_lock_count = 0

        self.corner_zone_lock_count += 1
        self.corner_zone_lock_last_sequence = sequence
        self.corner_zone_lock_last_time = now
        if self.corner_zone_lock_count < CORNER_ZONE_LOCK_SAMPLES:
            self.notice(
                f"[ROUND2] TF-LUNA CORNER ZONE: {self.corner_zone_lock_count}/"
                f"{CORNER_ZONE_LOCK_SAMPLES}, C={center} cm",
                period=0.15,
            )
            return False

        self.corner_zone_locked = True
        if 0 < self.pillar_slots_filled < MAX_PILLARS_PER_STRAIGHT:
            self.corner_next_after_pillars = True
            self.record_run_event(
                "one_slot_corner_fallback",
                f"confirmed end-wall geometry with {self.pillar_slots_filled}/"
                f"{MAX_PILLARS_PER_STRAIGHT} slots",
            )
            print(
                f"[ROUND2] ONE-SLOT CORNER FALLBACK: confirmed wall geometry; "
                f"accepting {self.pillar_slots_filled}/{MAX_PILLARS_PER_STRAIGHT}"
            )
        self.corner_after_pillar_pending = False
        self.corner_approach_from_lines = False
        self.vision_corner_armed = False
        self.locked_track_id = None
        self.locked_pillar_color = None
        self.pillar_was_near = False
        self.pillar_lost_started_at = None
        self.filtered_pillar_correction = 0.0
        self.last_pillar_steering = 0.0
        self.detector.release_active()
        self.reset_deferred_corner_track()
        self.reset_corner_evidence()
        self.transition(
            "APPROACH_CORNER",
            f"TF-Luna corner geometry locked at C={center} cm",
        )
        print(
            f"[ROUND2] CORNER LOCK -> APPROACH_CORNER: C={center} cm; "
            "TF-Luna geometry confirmed 3/3"
        )
        return True

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
        allowed_spread = (
            CORNER_NEXT_HEADING_SPREAD
            if self.corner_next_after_pillars
            else OBSTACLE_CORNER_HEADING_SPREAD
        )
        if max_error - min_error > allowed_spread:
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
        required_samples = (
            CORNER_NEXT_CONFIRMATION_SAMPLES
            if self.corner_next_after_pillars
            else OBSTACLE_CORNER_CONFIRMATION_SAMPLES
        )
        required_seconds = (
            CORNER_NEXT_CONFIRMATION_SECONDS
            if self.corner_next_after_pillars
            else OBSTACLE_CORNER_CONFIRMATION_SECONDS
        )
        return (
            self.corner_confirmation >= required_samples
            and self.corner_evidence_started_at is not None
            and self.corner_evidence_last_at is not None
            and self.corner_evidence_last_at - self.corner_evidence_started_at
            >= required_seconds
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
            "CORNER_CLEARANCE_RECOVERY",
        ):
            required |= VALID_LEFT | VALID_RIGHT
        return (
            telemetry_is_fresh()
            and (mask & required) == required
            and data[0] is not None
            and data[2] is not None
            and (self.state != "CORNER_CLEARANCE_RECOVERY" or (
                _distance_is_valid(data[1]) and _distance_is_valid(data[3])
            ))
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
        # Use the same steering limits for both colours; only pass direction
        # changes. Keep the shared clearance boost for either side.
        profile = dict(profile)
        profile["min_steer"] = min(
            60.0, profile["min_steer"] + PILLAR_AVOIDANCE_STEER_BOOST_DEGREES
        )
        profile["max_steer"] = min(
            60.0, profile["max_steer"] + PILLAR_AVOIDANCE_STEER_BOOST_DEGREES
        )
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
        if detection.color == "green" and detection.seen_this_frame:
            # Do not straighten as soon as green crosses its image target. Keep
            # a modest pass-side command until physical/camera clearance begins.
            # The heading hard limit below and the TF-Luna wall guard remain
            # authoritative safety constraints.
            steering = min(-PILLAR_GREEN_VISIBLE_HOLD_STEER, steering)
        # Keep the pass moving, but do not let one visible pillar turn the
        # chassis away from its lane heading for several seconds.
        excursion = abs(angle_diff(target_angle, heading))
        if excursion > PILLAR_HEADING_SOFT_LIMIT_DEGREES:
            fraction = max(
                0.0,
                (PILLAR_HEADING_HARD_LIMIT_DEGREES - excursion)
                / (PILLAR_HEADING_HARD_LIMIT_DEGREES - PILLAR_HEADING_SOFT_LIMIT_DEGREES),
            )
            steering = math.copysign(min(abs(steering), TURN_STEER * fraction), steering)
            self.notice(
                f"[ROUND2] PILLAR HEADING EASE: {excursion:.0f}deg, "
                f"steer={steering:+.0f}; pass continues",
                period=0.7,
            )
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
        if self.pillar_pass_commitment is not None:
            commitment = self.pillar_pass_commitment
            commitment.last_steering = (
                max(0.0, steering)
                if commitment.forward_steering_sign > 0
                else min(0.0, steering)
            )
            commitment.last_speed = speed
        self.command_robot(speed, 0, steering, force=force)
        return speed, steering

    def enter_acquire(self, detection):
        if self.post_turn_asymmetric_view_active and detection is not None:
            x, _, width, _ = detection.bbox
            center_x = x + width * 0.5
            left_fraction, right_fraction = self.post_turn_view_fractions(
                time.monotonic()
            )
            left_limit = CAMERA_WIDTH * left_fraction
            right_limit = CAMERA_WIDTH * (1.0 - right_fraction)
            if center_x < left_limit or center_x > right_limit:
                print(
                    f"[ROUND2] TRAILING PILLAR REJECTED: "
                    f"track={detection.track_id} centerX={center_x:.0f}"
                )
                return False
        if (
            detection is None
            or self.corner_zone_locked
            or self.pillar_limit_reached()
            or detection.track_id in self.committed_pillar_tracks_this_straight
        ):
            return False
        self.reset_deferred_corner_track()
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
        if self.post_turn_asymmetric_view_active:
            self.release_post_turn_view(
                f"new-straight pillar acquired; track={detection.track_id}",
                reset_detector=False,
            )
        return True

    def pillar_limit_reached(self):
        return (
            (self.parking_exit_first_pillar_counted and COUNTER == 0)
            or self.pillar_slots_filled >= MAX_PILLARS_PER_STRAIGHT
        )

    def candidate_proves_foreground(self, detection):
        """Use camera size and height, never tape depth, for a near pillar."""
        if detection is None or detection.blocked_by_corner:
            return False
        return (
            detection.normalized_area >= PILLAR_AREA_NEAR_RATIO
            and detection.bottom_norm >= PILLAR_NEAR_BOTTOM
        )

    def reset_deferred_corner_track(self):
        self.deferred_corner_track_id = None
        self.deferred_corner_initial_area = None
        self.deferred_corner_initial_bottom = None
        self.deferred_track_promoted = False
        self.detector.deferred_track_id = None

    def early_corner_side_context(self, data):
        side = {"anticlockwise": (1, VALID_LEFT), "clockwise": (3, VALID_RIGHT)}.get(DIRECTION)
        if side is None or not telemetry_is_fresh():
            return False
        side_index, side_bit = side
        required = VALID_HEADING | side_bit
        if (last_telemetry_status["valid_mask"] & required) != required:
            return False
        heading = data[0]
        opening = data[side_index]
        return (
            heading is not None
            and _distance_is_valid(opening)
            and opening >= BACKGROUND_EARLY_SIDE_OPEN_CM
            and abs(angle_diff(target_angle, heading))
            <= OBSTACLE_CORNER_HEADING_TOLERANCE
        )

    def defer_background_candidate(self, data, detection):
        """Defer one high next-lane track; never suppress its whole colour."""
        self.deferred_track_promoted = False
        if time.monotonic() < self.post_turn_block_priority_until:
            # The first post-turn view belongs to nearby pillars. The
            # early-side-opening rule must not hide a block during this grace.
            self.reset_deferred_corner_track()
            return False
        if detection is None:
            return False
        if not detection.seen_this_frame:
            if detection.miss_count >= PILLAR_REQUIRED_LOST_FRAMES:
                self.reset_deferred_corner_track()
            return self.deferred_corner_track_id == detection.track_id

        if detection.blocked_by_corner:
            return True

        same_deferred_track = self.deferred_corner_track_id == detection.track_id
        if same_deferred_track:
            area_growth = (
                self.deferred_corner_initial_area is not None
                and detection.normalized_area
                >= self.deferred_corner_initial_area * BACKGROUND_FOREGROUND_GROWTH_RATIO
            )
            bottom_growth = (
                self.deferred_corner_initial_bottom is not None
                and detection.bottom_norm
                >= self.deferred_corner_initial_bottom
                + BACKGROUND_FOREGROUND_BOTTOM_GROWTH
            )
            if self.candidate_proves_foreground(detection) or (
                area_growth and bottom_growth
            ):
                self.notice(
                    f"[ROUND2] DEFERRED TRACK PROMOTED TO FOREGROUND: "
                    f"track={detection.track_id}",
                    period=0.2,
                )
                self.reset_deferred_corner_track()
                self.deferred_track_promoted = True
                return False
            self.notice(
                f"[ROUND2] BACKGROUND DEFERRED: EARLY SIDE OPENING "
                f"track={detection.track_id}",
                period=0.5,
            )
            return True

        self.reset_deferred_corner_track()
        if self.early_corner_side_context(data) and not self.candidate_proves_foreground(detection):
            self.deferred_corner_track_id = detection.track_id
            self.deferred_corner_initial_area = detection.normalized_area
            self.deferred_corner_initial_bottom = detection.bottom_norm
            self.detector.deferred_track_id = detection.track_id
            self.notice(
                f"[ROUND2] BACKGROUND DEFERRED: EARLY SIDE OPENING "
                f"track={detection.track_id}",
                period=0.5,
            )
            return True
        return False


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
            and item.bottom_norm >= PILLAR_NEAR_BOTTOM
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

    def update_parking_block_avoidance(self, data, detection):
        """Gently steer around a confirmed wide obstacle during laps 1 and 2."""
        block = self.detector.parking_block
        if (
            COUNTER >= 8
            or self.state not in ("FOLLOW_STRAIGHT", "ACQUIRE_PILLAR")
            or self.corner_zone_locked
            or block is None
            or not block.confirmed
            or block.miss_count >= PARKING_BLOCK_MAX_MISSES
            or last_telemetry_status["hard_stop"]
            or data[2] <= PYTHON_EMERGENCY_RELEASE_CM
        ):
            return False

        # A confirmed foreground pillar always owns navigation.  The exception
        # is an unconfirmed colour track created from the wide parking object
        # itself; the shape proof is then allowed to cancel that acquisition.
        if (
            self.state == "ACQUIRE_PILLAR"
            and detection is not None
            and detection.confirmed
        ):
            return False

        x, _, width, _ = block.bbox
        block_left = x / float(CAMERA_WIDTH)
        block_right = (x + width) / float(CAMERA_WIDTH)
        overlaps_path = (
            block_right >= PARKING_BLOCK_PATH_LEFT_RATIO
            and block_left <= PARKING_BLOCK_PATH_RIGHT_RATIO
        )
        if not overlaps_path:
            return False

        if self.state == "ACQUIRE_PILLAR":
            self.detector.release_active()
            self.locked_track_id = None
            self.locked_pillar_color = None
            self.filtered_pillar_correction = 0.0
            self.transition(
                "FOLLOW_STRAIGHT",
                "wide parking block replaced unconfirmed colour track",
            )

        if block.center_x_norm < 0.45:
            steering = PARKING_BLOCK_AVOID_STEER
        elif block.center_x_norm > 0.55:
            steering = -PARKING_BLOCK_AVOID_STEER
        else:
            # A centred block has no image-side preference; use the TF-Luna
            # side with more clearance and still keep the correction gentle.
            steering = (
                PARKING_BLOCK_AVOID_STEER
                if data[3] >= data[1]
                else -PARKING_BLOCK_AVOID_STEER
            )

        side = "right" if steering > 0 else "left"
        self.command_robot(PARKING_BLOCK_AVOID_SPEED, 0, steering)
        self.notice(
            f"[ROUND2] PARKING BLOCK AVOID: steer {side} {abs(steering)} deg; "
            f"W/H={block.width_over_height:.1f}",
            period=0.5,
        )
        return True

    def update_follow_straight(self, data, detection, new_sample):
        heading, _, center, _, _ = data
        hard_stop = last_telemetry_status["hard_stop"]

        if new_sample and not self.corner_armed and center >= CORNER_ZONE_REARM_CM:
            self.corner_armed = True
            self.corner_confirmation = 0
            print("[ROUND2] Corner detector re-armed")

        if detection is not None and not detection.seen_this_frame and detection.miss_count >= PILLAR_REQUIRED_LOST_FRAMES:
            self.detector.release_active()
            detection = None

        if (
            detection is not None
            and detection.seen_this_frame
            and not detection.blocked_by_corner
            and not self.pillar_limit_reached()
            and detection.track_id not in self.committed_pillar_tracks_this_straight
        ):
            self.enter_acquire(detection)
            return

        if hard_stop or center <= PYTHON_EMERGENCY_RELEASE_CM:
            self.start_hard_stop_recovery(data, "FOLLOW_STRAIGHT", "release front interlock")
            return

        straight_speed = (
            min(FOLLOW_STRAIGHT_SPEED, POST_CORNER_FOLLOW_SPEED)
            if time.monotonic() < self.post_corner_slow_until
            else FOLLOW_STRAIGHT_SPEED
        )
        self.command_robot(
            straight_speed,
            0,
            self.heading_correction(heading),
        )

    def update_acquire_pillar(self, data, detection):
        heading = data[0]
        elapsed = time.monotonic() - self.state_started
        acquire_speed = (
            min(PILLAR_APPROACH_SPEED, 70)
            if data[2] <= CORNER_PILLAR_FRONT_NEAR_CM
            else PILLAR_APPROACH_SPEED
        )

        if (
            detection is None
            or detection.track_id != self.locked_track_id
            or not detection.seen_this_frame
        ):
            self.command_robot(min(SPEED, acquire_speed), 0, self.heading_correction(heading))
            if elapsed >= PILLAR_ACQUIRE_TIMEOUT_SECONDS:
                self.detector.release_active()
                self.transition("FOLLOW_STRAIGHT", "candidate did not confirm")
            return

        if detection.confirmed:
            # Detection confirmation means the block is real, not yet close
            # enough to turn around. The classic block controller started
            # avoidance at 4,000 pixels; use its resolution-normalized threshold.
            if detection.normalized_area < PILLAR_AREA_NEAR_RATIO:
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
            self.commit_pillar_pass(detection, heading, next_state)
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
            min(SPEED, acquire_speed),
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
            return_state = (
                self.pillar_pass_commitment.pass_state
                if self.pillar_pass_commitment is not None
                else "PASS_RED_RIGHT" if detection.color == "red" else "PASS_GREEN_LEFT"
            )
            self.transition(return_state, "pillar still visible; continue clearance")
            self.pillar_lost_started_at = None
            self.command_pillar_pass(heading, detection, force=True)
            return
       
       
        if elapsed < PILLAR_CLEARANCE_HOLD_SECONDS:
            clearance_steering = self.last_pillar_steering
            if self.locked_pillar_color == "red":
                remaining = max(
                    0.0,
                    1.0 - elapsed / PILLAR_CLEARANCE_HOLD_SECONDS,
                )
                clearance_steering = (
                    min(PILLAR_RED_CLEARANCE_MAX_STEER,
                        max(0.0, self.last_pillar_steering))
                    * remaining
                )
            self.command_robot(
                self.last_pillar_speed, 0, clearance_steering
            )
            return

        if elapsed >= PILLAR_CLEARANCE_HOLD_SECONDS:
            color = self.locked_pillar_color
            self.last_passed_pillar_color = color
            print(f"[ROUND2] PILLAR PASSED: {color}, track={self.locked_track_id}")
            self.detector.release_active()
            self.locked_track_id = None
            self.locked_pillar_color = None
            self.pillar_lost_started_at = None
            self.last_pillar_passed_at = time.monotonic()
            self.filtered_pillar_correction = 0.0
            self.navigation_release_count = 0
            self.deep_recovery_attempts = 0
            self.clear_pillar_pass_commitment("physical clearance movement complete")
            if self.pillar_limit_reached():
                self.corner_next_after_pillars = True
                # The 130 cm re-arm window can occur while a pillar owns
                # control, before FOLLOW_STRAIGHT gets a chance to set this
                # separate flag. Slot 2 clearance is the other valid re-arm
                # event; corner entry still needs fresh TF-Luna geometry.
                self.corner_armed = True
                self.corner_confirmation = 0
                self.reset_corner_zone_lock(armed=True)
                self.record_run_event(
                    "corner_next_after_pillar_limit",
                    "pillar limit reached; TF-Luna corner proof required",
                )
                print("[ROUND2] CORNER NEXT: both slots filled; waiting for TF-Luna confirmation")
               
            next_state = "FOLLOW_STRAIGHT"
            self.pillar_recenter_next_state = next_state
            self.pillar_recenter_centered_samples = 0
            self.transition("PILLAR_RECENTER", f"pillar {color} cleared; recenter before {next_state}")
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
            and self.candidate_proves_foreground(detection)
        )
        if (
            near_foreground_pillar
            and not self.corner_zone_locked
            and not self.pillar_limit_reached()
            and detection.track_id not in self.committed_pillar_tracks_this_straight
        ):
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

        self.collect_corner_evidence(data, new_sample)

        if self.corner_evidence_confirmed() and (
            not self.corner_reapproach_from_recovery or center >= HARD_STOP_RELEASE_CM
        ):
            if self.begin_corner(data):
                return

        if last_telemetry_status["hard_stop"] or center <= PYTHON_EMERGENCY_RELEASE_CM:
            self.reset_corner_evidence()
            self.start_hard_stop_recovery(data, "APPROACH_CORNER", "release during corner approach")
            return

        if self.corner_reapproach_from_recovery:
            # The recovery approach gathers fresh turn proof while stationary.
            # Driving forward here could erase the clearance just recovered.
            self.brake()
            if elapsed >= POST_RELEASE_RECHECK_SECONDS:
                self.corner_reapproach_from_recovery = False
                self.reset_corner_evidence()
                self.reset_corner_zone_lock(armed=True)
                self.transition(
                    "REASSESS_FRONT" if center < HARD_STOP_RELEASE_CM else "FOLLOW_STRAIGHT",
                    "recovered corner proof did not persist",
                )
            return

        if (
            not self.corner_zone_locked
            and not self.corner_approach_from_lines
            and not self.corner_sensors_agree(data, CORNER_TRIGGER_RELEASE_CM + 15)
        ):
            self.corner_confirmation = 0
            self.transition("FOLLOW_STRAIGHT", "two-sensor corner evidence cleared")
            return

        approach_timeout = (
            VISION_CORNER_APPROACH_TIMEOUT_SECONDS
            if self.corner_approach_from_lines or self.corner_zone_locked
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
                    self.command_robot(CORNER_PENDING_SPEED, 0, self.heading_correction(heading))
                    return
                self.corner_approach_from_lines = False
                self.corner_confirmation = 0
                self.transition(
                    "FOLLOW_STRAIGHT",
                    "vision corner not confirmed; continuing sensor navigation",
                )
                return
            self.reset_corner_zone_lock(armed=True)
            self.reset_corner_evidence()
            self.reset_deferred_corner_track()
            self.detector.reset_navigation_context()
            self.record_run_event(
                "corner_approach_abandoned",
                "opening not confirmed before timeout; recover and resume navigation",
            )
            if (
                not last_telemetry_status["hard_stop"]
                and center >= HARD_STOP_RELEASE_CM
            ):
                self.resume_camera_after_recovery()
                self.transition(
                    "FOLLOW_STRAIGHT",
                    "corner approach timeout; lock cleared and navigation resumed",
                )
            else:
                self.start_hard_stop_recovery(
                    data,
                    "FOLLOW_STRAIGHT",
                    "corner approach timeout near wall; bounded reverse",
                )
            return

        self.command_robot(
            CORNER_FINAL_APPROACH_SPEED if center <= CORNER_FINAL_APPROACH_CM else CORNER_PENDING_SPEED,
            0,
            self.heading_correction(heading),
        )

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
        if self.corner_next_after_pillars:
            self.record_run_event(
                "corner_next_confirmed",
                f"C={data[2]} L={data[1]} R={data[3]} headingError={angle_diff(target_angle, data[0]):+.1f}",
            )
        self.suspend_block_detection()
        self.corner_reapproach_from_recovery = False
        self.corner_armed = False
        self.corner_confirmation = 0
        self.corner_approach_from_lines = False
        self.pending_target = normalize_angle(
            target_angle + (-90 if DIRECTION == "anticlockwise" else 90)
        )
        self.corner_turn_mid_target = normalize_angle(
            target_angle
            + (-CORNER_TWO_STEP_SPLIT_DEGREES
               if DIRECTION == "anticlockwise"
               else CORNER_TWO_STEP_SPLIT_DEGREES)
        )
        self.turn_attempts = 0
        self.recenter_next_state = "FOLLOW_STRAIGHT"
        self.recenter_centered_samples = 0
        self.turn_mode = "reverse" if data[2] <= TURN_DIRECT_REVERSE_FRONT_CM else "forward"
        progress_target = self.pending_target if self.turn_mode == "reverse" else self.corner_turn_mid_target
        self.turn_progress_error = abs(angle_diff(progress_target, data[0]))
        self.turn_progress_at = time.monotonic()
        self.initial_corner_reverse_started_at = None
        self.transition(
            "TURN_90",
            (
                f"close wall C={data[2]}cm: reverse arc toward {self.pending_target:.1f}"
                if self.turn_mode == "reverse"
                else f"two-step forward arc to {self.corner_turn_mid_target:.1f}, "
                     f"then reverse arc to {self.pending_target:.1f}"
            ),
        )
        return True

    def finish_turn(self):
        global COUNTER, target_angle, last_turn, last_cooldown

        self.corner_clearance_attempts = 0
        self.reset_corner_recovery_evidence()
        self.initial_corner_reverse_started_at = None
        self.corner_turn_mid_target = None
        target_angle = self.pending_target
        COUNTER += 1
        if COUNTER == 1:
            self.parking_exit_first_pillar_counted = False
        self.committed_pillar_tracks_this_straight.clear()
        self.pillar_slots_filled = 0
        self.first_pillar_slot_filled_at = None
        self.last_completed_corner_at = time.monotonic()
        self.corner_next_after_pillars = False
        self.corner_next_ignored_tracks.clear()
        self.record_run_event("pillar_count_reset", f"corner={COUNTER}; new straight 0/{MAX_PILLARS_PER_STRAIGHT}")
        print(f"[ROUND2] PILLAR COUNTER RESET: 0/{MAX_PILLARS_PER_STRAIGHT} after corner {COUNTER}")
        self.post_corner_slow_until = time.monotonic() + POST_CORNER_FOLLOW_SECONDS
        last_turn = time.time()
        last_cooldown = time.time()
        self.pending_target = None
        self.turn_attempts = 0
        self.turn_progress_error = None
        self.turn_progress_at = None
        self.corner_previous_front_cm = None
        self.corner_previous_front_at = None
        self.next_corner_forward_seconds = None
        self.next_corner_forward_checked_at = None
        print(f"[NAV] CORNER {COUNTER}/{COUNTER_MAX} COMPLETE; target={target_angle:.1f}")
        if COUNTER >= COUNTER_MAX:
            if ENABLE_PARKING_IN:
                self.command_robot(0, 0, 0, force=True)
                self.transition(
                    "PARKING_PARALLEL_HANDOFF",
                    "final corner complete; start parallel parking controller",
                )
            else:
                self.transition("COMPLETE", "three laps / 12 corners complete")
                self.command_robot(0, 0, 0, force=True)
        else:
            self.transition("POST_TURN_BACKUP", "backing up to maximize camera view")
            self.post_turn_block_priority_until = time.monotonic() + POST_TURN_TAPE_IGNORE_GRACE_SECONDS
            self.resume_block_detection()
            self.activate_post_turn_view()

    def update_turn_90(self, data, new_sample):
        heading, _, center, _, _ = data
        now = time.monotonic()
        if abs(angle_diff(self.pending_target, heading)) <= TURN_HEADING_TOLERANCE:
            self.finish_turn()
            return
        if self.turn_mode == "forward":
            mid_error = angle_diff(self.corner_turn_mid_target, heading)
            if new_sample and (
                self.turn_progress_error is None
                or abs(mid_error) <= self.turn_progress_error - TURN_PROGRESS_DEGREES
            ):
                self.turn_progress_error = abs(mid_error)
                self.turn_progress_at = now
            forward_stalled = (
                self.turn_progress_at is not None
                and now - self.turn_progress_at >= TURN_NO_PROGRESS_SECONDS
            )
            if (
                abs(mid_error) <= TURN_HEADING_TOLERANCE
                or center <= TURN_DIRECT_REVERSE_FRONT_CM
                or forward_stalled
            ):
                self.turn_mode = "reverse"
                self.turn_progress_error = abs(angle_diff(self.pending_target, heading))
                self.turn_progress_at = now
                self.transition(
                    "TURN_90",
                    f"forward arc ended at {heading:.1f}, C={center}cm; reverse arc to {self.pending_target:.1f}",
                )
            else:
                self.command_robot(TURNING_SPEED, 0, self.turn_steering())
                return

        error = abs(angle_diff(self.pending_target, heading))
        if new_sample and (
            self.turn_progress_error is None
            or error <= self.turn_progress_error - TURN_PROGRESS_DEGREES
        ):
            self.turn_progress_error = error
            self.turn_progress_at = now

        if (
            self.turn_mode == "reverse"
            and self.turn_progress_at is not None
            and now - self.turn_progress_at >= TURN_NO_PROGRESS_SECONDS
            and center >= TURN_FORWARD_FALLBACK_MIN_FRONT_CM
        ):
            self.turn_mode = "forward_fallback"
            self.turn_progress_error = error
            self.turn_progress_at = now
            self.record_run_event(
                "corner_forward_fallback",
                f"reverse blocked at heading={heading:.1f}; front={center}cm; keeping target={self.pending_target:.1f}",
            )
            self.transition("TURN_90", "reverse blocked by wall; continue clockwise target with forward arc")

        if self.turn_mode == "forward_fallback":
            if center <= TURN_DIRECT_REVERSE_FRONT_CM:
                self.turn_mode = "reverse"
                self.turn_progress_error = error
                self.turn_progress_at = now
                self.transition("TURN_90", "forward clearance ended; continue same target in reverse")
            else:
                self.command_robot(TURN_FORWARD_FALLBACK_SPEED, 0, self.turn_steering())
                return

        self.command_robot(TURN_REVERSE_SPEED, 1, self.turn_steering())

    def update_post_turn_backup(self, data):
        if POST_TURN_BACKUP_SECONDS <= 0:
            self.transition("POST_TURN_SCAN_HOLD", "camera observation after corner reverse")
            self.resume_block_detection()
            return

        heading = data[0]
        steering = self.heading_correction(heading, reverse=True)
        self.command_robot(POST_TURN_BACKUP_SPEED, 1, steering)

        if time.monotonic() - self.state_started >= POST_TURN_BACKUP_SECONDS:
            self.transition("POST_TURN_SCAN_HOLD", "camera observation after corner reverse")
            self.resume_block_detection()

    def finish_post_turn_context(self, prepare_next_corner=True):
        """Release the completed corner while preserving a freshly seen pillar."""
        if self.alpha_corner_locked:
            self.alpha_corner_locked = False
            self.alpha_corner_reference_y = None
            self.corner_after_pillar_pending = False
            print("[ROUND2] ALPHA CORNER RELEASED: post-turn context complete")
        zone_was_locked = self.corner_zone_locked
        self.reset_corner_zone_lock(armed=False)
        self.reset_deferred_corner_track()
        self.detector.reset_navigation_context(preserve_active=True)
        self.detector.suppress_corner_filter(POST_TURN_TAPE_IGNORE_GRACE_SECONDS)
        self.post_turn_block_priority_until = time.monotonic() + POST_TURN_TAPE_IGNORE_GRACE_SECONDS
        if zone_was_locked:
            print("[ROUND2] TF-LUNA CORNER ZONE RELEASED: post-turn context complete")
        if prepare_next_corner:
            self.next_corner_eligible_at = time.monotonic() + NEXT_CORNER_MIN_FORWARD_SECONDS
            self.next_corner_forward_seconds = 0.0
            self.next_corner_forward_checked_at = time.monotonic()
            self.corner_previous_front_cm = None
            self.corner_previous_front_at = None

    def update_post_turn_scan_hold(self, detection):
        """Stop briefly so several stationary frames can prove a nearby pillar."""
        self.brake()
        if self.vision_enabled and self.last_detection_capture_at <= self.state_started:
            return
        if time.monotonic() - self.state_started < POST_TURN_SCAN_HOLD_SECONDS:
            return

        if (
            detection is not None
            and detection.seen_this_frame
            and not detection.blocked_by_corner
        ):
            self.finish_post_turn_context()
            print(
                f"[ROUND2] POST-TURN BLOCK FOUND: track={detection.track_id} "
                f"color={detection.color}; starting avoidance before recenter"
            )
            self.enter_acquire(detection)
            return

        self.transition("RECENTER", "0.2 s stationary camera scan complete")

    def update_recenter_round2(self, data, new_sample):
        heading, left, center, right, _ = data
        if last_telemetry_status["hard_stop"]:
            self.start_hard_stop_recovery(data, "RECENTER", "front interlock while recentering")
            return

        heading_error = angle_diff(target_angle, heading)
        side_walls_visible = (
            _distance_is_valid(left)
            and _distance_is_valid(right)
            and ROUND2_RECENTER_SIDE_MIN_CM <= left <= ROUND2_RECENTER_SIDE_MAX_CM
            and ROUND2_RECENTER_SIDE_MIN_CM <= right <= ROUND2_RECENTER_SIDE_MAX_CM
        )
        side_error = right - left if side_walls_visible else 0
        side_steering = 0.0
        if side_walls_visible and abs(heading_error) <= ROUND2_RECENTER_HEADING_GATE_DEGREES:
            side_steering = max(
                -ROUND2_RECENTER_SIDE_MAX_STEER,
                min(ROUND2_RECENTER_SIDE_MAX_STEER, side_error * ROUND2_RECENTER_SIDE_KP),
            )
        steering = self.heading_correction(heading) + side_steering
        steering = max(-HEADING_MAX_STEER, min(HEADING_MAX_STEER, steering))
        self.command_robot(ROUND2_RECENTER_SPEED, 0, steering)

        if new_sample:
            centered = (
                abs(heading_error) <= TURN_HEADING_TOLERANCE
                and (not side_walls_visible or abs(side_error) <= ROUND2_RECENTER_SIDE_TOLERANCE_CM)
            )
            self.recenter_centered_samples = self.recenter_centered_samples + 1 if centered else 0
        elapsed = time.monotonic() - self.state_started
        if (
            elapsed >= ROUND2_RECENTER_MAX_SECONDS
            or (
                elapsed >= ROUND2_RECENTER_MIN_SECONDS
                and self.recenter_centered_samples >= ROUND2_RECENTER_CONFIRMATION_SAMPLES)
        ):
            next_state = self.recenter_next_state
            self.finish_post_turn_context(prepare_next_corner=(next_state == "FOLLOW_STRAIGHT"))
            self.transition(next_state)

    def update_pillar_recenter(self, data, new_sample):
        heading, left, center, right, _ = data
        if (
            self.corner_next_after_pillars
            and self.update_corner_zone_lock(data, None, new_sample)
        ):
            self.pillar_recenter_next_state = None
            self.pillar_recenter_phase = None
            self.pillar_recenter_phase_started = None
            self.pillar_recenter_centered_samples = 0
            self.record_run_event(
                "corner_next_during_recenter",
                f"C={center} L={left} R={right}; corner geometry won over recenter",
            )
            return

        if last_telemetry_status["hard_stop"] or center <= PYTHON_EMERGENCY_RELEASE_CM:
            self.start_hard_stop_recovery(
                data, "PILLAR_RECENTER", "front interlock while recentering after pillar"
            )
            return

        now = time.monotonic()

        # --- Phase 1: active counter-steer -------------------------------
        # A red pass steers positive (right); counter with negative (left).
        # A green pass steers negative (left); counter with positive (right).
        if self.pillar_recenter_phase is None:
            self.pillar_recenter_phase = "counter"
            self.pillar_recenter_phase_started = now

        if self.pillar_recenter_phase == "counter":
            counter_sign = -1 if self.last_passed_pillar_color == "red" else 1
            counter_steer = counter_sign * PILLAR_RECENTER_COUNTER_STEER_DEGREES
            self.command_robot(PILLAR_RECENTER_SPEED, 0, counter_steer)

            phase_elapsed = now - self.pillar_recenter_phase_started
            heading_error = angle_diff(target_angle, heading)
            # Move on once the counter-steer has visibly swung the heading
            # back past straight (proof it actually cancelled the drift,
            # not just a timer), or once the bounded max time is used up.
            swung_back = (
                (counter_sign < 0 and heading_error <= -3)
                or (counter_sign > 0 and heading_error >= 3)
            )
            if phase_elapsed >= PILLAR_RECENTER_COUNTER_MAX_SECONDS or (
                phase_elapsed >= PILLAR_RECENTER_COUNTER_MIN_SECONDS and swung_back
            ):
                self.pillar_recenter_phase = "settle"
                self.pillar_recenter_phase_started = now
                self.pillar_recenter_centered_samples = 0
            return

        # --- Phase 2: settle onto lane centre with heading + side walls --
        heading_error = angle_diff(target_angle, heading)
        side_walls_visible = (
            _distance_is_valid(left)
            and _distance_is_valid(right)
            and PILLAR_RECENTER_SIDE_MIN_CM <= left <= PILLAR_RECENTER_SIDE_MAX_CM
            and PILLAR_RECENTER_SIDE_MIN_CM <= right <= PILLAR_RECENTER_SIDE_MAX_CM
        )
        side_error = right - left if side_walls_visible else 0
        side_steering = 0.0
        if side_walls_visible and abs(heading_error) <= PILLAR_RECENTER_HEADING_GATE_DEGREES:
            side_steering = max(
                -PILLAR_RECENTER_SIDE_MAX_STEER,
                min(PILLAR_RECENTER_SIDE_MAX_STEER, side_error * PILLAR_RECENTER_SIDE_KP),
            )
        steering = self.heading_correction(heading) + side_steering
        steering = max(-HEADING_MAX_STEER, min(HEADING_MAX_STEER, steering))
        self.command_robot(PILLAR_RECENTER_SPEED, 0, steering)

        if new_sample:
            centered = (
                abs(heading_error) <= TURN_HEADING_TOLERANCE
                and (not side_walls_visible or abs(side_error) <= PILLAR_RECENTER_SIDE_TOLERANCE_CM)
            )
            self.pillar_recenter_centered_samples = (
                self.pillar_recenter_centered_samples + 1 if centered else 0
            )

        settle_elapsed = now - self.pillar_recenter_phase_started
        if (
            settle_elapsed >= PILLAR_RECENTER_SETTLE_MAX_SECONDS
            or (
                settle_elapsed >= PILLAR_RECENTER_SETTLE_MIN_SECONDS
                and self.pillar_recenter_centered_samples >= PILLAR_RECENTER_CONFIRMATION_SAMPLES
            )
        ):
            next_state = self.pillar_recenter_next_state or "FOLLOW_STRAIGHT"
            self.pillar_recenter_next_state = None
            self.pillar_recenter_phase = None
            self.pillar_recenter_phase_started = None
            self.transition(next_state, "pillar recenter complete")
           
    def update(self, data, detection, camera_fresh):
        new_sample = self.is_new_sample()
        self.current_round2_sensor_data = data
        self.enforce_second_slot_timeout()
        # A committed pass keeps its camera track through clearance. Outside a
        # pass, suppress repeats and all new pillars after the second pass.
        if (
            detection is not None
            and self.state not in (
                "ACQUIRE_PILLAR", "PASS_RED_RIGHT", "PASS_GREEN_LEFT", "CONFIRM_PASSED"
            )
            and self.pillar_pass_commitment is None
            and (
                self.pillar_limit_reached()
                or detection.track_id in self.committed_pillar_tracks_this_straight
            )
        ):
            if (
                self.pillar_limit_reached()
                and detection.seen_this_frame
                and detection.track_id not in self.committed_pillar_tracks_this_straight
                and detection.track_id not in self.corner_next_ignored_tracks
            ):
                self.corner_next_ignored_tracks.add(detection.track_id)
                self.record_run_event(
                    "new_pillar_ignored_corner_next",
                    f"new pillar ignored: corner next; track={detection.track_id} color={detection.color}",
                )
                print(f"[ROUND2] NEW PILLAR IGNORED: corner next; track={detection.track_id}")
            detection = None
        if self.next_corner_forward_seconds is not None:
            now = time.monotonic()
            last_checked = self.next_corner_forward_checked_at
            self.next_corner_forward_checked_at = now
            if (
                last_checked is not None
                and self.command[0] >= 60
                and self.command[1] == 0
                and last_telemetry_status["applied_speed"] is not None
                and last_telemetry_status["applied_speed"] > 0
            ):
                self.next_corner_forward_seconds += max(0.0, min(0.15, now - last_checked))

        if self.state == "WAIT":
            self.brake()
            required = VALID_LEFT | VALID_CENTER | VALID_RIGHT | VALID_HEADING
            mask = last_telemetry_status["valid_mask"]
            if (not self.vision_enabled or camera_fresh) and telemetry_is_fresh() and (mask & required) == required:
                self.transition("DETECT_DIRECTION", "start sensors ready")
            return

        if self.state == "DETECT_DIRECTION":
            if self.vision_enabled and not camera_fresh:
                self.brake()
                self.notice("[ROUND2] Waiting for fresh camera frames")
                return
            self.update_detect_direction(data, new_sample)
            return

        if self.state in ("COMPLETE", "FAULT"):
            self.brake()
            return

        camera_required = (
            self.vision_enabled
            and not self.block_detection_suspended
            and not self.recovery_camera_paused
            and not self.corner_zone_locked
            and self.pillar_pass_commitment is None
            and self.state not in (
                "HARD_STOP_RELEASE", "POST_TURN_BACKUP", "CORNER_CLEARANCE_RECOVERY"
            )
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

        if self.state == "REASSESS_FRONT":
            self.update_reassess_front(data, detection, new_sample)
            return

        if self.state == "CORNER_CLEARANCE_RECOVERY":
            self.update_corner_clearance_recovery(data, new_sample)
            return

        if self.pillar_resume_waiting_frame and self.vision_enabled:
            if self.last_detection_capture_at <= self.state_started:
                self.brake()
                return
            self.pillar_resume_waiting_frame = False

        if self.state == "DEEP_REVERSE_RECOVERY":
            self.update_deep_recovery(data, detection, new_sample)
            return

        # A wide parking block can initially create an unconfirmed colour track.
        # Allow shape proof to cancel only that uncommitted acquisition.
        if (
            self.state == "ACQUIRE_PILLAR"
            and self.update_parking_block_avoidance(data, detection)
        ):
            return

        # A selected pillar owns control through clearance. Emergency braking
        # retains its identity and never triggers an automatic reverse.
        if self.state in ("ACQUIRE_PILLAR", "PASS_RED_RIGHT", "PASS_GREEN_LEFT", "CONFIRM_PASSED"):
            # Re-arm the next-corner recognizer while a pillar manoeuvre owns
            # control.  After a turn the robot can enter ACQUIRE_PILLAR after
            # only one FOLLOW_STRAIGHT sample, which previously prevented the
            # required open-straight samples from ever arming the corner zone.
            # update_corner_zone_lock cannot start a corner from these states;
            # its disarmed branch only collects the safe re-arm evidence.
            if not self.corner_zone_lock_armed and not self.corner_zone_locked:
                self.update_corner_zone_lock(data, detection, new_sample)
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

        if self.state == "FOLLOW_STRAIGHT":
            one_slot_corner_geometry = (
                self.pillar_slots_filled == 1
                and new_sample
                and self.corner_sensors_agree(data, CORNER_ZONE_LOCK_CM)
                and data[2] > PYTHON_EMERGENCY_RELEASE_CM
            )
            if one_slot_corner_geometry:
                # Do not let a colour track at the end wall steal ownership
                # from the existing three-fresh-sample corner proof.
                detection = None

            # A lower/near pillar always wins, including a genuine second pillar
            # of the same colour. Background deferral is track-specific.
            if (
                not one_slot_corner_geometry
                and
                self.candidate_proves_foreground(detection)
                and not self.pillar_limit_reached()
            ):
                if self.enter_acquire(detection):
                    return

            background_deferred = self.defer_background_candidate(data, detection)
            if self.deferred_track_promoted:
                if self.enter_acquire(detection):
                    return

            # The TF-Luna proof changes state only after three fresh samples.
            # There is no locked or slow FOLLOW_STRAIGHT mode.
            if self.update_corner_zone_lock(data, detection, new_sample):
                return

            if self.update_parking_block_avoidance(data, detection):
                return

            if background_deferred:
                detection = None

        # Any forward-driving state can encounter an obstacle. The ESP32
        # reports front distance while Python decides whether a controlled
        # release is needed. Avoid entering recovery during a proven turn.
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
            self.update_recenter_round2(data, new_sample)
        elif self.state == "PILLAR_RECENTER":
            self.update_pillar_recenter(data, new_sample)
        elif self.state == "APPROACH_CORNER":
            self.update_approach_corner(data, detection, new_sample)
        elif self.state == "TURN_90":
            self.update_turn_90(data, new_sample)
        elif self.state == "POST_TURN_BACKUP":
            self.update_post_turn_backup(data)
        elif self.state == "POST_TURN_SCAN_HOLD":
            self.update_post_turn_scan_hold(detection)
        elif self.state == "HARD_STOP_RELEASE":
            self.update_hard_stop_release(data, new_sample)
        elif self.state == "TURN_RETRY_PAUSE":
            self.brake()
            if time.monotonic() - self.state_started >= 0.25:
                self.transition("TURN_90")
        else:
            self.transition("FAULT", f"unknown Round 2 state {self.state}")


def round2_obstacle_main():
    global active_run_csv_logger
    vision_enabled = ENABLE_COLOR_DETECTION
    detector = UnifiedPillarDetector()
    run_logger = RunCsvLogger() if ENABLE_RUN_CSV_LOGGING else None
    active_run_csv_logger = run_logger
    controller = ObstacleChallengeController(detector, run_logger=run_logger)
    last_camera_sequence = -1
    detection = None
    display_frame = None
    print(
        f"[ROUND2] Unified obstacle controller; colour detector {'ON' if vision_enabled else 'OFF'}; parking exit "
        f"{'ON' if vision_enabled and ENABLE_PARKING_EXIT else 'OFF'}; parking in "
        f"{'ON' if ENABLE_PARKING_IN else 'OFF'}; "
        "red passes right, green passes left"
    )

    while True:
        telemetry = read_data()
        ret, frame, camera_sequence, captured_at = cap.read_with_metadata()
        left_fraction, right_fraction = controller.post_turn_view_fractions(
            time.monotonic()
        )
        camera_fresh = (
            ret
            and captured_at > 0
            and time.monotonic() - captured_at <= CAMERA_STALE_SECONDS
        )
        if ret:
            display_frame = frame
            if not vision_enabled or controller.block_detection_suspended or controller.recovery_camera_paused:
                detection = None
                # Consume sequence numbers without processing their frames. This
                # prevents frames captured during a turn or recovery movement
                # from being reused after normal detection resumes.
                last_camera_sequence = camera_sequence
            elif camera_sequence != last_camera_sequence:
                black_corner_search = controller.state == "REASSESS_FRONT"
                detector_frame = (
                    frame
                    if black_corner_search
                    else asymmetric_camera_view(frame, left_fraction, right_fraction)
                )
                detection = detector.update(
                    detector_frame,
                    black_corner_search=black_corner_search,
                    black_corner_side=(
                        "right" if DIRECTION == "clockwise" else "left"
                    ) if black_corner_search else None,
                )
                controller.last_detection_capture_at = captured_at
                last_camera_sequence = camera_sequence

        controller.update(telemetry, detection, camera_fresh)
        if run_logger is not None:
            run_logger.log_sample(controller, telemetry, detection, camera_fresh)

        if controller.state == "PARKING_PARALLEL_HANDOFF":
            controller.brake()
            if run_logger is not None:
                run_logger.close()
                active_run_csv_logger = None
            # main.py runs as __main__; give the standalone parking module the
            # same initialized camera, serial port, telemetry, and safety API.
            import sys
            sys.modules["main"] = sys.modules[__name__]
            if PARTIAL_PARKING == PARALLEL_PARKING:
                raise ValueError("Select exactly one parking mode: PARTIAL_PARKING or PARALLEL_PARKING")
            if PARTIAL_PARKING:
                import partial_parking
                partial_parking.partial_parking_main(DIRECTION)
            else:
                import parallel_parking
                parallel_parking.parallel_parking_main(DIRECTION, "auto")
            return

        if SHOW_LIVE_UI and display_frame is not None:
            annotated = detector.annotate(
                display_frame,
                controller.state,
                controller.last_steering,
                telemetry[0],
                telemetry,
                pillar_count=controller.pillar_slots_filled,
                corner_next=controller.corner_next_after_pillars,
                second_slot_grace=(
                    None if controller.first_pillar_slot_filled_at is None
                    else max(0.0, SECOND_PILLAR_SLOT_TIMEOUT_SECONDS
                             - (time.monotonic() - controller.first_pillar_slot_filled_at))
                ),
            )
            display_left, display_right = controller.post_turn_view_fractions(
                time.monotonic()
            )
            if display_left or display_right:
                left_margin = int(round(annotated.shape[1] * display_left))
                right_x = int(round(annotated.shape[1] * (1.0 - display_right)))
                cv2.line(annotated, (left_margin, 0),
                         (left_margin, annotated.shape[0]), (255, 255, 0), 2)
                cv2.line(annotated, (right_x, 0),
                         (right_x, annotated.shape[0]), (255, 255, 0), 2)
                cv2.putText(
                    annotated,
                    f"VIEW MASK L={display_left * 100:.0f}% R={display_right * 100:.0f}%",
                    (left_margin + 10, annotated.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2,
                )
            publish_detection_view(annotated)

        if controller.state == "COMPLETE" and ENABLE_PARKING_IN:
            controller.brake()
            return

        time.sleep(NAVIGATION_LOOP_SECONDS)





def waitForOk():
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print("ESP32:", line)
            if line.upper() == "OK":
                print("Sending back OK...")
                ser.write(b"OK\n")
                break
            if line.startswith("T,") and _parse_telemetry_line(line) is not None:
                print("ESP32 telemetry already streaming; continuing")
                break

if __name__ == "__main__":
    # MAIN
    try:
        print("Initialized")
        led.on()
        start_live_ui()
        waitForOk()
        print("STARTING")
        flush_serial()

        round2_obstacle_main()
    except KeyboardInterrupt:
        print("[STOP] Keyboard interrupt; applying electrical brake")
    finally:
        ui_running = False
        if active_run_csv_logger is not None:
            log_path = active_run_csv_logger.path
            active_run_csv_logger.close()
            active_run_csv_logger = None
            if log_path:
                print(f"[CSV] Run log closed: {log_path}")
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
