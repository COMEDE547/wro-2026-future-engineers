"""Standalone geometry-based parallel-parking test.

Place this file beside main.py and run it with the same virtual environment.
It reuses main.py only for camera, serial, telemetry, UI, and shutdown safety.
The detector deliberately does not use parking-block colour. It identifies the
two rectangular parking limitations, latches the first one, projects a virtual
empty-space cutout, and confirms that opening with heading and side TF-Luna.

Examples:
    python parallel_parking.py --direction clockwise --last-pillar red
    python parallel_parking.py --direction clockwise --last-pillar green
    python parallel_parking.py --direction anticlockwise --last-pillar none
"""

import argparse
import csv
import json
import math
import os
import time
from collections import deque
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path

import numpy as np

import main as robot


BUILD_ID = "slot-guided-entry-v29"
CALIBRATION_FILE = Path(__file__).with_name("parking_camera_calibration.json")
ENABLE_RUN_LOGGING = False

# Geometry is intentionally broad because the 200x20x100 mm limitation can be
# seen edge-on, face-on, obliquely, or clipped by the image boundary.
ROI_TOP_FRACTION = 0.28
MIN_WIDTH_PX = 8
MAX_WIDTH_RATIO = 0.30
MIN_HEIGHT_RATIO = 0.055
MAX_HEIGHT_RATIO = 0.72
MIN_BOX_AREA_RATIO = 0.0015
MAX_BOX_AREA_RATIO = 0.24
MIN_RECTANGULARITY = 0.42
MIN_BOUNDARY_BOTTOM_RATIO = 0.55
MIN_ASPECT = 1.15
MAX_ASPECT = 6.0
MAGENTA_HSV_LOW = np.array([135, 65, 45], dtype=np.uint8)
MAGENTA_HSV_HIGH = np.array([179, 255, 255], dtype=np.uint8)
MAGENTA_LAB_A_MIN = 145
MAGENTA_LAB_B_MAX = 175
MIN_MAGENTA_FILL = 0.45
MIN_GROUND_GAP_CM = 25.0
MAX_GROUND_GAP_CM = 120.0
MAX_TRACK_GROUND_DISTANCE_CM = 35.0
TRACK_MAX_MISSES = 8
OPTICAL_FLOW_MIN_POINTS = 4
SLOT_BEARING_KP = 2.0
SLOT_MAX_USABLE_BEARING_DEG = 30.0
SLOT_ALIGN_STEER_LIMIT = 25
SLOT_ENTRY_STEER_LIMIT = 12
SIDE_ENTRY_LIMIT = 0.62
TRACK_CENTER_DISTANCE = 0.18
TRACK_IOU_MIN = 0.04
BOUNDARY_HISTORY = 6
BOUNDARY_REQUIRED_HITS = 3
SECOND_REQUIRED_HITS = 3
BOUNDARY_LOST_BRAKE_SECONDS = 0.80

# The cutout width is scaled from the observed 100 mm block height. Images from
# the real robot can be used to tune this single perspective-independent ratio.
VIRTUAL_GAP_HEIGHT_MULTIPLIER = 1.55
MIN_GAP_WIDTH_RATIO = 0.08
MAX_GAP_WIDTH_RATIO = 0.42
GAP_TARGET_X_CLOCKWISE = 0.34
GAP_TARGET_X_ANTICLOCKWISE = 0.66

SEARCH_SPEED = 65
CAUTIOUS_SPEED = 65
ALIGN_SPEED = 40
# Slow, controlled approach once camera alignment has committed the robot to
# entering the parking space.  Search and pillar-pass speeds stay unchanged.
ENTRY_SPEED = 40
LAST_PILLAR_SPEED = 80
LAST_PILLAR_STEER = 20
GREEN_LAST_PILLAR_STEER = 30
LAST_PILLAR_REQUIRED_LOST_FRAMES = 3
CAMERA_STALE_GRACE_SECONDS = 0.40
NORMAL_SEARCH_STEER = 35
RED_SEARCH_STEER = 50
ALIGN_STEER_LIMIT = 42
ALIGN_KP = 75.0
ALIGN_TOLERANCE = 0.04
ALIGN_CONFIRM_SAMPLES = 8
TRACK_HEADING_MAX_STEER = 25
PARKING_SEARCH_HEADING_OFFSET_DEG = 45.0
GREEN_PARKING_SEARCH_HEADING_OFFSET_DEG = 5.0
GREEN_STRAIGHT_HEADING_TOLERANCE_DEG = 2.0
GREEN_STRAIGHT_CONFIRM_SAMPLES = 3
GREEN_STRAIGHT_TIMEOUT_SECONDS = 10.0
GREEN_STRAIGHT_FORWARD_SECONDS = 1.0
PARKING_SEARCH_OFFSET_STEP_DEG = 5.0
PARKING_SEARCH_MAX_OFFSET_DEG = 55.0
PARKING_SEARCH_TIMEOUT_SECONDS = 4.0
PARKING_SEARCH_TIMEOUT_STEP_SECONDS = 0.2
GREEN_PARKING_SEARCH_MAX_SECONDS = 15.0
PARKING_SEARCH_REVERSE_BASE_SECONDS = 0.60
PARKING_SEARCH_REVERSE_STEP_SECONDS = 0.10
PARKING_SEARCH_REVERSE_MAX_SECONDS = 1.0
PARKING_SEARCH_REVERSE_BASE_SPEED = 55
PARKING_SEARCH_REVERSE_SPEED_STEP = 5
PARKING_SEARCH_REVERSE_MAX_SPEED = 75
PARKING_SEARCH_REVERSE_BASE_STEER = 35
PARKING_SEARCH_REVERSE_STEER_STEP = 5
PARKING_SEARCH_REVERSE_MAX_STEER = 55
SEARCH_MAX_HEADING_CHANGE = 75.0

TF_FIRST_CLOSE_CM = 100
TF_OPEN_CM = 140
TF_SECOND_CLOSE_CM = 100
TF_CONFIRM_SAMPLES = 2

ENTRY_STOP_FRONT_CM = 5
PARALLEL_HEADING_TARGET = 0.0
PARALLEL_HEADING_TOLERANCE = 2.0
PARALLEL_HEADING_CONFIRM_SAMPLES = 3
PARALLEL_ALIGN_SPEED = 35
PARALLEL_ALIGN_STROKE_SECONDS = 0.80
PARALLEL_ALIGN_STEER_LIMIT = 45
FINAL_REVERSE_SECONDS = 1.5
FINAL_BRAKE_SECONDS = 0.20
FINAL_FORWARD_SECONDS = 0.30
FINAL_FORWARD_STEER = 10
FINAL_FORWARD_STOP_CM = 6
FULL_STEER = 60


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def distance_valid(value):
    return value is not None and 0 < value <= 400


def heading_valid(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


@dataclass
class RectangleCandidate:
    bbox: tuple
    center_x: float
    bottom: float
    width_ratio: float
    height_ratio: float
    aspect: float
    rectangularity: float
    clipped: bool
    score: float = 0.0
    rotated_box: tuple = ()
    ground_x_cm: float = 0.0
    ground_z_cm: float = 0.0
    confidence: float = 0.0
    source: str = "mask"


@dataclass
class BoundaryTrack:
    candidate: RectangleCandidate
    hits: int
    misses: int
    confirmed: bool
    last_seen_at: float


class GroundKalman:
    def __init__(self, x_cm, z_cm):
        self.filter = robot.cv2.KalmanFilter(4, 2)
        self.filter.transitionMatrix = np.array(
            [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=np.float32,
        )
        self.filter.measurementMatrix = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32
        )
        self.filter.processNoiseCov = np.eye(4, dtype=np.float32) * 0.08
        self.filter.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.5
        self.filter.errorCovPost = np.eye(4, dtype=np.float32)
        self.filter.statePost = np.array([[x_cm], [z_cm], [0], [0]], dtype=np.float32)

    def update(self, x_cm, z_cm):
        self.filter.predict()
        corrected = self.filter.correct(
            np.array([[x_cm], [z_cm]], dtype=np.float32)
        )
        return float(corrected[0, 0]), float(corrected[1, 0])


class ParkingCalibration:
    def __init__(self, matrix, image_size, source_path):
        self.matrix = np.asarray(matrix, dtype=np.float32)
        self.image_size = tuple(image_size)
        self.source_path = source_path

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        return cls(payload["homography"], payload["image_size"], path)

    def ground_point(self, pixel, frame_shape):
        height, width = frame_shape[:2]
        cal_width, cal_height = self.image_size
        scaled = np.array([[[
            pixel[0] * cal_width / float(width),
            pixel[1] * cal_height / float(height),
        ]]], dtype=np.float32)
        result = robot.cv2.perspectiveTransform(scaled, self.matrix)[0, 0]
        return float(result[0]), float(result[1])


class ParkingRunRecorder:
    FIELDNAMES = (
        "elapsed_s", "state", "heading", "left_cm", "front_cm", "right_cm",
        "speed", "drive_direction", "steering", "block1_x_cm", "block1_z_cm",
        "block1_confidence", "block2_x_cm", "block2_z_cm", "block2_confidence",
        "slot_x_cm", "slot_z_cm", "slot_heading_deg", "slot_bearing_deg",
        "slot_confident",
    )

    def __init__(self, base_directory):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.directory = Path(base_directory) / "run_logs"
        self.directory.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.directory / f"parking_run_{stamp}.csv"
        self.video_path = self.directory / f"parking_run_{stamp}.mp4"
        self.file = open(self.csv_path, "x", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        self.video = None
        self.started_at = time.monotonic()
        self.last_flush = self.started_at
        print(f"[PARKING LOG] CSV={self.csv_path}")

    def record(self, frame, telemetry, controller, detector):
        if self.video is None:
            height, width = frame.shape[:2]
            self.video = robot.cv2.VideoWriter(
                str(self.video_path), robot.cv2.VideoWriter_fourcc(*"mp4v"),
                15.0, (width, height),
            )
            print(f"[PARKING LOG] VIDEO={self.video_path}")
        self.video.write(frame)
        heading, left, front, right, _ = telemetry
        first = detector.first.candidate if detector.first else None
        second = detector.second.candidate if detector.second else None
        speed, drive_direction, steering = controller.last_command
        self.writer.writerow({
            "elapsed_s": f"{time.monotonic() - self.started_at:.3f}",
            "state": controller.state, "heading": heading, "left_cm": left,
            "front_cm": front, "right_cm": right, "speed": speed,
            "drive_direction": drive_direction, "steering": steering,
            "block1_x_cm": "" if first is None else f"{first.ground_x_cm:.2f}",
            "block1_z_cm": "" if first is None else f"{first.ground_z_cm:.2f}",
            "block1_confidence": "" if first is None else f"{first.confidence:.3f}",
            "block2_x_cm": "" if second is None else f"{second.ground_x_cm:.2f}",
            "block2_z_cm": "" if second is None else f"{second.ground_z_cm:.2f}",
            "block2_confidence": "" if second is None else f"{second.confidence:.3f}",
            "slot_x_cm": detector.slot_x_cm, "slot_z_cm": detector.slot_z_cm,
            "slot_heading_deg": detector.slot_heading_deg,
            "slot_bearing_deg": detector.slot_bearing_deg,
            "slot_confident": detector.slot_confident,
        })
        if time.monotonic() - self.last_flush >= 0.5:
            self.file.flush()
            self.last_flush = time.monotonic()

    def close(self):
        if self.video is not None:
            self.video.release()
        self.file.flush()
        self.file.close()


class ParkingGeometryDetector:
    """Track two physical rectangles without using their colour."""

    def __init__(self, direction, calibration):
        self.direction = direction
        self.calibration = calibration
        self.candidates = []
        self.first = None
        self.second = None
        self.first_history = deque(maxlen=BOUNDARY_HISTORY)
        self.second_history = deque(maxlen=BOUNDARY_HISTORY)
        self.virtual_gap = None
        self.gap_center_x = None
        self.slot_x_cm = None
        self.slot_z_cm = None
        self.slot_heading_deg = None
        self.slot_bearing_deg = None
        self.slot_confident = False
        self.kalman = {"first": None, "second": None}
        self.previous_gray = None

    @staticmethod
    def iou(first, second):
        ax, ay, aw, ah = first
        bx, by, bw, bh = second
        x1, y1 = max(ax, bx), max(ay, by)
        x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        union = aw * ah + bw * bh - intersection
        return intersection / union if union > 0 else 0.0

    @staticmethod
    def center_distance(first, second):
        return math.hypot(first.center_x - second.center_x, first.bottom - second.bottom)

    def extract(self, frame):
        height, width = frame.shape[:2]
        hsv = robot.cv2.cvtColor(frame, robot.cv2.COLOR_BGR2HSV)
        lab = robot.cv2.cvtColor(frame, robot.cv2.COLOR_BGR2LAB)
        hsv_mask = robot.cv2.inRange(hsv, MAGENTA_HSV_LOW, MAGENTA_HSV_HIGH)
        lab_mask = robot.cv2.inRange(
            lab,
            np.array([0, MAGENTA_LAB_A_MIN, 0], dtype=np.uint8),
            np.array([255, 255, MAGENTA_LAB_B_MAX], dtype=np.uint8),
        )
        mask = robot.cv2.bitwise_and(hsv_mask, lab_mask)
        mask[:int(height * ROI_TOP_FRACTION), :] = 0
        kernel = robot.cv2.getStructuringElement(robot.cv2.MORPH_RECT, (5, 5))
        mask = robot.cv2.morphologyEx(mask, robot.cv2.MORPH_OPEN, kernel, iterations=1)
        mask = robot.cv2.morphologyEx(mask, robot.cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = robot.cv2.findContours(
            mask, robot.cv2.RETR_EXTERNAL, robot.cv2.CHAIN_APPROX_SIMPLE
        )
        frame_area = float(width * height)
        candidates = []
        for contour in contours:
            rotated = robot.cv2.minAreaRect(contour)
            (_, _), (side_a, side_b), _ = rotated
            long_side, short_side = max(side_a, side_b), min(side_a, side_b)
            if short_side <= 0 or long_side <= short_side:
                continue
            rotated_box_array = robot.cv2.boxPoints(rotated).astype(np.float32)
            x, y, box_width, box_height = robot.cv2.boundingRect(rotated_box_array.astype(np.int32))
            if box_width < MIN_WIDTH_PX or box_height <= 0:
                continue
            width_ratio = box_width / float(width)
            height_ratio = box_height / float(height)
            box_area_ratio = box_width * box_height / frame_area
            aspect = long_side / float(short_side)
            contour_area = robot.cv2.contourArea(contour)
            rotated_area = long_side * short_side
            rectangularity = contour_area / float(max(rotated_area, 1.0))
            magenta_fill = contour_area / float(max(box_width * box_height, 1))
            bottom = (y + box_height) / float(height)
            center_x = (x + box_width * 0.5) / float(width)
            clipped = x <= 3 or x + box_width >= width - 3
            bottom_clipped = y + box_height >= height - 3
            if not (
                width_ratio <= MAX_WIDTH_RATIO
                and MIN_HEIGHT_RATIO <= height_ratio <= MAX_HEIGHT_RATIO
                and MIN_BOX_AREA_RATIO <= box_area_ratio <= MAX_BOX_AREA_RATIO
                and MIN_ASPECT <= aspect <= MAX_ASPECT
                and rectangularity >= MIN_RECTANGULARITY
                and magenta_fill >= MIN_MAGENTA_FILL
                and bottom >= MIN_BOUNDARY_BOTTOM_RATIO
                and not bottom_clipped
            ):
                continue
            two_lowest = sorted(rotated_box_array, key=lambda point: point[1])[-2:]
            foot_pixel = (
                float((two_lowest[0][0] + two_lowest[1][0]) * 0.5),
                float((two_lowest[0][1] + two_lowest[1][1]) * 0.5),
            )
            ground_x_cm, ground_z_cm = self.calibration.ground_point(
                foot_pixel, frame.shape
            )
            side_score = 1.0 - center_x if self.direction == "clockwise" else center_x
            edge_profile = 0.4 if clipped else 1.0
            score = (
                0.25 * bottom
                + 0.20 * min(1.0, height_ratio / 0.30)
                + 0.15 * side_score
                + 0.20 * edge_profile
                + 0.20 * rectangularity
            )
            candidates.append(RectangleCandidate(
                bbox=(x, y, box_width, box_height),
                center_x=center_x,
                bottom=bottom,
                width_ratio=width_ratio,
                height_ratio=height_ratio,
                aspect=aspect,
                rectangularity=rectangularity,
                clipped=clipped,
                score=score,
                rotated_box=tuple(tuple(float(value) for value in point) for point in rotated_box_array),
                ground_x_cm=ground_x_cm,
                ground_z_cm=ground_z_cm,
                confidence=clamp(0.5 * rectangularity + 0.3 * magenta_fill + 0.2 * min(1.0, aspect / 2.0), 0.0, 1.0),
            ))
        self.candidates = candidates
        return candidates

    def optical_flow_candidate(self, track, gray, frame_shape):
        if track is None or self.previous_gray is None:
            return None
        x, y, width, height = track.candidate.bbox
        mask = np.zeros_like(self.previous_gray)
        mask[max(0, y):y + height, max(0, x):x + width] = 255
        points = robot.cv2.goodFeaturesToTrack(
            self.previous_gray, maxCorners=30, qualityLevel=0.01,
            minDistance=4, mask=mask,
        )
        if points is None or len(points) < OPTICAL_FLOW_MIN_POINTS:
            return None
        moved, status, _ = robot.cv2.calcOpticalFlowPyrLK(
            self.previous_gray, gray, points, None,
            winSize=(21, 21), maxLevel=3,
        )
        if moved is None or status is None:
            return None
        valid_old = points[status.reshape(-1) == 1].reshape(-1, 2)
        valid_new = moved[status.reshape(-1) == 1].reshape(-1, 2)
        if len(valid_new) < OPTICAL_FLOW_MIN_POINTS:
            return None
        delta = np.median(valid_new - valid_old, axis=0)
        dx, dy = float(delta[0]), float(delta[1])
        shifted_bbox = (int(round(x + dx)), int(round(y + dy)), width, height)
        shifted_box = tuple((px + dx, py + dy) for px, py in track.candidate.rotated_box)
        foot_x = shifted_bbox[0] + width * 0.5
        foot_y = shifted_bbox[1] + height
        ground_x, ground_z = self.calibration.ground_point((foot_x, foot_y), frame_shape)
        return replace(
            track.candidate,
            bbox=shifted_bbox,
            center_x=(foot_x / float(frame_shape[1])),
            bottom=(foot_y / float(frame_shape[0])),
            rotated_box=shifted_box,
            ground_x_cm=ground_x,
            ground_z_cm=ground_z,
            # Good multi-point optical flow is a real observation.  Preserve
            # enough confidence for a confirmed block to remain useful after
            # it naturally leaves the edge of the camera view.
            confidence=max(0.60, track.candidate.confidence * 0.98),
            source="flow",
        )

    def smooth_ground(self, key, candidate):
        if candidate is None:
            return None
        if self.kalman[key] is None:
            self.kalman[key] = GroundKalman(
                candidate.ground_x_cm, candidate.ground_z_cm
            )
        x_cm, z_cm = self.kalman[key].update(
            candidate.ground_x_cm, candidate.ground_z_cm
        )
        return replace(candidate, ground_x_cm=x_cm, ground_z_cm=z_cm)

    def reset_tracks(self):
        self.first = None
        self.second = None
        self.first_history.clear()
        self.second_history.clear()
        self.virtual_gap = None
        self.gap_center_x = None
        self.slot_x_cm = None
        self.slot_z_cm = None
        self.slot_heading_deg = None
        self.slot_bearing_deg = None
        self.slot_confident = False
        self.kalman = {"first": None, "second": None}
        self.previous_gray = None

    def match(self, track, candidates):
        if track is None:
            return None
        matches = []
        for candidate in candidates:
            overlap = self.iou(track.candidate.bbox, candidate.bbox)
            distance = self.center_distance(track.candidate, candidate)
            ground_distance = math.hypot(
                candidate.ground_x_cm - track.candidate.ground_x_cm,
                candidate.ground_z_cm - track.candidate.ground_z_cm,
            )
            if (
                overlap >= TRACK_IOU_MIN
                or distance <= TRACK_CENTER_DISTANCE
                or ground_distance <= MAX_TRACK_GROUND_DISTANCE_CM
            ):
                matches.append((
                    2.0 * overlap - distance - ground_distance / 100.0 + candidate.score,
                    candidate,
                ))
        return max(matches, key=lambda item: item[0])[1] if matches else None

    def match_first(self, candidates):
        """Update block 1 without ever jumping inward to block 2."""
        candidate = self.match(self.first, candidates)
        if candidate is None or self.first is None or not self.first.confirmed:
            return candidate
        previous_x = self.first.candidate.center_x
        # As the robot advances, clockwise block 1 leaves through the left
        # edge; anticlockwise block 1 leaves through the right edge. A sudden
        # inward jump is therefore the next block, never a new block-1 sample.
        if self.direction == "clockwise" and candidate.center_x > previous_x + 0.05:
            return None
        if self.direction == "anticlockwise" and candidate.center_x < previous_x - 0.05:
            return None
        return candidate

    def update_track(self, track, candidate, history, required_hits):
        now = time.monotonic()
        history.append(candidate is not None)
        if candidate is None:
            if track is None:
                return None
            return replace(track, misses=track.misses + 1)
        hits = sum(history)
        return BoundaryTrack(
            candidate=candidate,
            hits=hits,
            misses=0,
            confirmed=(track.confirmed if track is not None else False)
                      or hits >= required_hits,
            last_seen_at=now,
        )

    def choose_first(self, candidates):
        if self.direction == "clockwise":
            eligible = [item for item in candidates if item.center_x <= SIDE_ENTRY_LIMIT]
        else:
            eligible = [item for item in candidates if item.center_x >= 1.0 - SIDE_ENTRY_LIMIT]
        return max(eligible, key=lambda item: item.score, default=None)

    def choose_second(self, candidates):
        if self.first is None:
            return None
        first = self.first.candidate
        eligible = []
        for candidate in candidates:
            if self.iou(first.bbox, candidate.bbox) > 0.03:
                continue
            ground_separation = math.hypot(
                candidate.ground_x_cm - first.ground_x_cm,
                candidate.ground_z_cm - first.ground_z_cm,
            )
            same_parking_line = abs(candidate.ground_x_cm - first.ground_x_cm) <= 55.0
            farther_boundary = candidate.ground_z_cm >= first.ground_z_cm + 15.0
            if (
                MIN_GROUND_GAP_CM <= ground_separation <= MAX_GROUND_GAP_CM
                and same_parking_line
                and farther_boundary
            ):
                eligible.append(candidate)
        return max(
            eligible,
            key=lambda item: item.confidence + 0.25 * item.score,
            default=None,
        )

    def update_virtual_gap(self, frame_width, frame_height):
        self.slot_x_cm = None
        self.slot_z_cm = None
        self.slot_heading_deg = None
        self.slot_bearing_deg = None
        self.slot_confident = False
        if self.first is None or self.second is None or not self.second.confirmed:
            self.virtual_gap = None
            self.gap_center_x = None
            return
        fx, fy, fw, fh = self.first.candidate.bbox
        sx, sy, sw, sh = self.second.candidate.bbox
        if self.direction == "clockwise":
            left_edge, right_edge = fx + fw, sx
        else:
            left_edge, right_edge = sx + sw, fx
        if right_edge - left_edge >= frame_width * 0.025:
            top = min(fy, sy)
            bottom = max(fy + fh, sy + sh)
            self.virtual_gap = (left_edge, top, right_edge - left_edge, bottom - top)
        else:
            self.virtual_gap = None
        if self.virtual_gap is not None:
            gx, _, gw, _ = self.virtual_gap
            self.gap_center_x = (gx + gw * 0.5) / float(frame_width)
        else:
            self.gap_center_x = None

        first = self.first.candidate
        second = self.second.candidate
        dx = second.ground_x_cm - first.ground_x_cm
        dz = second.ground_z_cm - first.ground_z_cm
        separation = math.hypot(dx, dz)
        self.slot_x_cm = 0.5 * (first.ground_x_cm + second.ground_x_cm)
        self.slot_z_cm = 0.5 * (first.ground_z_cm + second.ground_z_cm)
        self.slot_heading_deg = math.degrees(math.atan2(dx, dz))
        self.slot_bearing_deg = math.degrees(
            math.atan2(self.slot_x_cm, max(1.0, self.slot_z_cm))
        )
        self.slot_confident = (
            self.first.confirmed
            and self.second.confirmed
            and self.first.misses <= 2
            and self.second.misses <= 2
            and MIN_GROUND_GAP_CM <= separation <= MAX_GROUND_GAP_CM
            and first.confidence >= 0.55
            and second.confidence >= 0.55
        )

    def update(self, frame):
        gray = robot.cv2.cvtColor(frame, robot.cv2.COLOR_BGR2GRAY)
        candidates = self.extract(frame)
        first_candidate = self.match_first(candidates)
        if self.first is None:
            first_candidate = self.choose_first(candidates)
        if first_candidate is None:
            first_candidate = self.optical_flow_candidate(self.first, gray, frame.shape)
        first_candidate = self.smooth_ground("first", first_candidate)
        self.first = self.update_track(
            self.first, first_candidate, self.first_history, BOUNDARY_REQUIRED_HITS
        )
        # A single magenta contour must never update both persistent IDs.
        second_candidates = candidates
        if first_candidate is not None:
            second_candidates = [
                item for item in candidates
                if self.iou(first_candidate.bbox, item.bbox) <= 0.10
                and math.hypot(
                    first_candidate.ground_x_cm - item.ground_x_cm,
                    first_candidate.ground_z_cm - item.ground_z_cm,
                ) >= 15.0
            ]
        second_candidate = self.match(self.second, second_candidates)
        if self.first is not None and self.first.confirmed and self.second is None:
            second_candidate = self.choose_second(second_candidates)
        if second_candidate is None:
            second_candidate = self.optical_flow_candidate(self.second, gray, frame.shape)
        second_candidate = self.smooth_ground("second", second_candidate)
        self.second = self.update_track(
            self.second, second_candidate, self.second_history, SECOND_REQUIRED_HITS
        )
        self.update_virtual_gap(frame.shape[1], frame.shape[0])
        self.previous_gray = gray
        return self

    def annotate(self, frame, controller, telemetry):
        output = frame.copy()
        for candidate in self.candidates:
            if candidate.rotated_box:
                box = np.asarray(candidate.rotated_box, dtype=np.int32)
                robot.cv2.polylines(output, [box], True, (0, 165, 255), 2)
        if self.first is not None:
            x, y, width, height = self.first.candidate.bbox
            color = (0, 255, 0) if self.first.confirmed else (0, 255, 255)
            box = np.asarray(self.first.candidate.rotated_box, dtype=np.int32)
            robot.cv2.polylines(output, [box], True, color, 4)
            first_status = "LATCHED" if self.first.confirmed else f"{self.first.hits}/{BOUNDARY_HISTORY}"
            robot.cv2.putText(output, f"BLOCK 1 {first_status} "
                              f"x={self.first.candidate.ground_x_cm:.0f} "
                              f"z={self.first.candidate.ground_z_cm:.0f}cm",
                              (x, max(25, y - 8)), robot.cv2.FONT_HERSHEY_SIMPLEX,
                              0.7, color, 2)
        if self.second is not None:
            x, y, width, height = self.second.candidate.bbox
            color = (255, 0, 255) if self.second.confirmed else (255, 255, 0)
            box = np.asarray(self.second.candidate.rotated_box, dtype=np.int32)
            robot.cv2.polylines(output, [box], True, color, 4)
            second_status = "LATCHED" if self.second.confirmed else f"{self.second.hits}/{BOUNDARY_HISTORY}"
            robot.cv2.putText(output, f"BLOCK 2 {second_status} "
                              f"x={self.second.candidate.ground_x_cm:.0f} "
                              f"z={self.second.candidate.ground_z_cm:.0f}cm",
                              (x, max(25, y - 8)), robot.cv2.FONT_HERSHEY_SIMPLEX,
                              0.7, color, 2)
        if self.virtual_gap is not None:
            x, y, width, height = self.virtual_gap
            robot.cv2.rectangle(output, (x, y), (x + width, y + height), (255, 0, 0), 4)
            target_x = int((self.gap_center_x or 0.5) * output.shape[1])
            robot.cv2.line(output, (target_x, y), (target_x, output.shape[0]), (255, 0, 0), 3)
            robot.cv2.putText(output, "VIRTUAL PARKING SPACE", (x, max(25, y - 35)),
                              robot.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        if self.slot_x_cm is not None:
            slot_status = "LOCKED" if self.slot_confident else "CHECK"
            robot.cv2.putText(
                output,
                f"3D SLOT {slot_status}: x={self.slot_x_cm:.1f}cm "
                f"z={self.slot_z_cm:.1f}cm bearing={self.slot_bearing_deg:.1f}deg "
                f"axis={self.slot_heading_deg:.1f}deg",
                (25, 105), robot.cv2.FONT_HERSHEY_SIMPLEX, 0.60,
                (255, 0, 255) if self.slot_confident else (0, 255, 255), 2,
            )
        heading, left, center, right, _ = telemetry
        robot.cv2.putText(output, f"PARKING TEST: {controller.state}", (25, 35),
                          robot.cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        heading_text = f"{heading:.1f}" if heading_valid(heading) else "WAIT"
        robot.cv2.putText(output,
                          f"heading={heading_text} L={left} C={center} R={right} MAGENTA+HOMOGRAPHY",
                          (25, 70), robot.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        robot.cv2.putText(output, "FULL PARKING VIEW L=0% R=0%", (25, output.shape[0] - 25),
                          robot.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        return output


class ParallelParkingController:
    def __init__(self, direction, last_pillar):
        self.direction = direction
        self.last_pillar = last_pillar
        self.state = "SEARCH_FIRST_BOUNDARY" if last_pillar == "none" else "SEARCH_LAST_PILLAR"
        self.state_started = time.monotonic()
        self.started_at = self.state_started
        self.start_heading = None
        self.last_command = (0, 0, 0)
        self.last_command_sent = 0.0
        self.alignment_count = 0
        self.parallel_heading_count = 0
        self.parallel_align_direction = 1
        self.parallel_align_phase_started = self.state_started
        self.parking_search_attempt = 0
        self.parking_search_phase = "forward"
        self.parking_search_phase_started = self.state_started
        self.first_confirmed_at = None
        self.locked_last_pillar_color = None
        self.last_pillar_was_near = False
        self.last_pillar_lost_frames = 0
        self.green_straight_count = 0
        self.camera_stale_since = None
        self.complete = False
        self.fault_reason = None

    def transition(self, new_state, reason):
        if new_state != self.state:
            print(f"[PARALLEL PARKING] {self.state} -> {new_state}: {reason}")
        self.state = new_state
        self.state_started = time.monotonic()
        # Force the new state's command to be transmitted immediately even if
        # its numeric values happen to match the preceding state's command.
        self.last_command_sent = 0.0

    def command(self, speed, direction, steering):
        speed = int(clamp(speed, 0, 255))
        steering = float(clamp(steering, -60, 60))
        command = (speed, int(direction), steering)
        now = time.monotonic()
        if (
            command != self.last_command
            or now - self.last_command_sent >= robot.COMMAND_REFRESH_SECONDS
        ):
            robot.send_data(speed, int(direction), steering)
            self.last_command_sent = now
        self.last_command = command

    def brake(self):
        self.command(0, 0, 0)

    def fail(self, reason):
        self.brake()
        self.fault_reason = reason
        self.transition("FAULT", reason)

    def search_steering(self):
        magnitude = RED_SEARCH_STEER if self.pillar_behavior(self.last_pillar) == "red" else NORMAL_SEARCH_STEER
        return -magnitude if self.direction == "clockwise" else magnitude

    def pillar_behavior(self, color):
        """Use the opposite pillar path when the course direction reverses."""
        if self.direction == "anticlockwise":
            return {"red": "green", "green": "red"}.get(color, color)
        return color

    def last_pillar_steering(self, color):
        return (LAST_PILLAR_STEER if self.pillar_behavior(color) == "red"
                else -GREEN_LAST_PILLAR_STEER)

    def green_pillar_search(self):
        return self.pillar_behavior(self.locked_last_pillar_color) == "green"

    def green_straight_steering(self, heading):
        if not heading_valid(heading) or not heading_valid(self.start_heading):
            return 0
        return clamp(
            angle_diff(self.start_heading, heading),
            -TRACK_HEADING_MAX_STEER,
            TRACK_HEADING_MAX_STEER,
        )

    def gap_steering(self, gap_center_x):
        target = (
            GAP_TARGET_X_CLOCKWISE
            if self.direction == "clockwise"
            else GAP_TARGET_X_ANTICLOCKWISE
        )
        # Camera error and steering command have opposite signs: if the gap
        # appears to the right of its desired image position, steer left to
        # bring the robot toward it (and vice versa).
        return clamp((target - gap_center_x) * ALIGN_KP, -ALIGN_STEER_LIMIT, ALIGN_STEER_LIMIT)

    @staticmethod
    def slot_bearing_usable(detector):
        bearing = detector.slot_bearing_deg
        return (
            detector.slot_confident
            and detector.first is not None
            and detector.second is not None
            and detector.first.misses == 0
            and detector.second.misses == 0
            and isinstance(bearing, (int, float))
            and math.isfinite(bearing)
            and abs(bearing) <= SLOT_MAX_USABLE_BEARING_DEG
        )

    @staticmethod
    def slot_steering(detector, steer_limit):
        if not ParallelParkingController.slot_bearing_usable(detector):
            return 0
        return clamp(
            detector.slot_bearing_deg * SLOT_BEARING_KP,
            -steer_limit,
            steer_limit,
        )

    def parking_search_heading_hold(self, heading):
        if not heading_valid(heading) or not heading_valid(self.start_heading):
            return 0
        green_search = self.green_pillar_search()
        magnitude = min(
            PARKING_SEARCH_MAX_OFFSET_DEG,
            GREEN_PARKING_SEARCH_HEADING_OFFSET_DEG if green_search else
            PARKING_SEARCH_HEADING_OFFSET_DEG
            + self.parking_search_attempt * PARKING_SEARCH_OFFSET_STEP_DEG,
        )
        offset = (
            -magnitude
            if self.direction == "clockwise"
            else magnitude
        )
        target = self.start_heading + offset
        steer_limit = min(
            45,
            TRACK_HEADING_MAX_STEER
            + (0 if green_search else self.parking_search_attempt * 5),
        )
        return clamp(
            angle_diff(target, heading),
            -steer_limit,
            steer_limit,
        )

    def parking_search_reverse_command(self):
        speed = min(
            PARKING_SEARCH_REVERSE_MAX_SPEED,
            PARKING_SEARCH_REVERSE_BASE_SPEED
            + self.parking_search_attempt * PARKING_SEARCH_REVERSE_SPEED_STEP,
        )
        steer = min(
            PARKING_SEARCH_REVERSE_MAX_STEER,
            PARKING_SEARCH_REVERSE_BASE_STEER
            + self.parking_search_attempt * PARKING_SEARCH_REVERSE_STEER_STEP,
        )
        # Reverse steering is mirrored: positive steer rotates the chassis
        # left during clockwise reverse; anticlockwise uses negative steer.
        if self.direction == "anticlockwise":
            steer = -steer
        return speed, 1, steer

    def parking_search_reverse_duration(self):
        return min(
            PARKING_SEARCH_REVERSE_MAX_SECONDS,
            PARKING_SEARCH_REVERSE_BASE_SECONDS
            + self.parking_search_attempt * PARKING_SEARCH_REVERSE_STEP_SECONDS,
        )

    def parking_search_timeout(self):
        return (
            PARKING_SEARCH_TIMEOUT_SECONDS
            + self.parking_search_attempt * PARKING_SEARCH_TIMEOUT_STEP_SECONDS
        )

    def update(self, telemetry, detector, pillar_detection, camera_fresh):
        now = time.monotonic()
        heading, _, center, _, _ = telemetry
        if self.start_heading is None and heading_valid(heading):
            self.start_heading = heading
        # TF-Luna readings are deliberately not used while searching for or
        # aligning with the parking blocks.  Their geometry is established by
        # the full camera view; distance sensing becomes relevant only after
        # the robot has entered the space.

        if self.state in ("COMPLETE", "FAULT"):
            self.brake()
            return
        if camera_fresh:
            self.camera_stale_since = None
        elif self.camera_stale_since is None:
            self.camera_stale_since = now
        if (
            not camera_fresh
            and now - self.camera_stale_since >= CAMERA_STALE_GRACE_SECONDS
            and self.state not in ("FINAL_REVERSE", "FINAL_BRAKE", "FINAL_FORWARD")
        ):
            # Do not pulse the electrical brake between camera frames.  Keep
            # refreshing the already-selected motion until vision recovers.
            self.command(*self.last_command)
            return
        first_confirmed = detector.first is not None and detector.first.confirmed
        second_confirmed = detector.second is not None and detector.second.confirmed
        if first_confirmed and self.first_confirmed_at is None:
            self.first_confirmed_at = now

        if self.state == "SEARCH_LAST_PILLAR":
            visible_pillar = (
                pillar_detection is not None
                and pillar_detection.seen_this_frame
                and pillar_detection.color in ("red", "green")
                and (self.last_pillar == "auto" or pillar_detection.color == self.last_pillar)
            )
            seen_pillar = visible_pillar and pillar_detection.confirmed
            if seen_pillar:
                self.locked_last_pillar_color = pillar_detection.color
                self.last_pillar_was_near = (
                    pillar_detection.bottom_norm >= robot.PILLAR_NEAR_BOTTOM
                    or pillar_detection.normalized_area >= robot.PILLAR_AREA_NEAR_RATIO
                )
                self.last_pillar_lost_frames = 0
                self.transition(
                    "PASS_LAST_PILLAR",
                    f"{pillar_detection.color} final pillar acquired",
                )
                # Do not spend the confirmation frame continuing the previous
                # straight command.  At robot speed that one-frame delay can
                # put the final pillar beyond the useful steering window.
                self.command(
                    LAST_PILLAR_SPEED,
                    0,
                    self.last_pillar_steering(pillar_detection.color),
                )
                return
            if visible_pillar:
                # Confirmation deliberately takes several frames, but the
                # avoidance response must begin on the first valid sighting.
                # A false detection can therefore influence steering briefly,
                # while it still cannot latch the pass state by itself.
                self.command(
                    LAST_PILLAR_SPEED,
                    0,
                    self.last_pillar_steering(pillar_detection.color),
                )
                return
            if first_confirmed:
                self.transition(
                    "TRACK_OPENING",
                    "no final pillar visible; first parking boundary confirmed",
                )
                return
            self.command(SEARCH_SPEED, 0, 0)
            return

        if self.state == "PASS_LAST_PILLAR":
            color = self.locked_last_pillar_color
            steering = self.last_pillar_steering(color)
            same_visible_pillar = (
                pillar_detection is not None
                and pillar_detection.seen_this_frame
                and pillar_detection.color == color
            )
            if same_visible_pillar:
                self.last_pillar_lost_frames = 0
                self.last_pillar_was_near = self.last_pillar_was_near or (
                    pillar_detection.bottom_norm >= robot.PILLAR_NEAR_BOTTOM
                    or pillar_detection.normalized_area >= robot.PILLAR_AREA_NEAR_RATIO
                )
            elif camera_fresh:
                self.last_pillar_lost_frames += 1
            if (
                self.last_pillar_was_near
                and self.last_pillar_lost_frames >= LAST_PILLAR_REQUIRED_LOST_FRAMES
            ):
                detector.reset_tracks()
                # Keep the startup straight heading as the reference.  The
                # pillar-pass heading is intentionally off-axis and must never
                # become the new "normal" heading.
                self.parking_search_attempt = 0
                self.parking_search_phase = "forward"
                self.parking_search_phase_started = now
                if self.pillar_behavior(color) == "green":
                    self.green_straight_count = 0
                    self.transition(
                        "GREEN_RETURN_STRAIGHT",
                        "green final pillar passed; returning to straight heading",
                    )
                    self.command(
                        ALIGN_SPEED, 0, self.green_straight_steering(heading)
                    )
                    return
                self.transition(
                    "SEARCH_FIRST_BOUNDARY",
                    f"{color} final pillar passed; searching parking block 1",
                )
                # Replace the +38-degree pillar command immediately instead of
                # carrying it into the next control frame.
                self.command(
                    ALIGN_SPEED,
                    0,
                    self.parking_search_heading_hold(heading),
                )
                return
            self.command(LAST_PILLAR_SPEED, 0, steering)
            return

        if self.state == "GREEN_RETURN_STRAIGHT":
            if now - self.state_started >= GREEN_STRAIGHT_TIMEOUT_SECONDS:
                self.fail("green straight-heading recovery timed out")
                return
            aligned = (
                heading_valid(heading)
                and heading_valid(self.start_heading)
                and abs(angle_diff(self.start_heading, heading))
                <= GREEN_STRAIGHT_HEADING_TOLERANCE_DEG
            )
            self.green_straight_count = self.green_straight_count + 1 if aligned else 0
            if self.green_straight_count >= GREEN_STRAIGHT_CONFIRM_SAMPLES:
                self.transition(
                    "GREEN_FORWARD_STRAIGHT",
                    "straight heading restored; driving ahead for 1.0s",
                )
            self.command(ALIGN_SPEED, 0, self.green_straight_steering(heading))
            return

        if self.state == "GREEN_FORWARD_STRAIGHT":
            if now - self.state_started >= GREEN_STRAIGHT_FORWARD_SECONDS:
                self.parking_search_phase_started = now
                self.transition(
                    "SEARCH_FIRST_BOUNDARY",
                    "green straight-ahead step complete; searching parking block 1",
                )
                self.command(
                    ALIGN_SPEED, 0, self.parking_search_heading_hold(heading)
                )
                return
            self.command(ALIGN_SPEED, 0, self.green_straight_steering(heading))
            return

        if self.state == "SEARCH_FIRST_BOUNDARY":
            if first_confirmed:
                self.parking_search_phase = "forward"
                self.parking_search_phase_started = now
                self.transition("TRACK_OPENING", "first rectangular boundary confirmed")
                return
            phase_elapsed = now - self.parking_search_phase_started
            if self.green_pillar_search():
                if phase_elapsed >= GREEN_PARKING_SEARCH_MAX_SECONDS:
                    self.fail("green block-1 search timed out")
                    return
                self.command(
                    ALIGN_SPEED, 0, self.parking_search_heading_hold(heading)
                )
                return
            if self.parking_search_phase == "reverse":
                if phase_elapsed >= self.parking_search_reverse_duration():
                    self.parking_search_phase = "forward"
                    self.parking_search_phase_started = now
                    print(
                        "[PARALLEL PARKING] Resuming forward block search; "
                        f"attempt={self.parking_search_attempt}"
                    )
                else:
                    self.command(*self.parking_search_reverse_command())
                    return
            elif phase_elapsed >= self.parking_search_timeout():
                search_timeout = self.parking_search_timeout()
                self.parking_search_attempt += 1
                self.parking_search_phase = "reverse"
                self.parking_search_phase_started = now
                speed, direction, steering = self.parking_search_reverse_command()
                print(
                    f"[PARALLEL PARKING] Block 1 not locked within {search_timeout:.1f}s; "
                    f"reverse recovery attempt={self.parking_search_attempt} "
                    f"speed={speed} steer={steering:+.0f}"
                )
                self.command(speed, direction, steering)
                return
            # The old red-pillar path applied -50 degrees here, turning into
            # the parking area before even the first boundary was confirmed.
            # Clockwise search deliberately holds a left-offset heading so the
            # parking blocks enter the camera view (mirrored anticlockwise).
            self.command(
                ALIGN_SPEED,
                0,
                self.parking_search_heading_hold(heading),
            )
            return

        if self.state == "TRACK_OPENING":
            if second_confirmed and detector.slot_confident:
                self.transition(
                    "ALIGN_VIRTUAL_CUTOUT",
                    "second camera boundary confirmed",
                )
                return
            phase_elapsed = now - self.parking_search_phase_started
            if self.green_pillar_search():
                if phase_elapsed >= GREEN_PARKING_SEARCH_MAX_SECONDS:
                    self.fail("green block-2 search timed out")
                    return
                self.command(
                    ALIGN_SPEED, 0, self.parking_search_heading_hold(heading)
                )
                return
            if self.parking_search_phase == "reverse":
                if phase_elapsed >= self.parking_search_reverse_duration():
                    self.parking_search_phase = "forward"
                    self.parking_search_phase_started = now
                    print(
                        "[PARALLEL PARKING] Resuming forward block-2 search; "
                        f"attempt={self.parking_search_attempt}"
                    )
                else:
                    self.command(*self.parking_search_reverse_command())
                    return
            elif phase_elapsed >= self.parking_search_timeout():
                search_timeout = self.parking_search_timeout()
                self.parking_search_attempt += 1
                self.parking_search_phase = "reverse"
                self.parking_search_phase_started = now
                speed, direction, steering = self.parking_search_reverse_command()
                print(
                    f"[PARALLEL PARKING] Block 2 not locked within {search_timeout:.1f}s; "
                    f"reverse recovery attempt={self.parking_search_attempt} "
                    f"speed={speed} steer={steering:+.0f}"
                )
                self.command(speed, direction, steering)
                return
            # Continue the bounded left-offset search until block 2 confirms.
            self.command(
                ALIGN_SPEED,
                0,
                self.parking_search_heading_hold(heading),
            )
            return

        if self.state == "ALIGN_VIRTUAL_CUTOUT":
            if (
                robot.last_telemetry_status["hard_stop"]
                or (distance_valid(center) and center <= ENTRY_STOP_FRONT_CM)
            ):
                # The chassis is already inside and at the front boundary.
                # Do not keep chasing a stale camera projection; begin the
                # bounded reverse/TF-Luna-forward parallel correction cycle.
                self.parallel_align_direction = 1
                self.parallel_align_phase_started = now
                self.parallel_heading_count = 0
                self.transition(
                    "PARALLEL_ALIGN",
                    f"inside space during alignment; front TF-Luna reached {center}cm",
                )
                self.command(PARALLEL_ALIGN_SPEED, 1, 0)
                return
            if not self.slot_bearing_usable(detector):
                # A short camera miss must not electrically brake the chassis.
                # Continue straight until both foreground blocks give a usable
                # slot bearing again, then restart confirmation.
                self.alignment_count = 0
                self.command(ALIGN_SPEED, 0, 0)
                return
            steering = self.slot_steering(detector, SLOT_ALIGN_STEER_LIMIT)
            self.alignment_count += 1
            if self.alignment_count >= ALIGN_CONFIRM_SAMPLES:
                self.transition(
                    "ENTER_SPACE",
                    "parking slot confirmed; enter without heading target",
                )
                return
            self.command(ALIGN_SPEED, 0, steering)
            return

        if self.state == "ENTER_SPACE":
            if not distance_valid(center):
                steering = self.slot_steering(detector, SLOT_ENTRY_STEER_LIMIT)
                self.command(ENTRY_SPEED, 0, steering)
                return
            if robot.last_telemetry_status["hard_stop"] or center <= ENTRY_STOP_FRONT_CM:
                self.parallel_align_direction = 1
                self.parallel_align_phase_started = now
                self.parallel_heading_count = 0
                self.transition(
                    "PARALLEL_ALIGN",
                    f"inside space; front TF-Luna reached {center}cm",
                )
                self.command(PARALLEL_ALIGN_SPEED, 1, 0)
                return
            steering = self.slot_steering(detector, SLOT_ENTRY_STEER_LIMIT)
            self.command(ENTRY_SPEED, 0, steering)
            return

        if self.state == "PARALLEL_ALIGN":
            if not heading_valid(heading):
                self.command(PARALLEL_ALIGN_SPEED, 1, 0)
                return

            heading_error = angle_diff(PARALLEL_HEADING_TARGET, heading)
            within_tolerance = abs(heading_error) <= PARALLEL_HEADING_TOLERANCE
            self.parallel_heading_count = (
                self.parallel_heading_count + 1 if within_tolerance else 0
            )
            if self.parallel_heading_count >= PARALLEL_HEADING_CONFIRM_SAMPLES:
                self.brake()
                self.complete = True
                self.transition(
                    "COMPLETE",
                    f"parallel heading confirmed at {heading:.1f} degrees",
                )
                return

            # Reverse is capped at 0.8s so the robot cannot back out of the
            # parking space.  Forward has no timer: front TF-Luna alone ends
            # it at 5 cm, then another short reverse correction begins.
            if self.parallel_align_direction == 1:
                if now - self.parallel_align_phase_started >= PARALLEL_ALIGN_STROKE_SECONDS:
                    self.parallel_align_direction = 0
                    self.parallel_align_phase_started = now
            elif (
                robot.last_telemetry_status["hard_stop"]
                or (distance_valid(center) and center <= ENTRY_STOP_FRONT_CM)
            ):
                self.parallel_align_direction = 1
                self.parallel_align_phase_started = now

            correction_sign = 1 if heading_error > 0 else -1
            steering = correction_sign * PARALLEL_ALIGN_STEER_LIMIT
            if self.parallel_align_direction == 1:
                steering = -steering
            self.command(
                PARALLEL_ALIGN_SPEED,
                self.parallel_align_direction,
                steering,
            )
            return

        if self.state == "FINAL_REVERSE":
            if now - self.state_started >= FINAL_REVERSE_SECONDS:
                self.brake()
                self.transition(
                    "FINAL_BRAKE",
                    f"{FINAL_REVERSE_SECONDS:.1f}s full-lock reverse complete",
                )
                return
            steering = -FULL_STEER if self.direction == "clockwise" else FULL_STEER
            self.command(robot.PARKING_SPEED, 1, steering)
            return

        if self.state == "FINAL_BRAKE":
            self.brake()
            if now - self.state_started >= FINAL_BRAKE_SECONDS:
                self.transition(
                    "FINAL_FORWARD",
                    f"{FINAL_BRAKE_SECONDS:.1f}s brake complete",
                )
            return

        if self.state == "FINAL_FORWARD":
            elapsed = now - self.state_started
            if (
                robot.last_telemetry_status["hard_stop"]
                or (distance_valid(center) and center <= FINAL_FORWARD_STOP_CM)
                or elapsed >= FINAL_FORWARD_SECONDS
            ):
                self.brake()
                self.complete = True
                self.transition("COMPLETE", "final placement complete; electrical brake held")
                return
            steering = FINAL_FORWARD_STEER if self.direction == "clockwise" else -FINAL_FORWARD_STEER
            self.command(robot.PARKING_SPEED, 0, steering)


def parse_args():
    parser = argparse.ArgumentParser(description="Standalone geometry-based parallel parking")
    parser.add_argument(
        "--direction",
        choices=("clockwise", "anticlockwise"),
        default="clockwise",
        help="course direction (default: clockwise)",
    )
    parser.add_argument(
        "--last-pillar",
        choices=("auto", "red", "green", "none"),
        default="auto",
        help="final pillar handling (default: detect red/green automatically)",
    )
    parser.add_argument(
        "--calibrate", action="store_true",
        help="capture four floor points and save webcam ground-plane calibration",
    )
    parser.add_argument("--calibration-width-cm", type=float, default=100.0)
    parser.add_argument("--calibration-depth-cm", type=float, default=150.0)
    return parser.parse_args()


def calibrate_webcam(width_cm, depth_cm):
    print("[CALIBRATION] Click: near-left, near-right, far-right, far-left")
    print(f"[CALIBRATION] Rectangle is {width_cm:.1f} x {depth_cm:.1f} cm; R=reset, S=save")
    points = []
    frame = None
    window = "Parking ground calibration"

    def click(event, x, y, _flags, _parameter):
        if event == robot.cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((float(x), float(y)))

    robot.cv2.namedWindow(window, robot.cv2.WINDOW_NORMAL)
    robot.cv2.setMouseCallback(window, click)
    while True:
        ret, latest, _sequence, _captured_at = robot.cap.read_with_metadata()
        if ret:
            frame = latest
        if frame is None:
            time.sleep(0.02)
            continue
        display = frame.copy()
        for index, point in enumerate(points):
            pixel = tuple(int(value) for value in point)
            robot.cv2.circle(display, pixel, 8, (0, 255, 255), -1)
            robot.cv2.putText(display, str(index + 1), pixel,
                              robot.cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        if len(points) > 1:
            robot.cv2.polylines(display, [np.asarray(points, dtype=np.int32)],
                                len(points) == 4, (0, 255, 255), 3)
        robot.cv2.putText(display, "1 near-L  2 near-R  3 far-R  4 far-L | R reset | S save",
                          (20, 35), robot.cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        robot.cv2.imshow(window, display)
        key = robot.cv2.waitKey(20) & 0xFF
        if key in (ord("q"), 27):
            raise KeyboardInterrupt
        if key == ord("r"):
            points.clear()
        if key == ord("s") and len(points) == 4:
            image_points = np.asarray(points, dtype=np.float32)
            ground_points = np.asarray([
                [-width_cm / 2.0, 0.0], [width_cm / 2.0, 0.0],
                [width_cm / 2.0, depth_cm], [-width_cm / 2.0, depth_cm],
            ], dtype=np.float32)
            matrix = robot.cv2.getPerspectiveTransform(image_points, ground_points)
            payload = {
                "image_size": [int(frame.shape[1]), int(frame.shape[0])],
                "image_points": image_points.tolist(),
                "ground_points_cm": ground_points.tolist(),
                "homography": matrix.tolist(),
            }
            with open(CALIBRATION_FILE, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
            print(f"[CALIBRATION] Saved {CALIBRATION_FILE}")
            robot.cv2.destroyWindow(window)
            return 0


def parallel_parking_main(direction, last_pillar):
    robot.DIRECTION = direction
    if not CALIBRATION_FILE.exists():
        raise RuntimeError(
            f"Missing {CALIBRATION_FILE}. Run parallel_parking.py --calibrate first."
        )
    calibration = ParkingCalibration.load(CALIBRATION_FILE)
    detector = ParkingGeometryDetector(direction, calibration)
    pillar_detector = robot.UnifiedPillarDetector()
    controller = ParallelParkingController(direction, last_pillar)
    recorder = ParkingRunRecorder(Path(__file__).resolve().parent) if ENABLE_RUN_LOGGING else None
    last_camera_sequence = -1
    display_frame = None
    pillar_detection = None
    print(
        f"[PARALLEL PARKING] BUILD={BUILD_ID}; direction={direction}; "
        f"lastPillar={last_pillar}; full camera view; "
        "CAMERA-ONLY block search; continuous motor commands"
    )
    print(f"[PARALLEL PARKING] RUNNING_FILE={Path(__file__).resolve()}")

    try:
        while controller.state not in ("COMPLETE", "FAULT"):
            controller.command(*controller.last_command)
            telemetry = robot.read_data()
            ret, frame, camera_sequence, captured_at = robot.cap.read_with_metadata()
            camera_fresh = (
                ret and captured_at > 0
                and time.monotonic() - captured_at <= robot.CAMERA_STALE_SECONDS
            )
            if ret:
                display_frame = frame
                if camera_sequence != last_camera_sequence:
                    pillar_detection = pillar_detector.update(frame)
                    if controller.state != "PASS_LAST_PILLAR":
                        detector.update(frame)
                    last_camera_sequence = camera_sequence
            controller.update(telemetry, detector, pillar_detection, camera_fresh)
            if display_frame is not None and (recorder is not None or robot.SHOW_LIVE_UI):
                annotated = detector.annotate(display_frame, controller, telemetry)
                if recorder is not None:
                    recorder.record(annotated, telemetry, controller, detector)
                if robot.SHOW_LIVE_UI:
                    robot.publish_detection_view(annotated)
            time.sleep(robot.NAVIGATION_LOOP_SECONDS)
    finally:
        if recorder is not None:
            recorder.close()

    controller.brake()
    if controller.state == "FAULT":
        print(f"[PARALLEL PARKING] FAULT: {controller.fault_reason}; electrical brake applied")
        return 1
    print("[PARALLEL PARKING] COMPLETE; electrical brake applied permanently")
    return 0


if __name__ == "__main__":
    arguments = parse_args()
    exit_code = 1
    try:
        print("Initialized: standalone parallel-parking test")
        if arguments.calibrate:
            exit_code = calibrate_webcam(
                arguments.calibration_width_cm,
                arguments.calibration_depth_cm,
            )
        else:
            if not CALIBRATION_FILE.exists():
                print(
                    "[CALIBRATION] No saved ground calibration was found; "
                    "starting one-time interactive calibration now."
                )
                exit_code = calibrate_webcam(
                    arguments.calibration_width_cm,
                    arguments.calibration_depth_cm,
                )
                print("[CALIBRATION] Calibration saved. Restart normally to drive.")
            else:
                robot.start_live_ui()
                robot.led.on()
                robot.waitForOk()
                robot.flush_serial()
                exit_code = parallel_parking_main(arguments.direction, arguments.last_pillar)
    except KeyboardInterrupt:
        print("[PARALLEL PARKING] Keyboard interrupt; applying electrical brake")
        exit_code = 130
    finally:
        robot.ui_running = False
        try:
            robot.send_data(0, 0, 0)
        except Exception as brake_error:
            print(f"[STOP] Could not send final brake command: {brake_error}")
        try:
            robot.cap.release()
        finally:
            robot.cv2.destroyAllWindows()
            robot.led.off()
            robot.ser.close()
    raise SystemExit(exit_code)
