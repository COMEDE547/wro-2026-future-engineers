"""Direct parking with the same pillar and block search as parallel_parking.

Uses parking_camera_calibration.json beside parallel_parking.py. This file can
run alone, or main.py can hand control to partial_parking_main after corner 12.
"""

import argparse
import math
import time

import numpy as np

import parallel_parking as parking


MIN_VISIBLE_ENTRY_GAP_RATIO = 0.065
MAX_REPOSITION_ATTEMPTS = 1
SEARCH_HEADING_OFFSET_DEG = 30.0
SEARCH_HEADING_STEP_DEG = 5.0
SEARCH_HEADING_MAX_OFFSET_DEG = 55.0
SEARCH_STEER_LIMIT = 25.0
FIRST_BLOCK_SEARCH_SPEED = 50
SECOND_BLOCK_SEARCH_SPEED = 45
GREEN_FORWARD_SEARCH_SECONDS = 0.3
GREEN_CLOCKWISE_FRONT_STOP_CM = 30.0
GREEN_CLOCKWISE_APPROACH_SPEED = 80
GREEN_CLOCKWISE_SEARCH_SPEED = 45
GREEN_CLOCKWISE_RIGHT_STEER = 45.0
GREEN_CLOCKWISE_LEFT_STEER = 60.0
GREEN_CLOCKWISE_SEARCH_HEADING_DEG = 55.0
REPOSITION_AWAY_DEG = 12.0
REPOSITION_SPEED = 40
REPOSITION_PASSED_GAP_X = 0.12
REPOSITION_ADVANCE_CM = 35.0
REPOSITION_FRONT_CLEARANCE_CM = 15.0
REPOSITION_SIDE_CLEARANCE_CM = 8.0
REPOSITION_AWAY_STEER_LIMIT = 25.0
REPOSITION_RETURN_STEER_LIMIT = 18.0
SENSOR_RECOVERY_SAMPLES = 3
ORIENT_HEADING_TOLERANCE_DEG = 5.0
ENTRY_HEADING_HOLD_GAIN = 2.0
ENTRY_HEADING_HOLD_INTEGRAL_GAIN = 0.6
ENTRY_HEADING_INTEGRAL_LIMIT = 30.0
ENTRY_HEADING_HOLD_LIMIT = 24.0
ENTRY_GAP_TARGET_X = 0.50
ENTRY_GAP_STEER_GAIN = 30.0
ENTRY_GAP_STEER_LIMIT = 8.0
ENTRY_GAP_TRACK_MIN_Z_CM = 40.0
ENTRY_GAP_SMOOTHING = 0.35
ENTRY_AIM_MAX_OFFSET_DEG = 30.0
ENTRY_AIM_MIN_Z_CM = 20.0
PASS_LAST_PILLAR_TIMEOUT_SECONDS = 4.0
LAST_PILLAR_DECISION_SECONDS = 3.0
NO_PILLAR_STRAIGHT_SECONDS = 2.0
NO_PILLAR_STRAIGHT_SPEED = 80
REVERSE_BLOCK_SEARCH_SPEED = 50
REVERSE_BLOCK_SEARCH_STEER_LIMIT = 40.0
REVERSE_BLOCK_SEARCH_HEADING_TOLERANCE_DEG = 4.0
REVERSE_BLOCK_SEARCH_SIDE_STOP_CM = 6.0
PILLAR_FRONT_STOP_CM = 8.0
PILLAR_CONFIRM_FRAMES = 3
ENTRY_STOP_FRONT_CM = 5.0
EDGE_ON_MIN_ASPECT = 3.0
EDGE_ON_MAX_WIDTH_RATIO = 0.16
EDGE_ON_MIN_BOTTOM_RATIO = 0.65
EDGE_ON_MIN_MAGENTA_RATIO = 0.18


class PartialPillarDetector:
    """Use main.py's LAB calibration, with tall-shape pillar geometry."""

    def __init__(self):
        self.previous = None
        self.hits = 0
        self.next_track_id = 1

    def update(self, frame):
        robot = parking.robot
        cv2 = robot.cv2
        frame_height, frame_width = frame.shape[:2]
        frame_area = float(frame_height * frame_width)
        lab = cv2.cvtColor(robot.adjust_frame(frame), cv2.COLOR_BGR2LAB)
        lightness, channel_a, channel_b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        lab = cv2.merge((clahe.apply(lightness), channel_a, channel_b))
        kernel = np.ones((3, 3), dtype=np.uint8)
        roi_top = int(frame_height * robot.DETECTOR_ROI_TOP_RATIO)
        candidates = []

        for color in ("red", "green"):
            lower, upper = robot.COLOR_RANGES[color]
            mask = cv2.inRange(
                lab, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask[:roi_top, :] = 0
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                area = cv2.contourArea(contour)
                x, y, width, height = cv2.boundingRect(contour)
                # A clipped object at the image edge belongs to the next
                # lane, not the final pillar directly ahead of the robot.
                if (width <= 0 or height <= width or
                        x <= 2 or x + width >= frame_width - 2):
                    continue
                area_ratio = area / frame_area
                height_ratio = height / float(frame_height)
                if (area_ratio < robot.DETECTOR_MIN_AREA_RATIO[color] or
                        height_ratio < robot.DETECTOR_MIN_HEIGHT_RATIO[color]):
                    continue
                rectangularity = area / float(width * height)
                hull_area = cv2.contourArea(cv2.convexHull(contour))
                solidity = area / hull_area if hull_area > 0 else 0.0
                if (rectangularity < robot.DETECTOR_MIN_RECTANGULARITY or
                        solidity < robot.DETECTOR_MIN_SOLIDITY):
                    continue
                candidates.append(robot.PillarDetection(
                    color=color,
                    bbox=(x, y, width, height),
                    center=(x + width // 2, y + height // 2),
                    bottom=y + height,
                    normalized_area=area_ratio,
                    height_ratio=height_ratio,
                    aspect_ratio=height / float(width),
                    rectangularity=rectangularity,
                    solidity=solidity,
                    confidence=min(1.0, 0.5 * rectangularity + 0.5 * solidity),
                    center_x_norm=(x + width * 0.5) / float(frame_width),
                    bottom_norm=(y + height) / float(frame_height),
                    observed_at=time.monotonic(),
                ))

        if not candidates:
            self.previous = None
            self.hits = 0
            return None
        # The foreground rule in main.py: lower in the image, then larger area.
        selected = max(candidates, key=lambda item: (
            item.bottom_norm, item.normalized_area, item.confidence
        ))
        same_track = (
            self.previous is not None
            and selected.color == self.previous.color
            and abs(selected.center_x_norm - self.previous.center_x_norm) <= 0.25
            and abs(selected.bottom_norm - self.previous.bottom_norm) <= 0.25
        )
        if same_track:
            self.hits += 1
            track_id = self.previous.track_id
        else:
            self.hits = 1
            track_id = self.next_track_id
            self.next_track_id += 1
        selected.confirmed = self.hits >= PILLAR_CONFIRM_FRAMES
        selected.hit_count = self.hits
        selected.track_id = track_id
        self.previous = selected
        return selected


class PartialParkingGeometryDetector(parking.ParkingGeometryDetector):
    def extract(self, frame):
        # The broad face is wide. The same physical block becomes tall in the
        # image when seen edge-on, so require a strong magenta side profile.
        old_bottom = parking.MIN_BOUNDARY_BOTTOM_RATIO
        old_fill = parking.MIN_MAGENTA_FILL
        try:
            parking.MIN_BOUNDARY_BOTTOM_RATIO = 0.35
            parking.MIN_MAGENTA_FILL = 0.25
            candidates = super().extract(frame)
        finally:
            parking.MIN_BOUNDARY_BOTTOM_RATIO = old_bottom
            parking.MIN_MAGENTA_FILL = old_fill
        hsv = parking.robot.cv2.cvtColor(frame, parking.robot.cv2.COLOR_BGR2HSV)
        frame_height, frame_width = frame.shape[:2]
        accepted = []
        for item in candidates:
            box_x, box_y, box_width, box_height = item.bbox
            if box_width > box_height:
                accepted.append(item)
                continue
            if not (
                item.aspect >= EDGE_ON_MIN_ASPECT
                and item.width_ratio <= EDGE_ON_MAX_WIDTH_RATIO
                and item.bottom >= EDGE_ON_MIN_BOTTOM_RATIO
            ):
                continue
            region = hsv[max(0, box_y):min(frame_height, box_y + box_height),
                         max(0, box_x):min(frame_width, box_x + box_width)]
            if region.size == 0:
                continue
            true_magenta = (
                (region[:, :, 0] >= 135) & (region[:, :, 0] <= 174)
                & (region[:, :, 1] >= 65) & (region[:, :, 2] >= 45)
            )
            if float(np.mean(true_magenta)) >= EDGE_ON_MIN_MAGENTA_RATIO:
                accepted.append(item)
        self.candidates = accepted
        return accepted

    def update(self, frame):
        self.frame_width = frame.shape[1]
        super().update(frame)
        if self.first is not None and len(self.candidates) >= 2:
            first_x = self.first.candidate.center_x
            if self.direction == "clockwise":
                earlier = min(self.candidates, key=lambda item: item.center_x)
                wrong_order = earlier.center_x < first_x - 0.10
            else:
                earlier = max(self.candidates, key=lambda item: item.center_x)
                wrong_order = earlier.center_x > first_x + 0.10
            if wrong_order:
                self.reset_tracks()
                super().update(frame)
        return self

    def choose_second(self, candidates):
        if self.first is None:
            return None
        first = self.first.candidate
        eligible = []
        for item in candidates:
            if self.iou(first.bbox, item.bbox) > 0.03:
                continue
            separation = math.hypot(
                item.ground_x_cm - first.ground_x_cm,
                item.ground_z_cm - first.ground_z_cm,
            )
            correct_side = (item.center_x > first.center_x if self.direction == "clockwise"
                            else item.center_x < first.center_x)
            if (
                correct_side
                and parking.MIN_GROUND_GAP_CM <= separation <= parking.MAX_GROUND_GAP_CM
                and abs(item.ground_x_cm - first.ground_x_cm) <= 85.0
                and item.ground_z_cm >= first.ground_z_cm + 10.0
            ):
                eligible.append(item)
        return max(eligible, key=lambda item: item.confidence + 0.25 * item.score,
                   default=None)


class PartialParkingController(parking.ParallelParkingController):
    """Keep the parallel controller's approach, then use direct entry."""

    def __init__(self, direction, last_pillar):
        super().__init__(direction, last_pillar)
        self.locked_pillar_track_id = None
        self.locked_entry_heading = None
        self.entry_heading_integral = 0.0
        self.entry_heading_control_at = None
        self.entry_visual_steer = 0.0
        self.entry_gap_passed = False
        self.locked_slot_ground = None
        self.approach_heading = None
        self.current_heading = None
        self.current_detector = None
        self.reposition_attempts = 0
        self.invalid_slot_frames = 0
        self.pillar_decision_started = time.monotonic()
        self.no_pillar_straight_started = None
        self.reverse_search_offset_deg = SEARCH_HEADING_OFFSET_DEG
        self.green_cw_reduced_search = False
        self.sensor_hold_return_state = None
        self.sensor_hold_last_sample = None
        self.sensor_hold_clear_samples = 0
        self.sensor_hold_stop_samples = 0
        self.parallel_align_last_sample = None
        if last_pillar == "none":
            self.state = "NO_PILLAR_STRAIGHT"
            self.no_pillar_straight_started = time.monotonic()

    def command(self, speed, direction, steering):
        if self.state == "REVERSE_BLOCK_SEARCH" and direction == 0 and speed > 0:
            super().command(0, 0, 0)
            return
        if (self.pillar_behavior(self.locked_last_pillar_color) == "green"
                and self.state in ("PASS_LAST_PILLAR", "GREEN_CW_TURN_RIGHT",
                                   "GREEN_CW_TURN_LEFT", "RED_ACW_TURN_LEFT",
                                   "RED_ACW_TURN_RIGHT")
                and speed > 0):
            speed = GREEN_CLOCKWISE_APPROACH_SPEED
        elif direction == 0 and self.state in ("SEARCH_FIRST_BOUNDARY", "TRACK_OPENING"):
            speed = min(speed, GREEN_CLOCKWISE_SEARCH_SPEED
                        if self.green_cw_reduced_search else
                        (FIRST_BLOCK_SEARCH_SPEED if self.state == "SEARCH_FIRST_BOUNDARY"
                         else SECOND_BLOCK_SEARCH_SPEED))
        elif (direction == 1 and self.green_cw_reduced_search
              and self.state in ("SEARCH_FIRST_BOUNDARY", "TRACK_OPENING")):
            speed = min(speed, 40)
        super().command(speed, direction, steering)

    def green_pillar_search(self):
        if self.state in ("SEARCH_FIRST_BOUNDARY", "TRACK_OPENING"):
            return False
        return super().green_pillar_search()

    def hold_for_front_sensor(self, reason):
        self.sensor_hold_return_state = self.state
        self.sensor_hold_last_sample = None
        self.sensor_hold_clear_samples = 0
        self.sensor_hold_stop_samples = 0
        self.brake()
        self.transition("FRONT_SENSOR_HOLD", reason)

    def reacquire_green_blocks(self, detector, reason):
        detector.reset_tracks()
        self.invalid_slot_frames = 0
        self.reposition_attempts = 0
        self.parking_search_phase = "forward"
        self.parking_search_phase_started = time.monotonic()
        super().transition("SEARCH_FIRST_BOUNDARY", reason)
        self.command(GREEN_CLOCKWISE_SEARCH_SPEED, 0,
                     self.parking_search_heading_hold(self.current_heading))

    def transition(self, new_state, reason):
        if (self.state == "PASS_LAST_PILLAR"
                and self.pillar_behavior(self.locked_last_pillar_color) == "green"
                and new_state in ("GREEN_RETURN_STRAIGHT", "SEARCH_FIRST_BOUNDARY")):
            if self.direction == "clockwise":
                new_state = "GREEN_CW_TURN_RIGHT"
                reason = "green pillar cleared; steering +45 forward until central TF-Luna reaches 30 cm"
            else:
                new_state = "RED_ACW_TURN_LEFT"
                reason = "red pillar cleared; steering -45 forward until central TF-Luna reaches 30 cm"
        if new_state == "ALIGN_VIRTUAL_CUTOUT":
            if not parking.heading_valid(self.start_heading):
                self.fail("entry heading cannot be locked without IMU heading")
                return
            if not parking.heading_valid(self.current_heading):
                self.fail("cannot save approach heading without IMU heading")
                return
            detector = self.current_detector
            gap = detector.virtual_gap if detector is not None else None
            frame_width = getattr(detector, "frame_width", 0)
            gap_ratio = gap[2] / float(frame_width) if gap and frame_width else 0.0
            if self.green_cw_reduced_search and not gap:
                self.invalid_slot_frames += 1
                if self.invalid_slot_frames >= 3:
                    self.reacquire_green_blocks(detector,
                                                "opening not visible; reacquiring blocks")
                else:
                    self.command(GREEN_CLOCKWISE_SEARCH_SPEED, 0,
                                 self.parking_search_heading_hold(self.current_heading))
                return
            if gap_ratio < MIN_VISIBLE_ENTRY_GAP_RATIO:
                if self.reposition_attempts >= MAX_REPOSITION_ATTEMPTS:
                    if self.green_cw_reduced_search:
                        self.reacquire_green_blocks(detector,
                                                    "opening still narrow; searching for a clearer slot")
                        return
                    self.fail(f"visible parking opening still too narrow ({gap_ratio:.3f})")
                    return
                self.reposition_attempts += 1
                self.approach_heading = self.current_heading
                new_state = "REPOSITION_AWAY"
                reason = (f"opening {gap_ratio:.3f} too narrow; "
                          f"saved heading {self.approach_heading:.1f}; "
                          f"reposition {self.reposition_attempts}/{MAX_REPOSITION_ATTEMPTS}")
            else:
                bearing = detector.slot_bearing_deg
                if (not isinstance(bearing, (int, float)) or
                        not math.isfinite(bearing) or
                        detector.slot_z_cm is None or
                        detector.slot_z_cm < ENTRY_AIM_MIN_Z_CM):
                    if self.green_cw_reduced_search:
                        self.invalid_slot_frames += 1
                        if self.invalid_slot_frames >= 3:
                            self.reacquire_green_blocks(detector,
                                                        "slot bearing unusable; reacquiring blocks")
                        else:
                            self.command(GREEN_CLOCKWISE_SEARCH_SPEED, 0,
                                         self.parking_search_heading_hold(self.current_heading))
                        return
                    self.fail("parking opening is not ahead with a valid bearing")
                    return
                self.invalid_slot_frames = 0
                aim_offset = parking.clamp(
                    bearing, -ENTRY_AIM_MAX_OFFSET_DEG,
                    ENTRY_AIM_MAX_OFFSET_DEG,
                )
                self.locked_entry_heading = (self.current_heading + aim_offset) % 360.0
                new_state = "ENTER_SPACE"
                reason = (f"opening {gap_ratio:.3f} wide enough; "
                          f"bearing {bearing:+.1f}; aiming {aim_offset:+.1f} "
                          f"to heading {self.locked_entry_heading:.1f}")
        if new_state == "ENTER_SPACE" and self.state != "ENTER_SPACE":
            self.entry_heading_integral = 0.0
            self.entry_heading_control_at = None
            self.entry_visual_steer = 0.0
            self.entry_gap_passed = False
        super().transition(new_state, reason)

    def parking_search_heading_hold(self, heading):
        if not parking.heading_valid(heading) or not parking.heading_valid(self.start_heading):
            return 0
        offset = min(
            SEARCH_HEADING_MAX_OFFSET_DEG,
            (GREEN_CLOCKWISE_SEARCH_HEADING_DEG if self.green_cw_reduced_search
             else SEARCH_HEADING_OFFSET_DEG)
            + self.parking_search_attempt * SEARCH_HEADING_STEP_DEG,
        )
        if self.direction == "clockwise":
            offset = -offset
        return parking.clamp(
            parking.angle_diff(self.start_heading + offset, heading),
            -min(45.0, SEARCH_STEER_LIMIT + self.parking_search_attempt * 5.0),
            min(45.0, SEARCH_STEER_LIMIT + self.parking_search_attempt * 5.0),
        )

    def opening_passed(self, detector, camera_fresh):
        if not camera_fresh:
            return False
        if (detector.first is None or detector.second is None or
                detector.first.misses or detector.second.misses):
            return False
        if detector.gap_center_x is not None:
            if self.direction == "clockwise":
                passed_in_image = detector.gap_center_x <= REPOSITION_PASSED_GAP_X
            else:
                passed_in_image = detector.gap_center_x >= 1.0 - REPOSITION_PASSED_GAP_X
            if passed_in_image:
                return True
        return (
            self.locked_slot_ground is not None
            and detector.slot_z_cm is not None
            and detector.slot_z_cm <=
            self.locked_slot_ground[1] - REPOSITION_ADVANCE_CM
        )

    def update(self, telemetry, detector, pillar_detection, camera_fresh):
        heading, left, front, right, _ = telemetry
        self.current_heading = heading
        self.current_detector = detector
        now = time.monotonic()
        if self.start_heading is None and parking.heading_valid(heading):
            self.start_heading = heading

        if self.state == "FRONT_SENSOR_HOLD":
            self.brake()
            status = parking.robot.last_telemetry_status
            sample = (status.get("sequence"), status.get("received_at"))
            if sample == self.sensor_hold_last_sample:
                return
            self.sensor_hold_last_sample = sample
            if self.sensor_hold_return_state == "ENTER_SPACE":
                at_stop = (parking.robot.telemetry_is_fresh()
                           and parking.distance_valid(front)
                           and front <= ENTRY_STOP_FRONT_CM)
                self.sensor_hold_stop_samples = (self.sensor_hold_stop_samples + 1
                                                 if at_stop else 0)
                if self.sensor_hold_stop_samples >= SENSOR_RECOVERY_SAMPLES:
                    self.sensor_hold_return_state = None
                    self.parallel_align_direction = 1
                    self.parallel_align_phase_started = now
                    self.parallel_heading_count = 0
                    self.parallel_align_last_sample = None
                    self.transition("PARALLEL_ALIGN",
                                    f"central TF-Luna confirmed {front} cm; aligning heading")
                    self.command(parking.PARALLEL_ALIGN_SPEED, 1, 0)
                    return
            clear = (parking.robot.telemetry_is_fresh()
                     and not status["hard_stop"]
                     and parking.distance_valid(front)
                     and front > ENTRY_STOP_FRONT_CM)
            self.sensor_hold_clear_samples = (self.sensor_hold_clear_samples + 1
                                              if clear else 0)
            if self.sensor_hold_clear_samples >= SENSOR_RECOVERY_SAMPLES:
                resume_state = self.sensor_hold_return_state
                self.sensor_hold_return_state = None
                self.sensor_hold_clear_samples = 0
                if resume_state in ("SEARCH_FIRST_BOUNDARY", "TRACK_OPENING"):
                    self.parking_search_phase_started = now
                if resume_state == "ENTER_SPACE":
                    self.entry_heading_control_at = None
                self.transition(resume_state, "front TF-Luna clear on three fresh readings")
            return

        if self.state == "PARALLEL_ALIGN":
            if not parking.robot.telemetry_is_fresh() or not parking.heading_valid(heading):
                self.brake()
                return

            status = parking.robot.last_telemetry_status
            sample = (status.get("sequence"), status.get("received_at"))
            heading_error = parking.angle_diff(parking.PARALLEL_HEADING_TARGET, heading)
            if sample != self.parallel_align_last_sample:
                self.parallel_align_last_sample = sample
                within_tolerance = abs(heading_error) <= parking.PARALLEL_HEADING_TOLERANCE
                self.parallel_heading_count = (
                    self.parallel_heading_count + 1 if within_tolerance else 0
                )
            if self.parallel_heading_count >= parking.PARALLEL_HEADING_CONFIRM_SAMPLES:
                self.brake()
                self.complete = True
                self.transition("COMPLETE", f"parallel heading confirmed at {heading:.1f} degrees")
                return

            if self.parallel_align_direction == 1:
                if now - self.parallel_align_phase_started >= parking.PARALLEL_ALIGN_STROKE_SECONDS:
                    self.parallel_align_direction = 0
                    self.parallel_align_phase_started = now
            elif (status["hard_stop"] or
                  (parking.distance_valid(front) and front <= ENTRY_STOP_FRONT_CM)):
                self.parallel_align_direction = 1
                self.parallel_align_phase_started = now

            correction_sign = 1 if heading_error > 0 else -1
            steering = correction_sign * parking.PARALLEL_ALIGN_STEER_LIMIT
            if self.parallel_align_direction == 1:
                steering = -steering
            self.command(parking.PARALLEL_ALIGN_SPEED,
                         self.parallel_align_direction, steering)
            return

        if self.state in ("GREEN_CW_TURN_RIGHT", "GREEN_CW_TURN_LEFT",
                          "RED_ACW_TURN_LEFT", "RED_ACW_TURN_RIGHT",
                          "SEARCH_FIRST_BOUNDARY", "TRACK_OPENING"):
            if (not parking.robot.telemetry_is_fresh() or
                    parking.robot.last_telemetry_status["hard_stop"] or
                    not parking.distance_valid(front)):
                self.hold_for_front_sensor("front TF-Luna unavailable or ESP32 hard stop during parking block search")
                return

        if self.state == "GREEN_CW_TURN_RIGHT":
            if not parking.heading_valid(heading):
                self.brake()
                return
            if front <= GREEN_CLOCKWISE_FRONT_STOP_CM:
                self.brake()
                self.transition("GREEN_CW_TURN_LEFT",
                                f"central TF-Luna reached {front} cm; steering full left slowly")
                return
            self.command(GREEN_CLOCKWISE_APPROACH_SPEED, 0, GREEN_CLOCKWISE_RIGHT_STEER)
            return

        if self.state == "GREEN_CW_TURN_LEFT":
            if not parking.heading_valid(heading) or not parking.heading_valid(self.start_heading):
                self.brake()
                return
            target_heading = self.start_heading - GREEN_CLOCKWISE_SEARCH_HEADING_DEG
            error = parking.angle_diff(target_heading, heading)
            if abs(error) <= ORIENT_HEADING_TOLERANCE_DEG:
                self.brake()
                self.green_cw_reduced_search = True
                self.parking_search_attempt = 0
                self.parking_search_phase = "forward"
                self.parking_search_phase_started = now
                self.transition("SEARCH_FIRST_BOUNDARY",
                                "left heading reached; using red-pillar block search")
                return
            self.command(GREEN_CLOCKWISE_APPROACH_SPEED, 0,
                         -GREEN_CLOCKWISE_LEFT_STEER)
            return

        if self.state == "RED_ACW_TURN_LEFT":
            if not parking.heading_valid(heading):
                self.brake()
                return
            if front <= GREEN_CLOCKWISE_FRONT_STOP_CM:
                self.brake()
                self.transition("RED_ACW_TURN_RIGHT",
                                f"central TF-Luna reached {front} cm; steering full right slowly")
                return
            self.command(GREEN_CLOCKWISE_APPROACH_SPEED, 0, -GREEN_CLOCKWISE_RIGHT_STEER)
            return

        if self.state == "RED_ACW_TURN_RIGHT":
            if not parking.heading_valid(heading) or not parking.heading_valid(self.start_heading):
                self.brake()
                return
            target_heading = self.start_heading + GREEN_CLOCKWISE_SEARCH_HEADING_DEG
            error = parking.angle_diff(target_heading, heading)
            if abs(error) <= ORIENT_HEADING_TOLERANCE_DEG:
                self.brake()
                self.green_cw_reduced_search = True
                self.parking_search_attempt = 0
                self.parking_search_phase = "forward"
                self.parking_search_phase_started = now
                self.transition("SEARCH_FIRST_BOUNDARY",
                                "right heading reached; using green-pillar block search")
                return
            self.command(GREEN_CLOCKWISE_APPROACH_SPEED, 0,
                         GREEN_CLOCKWISE_LEFT_STEER)
            return

        if self.state == "SEARCH_LAST_PILLAR":
            if (parking.robot.last_telemetry_status["hard_stop"] or
                    (parking.distance_valid(front) and
                     front <= PILLAR_FRONT_STOP_CM)):
                self.fail("front obstruction during final pillar classification")
                return
            pillar_seen = (
                camera_fresh and pillar_detection is not None
                and pillar_detection.seen_this_frame
                and pillar_detection.color in ("red", "green")
                and (self.last_pillar == "auto"
                     or pillar_detection.color == self.last_pillar)
            )
            if not pillar_seen:
                if now - self.pillar_decision_started >= LAST_PILLAR_DECISION_SECONDS:
                    if not camera_fresh:
                        self.brake()
                        return
                    self.brake()
                    self.no_pillar_straight_started = now
                    self.transition("NO_PILLAR_STRAIGHT", "no final pillar confirmed within 3 seconds")
                    return
                self.command(FIRST_BLOCK_SEARCH_SPEED, 0, 0)
                return
            if (not pillar_detection.confirmed and
                    now - self.pillar_decision_started >= LAST_PILLAR_DECISION_SECONDS):
                if not camera_fresh:
                    self.brake()
                    return
                self.brake()
                self.no_pillar_straight_started = now
                self.transition("NO_PILLAR_STRAIGHT", "no confirmed final pillar within 3 seconds")
                return

        if self.state == "NO_PILLAR_STRAIGHT":
            if now - self.no_pillar_straight_started >= NO_PILLAR_STRAIGHT_SECONDS:
                self.brake()
                self.transition("REVERSE_BLOCK_SEARCH", "2-second straight approach complete")
                return
            if (parking.robot.last_telemetry_status["hard_stop"] or
                    (parking.distance_valid(front) and front <= PILLAR_FRONT_STOP_CM)):
                self.fail("front obstruction during no-pillar straight approach")
                return
            self.command(NO_PILLAR_STRAIGHT_SPEED, 0, 0)
            return
        if self.state == "REVERSE_BLOCK_SEARCH":
            if (detector.first is not None and detector.first.confirmed
                    and detector.first.misses == 0 and camera_fresh):
                self.brake()
                self.parking_search_phase = "forward"
                self.parking_search_phase_started = now
                self.transition("TRACK_OPENING", "first parking block confirmed during reverse search")
                return
            if (not camera_fresh or not parking.heading_valid(heading)
                    or not parking.robot.telemetry_is_fresh()):
                self.brake()
                return
            turn_side = left if self.direction == "clockwise" else right
            if not parking.distance_valid(turn_side):
                self.brake()
                return
            if turn_side <= REVERSE_BLOCK_SEARCH_SIDE_STOP_CM:
                self.fail("side clearance too small during reverse block search")
                return
            target_offset = (-self.reverse_search_offset_deg
                             if self.direction == "clockwise"
                             else self.reverse_search_offset_deg)
            target_heading = self.start_heading + target_offset
            heading_error = parking.angle_diff(target_heading, heading)
            if (abs(heading_error) <= REVERSE_BLOCK_SEARCH_HEADING_TOLERANCE_DEG
                    and self.reverse_search_offset_deg < SEARCH_HEADING_MAX_OFFSET_DEG):
                self.reverse_search_offset_deg = min(
                    SEARCH_HEADING_MAX_OFFSET_DEG,
                    self.reverse_search_offset_deg + SEARCH_HEADING_STEP_DEG,
                )
                target_offset = (-self.reverse_search_offset_deg
                                 if self.direction == "clockwise"
                                 else self.reverse_search_offset_deg)
                heading_error = parking.angle_diff(self.start_heading + target_offset, heading)
            steering = parking.clamp(
                -heading_error * 1.5,
                -REVERSE_BLOCK_SEARCH_STEER_LIMIT,
                REVERSE_BLOCK_SEARCH_STEER_LIMIT,
            )
            self.command(REVERSE_BLOCK_SEARCH_SPEED, 1, steering)
            return

        if self.state in ("REPOSITION_AWAY", "REPOSITION_FORWARD", "REPOSITION_RETURN"):
            away_side = right if self.direction == "clockwise" else left
            if (parking.robot.last_telemetry_status["hard_stop"] or
                    not parking.distance_valid(front) or
                    front <= REPOSITION_FRONT_CLEARANCE_CM):
                self.fail("insufficient front clearance while repositioning")
                return
            if (not parking.distance_valid(away_side) or
                    away_side <= REPOSITION_SIDE_CLEARANCE_CM):
                self.fail("insufficient away-side clearance while repositioning")
                return
            if not parking.heading_valid(heading):
                self.brake()
                return
            if self.state == "REPOSITION_AWAY":
                if self.opening_passed(detector, camera_fresh):
                    self.brake()
                    self.transition("REPOSITION_RETURN", "opening passed in camera; returning to saved heading")
                    return
                away_offset = (REPOSITION_AWAY_DEG if self.direction == "clockwise"
                               else -REPOSITION_AWAY_DEG)
                error = parking.angle_diff(self.approach_heading + away_offset, heading)
                if abs(error) <= ORIENT_HEADING_TOLERANCE_DEG:
                    self.brake()
                    self.transition("REPOSITION_FORWARD", "away offset reached; following opening past camera")
                    return
                self.command(REPOSITION_SPEED, 0, parking.clamp(
                    error * 2.0, -REPOSITION_AWAY_STEER_LIMIT,
                    REPOSITION_AWAY_STEER_LIMIT,
                ))
                return
            if self.state == "REPOSITION_FORWARD":
                if self.opening_passed(detector, camera_fresh):
                    self.brake()
                    self.transition("REPOSITION_RETURN", "opening passed in camera; returning to saved heading")
                    return
                if (not camera_fresh or detector.first is None or
                        detector.second is None or detector.first.misses or
                        detector.second.misses):
                    self.fail("parking opening lost before passing it")
                    return
                away_offset = (REPOSITION_AWAY_DEG if self.direction == "clockwise"
                               else -REPOSITION_AWAY_DEG)
                error = parking.angle_diff(self.approach_heading + away_offset, heading)
                self.command(REPOSITION_SPEED, 0, parking.clamp(error, -12.0, 12.0))
                return
            error = parking.angle_diff(self.approach_heading, heading)
            if abs(error) <= ORIENT_HEADING_TOLERANCE_DEG:
                self.brake()
                detector.reset_tracks()
                self.parking_search_attempt = 0
                self.parking_search_phase = "forward"
                self.parking_search_phase_started = now
                self.transition("SEARCH_FIRST_BOUNDARY", "approach heading restored; reacquiring both blocks")
                return
            self.command(REPOSITION_SPEED, 0, parking.clamp(
                error * 2.0, -REPOSITION_RETURN_STEER_LIMIT,
                REPOSITION_RETURN_STEER_LIMIT,
            ))
            return

        if (self.state == "GREEN_FORWARD_STRAIGHT" and
                now - self.state_started >= GREEN_FORWARD_SEARCH_SECONDS):
            self.parking_search_phase_started = now
            self.transition("SEARCH_FIRST_BOUNDARY", "short green forward step complete; searching parking block 1")
            self.command(FIRST_BLOCK_SEARCH_SPEED, 0, self.parking_search_heading_hold(heading))
            return

        if self.state in ("SEARCH_LAST_PILLAR", "PASS_LAST_PILLAR"):
            if parking.distance_valid(front) and front <= PILLAR_FRONT_STOP_CM:
                self.fail(f"front obstacle at {front} cm during final pillar pass")
                return
            if (self.state == "PASS_LAST_PILLAR" and
                    now - self.state_started >= PASS_LAST_PILLAR_TIMEOUT_SECONDS):
                self.fail("final pillar pass timed out")
                return
            if (self.state == "PASS_LAST_PILLAR" and
                    pillar_detection is not None and
                    pillar_detection.track_id != self.locked_pillar_track_id):
                pillar_detection = None

        if self.state == "ENTER_SPACE":
            if parking.distance_valid(front) and front <= ENTRY_STOP_FRONT_CM:
                self.hold_for_front_sensor(f"confirming central TF-Luna {front} cm at parking stop")
                return
            if parking.robot.last_telemetry_status["hard_stop"]:
                self.hold_for_front_sensor("front hard stop during entry")
                return
            if not parking.distance_valid(front) or not parking.robot.telemetry_is_fresh():
                self.hold_for_front_sensor("central TF-Luna unavailable during entry")
                return
            if not parking.heading_valid(heading):
                self.brake()
                return
            fresh_slot = (
                camera_fresh and detector.first is not None
                and detector.second is not None
                and detector.first.misses == 0
                and detector.second.misses == 0
                and detector.slot_z_cm is not None
            )
            if (fresh_slot and not self.entry_gap_passed
                    and detector.slot_z_cm < ENTRY_GAP_TRACK_MIN_Z_CM):
                self.entry_gap_passed = True
                self.entry_heading_integral = 0.0
                self.entry_visual_steer = 0.0
            heading_error = parking.angle_diff(self.locked_entry_heading, heading)
            dt = (0.0 if self.entry_heading_control_at is None else
                  parking.clamp(now - self.entry_heading_control_at, 0.0, 0.2))
            self.entry_heading_control_at = now
            proposed_integral = parking.clamp(
                self.entry_heading_integral + heading_error * dt,
                -ENTRY_HEADING_INTEGRAL_LIMIT,
                ENTRY_HEADING_INTEGRAL_LIMIT,
            )
            proposed_steering = (
                heading_error * ENTRY_HEADING_HOLD_GAIN
                + proposed_integral * ENTRY_HEADING_HOLD_INTEGRAL_GAIN
            )
            if (self.entry_gap_passed
                    and (abs(proposed_steering) <= ENTRY_HEADING_HOLD_LIMIT or
                         proposed_steering * heading_error < 0)):
                self.entry_heading_integral = proposed_integral
            if (fresh_slot and not self.entry_gap_passed
                    and detector.gap_center_x is not None
                    and detector.slot_z_cm >= ENTRY_GAP_TRACK_MIN_Z_CM):
                gap_steering = parking.clamp(
                    (detector.gap_center_x - ENTRY_GAP_TARGET_X)
                    * ENTRY_GAP_STEER_GAIN,
                    -ENTRY_GAP_STEER_LIMIT, ENTRY_GAP_STEER_LIMIT,
                )
                self.entry_visual_steer += ENTRY_GAP_SMOOTHING * (
                    gap_steering - self.entry_visual_steer
                )
            else:
                self.entry_visual_steer = 0.0
            steering = parking.clamp(
                heading_error * ENTRY_HEADING_HOLD_GAIN
                + self.entry_heading_integral * ENTRY_HEADING_HOLD_INTEGRAL_GAIN
                + self.entry_visual_steer,
                -ENTRY_HEADING_HOLD_LIMIT, ENTRY_HEADING_HOLD_LIMIT,
            )
            self.command(parking.ENTRY_SPEED, 0, steering)
            return

        previous_state = self.state
        super().update(telemetry, detector, pillar_detection, camera_fresh)
        if previous_state == "SEARCH_LAST_PILLAR" and self.state == "PASS_LAST_PILLAR":
            self.locked_pillar_track_id = pillar_detection.track_id
        if previous_state == "TRACK_OPENING" and self.state in (
                "REPOSITION_AWAY", "ENTER_SPACE"):
            self.locked_slot_ground = (detector.slot_x_cm, detector.slot_z_cm)
            self.brake()


def partial_parking_main(direction, last_pillar="auto"):
    robot = parking.robot
    robot.DIRECTION = direction
    if not parking.CALIBRATION_FILE.exists():
        raise RuntimeError(f"Missing calibration: {parking.CALIBRATION_FILE}")
    calibration = parking.ParkingCalibration.load(parking.CALIBRATION_FILE)
    detector = PartialParkingGeometryDetector(direction, calibration)
    pillar_detector = PartialPillarDetector()
    controller = PartialParkingController(direction, last_pillar)
    last_camera_sequence = -1
    last_status_at = 0.0
    display_frame = None
    pillar_detection = None
    print(f"[PARTIAL PARKING] direction={direction}; lastPillar={last_pillar}; "
          f"calibration={parking.CALIBRATION_FILE}")

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
            now = time.monotonic()
            if now - last_status_at >= 0.5:
                speed, drive_direction, steering = controller.last_command
                gap = ("none" if detector.gap_center_x is None
                       else f"{detector.gap_center_x:.2f}")
                pillar = ("none" if pillar_detection is None else
                          f"{pillar_detection.color}:{pillar_detection.hit_count}:"
                          f"{int(pillar_detection.seen_this_frame)}")
                slot = ("none" if detector.slot_x_cm is None else
                        f"({detector.slot_x_cm:.0f},{detector.slot_z_cm:.0f})"
                        f"/{detector.slot_bearing_deg:.0f}deg")
                print(f"[PARTIAL PARKING] {controller.state} "
                      f"heading={telemetry[0]} left={telemetry[1]} front={telemetry[2]} "
                      f"locked_heading={controller.locked_entry_heading} "
                      f"visual_steer={controller.entry_visual_steer:+.1f} "
                      f"pillar={pillar} "
                      f"gap_x={gap} slot={slot} "
                      f"slot_confident={detector.slot_confident} "
                      f"speed={speed} direction={drive_direction} steer={steering:+.0f}")
                last_status_at = now
            if display_frame is not None and robot.SHOW_LIVE_UI:
                robot.publish_detection_view(detector.annotate(display_frame, controller, telemetry))
            time.sleep(robot.NAVIGATION_LOOP_SECONDS)
    finally:
        controller.brake()

    if controller.state == "FAULT":
        print(f"[PARTIAL PARKING] FAULT: {controller.fault_reason}; brake applied")
        return 1
    print("[PARTIAL PARKING] COMPLETE; brake applied")
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description="Standalone 90 degree parking")
    parser.add_argument("--direction", choices=("clockwise", "anticlockwise"),
                        default="clockwise")
    parser.add_argument("--last-pillar", choices=("auto", "red", "green", "none"),
                        default="auto",
                        help="final pillar handling; auto detects red or green if present")
    return parser.parse_args()


if __name__ == "__main__":
    robot = parking.robot
    exit_code = 1
    try:
        args = parse_args()
        robot.start_live_ui()
        robot.led.on()
        robot.waitForOk()
        robot.flush_serial()
        exit_code = partial_parking_main(args.direction, args.last_pillar)
    except KeyboardInterrupt:
        print("[PARTIAL PARKING] Interrupted; applying electrical brake")
        exit_code = 130
    finally:
        robot.ui_running = False
        try:
            robot.send_data(0, 0, 0)
        finally:
            robot.cap.release()
            robot.cv2.destroyAllWindows()
            robot.led.off()
            robot.ser.close()
    raise SystemExit(exit_code)
