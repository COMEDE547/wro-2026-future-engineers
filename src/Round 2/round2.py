#!/usr/bin/env python3

import time
import threading
import queue
import subprocess
import re
import serial
import numpy as np
import cv2
import av
from collections import deque
from PIL import Image
# Danger zone – only react to blocks in the middle of the frame
DANGER_LEFT  = 70
DANGER_RIGHT = 170
# --- Minimum height (in pixels) before the robot starts to swerve ---
MIN_SWERVE_HEIGHT = 45   # block must be at least this tall to trigger avoidance
REVERSE_HEIGHT = 80  # if block is below this y, reverse instead of swerve
            # --- Thresholds for determining which side the block is on ---
LEFT_SIDE_MAX   = 90   # pixel x < this = left side
RIGHT_SIDE_MIN  = 150   # pixel x > this = right side
def rgb_to_lab(frame: np.ndarray) -> np.ndarray:
    """Convert RGB to LAB color space."""
    rgb = frame.astype(np.float32) / 255.0
    mask = rgb > 0.04045
    linear = np.where(mask, ((rgb + 0.055) / 1.055) ** 2.4, rgb / 12.92)
    M = np.array([
        [0.4124564, 0.3575761, 0.1804375],
        [0.2126729, 0.7151522, 0.0721750],
        [0.0193339, 0.1191920, 0.9503041],
    ], dtype=np.float32)
    xyz = linear @ M.T
    xyz = xyz / np.array([0.95047, 1.0, 1.08883], dtype=np.float32)
    delta = 6.0 / 29.0
    f = np.where(xyz > delta ** 3, np.cbrt(xyz), xyz / (3 * delta ** 2) + 4.0 / 29.0)
    fx, fy, fz = f[..., 0], f[..., 1], f[..., 2]

    L = 116.0 * fy - 16.0
    a = 500.0 * (fx - fy)
    b = 200.0 * (fy - fz)

    return np.stack([L, a, b], axis=-1)


def get_masks(lab: np.ndarray, calib: dict) -> tuple:
    L, a, b = lab[..., 0], lab[..., 1], lab[..., 2]
    chroma = np.sqrt(a ** 2 + b ** 2)
    min_chroma = 10.0

    red = calib.get('red')
    green = calib.get('green')

    red_mask = np.zeros(L.shape, dtype=bool)
    green_mask = np.zeros(L.shape, dtype=bool)

    if red is not None:
        dist = np.sqrt((a - red['a_center']) ** 2 + (b - red['b_center']) ** 2)
        red_mask = (dist < red['tol']) & (L > red['l_min']) & (chroma > min_chroma)

    if green is not None:
        dist = np.sqrt((a - green['a_center']) ** 2 + (b - green['b_center']) ** 2)
        green_mask = (dist < green['tol']) & (L > green['l_min']) & (chroma > min_chroma)

    return red_mask, green_mask


def extract_bounding_box(mask: np.ndarray, min_area: int = 60, min_extent: float = 0.55) -> dict:
    """Finds the largest connected blob in the mask and returns its box."""
    from scipy import ndimage

    if mask.sum() < min_area:
        return None

    labeled, n_components = ndimage.label(mask)
    if n_components == 0:
        return None

    sizes = ndimage.sum(mask, labeled, index=range(1, n_components + 1))
    largest_label = int(np.argmax(sizes)) + 1
    largest_size = sizes[largest_label - 1]

    if largest_size < min_area:
        return None

    rows, cols = np.where(labeled == largest_label)
    x, y = int(cols.min()), int(rows.min())
    w, h = int(cols.max() - x), int(rows.max() - y)
    if w < 6 or h < 6 or (w / max(h, 1)) > 6 or (h / max(w, 1)) > 6:
        return None

    bbox_area = (w + 1) * (h + 1)
    extent = largest_size / bbox_area
    if extent < min_extent:
        return None

    return {"x": x, "y": y, "width": w, "height": h,
            "center_x": x + w // 2, "center_y": y + h // 2}


def process_frame(frame: np.ndarray, calib: dict, edge_margin: int = 6) -> tuple:
    """Process a frame and detect red/green blocks."""
    if frame is None or frame.size == 0:
        return None, None

    lab = rgb_to_lab(frame)
    red_mask, green_mask = get_masks(lab, calib)

    if edge_margin > 0:
        red_mask[:edge_margin, :] = False
        red_mask[-edge_margin:, :] = False
        red_mask[:, :edge_margin] = False
        red_mask[:, -edge_margin:] = False
        green_mask[:edge_margin, :] = False
        green_mask[-edge_margin:, :] = False
        green_mask[:, :edge_margin] = False
        green_mask[:, -edge_margin:] = False

    red_box = extract_bounding_box(red_mask)
    green_box = extract_bounding_box(green_mask)
    return red_box, green_box


def upscale_for_display(frame_bgr: np.ndarray, scale: int = 3) -> np.ndarray:
    """Upscale for display."""
    h, w = frame_bgr.shape[:2]
    return cv2.resize(frame_bgr, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)


def draw_boxes(frame_bgr: np.ndarray, red_box: dict, green_box: dict, roi: tuple = None) -> np.ndarray:
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


def run_calibration_session(get_frame, roi: tuple, window_name: str) -> dict:
    """Interactive calibration session."""
    MAX_SAMPLES = 6
    samples = {'red': deque(maxlen=MAX_SAMPLES), 'green': deque(maxlen=MAX_SAMPLES)}
    calib = {}

    print("\n=== CALIBRATION SESSION ===")
    print("Tip: let the camera's auto-exposure settle for a second before sampling.")
    print("Hold the RED block inside the cyan box, press 1 to sample (3-4x, moving it slightly).")
    print("Hold the GREEN block inside the cyan box, press 2 to sample (3-4x).")
    print("Press N when satisfied with both, or R to clear the last color's samples.")
    print("Press Q to abort calibration.\n")

    last_color = None
    last_sample_time = 0.0
    debounce_s = 0.6

    while True:
        frame = get_frame()
        if frame is None:
            continue

        red_box, green_box = (None, None)
        if calib:
            red_box, green_box = process_frame(frame, calib)

        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        display = draw_boxes(bgr, red_box, green_box, roi=roi)
        display = upscale_for_display(display, scale=3)
        status = (f"RED samples:{len(samples['red'])} GREEN samples:{len(samples['green'])} | "
                  f"1=sample RED  2=sample GREEN  N=done  Q=abort")
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


def set_manual_camera_controls(camera_id: int, exposure_value: int = 156,
                                wb_temperature: int = 4500):
    """Lock auto_exposure/white_balance so LAB calibration stays stable across runs."""
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

def main(camera_id: int = 0, frame_size: int = 240):
    """Main function with improved camera handling."""

    # --- Lock exposure/white balance before opening the stream ---
    set_manual_camera_controls(camera_id, exposure_value=500, wb_temperature=4500)

    # --- Serial connection to ESP32 ---
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        print("Serial port opened")
    except Exception as e:
        print(f"Could not open serial port: {e}")
        ser = None

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
    cv2.namedWindow(window_name)

    roi_w, roi_h = frame_size // 5, frame_size // 5
    roi = ((frame_size - roi_w) // 2, (frame_size - roi_h) // 2, roi_w, roi_h)

    calib = run_calibration_session(get_frame, roi, window_name)

    print("=== LIVE DETECTION ===")
    print("Press C to recalibrate, Q to quit.\n")

    history_len = 7
    required = 5
    red_hist = deque(maxlen=history_len)
    green_hist = deque(maxlen=history_len)

    # --- Single Kalman filter for whichever color is currently tracked ---
    kf = create_kalman_filter()
    kf_initialized = False
    kf_color = None  # which color the filter is currently locked onto

    last_sent = None
    frame_count = 0
    clear_counter = 0      
    CLEAR_HISTORY = 10

    try:
        while True:
            frame = get_frame(timeout=0.5)
            if frame is None:
                continue

            red_box, green_box = process_frame(frame, calib)
            red_hist.append(red_box)
            green_hist.append(green_box)

            red_confirmed = sum(b is not None for b in red_hist) >= required
            green_confirmed = sum(b is not None for b in green_hist) >= required


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

            if primary_box is not None:
                block_height = primary_box['height']
                
                # --- Too far away? (below 80) ---
                if block_height < MIN_SWERVE_HEIGHT:   # REVERSE_HEIGHT = 80
                    current_detection = None
                    active_box = None
                
                # --- Dangerously close? (above 100) ---
                elif block_height > REVERSE_HEIGHT:   # MIN_SWERVE_HEIGHT = 100
                    if ser is not None:
                        ser.write(b'REVERSE\n')
                        print(">>> Sent REVERSE (too close)")
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

            if current_detection is not None and active_box is not None:
                if active_box['height'] > REVERSE_HEIGHT and ser is not None:
                    ser.write(b'REVERSE\n')
                    print(">>> Sent REVERSE (too close)")
                    # skip the normal command for this frame
                    current_detection = None   # so it doesn't send RED/GREEN

            # --- Kalman smoothing for steering (single filter, reset on target change) ---
            if current_detection != kf_color:
                kf = create_kalman_filter()
                kf_initialized = False
                kf_color = current_detection

            smoothed = None
            if current_detection is not None:
                smoothed, kf_initialized = kalman_update(kf, active_box, kf_initialized)
            if current_detection is None:
                clear_counter += 1
            else:
                clear_counter = 0

            # Send command only on change
            if current_detection != last_sent and ser is not None:
                if current_detection is None and clear_counter < CLEAR_HISTORY:
                    pass
                else:
                    if current_detection == 'red':
                        ser.write(b'RED\n')
                        print(">>> Sent RED")
                    elif current_detection == 'green':
                        ser.write(b'GREEN\n')
                        print(">>> Sent GREEN")
                    else:
                        ser.write(b'CLEAR\n')
                        print(">>> Sent CLEAR")
                    last_sent = current_detection

            # Send smoothed steering data every frame while tracking
            # if current_detection is not None and smoothed is not None and ser is not None:
            #     steer_msg = (f"{current_detection.upper()},"
            #                  f"{smoothed['center_x']},{smoothed['center_y']},"
            #                  f"{smoothed['vx']:.1f}\n")
            #     ser.write(steer_msg.encode())

            # Display
            display_red = red_box if red_confirmed else None
            display_green = green_box if green_confirmed else None
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            display = draw_boxes(bgr, display_red, display_green)

            cv2.line(display, (LEFT_SIDE_MAX, 0), (LEFT_SIDE_MAX, frame_size-1), (0, 165, 255), 1)
            cv2.putText(display, "RED SAFE <", (2, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 165, 255), 1)
            
            cv2.line(display, (RIGHT_SIDE_MIN, 0), (RIGHT_SIDE_MIN, frame_size-1), (0,255,0), 1)
            cv2.putText(display, "GREEN SAFE >", (RIGHT_SIDE_MIN+2, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

            cv2.putText(display, "DANGER", (LEFT_SIDE_MAX+5, frame_size-10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
            
            Ldisplay = upscale_for_display(display, scale=3)
            cv2.imshow(window_name, display)
            frame_count += 1
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

            print(f"Frame {frame_count} | {red_str} | {green_str} | {smoothed_str}", flush=True)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                calib = run_calibration_session(get_frame, roi, window_name)
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
        stop_flag.set()
        t.join(timeout=2.0)
        container.close()
        if ser is not None:
            ser.close()
        cv2.destroyAllWindows()
        print("\nFinal calibration used:")
        for color, c in calib.items():
            print(f"  {color}: {c}")

if __name__ == "__main__":
    main(camera_id=0, frame_size=240)