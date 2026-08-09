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



def rgb_to_lab(frame: np.ndarray) -> np.ndarray:
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
    """Finds the largest connected blob in the mask and returns its box.

    Two shape gates reject non-cuboid blobs (reflections, noise clusters,
    background clutter):
      - aspect ratio: not more than 6x longer than wide in either direction
      - extent: matched_pixels / (bbox_width * bbox_height) must be >=
        min_extent. A real cuboid face fills most of its bounding rectangle;
        scattered noise or thin/irregular shapes leave a lot of the box
        empty and get rejected here. This is the "does it look like a
        rectangle" check.
    """
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
    lab = rgb_to_lab(frame)
    red_mask, green_mask = get_masks(lab, calib)

    # Ignore a thin border — webcam vignetting/edge distortion commonly
    # produces false color matches right at frame corners/edges.
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
    """Upscale only for on-screen display; detection still runs on the small
    frame so this costs nothing on the compute side."""
    h, w = frame_bgr.shape[:2]
    return cv2.resize(frame_bgr, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)


def draw_boxes(frame_bgr: np.ndarray, red_box: dict, green_box: dict, roi: tuple = None) -> np.ndarray:
    out = frame_bgr.copy()
    if roi is not None:
        rx, ry, rw, rh = roi
        cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), (255, 255, 0), 1)
    if red_box:
        x, y, w, h = red_box['x'], red_box['y'], red_box['width'], red_box['height']
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(out, f"RED {w}x{h}", (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    if green_box:
        x, y, w, h = green_box['x'], green_box['y'], green_box['width'], green_box['height']
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(out, f"GREEN {w}x{h}", (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    return out


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def sample_roi_lab(frame: np.ndarray, roi: tuple) -> np.ndarray:
    x, y, w, h = roi
    patch = frame[y:y + h, x:x + w]
    return rgb_to_lab(patch)


def calibrate_color(lab_patches: list, margin: float = 5.0,
                     l_percentile: float = 5.0, max_tol: float = 15.0) -> dict:
    """Derive thresholds from one or more sampled patches.

    Accepts a LIST of patches (multiple key presses at slightly different
    block positions/angles get pooled together) rather than a single one —
    pooling multiple views of the same real object tightens the estimate
    instead of any single noisy sample dominating it.

    Uses median center + median-absolute-deviation for spread — both robust
    to a bit of background/hand contamination in any one sample. Tolerance
    is expressed as a single radius (for the circular distance mask) and
    hard-capped so a messy sample still yields a usable, non-degenerate
    mask instead of matching almost everything.
    """
    a_vals = np.concatenate([p[..., 1].flatten() for p in lab_patches])
    b_vals = np.concatenate([p[..., 2].flatten() for p in lab_patches])
    l_vals = np.concatenate([p[..., 0].flatten() for p in lab_patches])

    a_center = float(np.median(a_vals))
    b_center = float(np.median(b_vals))
    a_mad = float(np.median(np.abs(a_vals - a_center)))
    b_mad = float(np.median(np.abs(b_vals - b_center)))

    # Combine a/b spread into one radius (MAD*1.4826 ~= std-equivalent)
    spread = np.sqrt((a_mad * 1.4826) ** 2 + (b_mad * 1.4826) ** 2)
    tol = min(spread * 1.3 + margin, max_tol)

    return {
        'a_center': a_center,
        'b_center': b_center,
        'tol': tol,
        'l_min': float(np.percentile(l_vals, l_percentile)),
    }


def run_calibration_session(get_frame, roi: tuple, window_name: str) -> dict:
    """Interactive calibration: show ROI, user aligns block, presses key to
    sample. Each press ADDS a sample, but only the most recent MAX_SAMPLES
    per color are kept (sliding window) — this bounds calibration to recent,
    consistent readings instead of averaging in early samples that may have
    been taken under different auto-exposure/white-balance conditions if the
    camera drifted mid-session. Live boxes update after each sample so you
    can visually confirm accuracy before accepting with N."""
    from collections import deque
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
    """Open camera using V4L2 (Linux/Raspberry Pi)."""
    try:
        container = av.open(
            f'/dev/video{camera_id}',
            format='v4l2',
            options={
                'video_size': '640x480',
                'framerate': '30',
                'input_format': 'yuv420p'
            }
        )
        stream = container.streams.video[0]
        stream.thread_type = 'AUTO'
        return container, stream
    except Exception as e:
        print(f"Camera error: {e}")
        return None, None



def resize_frame(frame: np.ndarray, target_w: int = 240, target_h: int = 240) -> np.ndarray:
    # Cheap nearest/area-free resize via numpy stride slicing would alias badly;
    # use PIL only here since it's a one-time-per-frame cost, not per-pixel loop.
    from PIL import Image
    return np.array(Image.fromarray(frame).resize((target_w, target_h), Image.BILINEAR))


def start_capture_thread(container, stream, frame_size=240):
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
                    rgb_frame = frame.reformat(format='rgb24')
                    img = rgb_frame.to_ndarray(format='rgb24')
                    img = resize_frame(img, frame_size, frame_size)
                    if frame_q.full():
                        try:
                            frame_q.get_nowait()
                        except queue.Empty:
                            pass
                    frame_q.put(img)
        except Exception as e:
            print(f"\nCapture thread stopped: {e}")

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    return t, frame_q, stop_flag



def main(camera_id: int = 0, frame_size: int = 240):
    # --- Serial connection to ESP32 ---
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)   # Linux example
        # For Windows: serial.Serial('COM3', 115200, timeout=1)
        print("Serial port opened")
    except Exception as e:
        print(f"Could not open serial port: {e}")
        ser = None
    from collections import deque

    container, stream = open_camera(camera_id)
    if container is None:
        print("Cannot open webcam.")
        return

    t, frame_q, stop_flag = start_capture_thread(container, stream, frame_size)

    def get_frame(timeout=1.0):
        try:
            return frame_q.get(timeout=timeout)
        except queue.Empty:
            return None

    window_name = "WRO Block Detector"
    cv2.namedWindow(window_name)

    # ROI in the center of the frame for calibration sampling
    roi_w, roi_h = frame_size // 5, frame_size // 5
    roi = ((frame_size - roi_w) // 2, (frame_size - roi_h) // 2, roi_w, roi_h)

    calib = run_calibration_session(get_frame, roi, window_name)

    print("=== LIVE DETECTION ===")
    print("Press C to recalibrate (e.g. lighting changed), Q to quit.\n")

    history_len = 5
    required = 3
    red_hist = deque(maxlen=history_len)
    green_hist = deque(maxlen=history_len)

    last_sent = None          # 'red', 'green', or 'clear'
    frame_count = 0

    try:
        while True:
            frame = get_frame()
            if frame is None:
                continue

            red_box, green_box = process_frame(frame, calib)
            red_hist.append(red_box)
            green_hist.append(green_box)

            red_confirmed = sum(b is not None for b in red_hist) >= required
            green_confirmed = sum(b is not None for b in green_hist) >= required

            # Determine current detection
            current_detection = None
            if green_confirmed and not red_confirmed:
                current_detection = 'green'
            elif red_confirmed and not green_confirmed:
                current_detection = 'red'
            # If both or none, treat as 'clear'

            # Send command only on change
            if current_detection != last_sent and ser is not None:
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

            # --- display (unchanged) ---
            display_red = red_box if red_confirmed else None
            display_green = green_box if green_confirmed else None
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            display = draw_boxes(bgr, display_red, display_green)
            display = upscale_for_display(display, scale=3)
            cv2.imshow(window_name, display)

            frame_count += 1
            red_str = (f"R[x={display_red['x']},y={display_red['y']},"
                        f"w={display_red['width']},h={display_red['height']}]"
                        if display_red else "R:None")
            green_str = (f"G[x={display_green['x']},y={display_green['y']},"
                            f"w={display_green['width']},h={display_green['height']}]"
                            if display_green else "G:None")
            print(f"Frame {frame_count} | {red_str} | {green_str}", end='\r')

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                calib = run_calibration_session(get_frame, roi, window_name)
                red_hist.clear()
                green_hist.clear()
                last_sent = None      # reset to force re‑sending
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


    