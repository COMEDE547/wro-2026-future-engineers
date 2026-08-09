import cv2
import numpy as np
import serial
import time

# ==========================================
#      ESP32 SERIAL CONNECTION
# ==========================================
# Find your port with `ls /dev/tty*` before and after plugging in the ESP32
# (usually /dev/ttyUSB0 on a Pi, sometimes /dev/ttyACM0). Must match the
# ESP32's Serial.begin(115200) baud rate.
ESP32_PORT = "/dev/ttyUSB0"
ESP32_BAUD = 115200

esp32 = None
try:
    esp32 = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.05)
    time.sleep(2)  # let the ESP32 finish its reset-on-serial-open cycle
    print(f"Connected to ESP32 on {ESP32_PORT}")
except serial.SerialException as e:
    print(f"Could not open {ESP32_PORT}: {e}")
    print("Continuing in vision-only mode (no commands will be sent).")

# ==========================================
#      DODGE / STEERING CONFIG
# ==========================================
# Must match the ESP32's servo limits exactly (see Round2.ino).
SERVO_CENTER = 90
SERVO_MAX_LEFT = 135
SERVO_MAX_RIGHT = 45

# Matches Round 1's TURN_SPEED -- same controlled speed used for turning.
DODGE_SPEED = 180

# How many consecutive empty frames before we tell the ESP32 to resume
# autonomous driving. Avoids flickering control back and forth if detection
# briefly drops a frame or two.
RESUME_AFTER_MISSES = 5
consecutive_misses = 0
currently_dodging = False


def send_dodge_command(color, servo_angle, motor_speed):
    if esp32 is None:
        return
    color_str = "RED" if color == "red" else "GREEN"
    try:
        esp32.write(f"D,{color_str},{int(servo_angle)},{int(motor_speed)}\n".encode())
    except serial.SerialException as e:
        print(f"Serial write failed: {e}")


def send_resume_command():
    if esp32 is None:
        return
    try:
        esp32.write(b"R\n")
    except serial.SerialException as e:
        print(f"Serial write failed: {e}")


def compute_dodge_servo_angle(color, box_x, box_w, box_h, frame_w, frame_h):
    """
    Turns a detected obstacle into a servo angle.
    - RED: WRO rule says keep to the RIGHT side of the lane -> steer the
      robot right (toward SERVO_MAX_RIGHT) to pass with the pillar on its left.
    - GREEN: keep to the LEFT side of the lane -> steer left (toward
      SERVO_MAX_LEFT) to pass with the pillar on its right.
    The steering offset scales with how close the object looks (taller
    bounding box = closer), so distant objects get a gentle correction and
    close ones get a sharper one.
    """
    closeness = min(box_h / (frame_h * 0.6), 1.0)  # tune the 0.6 divisor to your camera/track distance
    max_offset = SERVO_MAX_LEFT - SERVO_CENTER  # assumes symmetric limits (45 here)
    offset = int(max_offset * closeness)

    if color == "red":
        servo_angle = SERVO_CENTER - offset
    else:  # green
        servo_angle = SERVO_CENTER + offset

    return max(SERVO_MAX_RIGHT, min(SERVO_MAX_LEFT, servo_angle))


# Open the webcam (0 is usually the built-in webcam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Press 'q' to close the camera window.")

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


def white_balance_gray_world(img):
    result = img.astype(np.float32)
    avg_b = np.mean(result[:, :, 0])
    avg_g = np.mean(result[:, :, 1])
    avg_r = np.mean(result[:, :, 2])
    avg_gray = (avg_b + avg_g + avg_r) / 3.0
    result[:, :, 0] *= (avg_gray / max(avg_b, 1e-6))
    result[:, :, 1] *= (avg_gray / max(avg_g, 1e-6))
    result[:, :, 2] *= (avg_gray / max(avg_r, 1e-6))
    return np.clip(result, 0, 255).astype(np.uint8)


def is_pillar_shaped(contour, min_aspect_ratio=1.2, min_solidity=0.85):
    """Rejects noisy blobs that pass the color mask but aren't pillar-shaped."""
    x, y, w, h = cv2.boundingRect(contour)
    if w == 0:
        return False
    aspect_ratio = h / float(w)
    contour_area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = contour_area / float(hull_area) if hull_area > 0 else 0
    return aspect_ratio >= min_aspect_ratio and solidity >= min_solidity


def compute_confidence(contour, hsv_frame, low_arr, high_arr, ideal_aspect_ratio=2.0):
    """
    Composite 0-100% confidence score combining:
    - Solidity (40%): how solid/convex the blob is vs. ragged noise
    - Aspect ratio fit (30%): how close to an ideal elongated pillar shape,
      not just barely above the minimum
    - Color centrality (30%): how deep into the S/V threshold range the
      object's actual pixels sit, vs. just barely scraping past the edge
    This is a heuristic score, not a statistical probability -- there's
    no ground truth to calibrate against, but it's useful for relatively
    ranking how trustworthy a detection is.
    """
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = h / float(w) if w > 0 else 0
    aspect_score = np.clip((aspect_ratio - 1.0) / (ideal_aspect_ratio - 1.0), 0, 1)

    contour_area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = contour_area / float(hull_area) if hull_area > 0 else 0

    obj_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
    cv2.drawContours(obj_mask, [contour], -1, 255, -1)
    mean_h, mean_s, mean_v = cv2.mean(hsv_frame, mask=obj_mask)[:3]

    s_center = (int(low_arr[1]) + int(high_arr[1])) / 2.0
    v_center = (int(low_arr[2]) + int(high_arr[2])) / 2.0
    s_half_range = (int(high_arr[1]) - int(low_arr[1])) / 2.0 + 1e-6
    v_half_range = (int(high_arr[2]) - int(low_arr[2])) / 2.0 + 1e-6
    s_score = 1 - min(abs(mean_s - s_center) / s_half_range, 1.0)
    v_score = 1 - min(abs(mean_v - v_center) / v_half_range, 1.0)
    color_score = (s_score + v_score) / 2.0

    confidence = (0.4 * solidity + 0.3 * aspect_score + 0.3 * color_score) * 100
    return int(round(confidence))


while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    balanced_frame = white_balance_gray_world(frame)
    blurred_frame = cv2.GaussianBlur(balanced_frame, (5, 5), 0)
    hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

    h_ch, s_ch, v_ch = cv2.split(hsv_frame)
    v_ch = clahe.apply(v_ch)
    hsv_frame = cv2.merge([h_ch, s_ch, v_ch])

    # --- DEFINE COLOR RANGES ---

    low_red1 = np.array([0, 140, 100])
    high_red1 = np.array([8, 255, 255])
    low_red2 = np.array([172, 140, 100])
    high_red2 = np.array([180, 255, 255])

    # RECALIBRATE with the updated hsv_calibrator.py (white balance + CLAHE
    # applied) -- don't reuse numbers found on the raw uncorrected feed.
    low_green = np.array([57, 0, 0])
    high_green = np.array([79, 255, 255])

    # --- CREATE MASKS ---
    mask_red = cv2.inRange(hsv_frame, low_red1, high_red1) + cv2.inRange(hsv_frame, low_red2, high_red2)
    mask_green = cv2.inRange(hsv_frame, low_green, high_green)

    # --- MORPHOLOGICAL CLEANUP ---
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

    # --- DETECT AND DRAW BOXES ---
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    frame_h, frame_w = frame.shape[:2]
    best_detection = None  # (confidence, color, x, y, w, h) -- highest confidence wins if both colors seen

    # Process Red Objects
    for contour in contours_red:
        if cv2.contourArea(contour) > 1500 and is_pillar_shaped(contour):
            conf = compute_confidence(contour, hsv_frame, low_red1, high_red1)
            if conf < 60:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, f"RED PILLAR {conf}%", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if best_detection is None or conf > best_detection[0]:
                best_detection = (conf, "red", x, y, w, h)

    # Process Green Objects
    for contour in contours_green:
        if cv2.contourArea(contour) > 1500 and is_pillar_shaped(contour):
            conf = compute_confidence(contour, hsv_frame, low_green, high_green)
            if conf < 60:
                continue
            hull = cv2.convexHull(contour)
            x, y, w, h = cv2.boundingRect(hull)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"GREEN PILLAR {conf}%", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if best_detection is None or conf > best_detection[0]:
                best_detection = (conf, "green", x, y, w, h)

    # --- SEND DODGE / RESUME COMMAND TO ESP32 ---
    if best_detection is not None:
        conf, color, x, y, w, h = best_detection
        servo_angle = compute_dodge_servo_angle(color, x, w, h, frame_w, frame_h)
        send_dodge_command(color, servo_angle, DODGE_SPEED)
        consecutive_misses = 0
        currently_dodging = True
        cv2.putText(frame, f"DODGE: {color.upper()} servo={servo_angle}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    else:
        consecutive_misses += 1
        if currently_dodging and consecutive_misses >= RESUME_AFTER_MISSES:
            send_resume_command()
            currently_dodging = False

    cv2.imshow("Live Object Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if esp32 is not None:
    send_resume_command()  # hand control back before we quit, just in case
    esp32.close()