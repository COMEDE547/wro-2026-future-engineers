#!/usr/bin/env python3
import cv2
import numpy as np
import onnxruntime as ort
import serial
import time
from collections import deque

# ── CONFIG ──
MODEL_PATH = "best224.onnx"
LABELS = ["green", "red"]
COLORS = [(0, 255, 0), (0, 0, 255)]
CONF_THRESH = 0.6
IMG_SIZE = 224

# Serial
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Distance estimation
FOCAL_CONSTANT = 3600          # Calibrate for your camera
DISTANCE_THRESHOLD = 30.0      # cm

# Steering commands (match ESP32 firmware)
LEFT = -90
CENTER = 0
RIGHT = 90

# Smoothing
SMOOTH_BUFFER = 5
STEER_COOLDOWN = 0.1

# ── Load model ──
session = ort.InferenceSession(
    MODEL_PATH,
    providers=["CPUExecutionProvider"]
)
input_name = session.get_inputs()[0].name


def preprocess(frame):
    h, w = frame.shape[:2]
    scale = IMG_SIZE / max(h, w)
    nh, nw = int(h * scale), int(w * scale)

    resized = cv2.resize(frame, (nw, nh))
    canvas = np.full((IMG_SIZE, IMG_SIZE, 3), 114, dtype=np.uint8)

    top = (IMG_SIZE - nh) // 2
    left = (IMG_SIZE - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized

    img = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)

    return img, scale, left, top


def postprocess(outputs, scale, left, top):
    preds = outputs[0][0]
    boxes = []

    for pred in preds:
        x1, y1, x2, y2, conf, cls_id = pred

        if conf < CONF_THRESH:
            continue

        x1 = int((x1 - left) / scale)
        y1 = int((y1 - top) / scale)
        x2 = int((x2 - left) / scale)
        y2 = int((y2 - top) / scale)

        boxes.append((x1, y1, x2, y2, float(conf), int(cls_id)))

    return boxes


def draw_boxes(frame, boxes):
    for x1, y1, x2, y2, conf, cls_id in boxes:
        color = COLORS[cls_id]
        label = f"{LABELS[cls_id]} {conf:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame,
            label,
            (x1, y1 - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    return frame


# ── Serial setup ──
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

while ser.in_waiting:
    print("ESP32:", ser.readline().decode().strip())

# Camera
cap = cv2.VideoCapture(0)

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_center_x = frame_width // 2

# Steering smoothing
steer_buffer = deque(maxlen=SMOOTH_BUFFER)

last_sent_command = CENTER
last_send_time = 0

print("Avoidance active. Press 'q' to quit.")

while True:

    ret, frame = cap.read()

    if not ret:
        break

    frame = cv2.flip(frame, 1)

    inp, scale, left, top = preprocess(frame)
    outputs = session.run(None, {input_name: inp})
    boxes = postprocess(outputs, scale, left, top)

    frame = draw_boxes(frame, boxes)

    steering_command = CENTER

    closest_distance = 9999

    # Find closest obstacle
    for (x1, y1, x2, y2, conf, cls_id) in boxes:

        box_width = x2 - x1

        if box_width <= 0:
            continue

        distance = FOCAL_CONSTANT / box_width

        if distance < DISTANCE_THRESHOLD and distance < closest_distance:

            closest_distance = distance

            block_center_x = (x1 + x2) / 2
            offset = block_center_x - frame_center_x

            if offset < 0:
                # Obstacle on LEFT -> steer RIGHT
                steering_command = RIGHT
            else:
                # Obstacle on RIGHT -> steer LEFT
                steering_command = LEFT

    # Majority vote smoothing
    steer_buffer.append(steering_command)

    smooth_command = max(set(steer_buffer), key=steer_buffer.count)

    current_time = time.time()

    if (
        smooth_command != last_sent_command
        and current_time - last_send_time > STEER_COOLDOWN
    ):

        ser.write(f"{smooth_command}\n".encode())

        print(f"Sent: {smooth_command}")

        last_sent_command = smooth_command
        last_send_time = current_time

        while ser.in_waiting:
            print("ESP32:", ser.readline().decode().strip())

    cv2.putText(
        frame,
        f"Command: {smooth_command}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        2,
    )

    if closest_distance != 9999:
        cv2.putText(
            frame,
            f"Distance: {closest_distance:.1f} cm",
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

    # cv2.imshow("Obstacle Avoidance", frame)

    # if cv2.waitKey(1) & 0xFF == ord("q"):
    #     break

cap.release()
# cv2.destroyAllWindows()
ser.close()