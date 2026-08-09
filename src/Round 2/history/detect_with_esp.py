import cv2
import numpy as np
import onnxruntime as ort
import serial
import time

# ── CONFIG ──
MODEL_PATH  = 'best224.onnx'
LABELS      = ['green', 'red']
COLORS      = [(0, 255, 0), (0, 0, 255)]
CONF_THRESH = 0.6
IMG_SIZE    = 224

# Serial
SERIAL_PORT = '/dev/ttyUSB0'   # adjust as needed
BAUD_RATE   = 115200

# Distance estimation
FOCAL_CONSTANT = 3600   # CALIBRATE: width_pixels * distance_cm at known distance
DISTANCE_THRESHOLD = 40.0  # cm, only act when closer than this

# ── Load model ──
session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])
input_name = session.get_inputs()[0].name

def preprocess(frame):
    h, w = frame.shape[:2]
    scale = IMG_SIZE / max(h, w)
    nh, nw = int(h * scale), int(w * scale)
    resized = cv2.resize(frame, (nw, nh))
    canvas = np.full((IMG_SIZE, IMG_SIZE, 3), 114, dtype=np.uint8)
    top = (IMG_SIZE - nh) // 2
    left = (IMG_SIZE - nw) // 2
    canvas[top:top+nh, left:left+nw] = resized
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

# def draw_boxes(frame, boxes):
#     for x1, y1, x2, y2, conf, cls_id in boxes:
#         color = COLORS[cls_id]
#         label = f"{LABELS[cls_id]} {conf:.2f}"
#         cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
#         cv2.putText(frame, label, (x1, y1-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
#     return frame

# ── Serial setup ──
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)
while ser.in_waiting:
    print("ESP32:", ser.readline().decode().strip())

cap = cv2.VideoCapture(0)

# One-shot tracking: only send a command once per "close block" event,
# and don't send again until the block is no longer close (or leaves frame).
block_active = False
last_sent_label = None

print("Color-trigger avoidance active. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)  # mirror for natural driving
    inp, scale, left, top = preprocess(frame)
    outputs = session.run(None, {input_name: inp})
    boxes = postprocess(outputs, scale, left, top)
    # frame = draw_boxes(frame, boxes)

    # Find the closest qualifying block (largest box width = nearest),
    # rather than just the first one YOLO happens to list.
    closest_box = None
    closest_distance = None
    for (x1, y1, x2, y2, conf, cls_id) in boxes:
        box_width = x2 - x1
        if box_width <= 0:
            continue
        distance = FOCAL_CONSTANT / box_width  # estimated distance in cm
        if distance < DISTANCE_THRESHOLD:
            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
                closest_box = (x1, y1, x2, y2, conf, cls_id)

    status_text = "No close block"

    if closest_box is not None:
        cls_id = closest_box[5]
        label = LABELS[cls_id].upper()  # "RED" or "GREEN"
        status_text = f"Close block: {label} ({closest_distance:.0f}cm)"

        if not block_active:
            # New close-block event -> send the one-shot command
            ser.write(f"{label}\n".encode())
            print(f"Sent: {label}")
            last_sent_label = label
            block_active = True

            # Read any feedback from the ESP32
            time.sleep(0.05)
            while ser.in_waiting:
                print("ESP32:", ser.readline().decode().strip())
    else:
        # No close block right now -> reset so the next close block (even
        # the same color) triggers a fresh send.
        block_active = False

    # Display info
    cv2.putText(frame, status_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.imshow("Color-Trigger Avoidance", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()