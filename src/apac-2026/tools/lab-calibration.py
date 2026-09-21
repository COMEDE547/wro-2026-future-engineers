import cv2
import numpy as np

BRIGHTNESS = 0
CONTRAST = 3.0
GAMMA = 0.7

ADJUST = True

def adjust_frame_with_clahe(frame):
    adjusted = cv2.convertScaleAbs(frame, alpha=CONTRAST, beta=BRIGHTNESS)

    inv_gamma = 1.0 / GAMMA
    table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)

    lab = cv2.cvtColor(adjusted, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l_clahe = clahe.apply(l)

    lab_clahe = cv2.merge((l_clahe, a, b))
    final_bgr = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

    return final_bgr

cv2.namedWindow("Mask")
cv2.namedWindow("Controls")

cv2.createTrackbar("L Lower", "Controls", 0, 255, lambda x: None)
cv2.createTrackbar("A Lower", "Controls", 0, 255, lambda x: None)
cv2.createTrackbar("B Lower", "Controls", 0, 255, lambda x: None)
cv2.createTrackbar("L Upper", "Controls", 255, 255, lambda x: None)
cv2.createTrackbar("A Upper", "Controls", 255, 255, lambda x: None)
cv2.createTrackbar("B Upper", "Controls", 255, 255, lambda x: None)

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if ADJUST:
        adjusted_frame = adjust_frame_with_clahe(frame)
    else:
        adjusted_frame = frame

    lab_frame = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2LAB)

    l_lower = cv2.getTrackbarPos("L Lower", "Controls")
    a_lower = cv2.getTrackbarPos("A Lower", "Controls")
    b_lower = cv2.getTrackbarPos("B Lower", "Controls")
    l_upper = cv2.getTrackbarPos("L Upper", "Controls")
    a_upper = cv2.getTrackbarPos("A Upper", "Controls")
    b_upper = cv2.getTrackbarPos("B Upper", "Controls")

    lower_bound = np.array([l_lower, a_lower, b_lower], dtype=np.uint8)
    upper_bound = np.array([l_upper, a_upper, b_upper], dtype=np.uint8)

    mask = cv2.inRange(lab_frame, lower_bound, upper_bound)
    result = cv2.bitwise_and(adjusted_frame, adjusted_frame, mask=mask)

    cv2.imshow("Mask", result)
    cv2.imshow("Original", adjusted_frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
