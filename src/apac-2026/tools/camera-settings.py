import cv2
import numpy as np

def apply_adjustments_with_clahe(frame, brightness=0, contrast=1.0, gamma=1.0):
    adjusted = cv2.convertScaleAbs(frame, alpha=contrast, beta=brightness)

    inv_gamma = 1.0 / gamma
    table = np.array([(i / 255.0) ** inv_gamma * 255
                      for i in np.arange(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)

    lab = cv2.cvtColor(adjusted, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l_clahe = clahe.apply(l)

    lab_clahe = cv2.merge((l_clahe, a, b))
    final = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

    return final

cv2.namedWindow("Camera Adjustments")

cv2.createTrackbar("Brightness", "Camera Adjustments", 50, 100, lambda x: None)
cv2.createTrackbar("Contrast", "Camera Adjustments", 10, 30, lambda x: None)
cv2.createTrackbar("Gamma", "Camera Adjustments", 10, 50, lambda x: None)

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    brightness = cv2.getTrackbarPos("Brightness", "Camera Adjustments") - 50
    contrast = cv2.getTrackbarPos("Contrast", "Camera Adjustments") / 10.0
    gamma = cv2.getTrackbarPos("Gamma", "Camera Adjustments") / 10.0
    gamma = max(gamma, 0.1)

    adjusted_frame = apply_adjustments_with_clahe(frame, brightness, contrast, gamma)

    cv2.imshow("Camera Adjustments", adjusted_frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
