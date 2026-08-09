import cv2
import os

# Create folders if they don't exist
os.makedirs("red", exist_ok=True)
os.makedirs("green", exist_ok=True)
os.makedirs("park", exist_ok=True)

# Open webcam
cap = cv2.VideoCapture(0)  # Change to 1 if you have multiple cameras

if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Scan existing files to set correct initial counts
red_count = len(os.listdir("red"))
green_count = len(os.listdir("green"))
park_count = len(os.listdir("park"))  # FIXED: Added missing initialization

print("=== Photo Capture Tool ===")
print("Press 'r' to save a RED cuboid photo")
print("Press 'g' to save a GREEN cuboid photo")
print("Press 'p' to save a PARK photo")  # FIXED: Added user instruction
print("Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Show live view
    cv2.imshow("Capture", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('r'):
        filename = f"red/red_{red_count+1:04d}.jpg"
        cv2.imwrite(filename, frame)
        red_count += 1
        print(f"Saved {filename} (total red: {red_count})")

    elif key == ord('g'):
        filename = f"green/green_{green_count+1:04d}.jpg"
        cv2.imwrite(filename, frame)
        green_count += 1
        print(f"Saved {filename} (total green: {green_count})")
    
    elif key == ord('p'):
        filename = f"park/park_{park_count+1:04d}.jpg"  # FIXED: Now works because park_count exists
        cv2.imwrite(filename, frame)
        park_count += 1  # FIXED: Increments park_count instead of green_count
        print(f"Saved {filename} (total park: {park_count})")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
# FIXED: Updated final summary to show all three categories
print(f"\nDone! Red: {red_count} photos, Green: {green_count} photos, Park: {park_count} photos")
