import cv2
import os

# Define chessboard size
chessboard_size = (9, 6)

# Create directory to save images
save_path = "calibration_images"
os.makedirs(save_path, exist_ok=True)

# Open webcam
cap = cv2.VideoCapture(1)  # Change index if multiple cameras are connected

img_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # Draw detected corners
    if ret:
        cv2.drawChessboardCorners(frame, chessboard_size, corners, ret)
        cv2.putText(frame, "Press SPACE to save", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Show the frame
    cv2.imshow("Chessboard Capture", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord(" "):  # Save image when SPACE is pressed
        img_name = os.path.join(save_path, f"calib_{img_count}.jpg")
        cv2.imwrite(img_name, frame)
        print(f"Saved {img_name}")
        img_count += 1

    elif key == ord("q"):  # Quit when 'q' is pressed
        break

# Release webcam and close windows
cap.release()
cv2.destroyAllWindows()
