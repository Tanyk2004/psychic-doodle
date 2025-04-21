import cv2
import numpy as np

# === Load Camera Calibration Parameters ===
camera_matrix = np.array([[647.074, 0, 353.319], [0, 653.396, 216.635], [0, 0, 1]])  # Replace with your values

# === Chessboard Parameters ===
chessboard_size = (9, 6)  # Adjust if needed
square_size = 25  # Adjust based on your actual chessboard square size (mm)

# === Define 3D Object Points for the Chessboard ===
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # Scale based on actual chessboard square size

# === Define 3D Cube Points (relative to the first chessboard corner) ===
cube_size = square_size  # Make cube the size of a single square
cube_points = np.float32([
    [0, 0, 0], [cube_size, 0, 0], [cube_size, cube_size, 0], [0, cube_size, 0],  # Bottom face
    [0, 0, -cube_size], [cube_size, 0, -cube_size], [cube_size, cube_size, -cube_size], [0, cube_size, -cube_size]  # Top face
])

# === Capture Image from Webcam ===
cap = cv2.VideoCapture(1)
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # === Find Chessboard Corners ===
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        # Refine corners
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                    (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001))

        # Estimate pose (rotation & translation)
        ret, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, None)  # No distortion coefficients

        if ret:
            # === Project Cube onto Image ===
            imgpts, _ = cv2.projectPoints(cube_points, rvec, tvec, camera_matrix, None)  # No distortion

            # Convert to integer values
            imgpts = np.int32(imgpts).reshape(-1, 2)

            # === Draw Cube ===
            # Bottom square
            frame = cv2.drawContours(frame, [imgpts[:4]], -1, (0, 255, 0), 3)
            # Vertical edges
            for i in range(4):
                frame = cv2.line(frame, tuple(imgpts[i]), tuple(imgpts[i + 4]), (255, 0, 0), 3)
            # Top square
            frame = cv2.drawContours(frame, [imgpts[4:]], -1, (0, 0, 255), 3)

        # Draw detected corners
        cv2.drawChessboardCorners(frame, chessboard_size, corners2, ret)

    # Show the frame
    cv2.imshow("Projected Cube (No Distortion)", frame)
    
    # Quit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
