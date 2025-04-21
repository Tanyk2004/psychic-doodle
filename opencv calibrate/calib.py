import cv2
import numpy as np
import glob

# Define chessboard size
chessboard_size = (9, 6)  # 9x6 internal corners
square_size = 1.0  # Adjust if you know the real size

# Prepare object points (0,0,0), (1,0,0), (2,0,0) ...
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # Scale if real square size is known

# Storage for object points and image points
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load calibration images
images = glob.glob("opencv calibrate/calibration_images/*.jpg")  # Update with your folder path

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# Calibrate the camera
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print results
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)
