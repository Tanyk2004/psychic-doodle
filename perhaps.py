import cv2
import numpy as np

# Camera matrix (intrinsics)
camera_matrix = np.array( [[190.1915536,    0.,         160.52495195],
 [  0.,         187.63877298, 147.50083495],
 [  0.,           0.,           1.        ]], dtype=np.float32)

# Distortion coefficients (example, replace with your actual values)
dist_coeffs = np.array([-0.08368914,  0.03974283, -0.0016545,   0.0050885,  -0.05173472], dtype=np.float32)

w, h = 344, 244
# Precompute undistortion and rectification maps
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)

real_diameter = 1.575  # cm
fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Undistort the frame
    undistorted_frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

    # Convert to HSV
    hsv_frame = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)

    # HSV range for white (adjust if needed)
    lower_color = np.array([0, 0, 199])
    upper_color = np.array([179, 8, 255])
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    segmented_frame = cv2.bitwise_and(undistorted_frame, undistorted_frame, mask=mask)

    gray = cv2.cvtColor(segmented_frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        (center, radius) = cv2.minEnclosingCircle(largest_contour)
        center = (int(center[0]), int(center[1]))
        radius = int(radius)

        cv2.circle(undistorted_frame, center, radius, (0, 255, 0), 2)
        cv2.drawContours(undistorted_frame, [largest_contour], -1, (0, 0, 255), 2)

        # 3D position estimate
        px_diameter = 2 * radius
        z = (fx * real_diameter) / px_diameter
        x = (center[0] - cx) * z / fx
        y = (center[1] - cy) * z / fy

        cv2.putText(undistorted_frame, f"Ball at {x:.2f},{y:.2f},{z:.2f}", (center[0], center[1]+20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Show the result
    cv2.imshow('Undistorted Camera Feed', undistorted_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
