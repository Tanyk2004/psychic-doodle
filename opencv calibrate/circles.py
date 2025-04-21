import cv2
import numpy as np

# Open the camera (0 is usually the default camera)
cap = cv2.VideoCapture(1)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture image.")
        break

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the HSV color range for the object (example range, adjust as needed)
    lower_color = np.array([0, 0, 199])  # Lower bound of the color
    upper_color = np.array([179, 8, 255])  # Upper bound of the color

    # Create the mask using the HSV range
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Apply the mask to the frame
    segmented_frame = cv2.bitwise_and(frame, frame, mask=mask)

    # Convert the segmented image to grayscale for contour detection
    gray = cv2.cvtColor(segmented_frame, cv2.COLOR_BGR2GRAY)

    # Threshold the grayscale image
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if contours were found
    if contours:
        # Find the largest contour by area
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the enclosing circle for the largest contour
        (center, radius) = cv2.minEnclosingCircle(largest_contour)

        # Convert the center to an integer
        center = (int(center[0]), int(center[1]))
        radius = int(radius)

        # Draw the enclosing circle on the original frame
        cv2.circle(frame, center, radius, (0, 255, 0), 2)

        # Optionally, draw the contour for comparison
        cv2.drawContours(frame, [largest_contour], -1, (0, 0, 255), 2)

        px_diameter = 2*radius
        real_diameter = 1.575
        fx = 647.07384177
        fy = 653.39571058
        cx = 353.31869253
        cy = 216.63488691
        z = ( fx * real_diameter)/ px_diameter
        x = (center[0] - cx) * z / fx
        y = (center[1] - cy) * z / fy
        cv2.putText(frame, f"Ball at {x:.2f},{y:.2f},{z:.2f}", (center[0], center[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)

    # Display the frame with the circle and contour
    cv2.imshow('Live Camera Feed', frame)

    # Wait for a key press to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture object and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()
