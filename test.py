import cv2
import numpy as np

gray = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

print("Success!", color.shape)
