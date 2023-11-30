import numpy as np
import cv2

# Boundaries
red_lower1 = np.array([0, 130, 150])
red_upper1 = np.array([10, 255, 255])

red_lower2 = np.array([160, 130, 150])
red_upper2 = np.array([179, 255, 255])

white_lower = np.array([0, 0, 240])
white_upper = np.array([180, 30, 255])

kernel = np.ones((5, 5), "uint8")

cap = cv2.VideoCapture(1)

""" # Image test 
image = cv2.imread("image.jpg")

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
red_full_mask = red_mask1 + red_mask2
red_full_mask = cv2.dilate(red_full_mask, kernel)

# Creating contour to track Red color
contours, hierarchy = cv2.findContours(red_full_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Drawing rectangle
for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if area > 300:
        x, y, w, h = cv2.boundingRect(contour)
        frame = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

# Display the image
cv2.imshow("image", image)

# Wait for 'q' key to close the window
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()
"""

while True:

    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Red Color Detection

    # Masking for Red Color
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_full_mask = red_mask1 + red_mask2
    red_full_mask = cv2.dilate(red_full_mask, kernel)

    # Creating contour to track Red color
    contours, hierarchy = cv2.findContours(red_full_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Drawing rectangle
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            x, y, w, h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

    # White Color Detection

    # Masking for White Color
    white_mask = cv2.inRange(hsv, white_lower, white_upper)
    white_mask = cv2.dilate(white_mask, kernel)

    # Creating contour to track White color
    contours, hierarchy = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Drawing rectangle
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            x, y, w, h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 0, 0), 2)

    # Program Termination
    cv2.imshow("Multiple Color Detection in Real-TIme", frame)
    cv2.imshow("red f kernel", red_full_mask)
    cv2.imshow("white f kernel", white_mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
