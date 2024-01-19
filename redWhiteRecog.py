import cv2
import numpy as np

camera_inst = np.array([[508.069, 0.0, 332.821], [0.0, 506.445, 242.984], [0.0, 0.0, 1.0]])
camera_dist = np.array([[-0.4146402, 0.2407384, 0.0, 0.0, -0.1040042]])

board_array_1 = [["." for _ in range(7)] for _ in range(6)]

# Boundaries
red_lower1 = np.array([0, 130, 150])
red_upper1 = np.array([10, 255, 255])

red_lower2 = np.array([160, 130, 150])
red_upper2 = np.array([179, 255, 255])

green_lower = np.array([0, 40, 0])
green_upper = np.array([100, 180, 125])

black_lower = np.array([0, 0, 0])
black_upper = np.array([179, 255, 74])

kernel = np.ones((5, 5), "uint8")
"""
cap = cv2.VideoCapture(1)

while True:
    ret, image = cap.read()
    image = cv2.undistort(image, camera_inst, camera_dist)
    image = cv2.flip(image, 0)
    # Display the image
    cv2.imshow("image", image)

    # Wait for 'q' key to close the window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        cap.release()
        cv2.imwrite("test.png", image)
"""
image = cv2.imread("test.png")


def row_detection(param_x1, param_x2, param_y1, param_y2, param_gap, param_row_number):
    global image

    for i in range(0, 7):
        x1 = param_x1 + (param_gap * i)
        x2 = param_x2 + (param_gap * i)
        y1 = param_y1
        y2 = param_y2
        if i == 0:
            box_frame = image[y1:y2, x1:x2]
        else:
            box_frame = image[y1:y2, x1:x2]
        hsv = cv2.cvtColor(box_frame, cv2.COLOR_BGR2HSV)

        # Red Coin Detection
        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_full_mask = red_mask1 + red_mask2
        red_full_mask = cv2.dilate(red_full_mask, kernel)

        # Creating contour to track Red color
        contours, hierarchy = cv2.findContours(red_full_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Drawing rectangle
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > 600:  # Adjust this threshold according to your needs
                image = cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                board_array_1[param_row_number-1][i] = 1

        # Green Coin Detection
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        green_mask = cv2.dilate(green_mask, kernel)
        # Creating contour to track Red color
        contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Drawing rectangle
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > 600:  # Adjust this threshold according to your needs
                image = cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                board_array_1[param_row_number-1][i] = 2


row_detection(90, 150, 90, 135, 67, 1)

row_detection(85, 140, 140, 185, 67, 2)

row_detection(75, 135, 195, 245, 70, 3)

row_detection(70, 130, 250, 300, 73, 4)

row_detection(60, 120, 310, 365, 75, 5)

row_detection(55, 120, 375, 430, 78, 6)

for k in range(0, 6):
    print(board_array_1[k])

cv2.imshow("frame", image)
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()





