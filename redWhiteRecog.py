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

# image = cv2.imread("pp.jpg")
cap = cv2.VideoCapture(3)
ret,image = cap.read()
image = cv2.undistort(image, camera_inst, camera_dist)
frame = image

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Board Detection
black_mask = cv2.inRange(hsv, black_lower, black_upper)
black_mask = cv2.dilate(black_mask, kernel)
contours, hierarchy = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if area > 5000:
        x1, y1, w, h = cv2.boundingRect(contour)
        frame = cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (255, 0, 0), 2)
        x2 = x1 + w
        y2 = y1 + h
        frame = frame[y1: y2, x1: x2]
        board_height = frame.shape[0]
        board_width = frame.shape[1]
        square_area_width = board_width / 7
        square_area_height = board_height / 6

        print(board_height, board_width)

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Red Coin Detection

red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
red_full_mask = red_mask1 + red_mask2
red_full_mask = cv2.dilate(red_full_mask, kernel)

cv2.imshow("rmask", red_full_mask)

# Creating contour to track Red color
contours, hierarchy = cv2.findContours(red_full_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Drawing rectangle
for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if area > 1800:  # Adjust this threshold according to your needs
        x1, y1, w, h = cv2.boundingRect(contour)
        frame = cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
        x2 = x1 + w
        y2 = y1 + h
        for j in range(0, 6):
            if y1 > (j * square_area_height) and y2 < ((j + 1) * square_area_height):
                for i in range(0, 7):
                    if x1 > i * square_area_width and x2 < (i + 1) * square_area_width:
                        board_array_1[j][i] = "1"

# Green Coin Detection

green_mask = cv2.inRange(hsv, green_lower, green_upper)

cv2.imshow("gmask", green_mask)

# Creating contour to track Red color
contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Drawing rectangle
for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if area > 1800:  # Adjust this threshold according to your needs
        x1, y1, w, h = cv2.boundingRect(contour)
        image = cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 0, 255), 2)
        x2 = x1 + w
        y2 = y1 + h
        for j in range(0, 6):
            if y1 > j * square_area_height and y2 < (j + 1) * square_area_height:
                for i in range(0, 7):
                    if x1 > i * square_area_width and x2 < (i + 1) * square_area_width:
                        board_array_1[j][i] = "2"

for k in range(0, 6):
    print(board_array_1[k])

# Display the image
cv2.imshow("image", image)

# Wait for 'q' key to close the window
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()
