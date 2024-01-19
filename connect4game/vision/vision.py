import cv2
import numpy as np
from connect4game.utils.logging import create_logger

class Vision:
    CAMERA_INST = np.array([[508.069, 0.0, 332.821], [0.0, 506.445, 242.984], [0.0, 0.0, 1.0]])
    CAMERA_DIST = np.array([[-0.4146402, 0.2407384, 0.0, 0.0, -0.1040042]])

    # Boundaries
    RED_LOWER_1 = np.array([0, 130, 150])
    RED_UPPER_1 = np.array([10, 255, 255])

    RED_LOWER_2 = np.array([160, 130, 150])
    RED_UPPER_2 = np.array([179, 255, 255])

    GREEN_LOWER = np.array([0, 40, 0])
    GREEN_UPPER = np.array([100, 180, 125])

    KERNEL = np.ones((5, 5), "uint8")

    def __init__(self, test=False):
        self.__logger = create_logger(name="VISION")
        self.test_mode = False
        self.board_array = [[0 for _ in range(7)] for _ in range(6)]
        self.image_rectangle = None

        if test:
            self.__logger.info("Test mode is active reading from the test img")
            self.test_image = cv2.imread("test.png")
            self.test_mode = True
        else:
            cam_find_loop = True
            current_cam_index = 0
            while cam_find_loop:
                self.cap = cv2.VideoCapture(current_cam_index)

                # Check if camera is working
                if not self.cap.isOpened():
                    current_cam_index += 1

                    # After 20 non functional cameras stop trying
                    if current_cam_index >= 20:
                        raise IndexError("Camera index limit reached")
                    
                    continue

                img = self.__get_image()
                self.__logger.info("To confirm the camera press y otherwise press n")
                cv2.imshow(f"Camera {current_cam_index}", img)

                while True:
                    key = cv2.waitKey(33)
                    if key == ord("y"):
                        cam_find_loop = False
                        break
                    elif key == ord("n"):
                        current_cam_index += 1
                        break
                    else:
                        img = self.__get_image()
                        cv2.imshow(f"Camera {current_cam_index}", img)
                        continue
                cv2.destroyAllWindows()

                # Release the capture of the camera if it is not correct
                if cam_find_loop:
                    self.cap.release()
    
    def __get_image(self):
        if self.test_mode:
            return self.test_image
        else:
            # Disregard for 5 frames for camera focus
            for _ in range(5):
                self.cap.read()
            
            # Get the image from camera
            ret, image = self.cap.read()

            # Apply undistort and flip coef
            image = cv2.undistort(image, self.CAMERA_INST, self.CAMERA_DIST)
            image = cv2.flip(image, 0)

        return image

    def __row_detection(self, image, param_x1, param_x2, param_y1, param_y2, param_gap, param_row_number):
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
            red_mask1 = cv2.inRange(hsv, self.RED_LOWER_1, self.RED_UPPER_1)
            red_mask2 = cv2.inRange(hsv, self.RED_LOWER_2, self.RED_UPPER_2)
            red_full_mask = red_mask1 + red_mask2
            red_full_mask = cv2.dilate(red_full_mask, self.KERNEL)

            # Creating contour to track Red color
            contours, hierarchy = cv2.findContours(red_full_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Drawing rectangle
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > 200:  # Adjust this threshold according to your needs
                    self.image_rectangle = cv2.rectangle(self.image_rectangle, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    self.board_array[param_row_number][i] = 1

            # Green Coin Detection
            green_mask = cv2.inRange(hsv, self.GREEN_LOWER, self.GREEN_UPPER)
            green_mask = cv2.dilate(green_mask, self.KERNEL)

            # Creating contour to track Red color
            contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Drawing rectangle
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > 1200:  # Adjust this threshold according to your needs
                    self.image_rectangle = cv2.rectangle(self.image_rectangle, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    self.board_array[param_row_number][i] = 2

        self.image_rectangle = image

    def detect_game(self):
        image = self.__get_image()
        self.image_rectangle = image

        self.__row_detection(image, 90, 150, 90, 135, 67, 0)
        self.__row_detection(image, 85, 140, 140, 185, 67, 1)
        self.__row_detection(image, 75, 135, 195, 245, 70, 2)
        self.__row_detection(image, 70, 130, 250, 300, 73, 3)
        self.__row_detection(image, 60, 120, 310, 365, 75, 4)
        self.__row_detection(image, 55, 120, 375, 430, 78, 5)

        return self.board_array

if __name__ == "__main__":

    vision = Vision()

    for i in range(5):

        board = vision.detect_game()

        for i in range(6):
            print(board[i])

        cv2.imshow("frame", vision.image_rectangle)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
