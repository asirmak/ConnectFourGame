from pyniryo2 import *
from roslibpy.core import RosTimeoutError
from src.utils.Logging import CreateLogger
import time
import traceback
import cv2
class Robot:
    # Constant variables

    # Logger for the robot
    __logger = CreateLogger(
        name="ROBOT"
    )

    # Positions of certain objects
    __first_piece_pos = PoseObject(
        x=0.14, y=0.0, z=0.152,
        roll=0.0, pitch=1.55, yaw=0
    ) # location of the piece at index 0 on the belt
    __second_piece_pos = PoseObject(
        x=0.2, y=0.0, z=0.148,
        roll=0.0, pitch=1.55, yaw=0
    ) # location of the piece at index 1 on the belt

    def __init__(self, robot_ip = "169.254.200.200"): # if ip addr is argument not provided then use the ethernet port
        # Connect to robot
        try:
            self.__robot = NiryoRobot(robot_ip)
        except:
            self.__logger.critical("Robot connection failed, check if the ip addr is correct")
            raise
        
        try:
            # Calibrate the robot
            self.__executeRobotAction(
                self.__robot.arm.calibrate_auto
            )
            self.__logger.info("Arm is calibrated and ready to use")

            # Detect the currently attached tool
            self.__executeRobotAction(
                self.__robot.tool.update_tool
            )
            current_tool = self.__executeRobotAction(
                self.__robot.tool.get_current_tool_id
            )
            # Open the gripper
            self.__executeRobotAction(
                self.__robot.tool.release_with_tool
            )
            self.__logger.info("Gripper is ready to use")

            # Set up the conveyer belt
            self.__conveyor_id = self.__executeRobotAction(
                self.__robot.conveyor.set_conveyor
            )
            self.__logger.info("Conveyer belt ready to use")

            # Move robot to its default position
            self.__moveToHome()
        except:
            # If any error happens during the calibration, end the robot so the code does not hang
            self.endRobot()
            raise

        self.__currentPiece = 0
        self.__beltSetUp = False

    # work around ros timing bug where the robot fails sometimes for no reason
    def __executeRobotAction(self, action, *args):
        action_retry = True
        result = None
        while action_retry:
            try:
                result = action(*args)
                action_retry = False
            except TypeError:
                self.__logger.critical("You did a coding error, do not pass function call instead pass a function reference")
                raise
            except RosTimeoutError as e:
                # robot internal bug safe to ignore
                # TODO in the future maybe add a retry limit
                self.__logger.warning("Robot internal timing bug, safe to ignore, retrying action...")
                continue
        return result

    # Function for moving back to the home pose
    def __moveToHome(self):
        self.__executeRobotAction(
            self.__robot.arm.move_to_home_pose
        )
        self.__logger.info("Moved to home position")

    # Move robot to specified position
    def __moveToPos(self, pos: PoseObject):
        self.__executeRobotAction(
            self.__robot.arm.move_pose, pos
        )
        self.__logger.info(f"Moved to position {pos}")
    
    # Belt set up function to place the piece on the belt before the game starts
    def setUpBelt(self, piece_count = 21, force = False):
        if not self.__beltSetUp or force:
            currentMoveCount = 0
            while piece_count:
                self.__logger.info(f"Currently setting up piece {self.__currentPiece}")
                piece_count -= 1
                
                # Determine which place to show to user
                index = self.__currentPiece % 2
                self.__logger.info(f"The piece will be placed on position {index}")
                self.__currentPiece += 1

                # Move the robot arm to that position and wait for user input
                self.__moveToPos(self.__first_piece_pos if index==0 else self.__second_piece_pos)
                self.__logger.info("Press enter to continue for the next piece...")
                input()
                currentMoveCount += 1

                # Move to home after piece is placed
                self.__moveToHome()

                # Move the belt since 2 pieces were placed
                if currentMoveCount == 2:
                    self.__logger.info("Piece stack full moving pieces to the left")
                    currentMoveCount = 0
                    self.__movePiecesOnBelt(ConveyorDirection.BACKWARD)
            
            self.__beltSetUp = True

    
    # Grab the piece at the specified index
    # TODO Change the behaviour of this function so that no outside parameter is required
    def grabPiece(self, piece_index):
        self.__moveToPos(self.__first_piece_pos if piece_index==0 else self.__second_piece_pos)
        self.__executeRobotAction(
            self.__robot.tool.grasp_with_tool
        )
        # TODO raise the piece

    # Move pieces in the belt
    def __movePiecesOnBelt(self, direction: ConveyorDirection):
        self.__executeRobotAction(
            self.__robot.conveyor.run_conveyor, self.__conveyor_id, 25, direction
        )
        time.sleep(2.5)
        self.__executeRobotAction(
            self.__robot.conveyor.stop_conveyor, self.__conveyor_id
        )

    # Function to end the control instance, must be called at the end
    def endRobot(self):
        self.__robot.end()


# Test function for robot
def __robotTest(ip):
    test_logger = CreateLogger("ROBOT_TEST")
    try:
        robot_ethernet = None
        try:
            robot_ethernet = Robot(ip)
        except Exception:
            test_logger.exception("Robot init failed!")
            sys.exit(1)
        
        robot_ethernet.setUpBelt()
    except KeyboardInterrupt:
        test_logger.info("Program ended with keyboard interrupt")
        sys.exit(130)
    finally:
        if robot_ethernet is not None:
            robot_ethernet.endRobot()

if __name__ == "__main__":
    import sys
    import argparse

    parser = argparse.ArgumentParser(description="Run tests for the robot")

    # Options ip argument for connecting to other robots, such as the simulation
    parser.add_argument(
        "--ip", 
        type=str,
        help="ip addr for robot, defaults to ethernet ip",
        default="169.254.200.200"
    )

    args = parser.parse_args()

    __robotTest(args.ip)