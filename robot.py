from pyniryo2 import *
from roslibpy.core import RosTimeoutError
from src.utils.Logging import CreateLogger
import time
from threading import Thread

class Robot:
    # Constant variables

    # Logger for the robot
    __logger = CreateLogger(
        name="ROBOT"
    )

    # Positions of certain objects

    # Better pos for robot when it should stay idle, got the pos from NiryoStudio
    __better_home_pos = PoseObject(
        x=0.14, y=0, z=0.203,
        roll=0, pitch=0.759, yaw=0
    )

    # Location of the piece at index 0 on the belt (robot side)
    __index0_pos = PoseObject(
        x=0.125, y=0.0, z=0.152,
        roll=0.0, pitch=1.55, yaw=0
    )

    # Location of the piece at index 1 on the belt (far side)
    __index1_pos = PoseObject(
        x=0.186, y=0.0, z=0.15,
        roll=0.0, pitch=1.55, yaw=0
    )

    # TODO Change this to actual game board positions later
    # Dummy position where the robot can drop the pieces by its side
    __drop_pos = PoseObject(
        x=-0.022, y=0.2, z=0.2,
        roll=0.013, pitch=0.023, yaw=1.653
    )

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
        
        # A thread which can manage belt actions async
        self.__beltThread = Thread()

        # Total number of pieces currently on the belt
        self.__currentPieceCount = 0

        # Piece of left on the current stack can be (0, 1, 2)
        self.__currentStackCount = 0

    # Returns the current total left pieces on the belt
    def getCurrentPieceCount():
        return self.__currentPieceCount
    
    # Returns the current count of piece left on the stack
    def getCurrentPieceStackCount():
        return self.__currentStackCount
    
    # Belt set up function to place the piece on the belt before the game starts
    def setUpBelt(self, piece_count=21, wait_time=0):
        while self.__currentPieceCount != piece_count:
            self.__logger.info(f"Currently setting up piece {self.__currentPieceCount}")
            
            # Determine which place to show to the user
            self.__logger.info(f"This piece will be placed on position {self.__currentStackCount + 1}")
            self.__currentPieceCount += 1

            # Move the robot arm to that position and wait for user input
            self.__moveToPos(self.__index0_pos if self.__currentStackCount==0 else self.__index1_pos)
            if wait_time == 0:
                self.__logger.info("Press enter to continue for the next piece...")
                input()
            else:
                self.__logger.info(f"Waiting {wait_time} seconds for piece to be placed")
                time.sleep(wait_time)
            self.__currentStackCount += 1

            # Move to home after piece is placed
            self.__moveToHome()

            # Move the belt since 2 pieces were placed
            if self.__currentStackCount == 2 and (self.__currentPieceCount != piece_count):
                self.__logger.info("Piece stack full moving pieces to the left")
                self.__currentStackCount = 0
                self.__movePiecesOnBelt(ConveyorDirection.BACKWARD)
    
    # Grab the next piece, which piece to grab is calculated by itself
    def grabPiece(self):
        # If there are no pieces left on the belt don't do anything
        if self.__currentPieceCount == 0:
            return
        
        # Wait if beltThread did not finish the task
        if self.__beltThread.is_alive():
            self.__beltThread.join()
        
        self.__moveToPos(self.__index0_pos if self.__currentStackCount==1 else self.__index1_pos)
        self.__executeRobotAction(
            self.__robot.tool.grasp_with_tool
        )
        self.__moveToHome()
        self.__currentStackCount -= 1
        
        # If stack is empty then move the new stones on the belt async
        if self.__currentStackCount == 0 and self.__currentPieceCount != 0:
            self.__currentStackCount = 2
            self.__beltThread = Thread(target=self.__movePiecesOnBelt, args=(ConveyorDirection.FORWARD,))
            self.__beltThread.start()

    # Take the piece and drop it next to the robot
    # TODO temp function remove this later
    def dropPiece(self):
        self.__moveToPos(self.__drop_pos)
        self.__executeRobotAction(
            self.__robot.tool.release_with_tool
        )
        self.__moveToHome()

    # Move pieces on the belt
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
        self.__moveToPos(self.__better_home_pos)
        self.__logger.info("Moved to home position")

    # Move robot to specified position
    def __moveToPos(self, pos: PoseObject):
        self.__executeRobotAction(
            self.__robot.arm.move_pose, pos
        )
        self.__logger.info(f"Moved to position {pos}")


# Test function for robot
def __robotTest(args):
    test_logger = CreateLogger("ROBOT_TEST")
    try:
        robot_ethernet = None
        try:
            robot_ethernet = Robot(args.ip)
        except Exception:
            test_logger.exception("Robot init failed!")
            sys.exit(1)
        
        original_piece = args.piece
        robot_ethernet.setUpBelt(args.piece)
        while original_piece:
            original_piece -= 1
            robot_ethernet.grabPiece()
            robot_ethernet.dropPiece()
        
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

    parser.add_argument(
        "--piece",
        type=int,
        help="game piece count",
        default=21
    )

    args = parser.parse_args()

    __robotTest(args)