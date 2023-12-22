from pyniryo2 import *
from roslibpy.core import RosTimeoutError
from src.utils.Logging import create_logger
from src.robot.enums import GripperAction
import time
from threading import Thread

class Robot:
    # Constant variables

    # Logger for the robot
    __logger = create_logger(
        name="ROBOT"
    )

    # Positions of certain objects

    # Better pos for robot when it should stay idle, got the pos from NiryoStudio
    __better_home_pos = PoseObject(
        x=0.14, y=0, z=0.203,
        roll=0, pitch=0.759, yaw=0
    )

    __mag_pos_bef = PoseObject(
        x=0.099, y=-0.236, z=0.2,
        roll=-3.137, pitch=1.245, yaw=3.085
    )

    __mag_pos = PoseObject(
        x=0.093, y=-0.24, z=0.16,
        roll=3.02, pitch=1.212, yaw=2.993
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

    # Board positions where the robot can drop the pieces to the lanes
    # Starting from 0 most right lane for the robot
    __board_pos = [
        (
            PoseObject(
                x=0.168, y=0.314, z=0.223,
                roll=-0.06, pitch=-0.016, yaw=1.551
            ),
            PoseObject(
                x=0.164, y=0.316, z=0.208,
                roll=0.063, pitch=-0.006, yaw=1.546
            )
        ),
    ]

    def __init__(self, robot_ip = "169.254.200.200"): # if ip addr is argument not provided then use the ethernet port
        # Connect to robot
        try:
            self.__robot = NiryoRobot(robot_ip)
        except:
            self.__logger.critical("Robot connection failed, check if the ip addr is correct")
            raise
        
        try:
            # Calibrate the robot
            self.__execute_robot_action(
                self.__robot.arm.calibrate_auto
            )
            self.__logger.info("Arm is calibrated and ready to use")

            # Detect the currently attached tool
            self.__execute_robot_action(
                self.__robot.tool.update_tool
            )
            current_tool = self.__execute_robot_action(
                self.__robot.tool.get_current_tool_id
            )
            # Open the gripper
            self.__execute_robot_action(
                self.__robot.tool.release_with_tool
            )
            self.__logger.info("Gripper is ready to use")

            # Set up the conveyer belt
            self.__conveyor_id = self.__execute_robot_action(
                self.__robot.conveyor.set_conveyor
            )
            self.__logger.info("Conveyer belt ready to use")

            # Move robot to its default position
            self.__move_to_home()
        except:
            # If any error happens during the calibration, end the robot so the code does not hang
            self.end_robot()
            raise
        
        # A thread which can manage belt actions async
        self.__belt_thread = Thread()

        # Total number of pieces currently on the belt
        self.__current_piece_count = 0

        # Piece of left on the current stack can be (0, 1, 2)
        self.__current_stack_count = 0

        self.__board_calibrated = False

    # Returns the current total left pieces on the belt
    def get_piece_count(self):
        return self.__current_piece_count
    
    # Returns the current count of piece left on the stack
    def get_stack_count(self):
        return self.__current_stack_count
    
    # Belt set up function to place the piece on the belt before the game starts
    def set_up_belt(self, piece_count=21, wait_time=0):
        while self.__current_piece_count != piece_count:
            self.__logger.info(f"Currently setting up piece {self.__current_piece_count}")
            
            # Determine which place to show to the user
            self.__logger.info(f"This piece will be placed on position {self.__current_stack_count + 1}")
            self.__current_piece_count += 1

            # Move the robot arm to that position and wait for user input
            self.__move_to_pos(self.__mag_pos_bef)

            self.__execute_robot_action(
                self.__robot.arm.set_arm_max_velocity, 30
            )

            self.__move_to_pos(self.__mag_pos)
            
            self.__control_gripper(GripperAction.CLOSE)

            self.__move_to_pos(self.__mag_pos_bef)

            self.__execute_robot_action(
                self.__robot.arm.set_arm_max_velocity, 100
            )

            self.__move_to_home()

            if not self.__board_calibrated:
                self.__calibrate_board()

            if self.__belt_thread.is_alive():
                self.__belt_thread.join()

            self.__move_to_pos(self.__index0_pos if self.__current_stack_count==0 else self.__index1_pos)
            self.__current_stack_count += 1

            self.__control_gripper(GripperAction.OPEN)

            # Move to home after piece is placed
            self.__move_to_home()

            # Move the belt since 2 pieces were placed
            if self.__current_stack_count == 2 and (self.__current_piece_count != piece_count):
                self.__logger.info("Piece stack full moving pieces to the left")
                self.__current_stack_count = 0
                self.__move_pieces_on_belt(ConveyorDirection.BACKWARD)
    
    # Grab the next piece, which piece to grab is calculated by itself
    def grab_piece(self):
        # If there are no pieces left on the belt don't do anything
        if self.__current_piece_count == 0:
            return
        
        # Wait if beltThread did not finish the task
        if self.__belt_thread.is_alive():
            self.__belt_thread.join()
        
        self.__move_to_pos(self.__index0_pos if self.__current_stack_count==1 else self.__index1_pos)
        self.__control_gripper(GripperAction.CLOSE)
        self.__move_to_home()
        self.__current_stack_count -= 1
        self.__current_piece_count -= 1
        
        # If stack is empty then move the new stones on the belt async
        if self.__current_stack_count == 0 and self.__current_piece_count != 0:
            self.__current_stack_count = 2
            self.__belt_thread = Thread(target=self.__move_pieces_on_belt, args=(ConveyorDirection.FORWARD,))
            self.__belt_thread.start()

    # Drop the piece to the specified lane starting from 0
    def drop_piece_to_board(self, index):
        self.__move_to_pos(self.__board_pos[index][0])
        self.__move_to_pos(self.__board_pos[index][1])

        self.__control_gripper(GripperAction.OPEN)

        self.__move_to_pos(self.__board_pos[index][0])
        self.__move_to_home()

    # Function to end the control instance, must be called at the end
    def end_robot(self):
        self.__robot.end()

    # Move pieces on the belt
    def __move_pieces_on_belt(self, direction: ConveyorDirection):
        self.__execute_robot_action(
            self.__robot.conveyor.run_conveyor, self.__conveyor_id, 25, direction
        )
        time.sleep(2.5)
        self.__execute_robot_action(
            self.__robot.conveyor.stop_conveyor, self.__conveyor_id
        )

    # work around ros timing bug where the robot fails sometimes for no reason
    def __execute_robot_action(self, action, *args):
        action_retry = True
        result = None
        try:
            name = action.__name__
        except AttributeError:
            name = None
        
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
                three_dot = "..."
                self.__logger.warning(f"Robot internal timing bug, safe to ignore, retrying action {three_dot if name is None else name}")
                continue
        return result

    # Function for moving back to the home pose
    def __move_to_home(self):
        self.__move_to_pos(self.__better_home_pos)
        self.__logger.info("Moved to home position")

    # Move robot to specified position
    def __move_to_pos(self, pos: PoseObject):
        self.__execute_robot_action(
            self.__robot.arm.move_pose, pos
        )
        self.__logger.info(f"Moved to position x={pos.x}, y={pos.y}, z={pos.z}, roll={pos.roll}, pitch={pos.pitch}, yaw={pos.yaw}")

    def __control_gripper(self, action: GripperAction):
        tool_state = self.__robot.arm.hardware_status.value.hardware_errors[7]

        while tool_state != 0:
            self.__logger.warning("Gripper overheadted or run into an error, restarting gripper")
            self.__execute_robot_action(
                self.__robot.tool.update_tool
            )
            tool_state = self.__robot.arm.hardware_status.value.hardware_errors[7]
        
        match action:
            case GripperAction.OPEN:
                self.__execute_robot_action(
                    self.__robot.tool.release_with_tool
                )
            case GripperAction.CLOSE:
                self.__execute_robot_action(
                    self.__robot.tool.grasp_with_tool
                )
    
    def __calibrate_board(self):
        self.__move_to_pos(self.__board_pos[0][0])
        self.__move_to_pos(self.__board_pos[0][1])
        self.__logger.info("Waiting for adjust the game board, press enter to continue...")
        input()
        self.__move_to_pos(self.__board_pos[0][0])
        self.__move_to_home()
        self.__board_calibrated = True

# TODO Implement unittest later
# Test function for robot
def __robotTest(args):
    test_logger = create_logger("ROBOT_TEST")
    try:
        robot_ethernet = None
        try:
            robot_ethernet = Robot(args.ip)
        except Exception:
            test_logger.exception("Robot init failed!")
            sys.exit(1)
        
        original_piece = args.piece
        robot_ethernet.set_up_belt(piece_count=args.piece, wait_time=args.wait)
        while original_piece:
            original_piece -= 1
            robot_ethernet.grab_piece()
            robot_ethernet.drop_piece_to_board(0)

        # robot_ethernet.hardware_info()
        
    except KeyboardInterrupt:
        test_logger.info("Program ended with keyboard interrupt")
        sys.exit(130)
    finally:
        if robot_ethernet is not None:
            robot_ethernet.end_robot()

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

    parser.add_argument(
        "--wait",
        type=int,
        help="wait time for the pieces",
        default=0
    )

    args = parser.parse_args()

    __robotTest(args)