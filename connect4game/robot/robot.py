import threading
import time
from contextlib import contextmanager
from random import randrange

from .enums import GripperAction
from pyniryo import NiryoRobot as OldAPI
from pyniryo2 import ConveyorDirection, NiryoRobot, PoseObject
from roslibpy.core import RosTimeoutError

from connect4game.utils.logging import create_logger


class Robot:
    # In the PoseObjects bellow there are some constant position for certain situations
    # Better pos for robot when it should stay idle, got the pos from NiryoStudio
    __BETTER_HOME_POS = PoseObject(
        x=0.14, y=0, z=0.203,
        roll=0, pitch=0.759, yaw=0
    )

    # Position for the magazine before it reaches it out
    __MAG_PRE_POS = PoseObject(
        x=0.115, y=-0.232, z=0.258,
        roll=-2.776, pitch=1.077, yaw=-2.773
    )

    # Piece position on the belt
    # TODO Reconsider keeping the pick pos it might not be needed anymore
    # Location of the piece at index 0 on the belt (robot side)
    __INDEX0_DROP_POS = PoseObject(
        x=0.125, y=0.0, z=0.152,
        roll=0.0, pitch=1.55, yaw=0
    )

    # Location of the piece at index 1 on the belt (far side)
    __INDEX1_DROP_POS = PoseObject(
        x=0.186, y=0.0, z=0.15,
        roll=0.0, pitch=1.55, yaw=0
    )

    # Board ref position to set up the game board
    # This position is used only one time during the initial set up
    __BOARD_REF = PoseObject(
            x=0.070, y=0.200, z=0.345,
            roll=0, pitch=0, yaw=1.576
    )
    # Constants for moving linearly on the x and y axises of the gameboard rows
    __BOARD_MOVE_REL_X = -0.042
    __BOARD_MOVE_REL_Y = -0.050

    def __init__(self, robot_ip = "169.254.200.200", simulation=False): # if ip addr is argument not provided then use the ethernet port
        # Logger for the robot
        self.__logger = create_logger(name="ROBOT")

        if simulation:
            self.__logger.info("Currently running in simulation mode")

        # Connect to robot
        try:
            # Pyniryo2 api for controlling the robot
            self.__robot = NiryoRobot(robot_ip)

            # Old pyniryo api for rebooting the tool
            self.__old_api = OldAPI(robot_ip)
        except:
            self.__logger.critical("Robot api connection failed, check if the ip addr is correct")
            raise

        self.__logger.info("Robot arm calibration will begin in 5 seconds")
        self.__logger.info("Make sure the area around the robot is clear")
        time.sleep(5)

        try:
            # Calibrate the robot
            self.__arm = self.__robot.arm
            self.__execute_robot_action(
                self.__arm.calibrate_auto
            )
            # Move robot to its default position
            self.__move_to_home()
            self.__logger.info("Arm is calibrated and ready to use")

            # Detect the gripper
            self.__tool = self.__robot.tool
            self.__execute_robot_action(
                self.__tool.update_tool
            )
            # Open the gripper
            self.__control_gripper(GripperAction.OPEN)
            self.__logger.info("Gripper is ready to use")

            # Set up the conveyer belt
            self.__conveyor = self.__robot.conveyor
            self.__conveyor_id = self.__execute_robot_action(
                self.__conveyor.set_conveyor
            )
            self.__logger.info("Conveyer belt is ready to use")
        except:
            self.__logger.critical("Robot calibration failed!")

            # If any error happens during the calibration close the api connections
            self.end_robot()
            raise

        # A lock to use the belt
        self.__belt_lock = threading.Lock()

        # Total number of pieces currently on the belt
        self.__current_piece_count = 0

        # Piece of left on the current stack can be (0, 1, 2)
        self.__current_stack_count = 0

        # Stores the position of the magazine
        self.__mag_pos = None

        # Stores the postiion of the first gameboard row
        self.__board_first_pos = None

        self.__simulation = simulation

    # Returns the current total left pieces on the belt
    @property
    def get_piece_count(self):
        return self.__current_piece_count

    # Returns the current count of piece left on the stack
    @property
    def get_stack_count(self):
        return self.__current_stack_count

    @property
    def is_board_rdy(self):
        return self.__board_first_pos is not None

    @property
    def is_mag_rdy(self):
        return self.__mag_pos is not None

    # Sets up the game, should be called before a game starts
    # This function makes sure that all pieces are ready on the belt, magazine is ready and the game board is calibrated
    def set_up_game(self, piece_count=21, rdy_piece=0, rdy_stack=0):
        self.__current_piece_count = rdy_piece
        self.__current_stack_count = rdy_stack

        # Set up magazine if not already set up
        if not self.is_mag_rdy:
            self.__calibrate_mag()

        while self.__current_piece_count != piece_count:
            self.__logger.info(f"Currently setting up piece {self.__current_piece_count}")

            # Determine which place to show to the user
            self.__logger.info(f"This piece will be placed on position {self.__current_stack_count + 1}")
            self.__current_piece_count += 1

            # Move to magazine to grab a piece
            self.__move_to_pos(self.__MAG_PRE_POS)

            # Decrease arm speed for precise actions
            with self.__slow_arm_control():
                self.__move_to_pos(self.__mag_pos)

                self.__control_gripper(GripperAction.CLOSE)

                self.__move_to_pos(self.__MAG_PRE_POS)

            # Wait because garbage api decides to execute the next action without completing the first one
            time.sleep(2)

            self.__move_to_home()

            # With the first piece calibrate the gameboard
            if not self.is_board_rdy:
                self.__calibrate_board()

            # Acquire belt control to place the piece
            with self.__belt_lock:
                self.__logger.debug(f"Belt locked by thread {threading.get_ident()}")

                self.__move_to_pos(self.__INDEX0_DROP_POS if self.__current_stack_count==0 else self.__INDEX1_DROP_POS)
                self.__current_stack_count += 1

                self.__control_gripper(GripperAction.OPEN)

                # Move to home after piece is placed
                self.__move_to_home()

                self.__logger.debug(f"Belt lock removed by thread {threading.get_ident()}")

            # Move the belt if 2 piece stack is full
            if self.__current_stack_count == 2 and (self.__current_piece_count != piece_count):
                self.__logger.info("Piece stack full moving pieces to the left")

                self.__current_stack_count = 0
                belt_action = threading.Thread(target=self.__move_pieces_on_belt, args=(ConveyorDirection.BACKWARD,))
                belt_action.start()

    # Grab the next piece, which piece to grab is calculated automatically
    def grab_piece(self):
        self.__logger.info("Grabing the next piece")

        # If there are no pieces left on the belt don't do anything
        if self.__current_piece_count == 0:
            self.__logger.warning("There are no pieces on the belt!")
            return

        # Make sure belt does not move while taking the pieces
        with self.__belt_lock:
            self.__logger.debug(f"Belt locked by thread {threading.get_ident()}")

            self.__move_to_pos(self.__INDEX0_DROP_POS if self.__current_stack_count==1 else self.__INDEX1_DROP_POS)
            self.__control_gripper(GripperAction.CLOSE)
            self.__move_to_home()
            self.__current_stack_count -= 1
            self.__current_piece_count -= 1

            self.__logger.debug(f"Belt lock removed by thread {threading.get_ident()}")

        # If stack is empty then move the new stones on the belt async
        if self.__current_stack_count == 0 and self.__current_piece_count != 0:
            self.__current_stack_count = 2
            belt_thread = threading.Thread(target=self.__move_pieces_on_belt, args=(ConveyorDirection.FORWARD,))
            belt_thread.start()

        self.__logger.info("Piece grabing action finished")
        self.__logger.info(
            "Currently there"
            f"{' is 1 piece' if self.__current_stack_count == 1 else f' are {self.__current_stack_count} pieces'} remaining on the stack"
            f" and there {'is 1 piece' if self.__current_piece_count == 1 else f' are {self.__current_piece_count} pieces'} remaining in total"
        )

    # Drop the piece to the specified lane starting from 0 upto 6
    def drop_piece_to_board(self, index):
        self.grab_piece()

        if index > 6 or index < 0:
            self.__logger.error(f"Index {index} is an invalid position for the game board. Use a value between 0-6")
            raise IndexError("Use a value between 0-6")

        self.__logger.info(f"Trying to place the piece on game board row {index}")

        # Distance to the row in x axis from the first row
        current_rel = self.__BOARD_MOVE_REL_X * index
        row_pos = PoseObject(
            x=self.__board_first_pos.x + current_rel, y=self.__board_first_pos.y, z=self.__board_first_pos.z,
            roll=self.__board_first_pos.roll, pitch=self.__board_first_pos.pitch, yaw=self.__board_first_pos.yaw
        )

        # Move towards to the row
        self.__move_to_pos(row_pos)

        time.sleep(1)

        # Get down to the row
        self.__move_relative_linear([0, 0, self.__BOARD_MOVE_REL_Y, 0, 0, 0])

        # Drop the piece to the row
        self.__control_gripper(GripperAction.OPEN)

        # Get up from the row
        self.__move_relative_linear([0, 0, -self.__BOARD_MOVE_REL_Y, 0, 0, 0])

        # Move Joint1 to avoid collision with the game board
        current_joints = self.__execute_robot_action(
            self.__arm.get_joints
        )

        current_joints[0] = 0.0

        self.__move_joints(current_joints)

        self.__move_to_home()

    # Function to end the control instance, must be called at the end
    def end_robot(self):
        self.__logger.info("Closing all robot api connections")

        self.__robot.end()
        self.__old_api.close_connection()

    # Move pieces on the belt
    def __move_pieces_on_belt(self, direction: ConveyorDirection):
        with self.__belt_lock:
            self.__logger.debug(f"Belt locked by thread {threading.get_ident()}")
            self.__logger.info(
                "Conveyor belt is currently moving in the "
                f"{'backward' if direction == ConveyorDirection.BACKWARD else 'forward'} direction"
            )

            self.__execute_robot_action(
                self.__conveyor.run_conveyor, self.__conveyor_id, 15, direction
            )
            time.sleep(4.3)
            self.__execute_robot_action(
                self.__conveyor.stop_conveyor, self.__conveyor_id
            )

            self.__logger.debug(f"Belt lock removed by thread {threading.get_ident()}")

    # work around ros timing bug where the robot fails sometimes for no reason
    def __execute_robot_action(self, action, *args):
        result = None

        # If this is a function get its name
        # If this is not a function (property) ignore the name since it will throw an exception
        try:
            name = action.__name__
        except AttributeError:
            name = None

        while True:
            try:
                result = action(*args)
            except TypeError:
                self.__logger.critical("You did a coding error, do not pass function call instead pass a function reference")
                raise
            except RosTimeoutError:
                # robot internal bug safe to ignore
                # TODO in the future maybe add a retry limit
                self.__logger.warning(f"Robot internal timing bug, safe to ignore, retrying action {'...' if name is None else name}")
                continue
            break
        return result

    # Function for moving back to the home pose
    def __move_to_home(self):
        self.__move_to_pos(self.__BETTER_HOME_POS)
        self.__logger.info("Moved to home position")

    # Move robot to specified position
    def __move_to_pos(self, pos: PoseObject):
        self.__execute_robot_action(
            self.__arm.move_pose, pos
        )
        
        self.__logger.debug(
            f"Moved to position x={pos.x} y={pos.y} z={pos.z} "
            f"roll={pos.roll} pitch={pos.pitch} yaw={pos.yaw}"
        )

    def __move_relative_linear(self, relative_arr: list):
        self.__execute_robot_action(
            self.__arm.move_linear_relative, relative_arr
        )

        self.__logger.debug(
            "Moved relative to current position by "
            f"x->{relative_arr[0]} y->{relative_arr[1]} z->{relative_arr[2]} "
            f"roll->{relative_arr[3]} pitch->{relative_arr[4]} yaw->{relative_arr[5]}"
        )

    def __move_joints(self, joint_list: list):
        self.__execute_robot_action(
            self.__arm.move_joints, joint_list
        )

        self.__logger.debug(
            "Moved joints to "
            f"Joint1->{joint_list[0]} Joint2->{joint_list[1]} Joint3->{joint_list[2]} "
            f"Joint4->{joint_list[3]} Joint5->{joint_list[4]} Joint6->{joint_list[5]}"
        )

    # Function for restarting gripper in case of hardware error
    # Works well for overheating issue, still can cause unexpected behaviour
    # with the robot, looks stupid but solves the issue
    def __check_gripper_errors(self) -> bool:
        result = False
        try:
            # Index for gripper error value
            tool_state = self.__arm.hardware_status.value.hardware_errors[7]
        except IndexError:
            # While using the simulation this value might not be present
            # If not present give a warning and skip checking for gripper errors
            self.__logger.warning("Gripper error status not readable, skip checking for gripper error")
            return False

        # If tool has an error, reboot the tool until error state clears
        # I know this looks stupid but this is how it works in NiryoStudio
        while tool_state != 0:
            self.__logger.warning("Gripper overheated or run into an error, restarting gripper")

            # Declare there was an error with the gripper
            result = True

            # Reboot the tool using the old api
            # This function is not present in the new one
            self.__execute_robot_action(
                self.__old_api.tool_reboot
            )

            # Update the tool just in case
            self.__execute_robot_action(
                self.__tool.update_tool
            )

            # Wait for hardware state to update
            time.sleep(5)

            try:
                tool_state = self.__arm.hardware_status.value.hardware_errors[7]
            except IndexError:
                self.__logger.warning("Gripper error status not readable after tool reboot")
                return True

        return result

    def __control_gripper(self, action: GripperAction):
        # Check for gripper errors before doing anything
        # Disregard the result during the first check since the function will fix it already
        self.__check_gripper_errors()

        while True:
            if action == GripperAction.CLOSE:
                self.__execute_robot_action(
                        self.__tool.grasp_with_tool
                )

                self.__logger.debug("Gripper close action done")
            else:
                self.__execute_robot_action(
                        self.__tool.release_with_tool
                )

                self.__logger.debug("Gripper open action done")

            # Gripper could fail during the execution of the action resulting in weird behaviour with the robot
            # That is why this check is present here
            error_status = self.__check_gripper_errors()
            # TODO Block the execution properly
            if error_status:
                self.__logger.warning("Error state after gripper command execution")
                self.__logger.warning("Do you want to retry the action? y/N")
                answ = input()
                answ = answ.upper()
                if answ.startswith("Y"):
                    continue
            break

    def __calibrate_mag(self):
        self.__move_to_pos(self.__MAG_PRE_POS)

        if self.__simulation:
            self.__logger.info("Mag position is constant while running in simulation")

            self.__mag_pos = PoseObject(
                x=0.05, y=-0.232, z=0.15,
                roll=-2.776, pitch=1.077, yaw=-2.773
            )
            with self.__slow_arm_control():
                self.__move_to_pos(self.__mag_pos)
                self.__move_to_pos(self.__MAG_PRE_POS)
        else:
            with self.__slow_arm_control():
                self.__logger.info("Move robot arm using freemove to the magazine")
                self.__logger.info("Press enter to continue after the magazine is ready")
                input()

                # Save the shown magazine position
                self.__mag_pos = self.__execute_robot_action(
                    self.__arm.get_pose
                )

                self.__logger.info("Saved current magazine position")

                self.__move_to_pos(self.__MAG_PRE_POS)

        self.__move_to_home()

    # This function calibrates the place of the game board
    def __calibrate_board(self):
        # Calibrate board positions
        self.__move_to_pos(self.__BOARD_REF)
        
        # Ask the user to move the arm of the robot to the first row of the gameboard
        self.__logger.info("Currently calibrating row 0")

        if self.__simulation:
            self.__logger.info("In simulation mode board position is constant")
            self.__move_relative_linear([2 * -self.__BOARD_MOVE_REL_X, 0, 0, 0, 0, 0])
        else:
            self.__logger.info("Move the arm to the position 0 of the gameboard by freemotion")
            self.__logger.info("Press enter to continue...")
            input()

            self.__move_relative_linear([0, 0, -self.__BOARD_MOVE_REL_Y, 0, 0, 0])

        self.__board_first_pos = self.__execute_robot_action(
            self.__arm.get_pose
        )

        self.__logger.info("Other row's positions will adjusted accordingly to the first row")

        for row in range(6):
            self.__logger.info(f"Currently calibrating row {(row+1)}")

            self.__move_relative_linear([self.__BOARD_MOVE_REL_X, 0, 0, 0, 0, 0])

            with self.__slow_arm_control():
                self.__move_relative_linear([0, 0, self.__BOARD_MOVE_REL_Y, 0, 0, 0])

                if not self.__simulation:
                    # Wait for user confirmation
                    self.__logger.info("Check if the row is aligned, press enter to continue...")
                    input()

                self.__move_relative_linear([0, 0, -self.__BOARD_MOVE_REL_Y, 0, 0, 0])

        # Move only Joint1 to avoid collision with the game board
        current_joints = self.__execute_robot_action(
            self.__arm.get_joints
        )

        current_joints[0] = 0.0

        self.__move_joints(current_joints)

        self.__move_to_home()

    @contextmanager
    def __slow_arm_control(self, slow_speed=30):
        self.__logger.info("Arm is moving slowly for precise action")
        
        slow_arm = self.__arm
        self.__execute_robot_action(
            slow_arm.set_arm_max_velocity, slow_speed
        )

        self.__logger.debug(f"Arm max speed changed to {slow_speed}%")

        try:
            yield slow_arm
        finally:
            self.__execute_robot_action(
                slow_arm.set_arm_max_velocity, 100
            )

            self.__logger.debug("Arm max speed changed back to 100%")

# TODO Implement unittest later
# Test function for robot
def __robotTest(args):
    test_logger = create_logger("ROBOT_TEST")

    try:
        robot_ethernet = Robot(args.ip)
    except:
        test_logger.exception("Robot init failed!")
        raise

    try:
        original_piece = args.piece
        robot_ethernet.set_up_game(piece_count=args.piece)

        row_info = []

        for i in range(7):
            current_row = {"val": i, "times": 0}
            row_info.append(current_row)

        while original_piece:
            robot_ethernet.grab_piece()

            chosen_index = randrange(len(row_info))
            chosen_row = row_info[chosen_index]["val"]

            row_info[chosen_index]["times"] = row_info[chosen_index]["times"] + 1

            if row_info[chosen_index]["times"] == 6:
                row_info.pop(chosen_index)

            original_piece -= 1

            robot_ethernet.drop_piece_to_board(chosen_row)
            # robot_ethernet.drop_piece_to_board(0)

    except KeyboardInterrupt:
        test_logger.info("Program ended with keyboard interrupt")
        sys.exit(130)
    finally:
        robot_ethernet.end_robot()

if __name__ == "__main__":
    import argparse
    import sys

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