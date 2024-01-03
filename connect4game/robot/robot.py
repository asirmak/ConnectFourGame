import threading
import time
from contextlib import contextmanager
from random import randrange

from enums import GripperAction
from pyniryo import NiryoRobot as OldAPI
from pyniryo2 import ConveyorDirection, NiryoRobot, PoseObject
from roslibpy.core import RosTimeoutError

from connect4game.utils.logging import create_logger


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
        x=0.107, y=-0.248, z=0.21,
        roll=-2.947, pitch=1.242, yaw=-2.934
    )

    # 15.9 - 16.0 cm from first speaker hole
    # __mag_pos = PoseObject(
    #     x=0.091, y=-0.254, z=0.161,
    #     roll=3.125, pitch=1.186, yaw=3.1
    # )

    __mag_pos = (
        -1.176, -0.4, -0.475,
        -0.411, -0.933, -0.865
    )

    # Location of the piece at index 0 on the belt (robot side)
    __index0_drop_pos = PoseObject(
        x=0.125, y=0.0, z=0.152,
        roll=0.0, pitch=1.55, yaw=0
    )
    __index0_pick_pos = PoseObject(
        x=0.136, y=0.0, z=0.152,
        roll=0.0, pitch=1.55, yaw=0.0
    )

    # Location of the piece at index 1 on the belt (far side)
    __index1_drop_pos = PoseObject(
        x=0.186, y=0.0, z=0.15,
        roll=0.0, pitch=1.55, yaw=0
    )
    __index1_pick_pos = PoseObject(
        x=0.196, y=0.0, z=0.15,
        roll=0.0, pitch=1.55, yaw=0.0
    )

    # Board complex will be placed 6 cm from the robot (keep in mind the error rate on the 30 cm)
    # Pre pos for the first row, move -0.05 on x for the next rows
    # For lining up the piece move -0.05 on z axis
    __board_ref = PoseObject(
            x=0.184, y=0.259, z=0.258,
            roll=0, pitch=0, yaw=1.65
    )
    __board_move_rel = -0.05

    def __init__(self, robot_ip = "169.254.200.200"): # if ip addr is argument not provided then use the ethernet port
        # Connect to robot
        try:
            self.__robot = NiryoRobot(robot_ip)
            self.__old_api = OldAPI(robot_ip)
        except:
            self.__logger.critical("Robot connection failed, check if the ip addr is correct")
            raise

        try:
            # Calibrate the robot
            self.__arm = self.__robot.arm
            self.__execute_robot_action(
                self.__arm.calibrate_auto
            )
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

            # Move robot to its default position
            self.__move_to_home()
        except:
            # If any error happens during the calibration, end the robot so the code does not hang
            self.end_robot()
            raise

        # A lock to use the belt
        self.__belt_lock = threading.Lock()

        # Total number of pieces currently on the belt
        self.__current_piece_count = 0

        # Piece of left on the current stack can be (0, 1, 2)
        self.__current_stack_count = 0

        self.__board_calibrated = False

        self.__magazine_ready = False

    # Returns the current total left pieces on the belt
    @property
    def get_piece_count(self):
        return self.__current_piece_count

    # Returns the current count of piece left on the stack
    @property
    def get_stack_count(self):
        return self.__current_stack_count

    @property
    def get_board_status(self):
        return self.__board_calibrated

    @property
    def get_magazine_status(self):
        return self.__magazine_ready

    # Belt set up function to place the piece on the belt before the game starts
    def set_up_game(self, piece_count=21):
        # Set up magazine if not already set up
        if not self.__magazine_ready:
            self.__move_to_pos(self.__mag_pos_bef)

            with self.__slow_arm_control():
                self.__move_joints(self.__mag_pos)

                self.__logger.info("Magazine should be placed to the shown position by the robot")
                self.__logger.info("Press enter to continue after the magazine is ready")
                input()

                self.__move_to_pos(self.__mag_pos_bef)

            self.__move_to_home()

            self.__magazine_ready = True

        while self.__current_piece_count != piece_count:
            self.__logger.info(f"Currently setting up piece {self.__current_piece_count}")

            # Determine which place to show to the user
            self.__logger.info(f"This piece will be placed on position {self.__current_stack_count + 1}")
            self.__current_piece_count += 1

            # Move to magazine to grab a piece
            self.__move_to_pos(self.__mag_pos_bef)

            # Decrease arm speed for precise actions
            with self.__slow_arm_control():
                self.__move_joints(self.__mag_pos)

                self.__control_gripper(GripperAction.CLOSE)

                self.__move_to_pos(self.__mag_pos_bef)

            # Wait because garbage api decides to execute the next action without completing the first one
            time.sleep(2)

            self.__move_to_home()

            # Acquire belt control to place the piece
            with self.__belt_lock:
                self.__logger.debug(f"Belt locked by thread {threading.get_ident()}")

                self.__move_to_pos(self.__index0_drop_pos if self.__current_stack_count==0 else self.__index1_drop_pos)
                self.__current_stack_count += 1

                self.__control_gripper(GripperAction.OPEN)

                # Move to home after piece is placed
                self.__move_to_home()

                self.__logger.debug(f"Belt lock removed by thread {threading.get_ident()}")

            if not self.__board_calibrated:
                self.__calibrate_board()

            # Move the belt since 2 pieces were placed
            if self.__current_stack_count == 2 and (self.__current_piece_count != piece_count):
                self.__logger.info("Piece stack full moving pieces to the left")

                self.__current_stack_count = 0
                belt_action = threading.Thread(target=self.__move_pieces_on_belt, args=(ConveyorDirection.BACKWARD,))
                belt_action.start()

    # Grab the next piece, which piece to grab is calculated by itself
    def grab_piece(self):
        self.__logger.info("Grabing the next piece")

        # If there are no pieces left on the belt don't do anything
        if self.__current_piece_count == 0:
            return

        # Make sure belt does not move while taking the pieces
        with self.__belt_lock:
            self.__logger.debug(f"Belt locked by thread {threading.get_ident()}")

            self.__move_to_pos(self.__index0_pick_pos if self.__current_stack_count==1 else self.__index1_pick_pos)
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

    # Drop the piece to the specified lane starting from 0
    def drop_piece_to_board(self, index):
        if index > 6 or index < 0:
            self.__logger.error(f"Index {index} is an invalid position for the game board. Use a value between 0-6")
            raise IndexError("Use a value between 0-6")

        self.__logger.info(f"Trying to place the piece on game board row {index}")

        current_rel = self.__board_move_rel * index
        if current_rel != 0:
            pos_to_move = PoseObject(
                x=self.__board_ref.x + current_rel, y=self.__board_ref.y, z=self.__board_ref.z,
                roll=self.__board_ref.roll, pitch=self.__board_ref.pitch, yaw=self.__board_ref.yaw
            )
        else:
            pos_to_move = self.__board_ref

        self.__move_to_pos(pos_to_move)
        self.__move_relative_linear([0, 0, self.__board_move_rel, 0, 0, 0])

        self.__control_gripper(GripperAction.OPEN)

        self.__move_relative_linear([0, 0, -self.__board_move_rel, 0, 0, 0])
        self.__move_to_home()

    # Function to end the control instance, must be called at the end
    def end_robot(self):
        self.__logger.info("Closing all robot connections")

        self.__robot.end()
        self.__old_api.close_connection()

    # Move pieces on the belt
    def __move_pieces_on_belt(self, direction: ConveyorDirection):
        with self.__belt_lock:
            move_time = 4.3
            if direction == ConveyorDirection.FORWARD:
                move_time += 0.1

            self.__logger.debug(f"Belt locked by thread {threading.get_ident()}")
            self.__logger.info(
                "Conveyor belt is currently moving in the "
                f"{'backward' if direction == ConveyorDirection.BACKWARD else 'forward'} direction"
            )

            self.__execute_robot_action(
                self.__conveyor.run_conveyor, self.__conveyor_id, 15, direction
            )
            time.sleep(move_time)
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
        self.__move_to_pos(self.__better_home_pos)
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
        if len(relative_arr) != 6:
            raise TypeError

        self.__execute_robot_action(
            self.__arm.move_linear_relative, relative_arr
        )
        self.__logger.debug(
            "Moved relative to current position by "
            f"x->{relative_arr[0]} y->{relative_arr[1]} z->{relative_arr[2]} "
            f"roll->{relative_arr[3]} pitch->{relative_arr[4]} yaw->{relative_arr[5]}"
        )

    def __move_joints(self, joints):
        self.__execute_robot_action(
            self.__arm.move_joints, joints
        )
        self.__logger.debug(
            f"Moved joints to 1={joints[0]}, 2={joints[1]}, 3={joints[2]}, "
            f"4={joints[3]}, 5={joints[4]}, 6={joints[5]}"
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

            # Adjust the return result
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
        # Disregard the result
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

    # This function calibrates the place of the game board
    def __calibrate_board(self):
        # Grab the first piece for calibration
        self.grab_piece()

        # Calibrate board positions
        self.__move_to_pos(self.__board_ref)
        for row in range(7):
            self.__logger.info(f"Currently calibrating row {row}")

            if row != 0:
                self.__move_relative_linear([self.__board_move_rel, 0, 0, 0, 0, 0])

            with self.__slow_arm_control():
                self.__move_relative_linear([0, 0, self.__board_move_rel, 0, 0, 0])

                # Wait for user confirmation
                self.__logger.info("Waiting for you to adjust the game board, press enter to continue...")
                input()

                self.__move_relative_linear([0, 0, -self.__board_move_rel, 0, 0, 0])

        self.__move_to_home()

        self.__current_stack_count += 1
        self.__current_piece_count += 1

        with self.__belt_lock:
            self.__logger.debug("Belt locked to place back the piece")
            self.__move_to_pos(self.__index0_pick_pos if self.__current_stack_count==1 else self.__index1_pick_pos)
            self.__control_gripper(GripperAction.OPEN)

            self.__move_to_home()

        self.__board_calibrated = True

    @contextmanager
    def __slow_arm_control(self, slow_speed=30):
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