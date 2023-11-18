from pyniryo2 import *
import time

class Robot:
    # Constant variables
    __first_piece_pos = [0.14, 0.0, 0.152, 0.0, 1.55, 0] # location of the piece at index 0 on the belt
    __second_piece_pos = [0.2, 0.0, 0.148, 0.0, 1.55, 0] # location of the piece at index 1 on the belt

    def __init__(self, robot_ip: str = "169.254.200.200"): # if ip addr is argument not provided then use the ethernet port
        # Connect to robot
        try:
            self.__robot = NiryoRobot(robot_ip)
        except:
            raise ConnectionError("Connection to robot failed!")
        
        # Calibrate the robot
        self.__executeRobotAction(
            self.__robot.arm.calibrate_auto
        )

        # Detect the currently attached tool
        self.__executeRobotAction(
            self.__robot.tool.update_tool
        )

        # Set up the conveyer belt
        self.__conveyor_id = self.__executeRobotAction(
            self.__robot.conveyor.set_conveyor
        )

        # Move robot to its default position
        self.__executeRobotAction(
            self.__robot.arm.move_to_home_pose()
        )

    # work around ros timing bug where the robot fails sometimes for no reason
    def __executeRobotAction(self, action, *args):
        action_retry = True
        result = None
        while action_retry:
            try:
                result = action(*args)
                action_retry = False
            except Exception as e:
                print(e)
                continue
        return result

    # Move robot to specified position
    def __moveToPos(self, pos):
        self.__executeRobotAction(
            self.__robot.arm.move_pose, pos
        )
    
    # Grab the piece at the specified index
    # TODO Change the behaviour of this function so that no outside parameter is required
    def grabPiece(self, piece_index):
        self.__moveToPos(self.__first_piece_pos if piece_index==0 else self.__second_piece_pos)

    # Move pieces in the belt
    # TODO Change this function to private, this function should be only managed internally
    def movePieces(self):
        self.__executeRobotAction(
            self.__robot.conveyor.run_conveyor, self.__conveyor_id, 25, ConveyorDirection.FORWARD
        )
        time.sleep(3)
        self.__executeRobotAction(
            self.__robot.conveyor.stop_conveyor, self.__conveyor_id
        )

    # Function to end the control instance, must be called at the end
    def endRobot(self):
        self.__robot.end()

if __name__ == "__main__":
    import sys
    import argparse

    parser = argparse.ArgumentParser(description="Run unit test of the robot")

    # Options ip argument for connecting to other robots, such as the simulation
    parser.add_argument(
        "--ip", type=str,
        help="ip addr for robot, defaults to ethernet ip",
        default="169.254.200.200"
    )

    args = parser.parse_args()

    try:
        robot_ethernet = Robot(args.ip)
    except ConnectionError as e:
        print(e)
        print("An error occured during the creation of the robot object!")
        print("Check if you have entered the correct ip addr")
        sys.exit(1)
    except Exception as e:
        print("Robot calibration functions failed!")
        print("Please make sure that the area around the robot is clear")
        print(e)
        sys.exit(1)

    robot_ethernet.grabPiece(1)
    #robot_ethernet.movePieces()

    robot_ethernet.endRobot()