# The board is 6 height x 7 width
from vision import Vision
from connect4game.robot import Robot
from connect4game.utils.logging import create_logger
import numpy as np
import time
import argparse


class ConnectFour:
    def __init__(self, rdy_piece=0, rdy_stack=0):
        self.__logger = create_logger("GAME")
        self.camera = Vision()
        self.robot = Robot()
        self.robot.set_up_game(piece_count=21, rdy_piece=rdy_piece, rdy_stack=rdy_stack)
        self.board = [[0 for _ in range(7)] for _ in range(6)]
        self.board = np.array(self.board)
        self.depth = 3
        self.turns = 0

    # Useful functions
    def display_board(self):
        self.__logger.info("Current game status:")
        self.__logger.info(f"Row 1: {self.board[0]}")
        self.__logger.info(f"Row 2: {self.board[1]}")
        self.__logger.info(f"Row 3: {self.board[2]}")
        self.__logger.info(f"Row 4: {self.board[3]}")
        self.__logger.info(f"Row 5: {self.board[4]}")
        self.__logger.info(f"Row 6: {self.board[5]}")

    # Win conditions
    def is_full(self):
        return all(cell != 0 for row in self.board for cell in row)

    # Horizontal, vertical, and diagonal checks
    def check_win(self, player):
        piece = 1 if player == "RED" else 2

        # Check horizontal locations
        for r in range(6):
            for c in range(4):  # Only check up to column 4, as max possible column is 6
                if (
                    self.board[r][c] == piece
                    and self.board[r][c + 1] == piece
                    and self.board[r][c + 2] == piece
                    and self.board[r][c + 3] == piece
                ):
                    return True

        # Check vertical locations
        for r in range(3):  # Only check up to row 3, as max possible row is 5
            for c in range(7):
                if (
                    self.board[r][c] == piece
                    and self.board[r + 1][c] == piece
                    and self.board[r + 2][c] == piece
                    and self.board[r + 3][c] == piece
                ):
                    return True

        # Check positively sloped diagonals
        for r in range(3):
            for c in range(4):
                if (
                    self.board[r][c] == piece
                    and self.board[r + 1][c + 1] == piece
                    and self.board[r + 2][c + 2] == piece
                    and self.board[r + 3][c + 3] == piece
                ):
                    return True

        # Check negatively sloped diagonals
        for r in range(3, 6):
            for c in range(4):
                if (
                    self.board[r][c] == piece
                    and self.board[r - 1][c + 1] == piece
                    and self.board[r - 2][c + 2] == piece
                    and self.board[r - 3][c + 3] == piece
                ):
                    return True

        return False

    def evaluate(self):
        if self.check_win("RED"):
            return 10000
        elif self.check_win("YELLOW"):
            return -10000
        return 0

    # Minimax function
    def minimax(self, depth, isMax):
        score = self.evaluate()

        # Depth 3 is instant
        # Depth 5 takes a second to respond
        if depth == self.depth:
            return score

        # If Yellow (AI) wins
        if score == 10000:
            return score

        # If Red (Player) wins
        if score == -10000:
            return score

        # If board is full, return 0
        if self.is_full():
            return 0

        if isMax:
            best = -1000
            for col in range(7):
                if self.can_place_coin(col):
                    self.place_coin(col, "RED")
                    best = max(best, self.minimax(depth + 1, not isMax))
                    # Undo move
                    self.remove_coin(col)
            return best

        else:
            best = 1000
            for col in range(7):
                if self.can_place_coin(col):
                    self.place_coin(col, "YELLOW")
                    best = min(best, self.minimax(depth + 1, not isMax))
                    # Undo move
                    self.remove_coin(col)
            return best

    def remove_coin(self, col):
        row = 0
        while row < 6 and self.board[row][col] == 0:
            row += 1
        if row < 6:
            self.board[row][col] = 0

    # Improved AI Play
    def ai_play(self):
        best_val = -1000
        best_col = 4

        for col in range(7):
            if self.can_place_coin(col):
                self.place_coin(col, "RED")
                move_val = self.minimax(0, False)
                # Undo move
                self.remove_coin(col)

                if move_val > best_val:
                    best_col = col
                    best_val = move_val

        # Best move has been calculated, we ask the robot to place the coin
        self.robot.drop_piece_to_board(best_col)
        self.place_coin(best_col, "RED")
        # Return a value in the future for Robot

        # Placing a coin and error handling

    def can_place_coin(self, col):
        if col == -1:
            return False
        for i in range(6):
            if self.board[i][col] == 0:
                return True
        return False

    def place_coin(self, col, color):
        if color not in ["RED", "YELLOW"]:
            raise ValueError("Color is limited to Red & Yellow")
        if col < 0 or col > 6:
            raise ValueError("Column must be between 0 & 6")
        bottom = 5

        while self.board[bottom][col] != 0:
            bottom -= 1

        if color == "RED":
            self.board[bottom][col] = 1
        else:
            self.board[bottom][col] = 2

    # Player choice
    def get_player_input(self, case):
        cases_dict = {"DIFFICULTY": 1, "COIN": 2}
        value = cases_dict[case]
        match value:
            case 1:
                self.__logger.info("1 Easy | 2 Medium | 3 Hard | 4 Impossible")
                player_choice = input()
                if player_choice == "":
                    return 2
                player_choice = int(player_choice)
                if player_choice < 1 or player_choice > 4:
                    return self.get_player_input("DIFFICULTY")
                return player_choice
            case 2:
                self.__logger.info("It is your turn ! Please choose a row to place your coin in.")
                player_choice = input()
                if (
                    player_choice == ""
                    or int(player_choice) < 1
                    or int(player_choice) > 7
                ):
                    return self.get_player_input("COIN")
                return int(player_choice) - 1
            case _:
                pass

    # Game difficulty
    def choose_difficulty(self):
        self.__logger.info("Choose your difficulty. No value will be medium by default.")
        player_choice = self.get_player_input("DIFFICULTY")
        self.depth = player_choice + 2

    # Game loop
    def play_game(self):
        for _ in range(42):
            connectFour.display_board()
            if connectFour.check_win("YELLOW"):
                self.__logger.info("Player wins!")
                break
            if connectFour.check_win("RED"):
                self.__logger.info("AI wins!")
                break
            if connectFour.is_full():
                self.__logger.info("It is a tie!")
                break
            # We decide the player starts playing first
            if self.turns % 2 == 0:
                # player_choice = -1
                # while connectFour.can_place_coin(player_choice) == False:
                #     player_choice = self.get_player_input("COIN")
                # connectFour.place_coin(player_choice, "YELLOW")
                self.__logger.info("Your turn!")
                while True:
                    time.sleep(8)
                    current_board = self.camera.detect_game()
                    if not np.array_equal(current_board, self.board):
                        self.board = current_board
                        break
            else:
                # Updating the board for the robot
                self.board = self.camera.detect_game()
                self.__logger.info("AI is thinking...")
                connectFour.ai_play()
            self.turns += 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--rdy_piece",
        type=int,
        default=0
    )

    parser.add_argument(
        "--rdy_stack",
        type=int,
        default=0
    )

    args = parser.parse_args()

    try:
        connectFour = ConnectFour(rdy_piece=args.rdy_piece, rdy_stack=args.rdy_stack)
        connectFour.choose_difficulty()
        connectFour.play_game()
    finally:
        connectFour.robot.end_robot()
