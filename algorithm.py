# The board is 6 height x 7 width


class ConnectFour:
    def __init__(self):
        self.board = [["." for _ in range(7)] for _ in range(6)]

    # Useful functions
    def display_board(self):
        for row in self.board:
            print(" ".join(row))

    # Win conditions
    def is_full(self):
        return all(cell != "." for row in self.board for cell in row)

    # Horizontal, vertical, and diagonal checks
    def check_win(self, player):
        piece = "X" if player == "RED" else "O"

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
            return -10
        elif self.check_win("YELLOW"):
            return 10
        return 0

    # Minimax function
    def minimax(self, depth, isMax):
        score = self.evaluate()

        # Depth 3 is instant
        # Depth 5 takes a second to respond
        if depth == 5:
            return score

        # If Yellow (AI) wins
        if score == 10:
            return score

        # If Red (Player) wins
        if score == -10:
            return score

        # If board is full, return 0
        if self.is_full():
            return 0

        if isMax:
            best = -1000
            for col in range(7):
                if self.can_place_coin(col):
                    self.place_coin(col, "YELLOW")
                    best = max(best, self.minimax(depth + 1, not isMax))
                    # Undo move
                    self.remove_coin(col)
            return best

        else:
            best = 1000
            for col in range(7):
                if self.can_place_coin(col):
                    self.place_coin(col, "RED")
                    best = min(best, self.minimax(depth + 1, not isMax))
                    # Undo move
                    self.remove_coin(col)
            return best

    def remove_coin(self, col):
        row = 0
        while row < 6 and self.board[row][col] == ".":
            row += 1
        if row < 6:
            self.board[row][col] = "."

    # Improved AI Play
    def ai_play(self):
        best_val = -1000
        best_col = 4

        for col in range(7):
            if self.can_place_coin(col):
                self.place_coin(col, "YELLOW")
                move_val = self.minimax(0, False)
                # Undo move
                self.remove_coin(col)

                if move_val > best_val:
                    best_col = col
                    best_val = move_val

        self.place_coin(best_col, "YELLOW")
        # Return a value in the future for Robot

        # Placing a coin and error handling

    def can_place_coin(self, col):
        bottom = 5
        while bottom >= 0 and self.board[bottom][col] != ".":
            bottom -= 1
        if bottom < 0:
            return False
        return True

    def place_coin(self, col, color):
        if color not in ["RED", "YELLOW"]:
            raise ValueError("Color is limited to Red & Yellow")
        if col < 0 or col > 6:
            raise ValueError("Column must be between 0 & 6")
        bottom = 5

        while self.board[bottom][col] != ".":
            bottom -= 1

        if color == "RED":
            self.board[bottom][col] = "X"
        else:
            self.board[bottom][col] = "O"


if __name__ == "__main__":
    connectFour = ConnectFour()
    turns = 0

    while True:
        connectFour.display_board()
        if connectFour.check_win("RED") == True:
            print("Player wins !")
            break
        if connectFour.check_win("YELLOW") == True:
            print("AI wins !")
            break
        if connectFour.is_full() == True:
            print("It is a tie !")
            break
        # We decide the player starts playing first
        if turns % 2 == 0:
            player_choice = int(
                input("It is your turn ! Please choose a row to place your coin in.\n")
            )
            connectFour.place_coin(player_choice, "RED")
        else:
            print("The AI is thinking...")
            connectFour.ai_play()
        turns += 1
