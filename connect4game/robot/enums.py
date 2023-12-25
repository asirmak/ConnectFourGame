from enum import Enum, unique

@unique
class GripperAction(Enum):
    OPEN = 0
    CLOSE = 1