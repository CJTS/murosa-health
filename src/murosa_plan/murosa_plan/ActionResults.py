from enum import Enum

class ActionResult(Enum):
    SUCCESS = 0
    FAILURE = 1
    WAITING = 2
    BATTERY_FAILURE = 3
