from enum import Enum

class FlightMode(Enum):
    MANUAL = 0
    ACRO = 1
    STEERING = 3
    HOLD = 4
    LOITER = 5
    FOLLOW = 6
    AUTO = 10
    RTL = 11
    SMART_RTL = 12
    GUIDED = 15