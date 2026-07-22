from enum import Enum, auto

class MissionState(Enum):
    NO_MISSION = auto()
    NOT_STARTED = auto()
    ACTIVE = auto()
    PAUSED = auto()
    COMPLETED = auto()
    ABORTED = auto()