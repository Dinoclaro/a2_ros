"""Mission state machine states."""

from enum import Enum, auto


class MissionState(Enum):
    """Ordered states for the autonomous survey mission."""

    CHECK_PREREQS = auto()
    STAND = auto()
    WAIT_STAND = auto()
    UNLOCK = auto()
    WALK = auto()
    RECORD_HOME = auto()
    SPAWN_EXPLORE = auto()
    EXPLORING = auto()
    KILL_EXPLORE = auto()
    SAVE_MAP = auto()
    SPAWN_NAV = auto()
    NAV_HOME = auto()
    KILL_NAV = auto()
    DONE = auto()
    FAILED = auto()
