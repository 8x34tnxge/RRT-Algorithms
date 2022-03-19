import enum


class Status(enum.Enum):
    Success = enum.auto()
    Failure = enum.auto()


class MapType(enum.IntEnum):
    EMPTY = 0
    ORIGIN = 1
    TARGET = 2
    WALL = 3


class AlgStatus(enum.Enum):
    Trapped = enum.auto()
    Advanced = enum.auto()
    Reached = enum.auto()
