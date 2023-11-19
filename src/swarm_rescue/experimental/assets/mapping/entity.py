from enum import Enum

class Entity(Enum):
    CLOUD = 0   # an unexplored tile
    VOID = 1   # an empty tile (explored)
    BASE = 2   # an access to the base (= rescue center)
    WALL = 3
    NOGPS = 4
    NOCOM = 5
    DRONE = 6
    KILL = 7