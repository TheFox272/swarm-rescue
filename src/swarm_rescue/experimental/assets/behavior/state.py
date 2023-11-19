from enum import Enum

class State(Enum):
    BOOT = 0   # the drone is establishing roles with its comrades
    EXPLORE = 1   # the drone is exploring the map (using waypoints)
    RESCUE = 2   # the drone saw a victim and is about to grab it
    SAVE = 3   # the drone is carrying a victim back to the (closest) rescue center
    DROP = 4    # the drone is touching a rescue center and is carrying a victim
    DONE = 5   # the drone is done exploring his zones
