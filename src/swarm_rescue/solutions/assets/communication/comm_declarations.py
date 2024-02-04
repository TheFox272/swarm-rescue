from enum import Enum

#TODO: Experiment with different values for these constants
VICTIM_MIN_DIST = 4  # Minimum distance between 2 victims to be considered as different victims when sharing victims
                     # used in share.py in share_victims function

class MsgType(Enum):
    """Message types"""
    # !! All MsgType names must be in uppercase (for good practice) !!
    ALIVE = 0
    SHARE_WAYPOINTS = 1
    SHARE_BASES = 2
    SHARE_OCCUPANCY_MAP = 3
    SHARE_ENTITY_MAP = 4
    SHARE_VICTIMS = 5


    # Message types from older version
    # group_req = 0
    # group_acc = 1
    # group_ref = 2
    # pos_speed = 3
    # alive = 4
    # to_rescue = 5
    # waypoints = 6
    # beleader = 7
    # drop = 8
    # eom = 9   # end of mission
    # kill = 10
    # path = 11
    # nocom = 12
    # base = 13
    # safe = 14
    # rescue_center = 15
    # follow = 16
    # nogps = 17