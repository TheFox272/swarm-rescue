from enum import Enum


class MsgType(Enum):
    """Message types"""
    # !! All MsgType names must be in uppercase (for good practice) !!
    ALIVE = 0
    SHARE_WAYPOINTS = 1
    SHARE_BASES = 2
    SHARE_OCCUPANCY_MAP = 3
    SHARE_ENTITY_MAP = 4
    SHARE_VICTIMS = 5
