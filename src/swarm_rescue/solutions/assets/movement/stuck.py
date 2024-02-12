import numba as nb
import numpy as np
import math as m
from typing import List, Tuple

from swarm_rescue.solutions.assets.mapping.entity import Entity

# region local constants
STUCK_SPEED = 0.6
STUCK_WAIT = 20
PATH_TILE_TO_CHANGE = 2


# endregion

def stuck_manager(timers, speed, entity_map, path, drone_id):
    if np.linalg.norm(speed[:2]) < STUCK_SPEED:
        timers['stuck'] += 1
        if timers['stuck'] >= STUCK_WAIT and len(path) > PATH_TILE_TO_CHANGE:
            entity_map[path[PATH_TILE_TO_CHANGE][0], path[PATH_TILE_TO_CHANGE][1]] = Entity.WALL.value
            timers['stuck'] = 0
            print(drone_id, " I'm stuck !")

    else:
        timers['stuck'] = 0
