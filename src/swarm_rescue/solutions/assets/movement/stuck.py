import numba as nb
import numpy as np
import math as m
from typing import List, Tuple

from swarm_rescue.solutions.assets.mapping.entity import Entity, add_entity
from swarm_rescue.solutions.assets.mapping.lidarMapping import THRESHOLD_MAX
from swarm_rescue.solutions.assets.mapping.semanticMapping import CLEAR_VALUE

# region local constants
STUCK_SPEED = 0.6
STUCK_WAIT = 14
PATH_TILE_TO_CHANGE = 1
STUCK_WALL_RADIUS = 2
STUCK_CLEAR_VALUE = THRESHOLD_MAX / 2


# endregion

def stuck_manager(timers, speed, entity_map, path, tile_map_size, occupancy_map):
    if np.linalg.norm(speed[:2]) < STUCK_SPEED:
        timers['stuck'] += 1
        if timers['stuck'] >= STUCK_WAIT and len(path) > PATH_TILE_TO_CHANGE:
            add_entity(path[PATH_TILE_TO_CHANGE][0], path[PATH_TILE_TO_CHANGE][1], Entity.WALL.value, entity_map, tile_map_size, STUCK_WALL_RADIUS)
            occupancy_map[path[PATH_TILE_TO_CHANGE][0], path[PATH_TILE_TO_CHANGE][1]] += STUCK_CLEAR_VALUE
            timers['stuck'] = STUCK_WAIT / 2
            # print(" I'm stuck !")

    else:
        timers['stuck'] = 0
