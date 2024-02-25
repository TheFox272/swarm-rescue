import numba as nb
import numpy as np
import math as m
from typing import List, Tuple

from scipy.spatial.distance import cdist

from swarm_rescue.solutions.assets.mapping.entity import Entity
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE
from swarm_rescue.solutions.assets.movement.pathfinding import DRONE_RUNOFF

# region local constants
STUCK_SPEED = 0.6
STUCK_WAIT = 15
PATH_TILE_TO_CHANGE = 2
MAX_PATH_FORECAST = 6

# endregion


@nb.njit
def compare_vectors(v1: np.ndarray, v2: np.ndarray):
    if v1[0] < v2[0]:
        return True
    elif v1[0] > v2[0]:
        return False
    else:
        if v1[1] < v2[1]:
            return True
        elif v1[1] > v2[1]:
            return False
    return True


def slow_down(pos, path, detected_drones):
    if len(path) == 0 or len(detected_drones) == 0:
        return False

    detected_drones_array = np.asarray(detected_drones, dtype=np.float32)
    detected_drones_array = detected_drones_array.reshape(-1, 2)
    forecast = min(len(path), MAX_PATH_FORECAST)
    path_array = np.asarray(path[:forecast], dtype=np.float32)
    path_array *= TILE_SIZE
    path_array = path_array.reshape(-1, 2)

    distances = cdist(path_array, detected_drones_array)
    index_min_distance = np.unravel_index(np.argmin(distances), distances.shape)
    distance_min = distances[index_min_distance]

    if distance_min <= DRONE_RUNOFF * TILE_SIZE and compare_vectors(pos, detected_drones[index_min_distance[1]]):
        # print("slower...")
        return True
    else:
        return False


