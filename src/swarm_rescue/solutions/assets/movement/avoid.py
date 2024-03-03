import numba as nb
import numpy as np
import math as m

from scipy.spatial.distance import cdist

from swarm_rescue.solutions.assets.behavior.anti_kill_zone import CLOSE_DRONE_DISTANCE
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE
from swarm_rescue.solutions.assets.movement.pathfinding import DRONE_RUNOFF

# region local constants
STUCK_SPEED = 0.6
STUCK_WAIT = 15
PATH_TILE_TO_CHANGE = 2
MAX_PATH_FORECAST = 5
DRONE_SLOW_DISTANCE = DRONE_RUNOFF
# endregion


@nb.njit
def compare_vectors(v1: np.ndarray, v2: np.ndarray):
    dx = v2[0] - v1[0]
    dy = v2[1] - v1[1]
    if dx < dy:
        return True
    elif dx > dy:
        return False
    else:
        if dx < -dy:
            return True
        elif dx > -dy:
            return False
    return True


def slow_down(pos, path, detected_drones, dead_drones):
    if len(path) == 0 or len(detected_drones) == 0:
        return False

    detected_drones_array = np.asarray(detected_drones, dtype=np.float32).reshape(-1, 2)
    dead_drones_array = np.asarray(dead_drones, dtype=np.float32).reshape(-1, 2)  # Correction ici
    forecast = min(len(path), MAX_PATH_FORECAST)
    path_array = np.asarray(path[:forecast], dtype=np.int32) * TILE_SIZE
    path_array = path_array.reshape(-1, 2)

    detected_drones_array = np.array([drone for drone in detected_drones_array if not any(m.dist(drone, dead) <= CLOSE_DRONE_DISTANCE for dead in dead_drones_array)])

    if len(detected_drones_array) == 0:
        return False
    distances = cdist(path_array, detected_drones_array)
    index_min_distance = np.unravel_index(np.argmin(distances), distances.shape)
    distance_min = distances[index_min_distance]

    if distance_min <= DRONE_SLOW_DISTANCE * TILE_SIZE and compare_vectors(pos, detected_drones[index_min_distance[1]]):
        # print("slower...")
        return True
    else:
        return False



