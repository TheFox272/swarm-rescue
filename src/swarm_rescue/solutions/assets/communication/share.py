from typing import List
import numpy as np
import numba as nb

from solutions.assets.communication.comm_declarations import MsgType

from swarm_rescue.solutions.assets.behavior.think import VICTIM_RESCUED_NB, NOGPS_WAYPOINT
from swarm_rescue.solutions.assets.mapping.entity import max_entity
from swarm_rescue.solutions.assets.mapping.semanticMapping import VICTIM_DETECTION_MARGIN

# region local constants
OWN_OCCUPANCY_WEIGHT = 0.6
"""


:type: float
:domain: [0, 1]
"""
VICTIM_MIN_DIST = VICTIM_DETECTION_MARGIN


# endregion


def intersect_waypoints(waypoints: np.ndarray, other_waypoints: np.ndarray):
    waypoints[other_waypoints == 0] = 0
    # waypoints[(waypoints != 0) & (other_waypoints == NOGPS_WAYPOINT)] = NOGPS_WAYPOINT


def intersect_bases(bases: List, other_bases: List):
    coord_set_bases = {(x, y) for x, y, _ in bases}
    for x, y, b in other_bases:
        if (x, y) not in coord_set_bases:
            bases.append([x, y, b])
            coord_set_bases.add((x, y))
        else:
            index = next((i for i, [ex, ey, eb] in enumerate(bases) if ex == x and ey == y), None)
            if index is not None:
                if bases[index][2] is not b:
                    bases[index] = [x, y, False]

    print(bases, other_bases)


def intersect_occupancy(occupancy: np.ndarray, other_occupancy: np.ndarray):
    np.add(np.multiply(occupancy, OWN_OCCUPANCY_WEIGHT), np.multiply(other_occupancy, 1 - OWN_OCCUPANCY_WEIGHT), out=occupancy)


@nb.njit
def intersect_entity(entity_map: np.ndarray, other_entity_map: np.ndarray, tile_map_size):
    for i in np.arange(tile_map_size[0]):
        for j in np.arange(tile_map_size[1]):
            entity_map[i, j] = max_entity(entity_map[i, j], other_entity_map[i, j])


def intersect_victims(victims: List, other_victims: List, drone_id: np.int32):
    abandon_victim = False

    # We'll use arrays to be faster
    victims_array = np.array(victims, dtype=np.int32)
    other_victims_array = np.array(other_victims, dtype=np.int32)

    # Here we compute all the distances between the victims of both lists, to eventually find those who are the same
    distances = np.linalg.norm(other_victims_array[:, :2] - victims_array[:, :2].reshape(-1, 1, 2), axis=2)

    for i, (tile_x, tile_y, savior) in enumerate(victims_array):
        if savior == VICTIM_RESCUED_NB:
            continue

        close_indices = np.where(distances[i] < VICTIM_MIN_DIST)[0]

        if close_indices.size > 0:
            min_savior = other_victims_array[close_indices, 2].min()

            if min_savior == VICTIM_RESCUED_NB:
                victims_array[i, 2] = VICTIM_RESCUED_NB
                if savior == drone_id:
                    abandon_victim = True
                    # print(f"{drone_id} was told there is no victim here anymore")

    victims[:] = victims_array.tolist()

    remaining_indices = np.setdiff1d(np.arange(len(other_victims_array)), np.where(np.any(distances < VICTIM_MIN_DIST, axis=0))[0])
    victims.extend(other_victims_array[remaining_indices].tolist())

    return abandon_victim
