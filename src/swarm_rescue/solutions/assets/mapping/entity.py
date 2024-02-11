from enum import Enum
import numpy as np
import numba as nb
from typing import Tuple, List


class Entity(Enum):
    """
    The different entities that a drone can classify.
    Be careful not to change the order, it corresponds to the priority of the entity. For example, if a drone thinks he sees a DRONE on a KILL tile, he will not replace the tile by
    a DRONE entity, because it's of lower priority.
    """
    CLOUD = 0  # an unexplored tile, don't change this value since entity map is initialised with np.zeros
    VOID = 1  # an empty tile (explored)
    WALL = 2
    NOCOM = 3
    NOGPS = 4
    BASE = 5  # an access to the base (= rescue center)
    KILL = 6
    SAFE = 7


priority_table = {Entity.CLOUD.value: 0, Entity.VOID.value: 1, Entity.WALL.value: 1, Entity.NOCOM.value: 2, Entity.NOGPS.value: 3, Entity.BASE.value: 4, Entity.KILL.value: 5,
                  Entity.SAFE.value: 5}
priority_table = np.array(list(priority_table.values()))
"""

"""


# Important TODO: change the priority system so that some can be changed both ways (like drones and void, or SAFE and NOGPS)
# hashmap ?

@nb.njit
def compare_entity(entity: np.uint, other_entity: np.uint):  # returns entity >= other_entity
    return priority_table[entity] >= priority_table[other_entity]

@nb.njit
def max_entity(entity: np.uint, other_entity: np.uint):
    if compare_entity(entity, other_entity):
        return entity
    else:
        return other_entity


@nb.njit(parallel=True)
def add_entity(x: np.uint, y: np.uint, entity: int, entity_map: np.ndarray, tile_map_size: Tuple[np.int32, np.int32], radius=0):
    height, width = tile_map_size

    if radius == 0:
        entity_map[x, y] = max_entity(entity, entity_map[x, y])
    else:
        for i in nb.prange(max(0, x - radius), min(height, x + radius + 1)):
            for j in nb.prange(max(0, y - radius), min(width, y + radius + 1)):
                if (x - i) ** 2 + (y - j) ** 2 <= radius ** 2:
                    entity_map[i, j] = max_entity(entity, entity_map[i, j])


