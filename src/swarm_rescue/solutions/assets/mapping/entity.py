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
    NOGPS = 3
    NOCOM = 4
    BASE = 5  # an access to the base (= rescue center)
    KILL = 6
    SAFE = 7


priority_table = {Entity.CLOUD.value: 0, Entity.VOID.value: 1, Entity.WALL.value: 1, Entity.NOCOM.value: 3, Entity.NOGPS.value: 4, Entity.BASE.value: 5, Entity.KILL.value: 2,
                  Entity.SAFE.value: 4}
priority_table = np.array(list(priority_table.values()))


@nb.njit
def compare_entity(entity: np.uint, other_entity: np.uint):
    if entity == other_entity:
        return True
    elif entity == Entity.CLOUD.value:
        return False
    elif other_entity == Entity.CLOUD.value:
        return True
    elif entity == Entity.VOID.value:
        if other_entity == Entity.WALL.value:
            return True
        elif other_entity == Entity.NOGPS.value:
            return False
        elif other_entity == Entity.NOCOM.value:
            return False
        elif other_entity == Entity.BASE.value:
            return False
        elif other_entity == Entity.KILL.value:
            return False
        elif other_entity == Entity.SAFE.value:
            return False
    elif entity == Entity.WALL.value:
        if other_entity == Entity.VOID.value:
            return True
        elif other_entity == Entity.NOGPS.value:
            return True
        elif other_entity == Entity.NOCOM.value:
            return True
        elif other_entity == Entity.BASE.value:
            return False
        elif other_entity == Entity.KILL.value:
            return True
        elif other_entity == Entity.SAFE.value:
            return True
    elif entity == Entity.NOGPS.value:
        if other_entity == Entity.VOID.value:
            return True
        elif other_entity == Entity.WALL.value:
            return False
        elif other_entity == Entity.NOCOM.value:
            return False
        elif other_entity == Entity.BASE.value:
            return False
        elif other_entity == Entity.KILL.value:
            return False
        elif other_entity == Entity.SAFE.value:
            return False
    elif entity == Entity.NOCOM.value:
        if other_entity == Entity.VOID.value:
            return True
        elif other_entity == Entity.WALL.value:
            return False
        elif other_entity == Entity.NOGPS.value:
            return True
        elif other_entity == Entity.BASE.value:
            return False
        elif other_entity == Entity.KILL.value:
            return True
        elif other_entity == Entity.SAFE.value:
            return False
    elif entity == Entity.BASE.value:
        return True
    elif entity == Entity.KILL.value:
        if other_entity == Entity.VOID.value:
            return True
        elif other_entity == Entity.WALL.value:
            return False
        elif other_entity == Entity.NOGPS.value:
            return True
        elif other_entity == Entity.NOCOM.value:
            return False
        elif other_entity == Entity.BASE.value:
            return False
        elif other_entity == Entity.SAFE.value:
            return False
    elif entity == Entity.SAFE.value:
        if other_entity == Entity.VOID.value:
            return True
        elif other_entity == Entity.WALL.value:
            return False
        elif other_entity == Entity.NOGPS.value:
            return True
        elif other_entity == Entity.NOCOM.value:
            return True
        elif other_entity == Entity.BASE.value:
            return False
        elif other_entity == Entity.KILL.value:
            return True

    # return priority_table[entity] >= priority_table[other_entity]


@nb.njit
def max_entity(entity: np.uint, other_entity: np.uint):
    if compare_entity(entity, other_entity):
        return entity
    else:
        return other_entity


@nb.njit
def bounded_variation(x, variation, x_sup, x_inf=0):
    """
    returns a range a values around x so that it stays bounded by x_sup and x_inf
    """
    return np.arange(max(int(x - variation), x_inf), min(int(x + variation), x_sup))


@nb.njit
def add_entity(x: np.uint, y: np.uint, entity: int, entity_map: np.ndarray, tile_map_size: Tuple[np.int32, np.int32], radius=0):
    if radius == 0:
        entity_map[x, y] = max_entity(entity, entity_map[x, y])
    else:
        for i in bounded_variation(x, radius, tile_map_size[0]):
            for j in bounded_variation(y, radius, tile_map_size[1]):
                if (x - i) ** 2 + (y - j) ** 2 <= radius ** 2:
                    entity_map[i, j] = max_entity(entity, entity_map[i, j])
