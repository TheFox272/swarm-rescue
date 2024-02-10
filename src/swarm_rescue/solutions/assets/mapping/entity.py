from enum import Enum
import numpy as np
import numba as nb
from typing import Tuple


class Entity(Enum):
    """
    The different entities that a drone can classify.
    Be careful not to change the order, it corresponds to the priority of the entity. For example, if a drone thinks he sees a DRONE on a KILL tile, he will not replace the tile by
    a DRONE entity, because it's of lower priority.
    """
    CLOUD = 0  # an unexplored tile
    VOID = 1  # an empty tile (explored)
    WALL = 2
    BASE = 3  # an access to the base (= rescue center)
    DRONE = 4
    NOGPS = 5
    NOCOM = 6
    KILL = 7
    SAFE = 8


@nb.njit
def add_entity(x: np.uint, y: np.uint, entity: int, entity_map: np.array, tile_map_size: Tuple[np.int32, np.int32], radius=0):
    height, width = tile_map_size

    if radius == 0:
        # Cas où le rayon est égal à zéro, mise à jour seulement du point (x, y)
        entity_map[x, y] = max(entity, entity_map[x, y])
    else:
        # Cas où le rayon est supérieur à zéro, mise à jour dans la zone définie par le rayon
        radius_squared = radius * radius

        for i in range(max(0, x - radius), min(height, x + radius + 1)):
            for j in range(max(0, y - radius), min(width, y + radius + 1)):
                distance_squared = (x - i) ** 2 + (y - j) ** 2

                if distance_squared <= radius_squared:
                    entity_map[i, j] = max(entity, entity_map[i, j])

