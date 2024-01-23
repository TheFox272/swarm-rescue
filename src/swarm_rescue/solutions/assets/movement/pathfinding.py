from math import sqrt
import numpy as np
import math as m
import tcod.path
import sys, os
import numba as nb
from typing import List, Tuple

from swarm_rescue.solutions.assets.mapping.entity import Entity
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE
from swarm_rescue.solutions.assets.behavior.state import State


# region local constants
WALL_WEIGHT = 8
"""
Constant used in :py:func:`f_runoff`, corresponding to the additional weight that the drone will put on a wall in his 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`. In other words, how much the drone will avoid the surroundings of a wall.

:type: int
:domain: [0, inf]
"""
WALL_RUNOFF = 5
"""
Constant used in :py:func:`f_runoff`, corresponding to how far a wall will impact the weights of his surroundings tiles in the 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map` of the drone. In other words, the distance that the drone will try to take 
when moving alongside a wall.

:type: int
:domain: [1, inf]
"""
BASE_WEIGHT = 5
"""
Constant used in :py:func:`compute_path_map`, corresponding to the base weight of the tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.

:type: int
:domain: [:py:data:`CLOUD_BONUS` + 1, inf]
"""
CLOUD_BONUS = 2
"""
Constant used in :py:func:`compute_path_map`, corresponding to the bonus weight of the cloud tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map` when the drone is exploring. This allows for more exploration from the drone, 
instead of always taking the same paths.

:type: int
:domain: [0, :py:data:`BASE_WEIGHT` - 1]
"""
PRUDENCE = 10
"""
Constant used in :py:func:`compute_path_map`, corresponding to the malus weight of the cloud tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map` when the drone is carrying a victim. It avoids the drone from going into a 
kill zone with a victim.

:type: int
:domain: [0, inf]
"""
DRONE_RELUCTANCE = 2
"""
Constant used in :py:func:`compute_path_map`, corresponding to the malus weight of the other drone's tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.

:type: int
:domain: [0, inf]
"""
KILL_RELUCTANCE = 5
"""
Constant used in :py:func:`compute_path_map`, corresponding to the malus weight of the kill zone tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.

:type: int
:domain: [0, inf]
"""
VICTIM_ZONE = 3
# endregion

f_runoff = nb.njit(lambda distance, weight: np.int8(round(BASE_WEIGHT + weight * (1 - (distance-1) / WALL_RUNOFF), 0)))
"""Used to compute the impact of a wall on :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`. Used in 
:py:func:`compute_path_map`.
"""

@nb.njit
def compute_path_map(tile_map_size: Tuple[np.int32, np.int32], occupancy_map: np.ndarray, entity_map: np.ndarray, state: State, target) -> np.ndarray:
    """Computes the path that the drone will use for its pathfinding

    Args:
        tile_map_size: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.tile_map_size`.
        occupancy_map: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.occupancy_map`.
        entity_map: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.entity_map`.
        state: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.state`.

    Returns:
        The resulting :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.
    """
    path_map = BASE_WEIGHT * np.ones(tile_map_size, dtype=np.int8)
    for x in np.arange(0, tile_map_size[0]):
        for y in np.arange(0, tile_map_size[1]):
            # region updates entity_map
            if occupancy_map[x, y] < 0:
                entity_map[x, y] = Entity.VOID.value
            elif occupancy_map[x, y] > 0 and entity_map[x, y] != Entity.BASE.value and entity_map[x, y] != Entity.DRONE.value:
                entity_map[x, y] = Entity.WALL.value

            # endregion
            if x == 0 or x == tile_map_size[0] - 1 or y == 0 or y == tile_map_size[1] - 1 or occupancy_map[x, y] > 0 or entity_map[x, y] == Entity.KILL.value:
                path_map[x, y] = 0
                if entity_map[x, y] == Entity.KILL.value:
                    weight = WALL_WEIGHT * KILL_RELUCTANCE
                elif entity_map[x, y] == Entity.DRONE.value:
                    weight = WALL_WEIGHT * DRONE_RELUCTANCE
                else:
                    weight = WALL_WEIGHT
                for i in np.arange(max(int(x - WALL_RUNOFF), 0), min(int(x + WALL_RUNOFF), tile_map_size[0]-1)):
                    for j in np.arange(max(int(y - WALL_RUNOFF), 0), min(int(y + WALL_RUNOFF), tile_map_size[1]-1)):
                        distance = m.sqrt((x - i) ** 2 + (y - j) ** 2)
                        if path_map[i, j] != 0 and distance <= WALL_RUNOFF:
                            path_map[i, j] = max(path_map[i, j], f_runoff(distance, weight))
            elif entity_map[x, y] == Entity.CLOUD.value and path_map[x, y] != 0:
                if state == State.SAVE.value:
                    path_map[x, y] += PRUDENCE
                else:
                    path_map[x, y] -= CLOUD_BONUS

    if state == State.RESCUE.value or state == State.SAVE.value:
        for i in np.arange(max(int(target[0] - VICTIM_ZONE), 0), min(int(target[0] + VICTIM_ZONE), tile_map_size[0]-1)):
            for j in np.arange(max(int(target[1] - VICTIM_ZONE), 0), min(int(target[1] + VICTIM_ZONE), tile_map_size[1]-1)):
                path_map[i, j] = BASE_WEIGHT

    return path_map

def anticipate_pos(tile_pos, speed, tile_map_size):
    """Deduces where the drone will soon be from his estimated position and speed

    Args:
        tile_pos:
        speed:
        tile_map_size:

    Returns:
        The anticipated position
    """
    x = min(max(0, tile_pos[0] + round(speed[0] / TILE_SIZE, 0)), tile_map_size[0]-1)
    y = min(max(0, tile_pos[1] + round(speed[1] / TILE_SIZE, 0)), tile_map_size[1]-1)
    return np.array([x, y], dtype=np.int8)

def find_path(tile_map_size: Tuple[int, int], path_map: np.ndarray, tile_pos: np.ndarray, target_pos: Tuple[int, int], drone_speed: np.ndarray) -> (
        List)[Tuple[int, int]]:
    """Computes the shortest path

    Args:
        tile_map_size: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.tile_map_size`.
        path_map: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.
        tile_pos: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.tile_pos`.
        target_pos: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.target_pos`.
        drone_speed: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.drone_speed`.

    Returns:
        The r
    """
    graph = tcod.path.SimpleGraph(cost=path_map, cardinal=5, diagonal=7)
    pf = tcod.path.Pathfinder(graph)
    pf.add_root(anticipate_pos(tile_pos, drone_speed, tile_map_size))

    computed_path = pf.path_to(target_pos)

    if len(computed_path) > 2:
        return computed_path[1:]
    else:
        return computed_path
