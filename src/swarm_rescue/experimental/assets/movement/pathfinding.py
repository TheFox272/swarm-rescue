from math import sqrt
import numpy as np
import math as m
import tcod.path
import sys, os
import numba as nb
from typing import List, Tuple

from src.swarm_rescue.experimental.assets.mapping.entity import Entity
from src.swarm_rescue.experimental.assets.behavior.state import State


# region local constants
WALL_WEIGHT = 8
"""
Constant used in :py:func:`f_runoff`, corresponding to the additional weight that the drone will put on a wall in his 
:py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map`. In other words, how much the drone will avoid the surroundings of a wall.

:type: int
:domain: [0, inf]
"""
WALL_RUNOFF = 5
"""
Constant used in :py:func:`f_runoff`, corresponding to how far a wall will impact the weights of his surroundings tiles in the 
:py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map` of the drone. In other words, the distance that the drone will try to take 
when moving alongside a wall.

:type: int
:domain: [1, inf]
"""
BASE_WEIGHT = 5
"""
Constant used in :py:func:`compute_path_map`, corresponding to the base weight of the tiles of 
:py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map`.

:type: int
:domain: [:py:data:`CLOUD_BONUS` + 1, inf]
"""
CLOUD_BONUS = 2
"""
Constant used in :py:func:`compute_path_map`, corresponding to the bonus weight of the cloud tiles of 
:py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map` when the drone is exploring. This allows for more exploration from the drone, 
instead of always taking the same paths.

:type: int
:domain: [0, :py:data:`BASE_WEIGHT` - 1]
"""
PRUDENCE = 10
"""
Constant used in :py:func:`compute_path_map`, corresponding to the malus weight of the cloud tiles of 
:py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map` when the drone is carrying a victim. It avoids the drone from going into a 
kill zone with a victim.

:type: int
:domain: [0, inf]
"""
KILL_RELUCTANCE = 5
"""
Constant used in :py:func:`compute_path_map`, corresponding to the malus weight of the kill zone tiles of 
:py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map`.

:type: int
:domain: [0, inf]
"""
# endregion

f_runoff = nb.njit(lambda distance, weight: np.int8(round(BASE_WEIGHT + weight * (1 - (distance-1) / WALL_RUNOFF), 0)))
"""Used to compute the impact of a wall on :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map`. Used in 
:py:func:`compute_path_map`.
"""

@nb.njit
def compute_path_map(tile_map_size: Tuple[int, int], occupancy_map: np.ndarray, entity_map: np.ndarray, state: State) -> np.ndarray:
    """Computes the path that the drone will use for its pathfinding

    Args:
        tile_map_size: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.tile_map_size`.
        occupancy_map: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.occupancy_map`.
        entity_map: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.entity_map`.
        state: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.state`.

    Returns:
        The resulting :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map`.
    """
    path_map = BASE_WEIGHT * np.ones(tile_map_size, dtype=np.int8)
    for x in np.arange(0, tile_map_size[0]):
        for y in np.arange(0, tile_map_size[1]):
            if x == 0 or x == tile_map_size[0] - 1 or y == 0 or y == tile_map_size[1] - 1 or occupancy_map[x, y] > 0 or entity_map[x, y] == Entity.KILL.value:
                path_map[x, y] = 0
                if entity_map[x, y] == Entity.KILL.value:
                    weight = WALL_WEIGHT * KILL_RELUCTANCE
                else:
                    weight = WALL_WEIGHT
                for i in np.arange(max(int(x - WALL_RUNOFF), 0), min(int(x + WALL_RUNOFF), tile_map_size[0])):
                    for j in np.arange(max(int(y - WALL_RUNOFF), 0), min(int(y + WALL_RUNOFF), tile_map_size[1])):
                        distance = m.sqrt((x - i) ** 2 + (y - j) ** 2)
                        if path_map[i, j] != 0 and distance <= WALL_RUNOFF:
                            path_map[i, j] = max(path_map[i, j], f_runoff(distance, weight))
            elif entity_map[x, y] == Entity.CLOUD.value and path_map[x, y] != 0:
                if state.value == State.SAVE.value or state.value == State.DROP.value:
                    path_map[x, y] += PRUDENCE
                else:
                    path_map[x, y] -= CLOUD_BONUS

    return path_map

def anticipate_pos(position, speed, tile_size, map_size):
    """Deduces where the drone will soon be from his estimated position and speed

    Args:
        position:
        speed:
        tile_size:
        map_size:

    Returns:
        The anticipated position
    """
    x = min(max(0, position[0] + round(speed[0] / tile_size, 0)), map_size[0])
    y = min(max(0, position[1] + round(speed[1] / tile_size, 0)), map_size[1])
    return np.array([x, y], dtype=np.int8)

def find_path(tile_map_size: Tuple[int, int], path_map: np.ndarray, drone_pos: np.ndarray, target_pos: np.ndarray, drone_speed: np.ndarray, tile_size: int) -> (
        List)[Tuple[int, int]]:
    """Computes the shortest path

    Args:
        tile_map_size: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.tile_map_size`.
        path_map: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map`.
        drone_pos: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.drone_pos`.
        target_pos: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.target_pos`.
        drone_speed: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.drone_speed`.
        tile_size: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.tile_size`.

    Returns:
        The r
    """
    graph = tcod.path.SimpleGraph(cost=path_map, cardinal=5, diagonal=7)
    pf = tcod.path.Pathfinder(graph)
    pf.add_root(anticipate_pos(drone_pos, drone_speed, tile_size, tile_map_size))
    return pf.path_to(target_pos)[1:-1]
