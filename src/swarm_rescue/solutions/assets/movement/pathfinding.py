import numpy as np
import math as m
import tcod.path
import numba as nb
from typing import List, Tuple

from swarm_rescue.solutions.assets.mapping.entity import Entity, add_entity, bounded_variation
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE
from swarm_rescue.solutions.assets.behavior.state import State

# region local constants
BASIC_WEIGHT = 3
"""
Constant used in :py:func:`compute_path_map`, corresponding to the base weight of the tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.

:type: int
:domain: [:py:data:`CLOUD_BONUS` + 1, inf]
"""
WALL_WEIGHT = BASIC_WEIGHT * 13
"""
Constant used in :py:func:`f_runoff`, corresponding to the additional weight that the drone will put on a wall in his 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`. In other words, how much the drone will avoid the surroundings of a wall.

:type: int
:domain: [0, inf]
"""
WALL_RUNOFF = 3
"""
Constant used in :py:func:`f_runoff`, corresponding to how far a wall will impact the weights of his surroundings tiles in the 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map` of the drone. In other words, the distance that the drone will try to take 
when moving alongside a wall.

:type: int
:domain: [1, inf]
"""
DRONE_RUNOFF = 2
"""


:type: int
:domain: [1, inf]
"""
CLOUD_BONUS = 1
"""
Constant used in :py:func:`compute_path_map`, corresponding to the bonus weight of the cloud tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map` when the drone is exploring. This allows for more exploration from the drone, 
instead of always taking the same paths.

:type: int
:domain: [0, :py:data:`BASE_WEIGHT` - 1]
"""
# PRUDENCE = SAFE_PRUDENCE
SAFE_PRUDENCE = BASIC_WEIGHT * 24
"""
Constant used in :py:func:`compute_path_map`, corresponding to the malus weight of the cloud tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map` when the drone is carrying a victim. It avoids the drone from going into a 
kill zone with a victim.

:type: int
:domain: [0, inf]
"""
DRONE_RELUCTANCE = BASIC_WEIGHT * 12
"""
Constant used in :py:func:`compute_path_map`, corresponding to the malus weight of the other drone's tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.

:type: int
:domain: [0, inf]
"""
KILL_RELUCTANCE = BASIC_WEIGHT * 24
"""
Constant used in :py:func:`compute_path_map`, corresponding to the malus weight of the kill zone tiles of 
:py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.

:type: int
:domain: [0, inf]
"""
KILL_RUNOFF = 5.5

VICTIM_ZONE = 2
DRONE_ZONE = 1

NO_GO_ENTITIES = (Entity.WALL.value, Entity.BASE.value, Entity.KILL.value)
# endregion

f_runoff = nb.njit(lambda distance, weight, runoff: np.int64(round(BASIC_WEIGHT + weight - (distance * m.sqrt(weight) / (runoff + 0.5))**2, 0)))
"""Used to compute the impact of a wall on :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`. Used in 
:py:func:`compute_path_map`.
"""


@nb.njit
def is_drone_close(x, y, detected_drones):
    distances = np.sqrt((x - detected_drones[:, 0])**2 + (y - detected_drones[:, 1])**2)
    return np.any(distances < TILE_SIZE)


@nb.njit
def weight_propagate(x, y, runoff, weight, tile_map_size, path_map, entity_map):
    for i in bounded_variation(x, runoff, tile_map_size[0]):
        for j in bounded_variation(y, runoff, tile_map_size[1]):
            distance = m.sqrt((x - i) ** 2 + (y - j) ** 2)
            if path_map[i, j] != 0 and entity_map[i, j] != Entity.CLOUD.value and distance <= runoff:
                path_map[i, j] = max(path_map[i, j], f_runoff(distance, weight, runoff))


@nb.njit(parallel=True)
def compute_path_map(tile_map_size: Tuple[np.int32, np.int32], occupancy_map: np.ndarray, entity_map: np.ndarray, state: State, target, detected_drones: np.ndarray) -> np.ndarray:
    """Computes the path that the drone will use for its pathfinding

    Args:
        tile_map_size: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.tile_map_size`.
        occupancy_map: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.occupancy_map`.
        entity_map: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.entity_map`.
        state: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.state`.

    Returns:
        The resulting :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.
    """
    path_map = BASIC_WEIGHT * np.ones(tile_map_size, dtype=np.int64)
    for x in nb.prange(0, tile_map_size[0]):
        for y in nb.prange(0, tile_map_size[1]):
            # region updates entity_map
            if occupancy_map[x, y] < 0:
                add_entity(x, y, Entity.VOID.value, entity_map, tile_map_size)
            elif occupancy_map[x, y] > 0:
                add_entity(x, y, Entity.WALL.value, entity_map, tile_map_size)
            # endregion

            tile_is_drone = is_drone_close(x * TILE_SIZE, y * TILE_SIZE, detected_drones)
            if x == 0 or x == tile_map_size[0] - 1 or y == 0 or y == tile_map_size[1] - 1 or entity_map[x, y] in NO_GO_ENTITIES:
                path_map[x, y] = 0
                runoff = WALL_RUNOFF
                if entity_map[x, y] == Entity.KILL.value:
                    runoff = KILL_RUNOFF
                    weight = KILL_RELUCTANCE
                else:
                    weight = WALL_WEIGHT
                weight_propagate(x, y, runoff, weight, tile_map_size, path_map, entity_map)
            elif tile_is_drone:
                add_entity(x, y, Entity.VOID.value, entity_map, tile_map_size, DRONE_ZONE)
                weight_propagate(x, y, DRONE_RUNOFF, DRONE_RELUCTANCE, tile_map_size, path_map, entity_map)
            elif path_map[x, y] != 0:
                if entity_map[x, y] == Entity.CLOUD.value and state != State.SAVE.value:
                    path_map[x, y] = BASIC_WEIGHT - CLOUD_BONUS
                elif state == State.SAVE.value and entity_map[x, y] not in [Entity.SAFE.value, Entity.NOCOM.value, Entity.NOGPS.value]:
                    path_map[x, y] += SAFE_PRUDENCE

    if state == State.RESCUE.value:
        for i in bounded_variation(target[0], VICTIM_ZONE, tile_map_size[0]):
            for j in bounded_variation(target[1], VICTIM_ZONE, tile_map_size[1]):
                path_map[i, j] = BASIC_WEIGHT

    return path_map


f_anticipate = nb.njit(lambda dx: np.sign(dx) * 8 * np.power(np.tanh(0.6*dx), 4))


def anticipate_pos(tile_pos, speed, tile_map_size, entity_map):
    """Deduces where the drone will soon be from his estimated position and speed

    Args:
        tile_pos:
        speed:
        tile_map_size:

    Returns:
        The anticipated position
    """
    if not m.isnan(speed[0]):
        x = min(max(0, tile_pos[0] + int(f_anticipate(speed[0]))), tile_map_size[0] - 1)
        y = min(max(0, tile_pos[1] + int(f_anticipate(speed[1]))), tile_map_size[1] - 1)
        if entity_map[x, y] in NO_GO_ENTITIES:
            x = tile_pos[0]
            y = tile_pos[1]
    else:
        x = tile_pos[0]
        y = tile_pos[1]

    return np.array([x, y], dtype=np.int32)


def find_path(path_map: np.ndarray, tile_pos: np.ndarray, target_pos: np.ndarray) -> List[Tuple[int, int]]:
    """Computes the shortest path

    Args:
        path_map: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.path_map`.
        tile_pos: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.tile_pos`.
        target_pos: see :py:attr:`~swarm_rescue.solutions.myFirstDrone.MyFirstDrone.target_pos`.

    Returns:
        The r
    """
    old_pos_value = path_map[tile_pos[0], tile_pos[1]]
    old_target_value = path_map[target_pos[0], target_pos[1]]
    path_map[tile_pos[0], tile_pos[1]] = 1
    path_map[target_pos[0], target_pos[1]] = 1  # important to be able to reach bases
    graph = tcod.path.SimpleGraph(cost=path_map, cardinal=1, diagonal=0, greed=BASIC_WEIGHT-CLOUD_BONUS)
    pf = tcod.path.Pathfinder(graph)
    pf.add_root(tile_pos)

    target = tuple(target_pos.copy())
    computed_path = pf.path_to(target).tolist()

    path_map[tile_pos[0], tile_pos[1]] = old_pos_value
    path_map[target_pos[0], target_pos[1]] = old_target_value

    n = len(computed_path)
    ignored_path_tiles = (0 if n in (1, 2) else 1)
    return computed_path[ignored_path_tiles:]


