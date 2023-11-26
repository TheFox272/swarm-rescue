import math as m
from typing import List, Tuple

import numba as nb
import numpy as np

from swarm_rescue.experimental.assets.mapping.mapping_constants import TILE_SIZE
from swarm_rescue.experimental.assets.movement.pathfinding import BASE_WEIGHT, CLOUD_BONUS

# region local constants
FORESEE = 7
"""
Constant corresponding to the maximum number of points from the :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path` that the drone will take into 
consideration in :py:func:`compute_command`

:type: int
:domain: [1, inf]
"""
K_FOR = 0.4
"""
Constant used in :py:func:`compute_command` to compute the forward thrust command of the drone

:type: float
:domain: [0, 1]list(list())
"""
K_LAT = 0.7
"""
Constant used in :py:func:`compute_command` to compute the lateral thrust command of the drone

:type: float
:domain: [0, 1]
"""
K_ROT = 0.9
"""
Constant used in :py:func:`compute_command` to compute the rotational thrust command of the drone

:type: float
:domain: [0, 1]
"""
# endregion

def signed_angle(a: float) -> float:
    """Brings back an angle in [-π, π]

    Args:
        a: The angle that must be converted.

    Returns:
        The resulting angle.
    """
    if a > m.pi:
        return signed_angle(a - 2 * m.pi)
    elif a < - m.pi:
        return signed_angle(a + 2 * m.pi)
    else:
        return a


def first_weighted_avg(nList: List[float], n: np.uint) -> float:
    """Computes the weighted average of n points, the first elements being the heaviest

    Args:
        nList: The list on which the average is made
        n: The size of the list

    Returns:
        The resulting weighted average
    """
    res = 0
    for i in range(n):
        res += nList[i] * (n-i)
    res /= n * (n+1) / 2
    return res


def compute_command(path: List[Tuple[int, int]], path_map: np.ndarray, tile_pos: np.ndarray) -> dict[str, float]:
    """Computes the command of the drone

    Uses path to determine the next point that the drone needs to reach, using the :py:function:first_weighted_avg function to anticipate the trajectory. Uses the
    path_map to determine if the path is risky, and if so anticipate less.

    Args:
        path: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path`.
        path_map: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.path_map`.
        tile_pos: see :py:attr:`~swarm_rescue.experimental.droneClasses.myFirstDrone.MyFirstDrone.tile_pos`.

    Returns:
        The resulting control command.
    """
    command = {"forward": 0.0,
               "lateral": 0.0,
               "rotation": 0.0,
               "grasper": 0}

    if len(path) == 0:
        return command
    else:
        foresee = min(FORESEE, len(path))

        # less anticipation if the path is risky
        foresee -= min(max(0, int(sum([path_map[tuple(path_tile)] for path_tile in path[:foresee]]) / BASE_WEIGHT) - foresee), foresee-1)

        # computes target point that the drone needs to follow in order to follow the path
        dx = first_weighted_avg([path_tile[0] - tile_pos[0] for path_tile in path[:foresee]], foresee)
        dy = first_weighted_avg([path_tile[1] - tile_pos[1] for path_tile in path[:foresee]], foresee)

        path_angle = m.atan2(dy, dx)

        forward_diff = dx * m.cos(tile_pos[2]) + dy * m.sin(tile_pos[2])
        lateral_diff = -dx * m.sin(tile_pos[2]) + dy * m.cos(tile_pos[2])
        rot_diff = signed_angle(path_angle - tile_pos[2])

        command["forward"] = min(max(-1, forward_diff * K_FOR), 1)
        command["lateral"] = min(max(-1, lateral_diff * K_LAT), 1)
        command["rotation"] = min(max(-1, rot_diff * K_ROT), 1)

        return command