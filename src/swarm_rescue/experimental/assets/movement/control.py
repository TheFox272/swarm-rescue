import math as m

import numpy as np

from swarm_rescue.experimental.assets.mapping.lidarMapping import TILE_SIZE
from swarm_rescue.experimental.assets.movement.pathfinding import BASE_WEIGHT, CLOUD_BONUS

# region local constants
FORESEE = 8
# endregion

def signed_angle(a):
    if a > m.pi:
        return signed_angle(a - 2 * m.pi)
    elif a < - m.pi:
        return signed_angle(a + 2 * m.pi)
    else:
        return a

def compute_command(path, path_map, tile_pos, speed):

    command = {"forward": 0.0,
               "lateral": 0.0,
               "rotation": 0.0,
               "grasper": 0}

    dx, dy = 0, 0
    foresee = min(FORESEE, len(path))
    i = 0
    while foresee > 0:
        # dx = avg([x - self.x_cell for x in np.array(self.path)[:preplanning,0]])
        # dy = avg([y - self.y_cell for y in np.array(self.path)[:preplanning,1]])
        dx += path[i][0] - tile_pos[0]
        dy += path[i][1] - tile_pos[1]
        cost = path_map[tuple(path[i])]
        if cost > BASE_WEIGHT:
            foresee -= 1
        foresee -= 1
        i += 1

    dx -= speed[0] / TILE_SIZE
    dy -= speed[1] / TILE_SIZE
    command["forward"] = min(max(-1, (dx * m.cos(tile_pos[2]) + dy * m.sin(tile_pos[2]))), 1)
    command["lateral"] = min(max(-1, (-dx * m.sin(tile_pos[2]) + dy * m.cos(tile_pos[2]))), 1)

    path_angle = m.atan2(dy, dx)
    command["rotation"] = min(max(-1, signed_angle(path_angle - tile_pos[2]) / m.pi), 1)

    return command