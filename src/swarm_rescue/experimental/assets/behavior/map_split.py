import numpy as np
import numba as nb
import math as m

from swarm_rescue.experimental.assets.mapping.mapping_constants import TILE_SIZE

# region local constants
PROJECT_COEF = 0.3
ZONE_SIZE = 10
# endregion

def base_project(x, y, map_size):
    w = map_size[0]
    h = map_size[1]
    w_dist = min(x, w-x)
    h_dist = min(y, h-y)
    if w_dist <= h_dist:
        if w_dist == x:
            x_res = 0
        else:
            x_res = w
        if h_dist < PROJECT_COEF * h:
            if h_dist == y:
                y_res = 0
            else:
                y_res = h
        else:
            y_res = y
    else:
        x_res = x
        if h_dist == y:
            y_res = 0
        else:
            y_res = h
        if w_dist < PROJECT_COEF * w:
            if w_dist == x:
                x_res = 0
            else:
                x_res = w
        else:
            y_res = y
    return tuple((x_res, y_res))

def waypoint_pos(i, j):
    x, y = 0, 0
    if i%2 == 0:
        x += ZONE_SIZE / 2 + (i/2) * ZONE_SIZE
    else:
        x += ZONE_SIZE + ((i-1)/2) * ZONE_SIZE
        y += ZONE_SIZE / 2
    y += ZONE_SIZE / 2 + j * ZONE_SIZE
    return tuple((int(x), int(y)))

def zone_split(tile_map_size, base_pos, nb_teams, n_team):

    n_width = int(2 * tile_map_size[0] / ZONE_SIZE)
    n_height = int(tile_map_size[1] / ZONE_SIZE)
    waypoints = np.ones((n_width, n_height), dtype=np.int8)

    if base_pos[0] == 0 and base_pos[1] == 0 or base_pos[0] == tile_map_size[0] and base_pos[1] == tile_map_size[1]:
        min_angle = 0
        max_angle = m.pi / 2
    elif base_pos[0] == 0 and base_pos[1] == tile_map_size[1] or base_pos[0] == tile_map_size[0] and base_pos[1] == 0:
        min_angle = - m.pi / 2
        max_angle = 0
    else:
        min_angle = - m.pi / 2
        max_angle = m.pi / 2

    for i in range(n_width):
        for j in range(n_height):
            pos = waypoint_pos(i, j)
            if pos[0] - base_pos[0] != 0:
                angle_waypoint = m.atan((pos[1] - base_pos[1])/(pos[0] - base_pos[0]))
            else:
                angle_waypoint = 0
            if min_angle + (n_team - 1) / nb_teams * (max_angle - min_angle) <= angle_waypoint <= min_angle + n_team / nb_teams * (max_angle - min_angle):
                waypoints[i, j] = 2

    return waypoints, n_width, n_height