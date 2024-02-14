import numpy as np
import numba as nb
import math as m

from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE

# region local constants
PROJECT_COEF = 0.35
ZONE_SIZE = 10  # multiple of 2


# endregion

@nb.njit
def base_project(x, y, map_size):
    w = map_size[0]
    h = map_size[1]
    w_dist = min(x, w - x)
    h_dist = min(y, h - y)
    if w_dist >= PROJECT_COEF * w and h_dist >= PROJECT_COEF * h:
        x_res = x
        y_res = y
    elif w_dist <= h_dist:
        if w_dist == x:
            x_res = 0
        else:
            x_res = w - 1
        if h_dist < PROJECT_COEF * h:
            if h_dist == y:
                y_res = 0
            else:
                y_res = h - 1
        else:
            y_res = y
    else:
        if h_dist == y:
            y_res = 0
        else:
            y_res = h - 1
        if w_dist < PROJECT_COEF * w:
            if w_dist == x:
                x_res = 0
            else:
                x_res = w - 1
        else:
            x_res = x
    return tuple((x_res, y_res))


@nb.njit
def waypoint_pos(i, j):
    half_zone_side = ZONE_SIZE / 2
    x = (i + 1) * half_zone_side - 1
    y = ((i % 2) + 1 + 2 * j) * half_zone_side - 1
    return tuple((int(x), int(y)))


@nb.njit
def init_zone_split(tile_map_size, waypoints_dims):
    waypoints_dims[0] = int(2 * tile_map_size[0] / ZONE_SIZE)
    waypoints_dims[1] = int(tile_map_size[1] / ZONE_SIZE)
    return np.ones((waypoints_dims[0], waypoints_dims[1]), dtype=np.uint8)


@nb.njit
def zone_split(tile_map_size, drone_pos, nb_teams, n_team, waypoints, waypoints_dims):
    if (2 * tile_map_size[0]) % ZONE_SIZE == 0:
        auto_explore_max_width = True
    else:
        auto_explore_max_width = False
    if tile_map_size[1] % ZONE_SIZE == 0:
        auto_explore_max_height = True
    else:
        auto_explore_max_height = False

    base_pos = base_project(drone_pos[0], drone_pos[1], tile_map_size)

    split_mode = 'normal'
    if (base_pos[0] == 0 and base_pos[1] == 0) or (base_pos[0] == tile_map_size[0] - 1 and base_pos[1] == tile_map_size[1] - 1):
        min_angle = 0
        max_angle = m.pi / 2
    elif (base_pos[0] == 0 and base_pos[1] == tile_map_size[1] - 1) or (base_pos[0] == tile_map_size[0] - 1 and base_pos[1] == 0):
        min_angle = - m.pi / 2
        max_angle = 0
    elif base_pos[0] == 0 or base_pos[0] == tile_map_size[0] - 1 or base_pos[1] == 0 or base_pos[1] == tile_map_size[1] - 1:
        min_angle = - m.pi / 2
        max_angle = m.pi / 2
        if base_pos[1] == 0 or base_pos[1] == tile_map_size[1] - 1:
            split_mode = 'decal'
    else:
        min_angle = - m.pi
        max_angle = m.pi
        split_mode = 'whole'

    for i in np.arange(waypoints_dims[0]):
        for j in np.arange(waypoints_dims[1]):
            if (auto_explore_max_width and i == waypoints_dims[0] - 1) or (auto_explore_max_height and j == waypoints_dims[1] - 1 and i % 2 == 1):
                waypoints[i, j] = 0
                continue
            pos = waypoint_pos(i, j)
            if pos[0] - base_pos[0] != 0:
                if split_mode == 'normal':
                    angle_waypoint = m.atan((pos[1] - base_pos[1]) / (pos[0] - base_pos[0]))
                elif split_mode == 'whole':
                    angle_waypoint = m.atan2(pos[1] - base_pos[1], pos[0] - base_pos[0])
                elif split_mode == 'decal':
                    angle_waypoint = m.atan((pos[1] - base_pos[1]) / (pos[0] - base_pos[0])) + m.pi / 2
                    if angle_waypoint > max_angle:
                        angle_waypoint -= m.pi
            else:
                angle_waypoint += min_angle
            if min_angle + n_team / nb_teams * (max_angle - min_angle) <= angle_waypoint <= min_angle + (n_team + 1) / nb_teams * (max_angle - min_angle):
                waypoints[i, j] = 2
