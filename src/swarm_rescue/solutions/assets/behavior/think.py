import numpy as np
import numba as nb
import math as m
import tcod.path

from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE, VICTIM_RADIUS
from swarm_rescue.solutions.assets.mapping.entity import Entity
from swarm_rescue.solutions.assets.movement.pathfinding import anticipate_pos
from swarm_rescue.solutions.assets.behavior.map_split import ZONE_SIZE, waypoint_pos
from swarm_rescue.solutions.assets.behavior.state import State

# region local constants
GRAB_DISTANCE = 28
DROP_DISTANCE = 50
EXPLORED_PATH_DISTANCE = 2
ZONE_COMPLETION = 85
VICTIM_RESCUED_NB = -2
VICTIM_WAITING_NB = -1

WAYPOINTS_SCAN = 10
VICTIM_WAIT = 10


# endregion

# @nb.njit
def next_waypoint(tile_pos, drone_speed, waypoints, n_width, n_height, tile_map_size, path_map):
    if 2 in waypoints:
        exigence = 2
    elif 1 in waypoints:
        exigence = 1
    else:
        return None

    graph = tcod.path.SimpleGraph(cost=path_map, cardinal=5, diagonal=7)
    pf = tcod.path.Pathfinder(graph)
    pf.add_root(anticipate_pos(tile_pos, drone_speed, tile_map_size))
    best_waypoint = None
    best_distance = m.inf

    for i in np.arange(n_width):
        for j in np.arange(n_height):
            if waypoints[i, j] == exigence:
                x, y = waypoint_pos(i, j)
                distance = len(pf.path_to((x, y)))
                if distance < best_distance:
                    best_waypoint = (i, j)
                    best_distance = distance

    return best_waypoint


def next_victim(tile_pos, drone_speed, victims, tile_map_size):
    drone_x, drone_y = anticipate_pos(tile_pos, drone_speed, tile_map_size)
    best_victim_index = None
    best_distance = m.inf

    for i in np.arange(len(victims)):
        if victims[i][2] == VICTIM_WAITING_NB:
            distance = m.sqrt((drone_x - victims[i][0]) ** 2 + (drone_y - victims[i][1]) ** 2)
            if distance < best_distance:
                best_victim_index = i
                best_distance = distance

    return best_victim_index


def next_base(tile_pos, drone_speed, bases, tile_map_size):
    drone_x, drone_y = anticipate_pos(tile_pos, drone_speed, tile_map_size)
    best_base = None
    best_distance = m.inf

    for base in bases:
        distance = m.sqrt((drone_x - base[0]) ** 2 + (drone_y - base[1]) ** 2)
        if distance < best_distance:
            best_base = base
            best_distance = distance

    return best_base


@nb.njit
def bounded_variation(x, variation, x_sup, x_inf=0):
    """
    returns a range a values around x so that it stays bounded by x_sup and x_inf
    """
    return np.arange(max(int(x - variation), x_inf), min(int(x + variation), x_sup))


@nb.njit
def waypoints_scan(n_width, n_height, waypoints, tile_map_size, entity_map):
    for i in np.arange(n_width):
        for j in np.arange(n_height):
            if waypoints[i, j] != 0 and partially_explored(waypoint_pos(i, j), tile_map_size, entity_map):
                waypoints[i, j] = 0


@nb.njit
def partially_explored(tile_pos, tile_map_size, entity_map):
    """
    check is the zone of center pos and size map_size is explored or not
    """
    zone_area = ZONE_SIZE * ZONE_SIZE / 2
    zone_knowledge = zone_area
    for i in bounded_variation(tile_pos[0], ZONE_SIZE / 2, tile_map_size[0] - 1):
        for j in bounded_variation(tile_pos[1], ZONE_SIZE / 2 - tile_pos[0] + i, tile_map_size[1] - 1):
            if entity_map[i, j] == Entity.BASE.value:
                zone_knowledge += 5
            elif entity_map[i, j] == Entity.WALL.value:
                zone_knowledge += 1
            elif entity_map[i, j] == Entity.CLOUD.value and 0 <= i < tile_map_size[0] and 0 <= j < tile_map_size[1]:
                zone_knowledge -= 1
                if zone_knowledge < zone_area * ZONE_COMPLETION / 100:
                    return False
    return True


def is_explored(waypoints, target_waypoint, path, tile_map_size, entity_map):
    if waypoints[target_waypoint] == 0 or len(path) <= EXPLORED_PATH_DISTANCE:
        return True
    else:
        return partially_explored(waypoint_pos(target_waypoint[0], target_waypoint[1]), tile_map_size, entity_map)


def is_defined(target):
    return target != undefined_target


undefined_target = (-1, -1)


def compute_behavior(id, target, target_waypoint, tile_pos, path, speed, state, victims, distance_from_closest_victim, bases, distance_from_closest_base,
                     got_victim, waypoints, n_width, n_height, tile_map_size, entity_map, path_map, timers):
    if state == State.BOOT.value:
        # TODO boot
        return State.EXPLORE.value, undefined_target, undefined_target

    elif state == State.EXPLORE.value:
        best_victim_index = next_victim(tile_pos, speed, victims, tile_map_size)
        if best_victim_index is not None:
            timers["waypoints_scan"] = 0  # resets the timer for next time it explores
            victims[best_victim_index][2] = id
            return State.RESCUE.value, tuple(victims[best_victim_index][:2]), undefined_target
        else:
            if timers["waypoints_scan"] == WAYPOINTS_SCAN:
                timers["waypoints_scan"] = 0
                waypoints_scan(n_width, n_height, waypoints, tile_map_size, entity_map)
            else:
                timers["waypoints_scan"] += 1
            if is_defined(target_waypoint):
                if is_explored(waypoints, target_waypoint, path, tile_map_size, entity_map):
                    waypoints[target_waypoint] = 0
                    return State.EXPLORE.value, undefined_target, undefined_target
                else:
                    return State.EXPLORE.value, target, target_waypoint
            else:
                nw = next_waypoint(tile_pos, speed, waypoints, n_width, n_height, tile_map_size, path_map)
                if nw is None:
                    return State.DONE.value, undefined_target, undefined_target
                else:
                    return State.EXPLORE.value, waypoint_pos(nw[0], nw[1]), nw

    elif state == State.RESCUE.value:
        if distance_from_closest_victim is not None and distance_from_closest_victim < GRAB_DISTANCE:
            return State.SAVE.value, undefined_target, undefined_target

        elif distance_from_closest_victim is not None and distance_from_closest_victim >= GRAB_DISTANCE:
            return State.RESCUE.value, target, undefined_target

        elif distance_from_closest_victim is None:
            # TODO : shouldn't happen so error message
            return State.RESCUE.value, target, undefined_target

    elif state == State.SAVE.value:
        if not is_defined(target) and got_victim:
            best_target_index, best_distance = None, m.inf
            for i in range(len(victims)):
                distance = m.dist(victims[i][:2], (tile_pos[0], tile_pos[1]))
                if distance < best_distance:
                    best_target_index = i
                    best_distance = distance
            victims[best_target_index][2] = VICTIM_RESCUED_NB
            nb = next_base(tile_pos, speed, bases, tile_map_size)
            if nb is None:
                return State.SAVE.value, tuple(bases[0]), undefined_target  # TODO : change that
            else:
                return State.SAVE.value, tuple(nb), undefined_target

        elif not is_defined(target) and not got_victim:
            return State.SAVE.value, undefined_target, undefined_target  # TODO wait at victim's position

        elif is_defined(target) and not got_victim:
            return State.EXPLORE.value, undefined_target, undefined_target

        elif is_defined(target) and got_victim:
            return State.SAVE.value, target, undefined_target

        else:
            return state, undefined_target, undefined_target

    else:  # state == State.DONE.value
        return state, undefined_target, undefined_target
