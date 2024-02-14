from typing import Tuple

import numpy as np
import numba as nb
import math as m
import tcod.path

from swarm_rescue.solutions.assets.mapping.entity import Entity, bounded_variation
from swarm_rescue.solutions.assets.movement.pathfinding import anticipate_pos, find_path
from swarm_rescue.solutions.assets.behavior.map_split import ZONE_SIZE, waypoint_pos, zone_split
from swarm_rescue.solutions.assets.behavior.state import State

# region local constants
GRAB_DISTANCE = 28
DROP_DISTANCE = 60
EXPLORED_PATH_DISTANCE = 2
ZONE_COMPLETION = 92
VICTIM_RESCUED_NB = -2
VICTIM_WAITING_NB = -1

WAYPOINTS_SCAN = 8
BASE_SCAN = 16
BELIEVE_WAIT = 2
VICTIM_WAIT = 4

EXPLORED_WP_VALUE = 0

MAX_BASES_SIZE = 8


# endregion


def next_waypoint(tile_pos, drone_speed, waypoints, waypoints_dims, tile_map_size, path_map):
    if 2 in waypoints:
        exigence = 2
    elif 1 in waypoints:
        exigence = 1
    else:
        return None

    graph = tcod.path.SimpleGraph(cost=path_map, cardinal=1, diagonal=0)
    pf = tcod.path.Pathfinder(graph)
    pf.add_root(anticipate_pos(tile_pos, drone_speed, tile_map_size))
    best_waypoint = None
    best_distance = m.inf

    for i in np.arange(waypoints_dims[0]):
        for j in np.arange(waypoints_dims[1]):
            if waypoints[i, j] == exigence:
                x, y = waypoint_pos(i, j)
                distance = len(pf.path_to((x, y)))
                if distance <= EXPLORED_PATH_DISTANCE:
                    waypoints[i, j] = EXPLORED_WP_VALUE
                elif distance < best_distance:
                    best_waypoint = (i, j)
                    best_distance = distance

    return best_waypoint


@nb.njit
def closest_victim(victim, victims, only_savable=False):
    if only_savable:
        indices = np.where(victims[:, 2] == VICTIM_WAITING_NB)[0]
    else:
        indices = np.where(victims[:, 2] != VICTIM_RESCUED_NB)[0]

    if not indices.size:
        return None

    # nodes = np.delete(nodes, 2, axis=1)
    dist_2 = np.sum((victims[indices, :2] - victim) ** 2, axis=1)
    return indices[np.argmin(dist_2)]


def next_victim(tile_pos, drone_speed, victims, tile_map_size):
    if len(victims) == 0:
        return None

    drone_pos = anticipate_pos(tile_pos, drone_speed, tile_map_size)

    victims_array = np.asarray(victims, dtype=np.int32)
    best_victim_index = closest_victim(drone_pos, victims_array, True)

    return best_victim_index


def next_base(tile_pos, drone_speed, bases, tile_map_size, path_map, distance_from_closest_base):
    if len(bases) == 0:
        return None

    best_distance = m.inf
    best_base = None

    if len(bases) < MAX_BASES_SIZE:
        valid_bases = []
        max_lookup = len(bases)
    else:
        valid_bases = bases[MAX_BASES_SIZE:]
        max_lookup = MAX_BASES_SIZE
    for base in bases[:max_lookup]:
        distance = len(find_path(tile_map_size, path_map, tile_pos, np.asarray(base, dtype=np.int32), drone_speed))
        if distance == 1 and m.isinf(distance_from_closest_base):  # means that base is isolated by walls somehow
            continue
        elif distance < best_distance:
            best_distance = distance
            best_base = base
        valid_bases.append(base)

    bases[:] = valid_bases

    if best_base is None and len(bases) != 0:
        return next_base(tile_pos, drone_speed, bases, tile_map_size, path_map, distance_from_closest_base)

    return best_base


@nb.njit
def waypoints_scan(waypoints_dims, waypoints, tile_map_size, entity_map):
    for i in np.arange(waypoints_dims[0]):
        for j in np.arange(waypoints_dims[1]):
            if waypoints[i, j] != EXPLORED_WP_VALUE and partially_explored(waypoint_pos(i, j), tile_map_size, entity_map):
                waypoints[i, j] = EXPLORED_WP_VALUE


@nb.njit
def partially_explored(tile_pos, tile_map_size, entity_map):
    """
    check is the zone of center pos and size map_size is explored or not
    """
    zone_area = ZONE_SIZE * ZONE_SIZE / 2
    zone_knowledge = zone_area
    for i in bounded_variation(tile_pos[0], ZONE_SIZE / 2, tile_map_size[0]):
        for j in bounded_variation(tile_pos[1], ZONE_SIZE / 2 - tile_pos[0] + i, tile_map_size[1]):
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
    if waypoints[target_waypoint[0], target_waypoint[1]] == EXPLORED_WP_VALUE or len(path) <= EXPLORED_PATH_DISTANCE:
        return True
    else:
        return partially_explored(waypoint_pos(target_waypoint[0], target_waypoint[1]), tile_map_size, entity_map)


@nb.njit
def is_defined(target_i: int):
    return target_i != undefined_index


@nb.njit
def target_is_defined(target: np.ndarray):
    return not np.array_equal(target, undefined_target)


@nb.njit
def waypoint_is_defined(wp: Tuple):
    return wp != undefined_waypoint

@nb.njit
def undefine_target(target: np.ndarray):
    target[:2] = undefined_target[:2].copy()


undefined_target = np.array([-1, -1], dtype=np.int32)
undefined_waypoint = (-1, -1)
undefined_index = -1


def compute_behavior(drone_id, target, target_indication, tile_pos, path, speed, state, victims, distance_from_closest_victim, bases, distance_from_closest_base,
                     got_victim, waypoints, waypoints_dims, tile_map_size, entity_map, path_map, timers, abandon_victim, nb_drones):
    if timers["waypoints_scan"] == WAYPOINTS_SCAN:
        timers["waypoints_scan"] = 0
        waypoints_scan(waypoints_dims, waypoints, tile_map_size, entity_map)
    else:
        timers["waypoints_scan"] += 1

    if state == State.BOOT.value:
        zone_split(tile_map_size, tile_pos, nb_drones, drone_id, waypoints, waypoints_dims)
        return State.EXPLORE.value

    elif state == State.DONE.value:
        return state

    elif state == State.EXPLORE.value:
        best_victim_index = next_victim(tile_pos, speed, victims, tile_map_size)
        if best_victim_index is not None and len(bases) != 0:
            timers["waypoints_scan"] = 0  # resets the timer for next time it explores
            target_indication["waypoint"] = undefined_waypoint
            victims[best_victim_index][2] = drone_id
            target_indication["victim"] = best_victim_index
            target[:2] = tuple(victims[best_victim_index][:2])
            return State.RESCUE.value
        else:
            if waypoint_is_defined(target_indication["waypoint"]):
                if is_explored(waypoints, target_indication["waypoint"], path, tile_map_size, entity_map):
                    waypoints[target_indication["waypoint"]] = EXPLORED_WP_VALUE
                    target_indication["waypoint"] = undefined_waypoint
                    undefine_target(target)
                    return State.EXPLORE.value
                else:
                    return State.EXPLORE.value
            else:
                target_waypoint = next_waypoint(tile_pos, speed, waypoints, waypoints_dims, tile_map_size, path_map)
                if target_waypoint is None:
                    undefine_target(target)
                    return State.DONE.value
                else:
                    target_indication["waypoint"] = target_waypoint
                    target[:2] = waypoint_pos(target_waypoint[0], target_waypoint[1])
                    return State.EXPLORE.value

    elif state == State.RESCUE.value:

        if abandon_victim[0]:
            abandon_victim[0] = False
            target_indication["victim"] = undefined_index
            undefine_target(target)
            # print(f"{drone_id}: leaving my target")
            return State.EXPLORE.value

        # actualise victim position
        target[:2] = tuple(victims[target_indication["victim"]][:2])

        believed_distance = m.dist(tile_pos[:2], target)
        if believed_distance < EXPLORED_PATH_DISTANCE:
            if timers["believe_wait"] >= BELIEVE_WAIT:
                timers["believe_wait"] = 0

                victims[target_indication["victim"]][2] = VICTIM_RESCUED_NB
                target_indication["victim"] = undefined_index
                undefine_target(target)
                return State.EXPLORE.value
            else:
                timers["believe_wait"] += 1

        if distance_from_closest_victim is not m.inf and distance_from_closest_victim <= GRAB_DISTANCE:
            timers["believe_wait"] = 0
            timers["victim_wait"] = 0
            timers["base_scan"] = 0
            target_indication["base"] = False
            return State.SAVE.value

        else:
            return State.RESCUE.value

    elif state == State.SAVE.value:

        if abandon_victim[0]:  # if told to let the victim while he was about to catch it, catch is nonetheless
            abandon_victim[0] = False
            victims[target_indication["victim"]][2] = drone_id
            # print(f"{drone_id}: I do not care, I'll catch it")

        if not target_indication["base"] and got_victim:
            target_indication["base"] = True

            if is_defined(target_indication["victim"]):  # only happens when just getting hist victim
                timers["victim_wait"] = 0
                victims_array = np.asarray(victims, dtype=np.int32)
                best_victim_index = closest_victim(target, victims_array)
                victims[target_indication["victim"]][2] = VICTIM_WAITING_NB
                target_indication["victim"] = undefined_index
                victims[best_victim_index][2] = VICTIM_RESCUED_NB

            target_base = next_base(tile_pos, speed, bases, tile_map_size, path_map, distance_from_closest_base)
            if target_base is None:
                target_indication["victim"] = undefined_index
                undefine_target(target)
                return State.EXPLORE.value  # TODO : change that ?
            else:
                target[:2] = tuple(target_base)
                return State.SAVE.value

        elif not target_indication["base"] and not got_victim:
            if timers["victim_wait"] >= VICTIM_WAIT:
                victims[target_indication["victim"]][2] = VICTIM_RESCUED_NB
                target_indication["victim"] = undefined_index
                undefine_target(target)
                return State.EXPLORE.value
            else:
                timers["victim_wait"] += 1
                return State.SAVE.value

        elif target_indication["base"] and not got_victim:  # means he lost his victim somehow, or dropped it
            undefine_target(target)
            return State.EXPLORE.value

        elif target_indication["base"] and got_victim:

            if timers["base_scan"] >= BASE_SCAN:
                timers["base_scan"] = 0
                target_indication["base"] = False
            else:
                timers["base_scan"] += 1

            return State.SAVE.value
