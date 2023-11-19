from math import sqrt
import numpy as np
import math as m
import tcod.path
import sys, os

from src.swarm_rescue.experimental.assets.Mapping.entity import Entity
from src.swarm_rescue.experimental.assets.behavior.state import State


# region local constants
WALL_WEIGHT = 6
WALL_RUNOFF = 5
BASE_WEIGHT = 5
"""
Must be < 5
"""
CLOUD_BONUS = 2
PRUDENCE = 10
KILL_RELUCTANCE = 5
# endregion

f_runoff = lambda distance, weight: round(BASE_WEIGHT + weight * (1 - (distance-1) / WALL_RUNOFF), 0)

def compute_path_map(tile_map_size, occupancy_map, entity_map, target, state):
    path_map = BASE_WEIGHT * np.ones(tile_map_size, dtype=np.int8)
    for x in range(0, tile_map_size[0]):
        for y in range(0, tile_map_size[1]):
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

    if state.value == State.SAVE or state.value == State.DROP:
        path_map[(entity_map == Entity.CLOUD.value) & (path_map != 0)] += PRUDENCE
    else:
        path_map[(entity_map == Entity.CLOUD.value) & (path_map != 0)] -= CLOUD_BONUS
    return path_map

def anticipate_pos(position, speed, tile_size, map_size):
    x = min(max(0, position[0] + round(speed[0] / tile_size, 0)), map_size[0])
    y = min(max(0, position[1] + round(speed[1] / tile_size, 0)), map_size[1])
    return np.array([x, y], dtype=np.int8)

def find_path(tile_map_size, path_map, drone_pos, target_pos, drone_speed, tile_size):
    graph = tcod.path.SimpleGraph(cost=path_map, cardinal=5, diagonal=7)
    pf = tcod.path.Pathfinder(graph)
    pf.add_root(anticipate_pos(drone_pos, drone_speed, tile_size, tile_map_size))
    return pf.path_to(target_pos)[1:-1]