import numba as nb
import numpy as np
import math as m
from typing import List, Tuple

from swarm_rescue.solutions.assets.mapping.entity import add_entity, Entity
from swarm_rescue.solutions.assets.mapping.mapping_constants import INV_TILE_SIZE, DRONE_RADIUS

# region local constants
CLOSE_DRONE_DISTANCE = DRONE_RADIUS * 2.5
DEAD_THRESHOLD = 10
KILL_RADIUS = 2
# endregion


def detect_kills(detected_drones, alive_received, silent_drones, entity_map, tile_map_size):
    for drone_coord in detected_drones:
        x, y = drone_coord
        x_tile = int(x * INV_TILE_SIZE)
        y_tile = int(y * INV_TILE_SIZE)
        if entity_map[x_tile, y_tile] == Entity.NOCOM.value:
            continue
        drone_in_alive = any(m.dist(drone_coord, alive_coord[1:]) <= CLOSE_DRONE_DISTANCE for alive_coord in alive_received)
        closest_silent = next((silent_coord for silent_coord in list(silent_drones.keys()) if m.dist(drone_coord, silent_coord) <= CLOSE_DRONE_DISTANCE), None)
        if drone_in_alive:
            if closest_silent in silent_drones:
                # silent_drones[key] -= 1
                # if silent_drones[key] <= 0:
                del silent_drones[closest_silent]
        else:
            if closest_silent is not None:
                silent_drones[closest_silent] += 1
                if silent_drones[closest_silent] == DEAD_THRESHOLD:
                    add_entity(x_tile, y_tile, Entity.KILL.value, entity_map, tile_map_size, KILL_RADIUS)
            else:
                key = (int(x), int(y))
                silent_drones[key] = 1








