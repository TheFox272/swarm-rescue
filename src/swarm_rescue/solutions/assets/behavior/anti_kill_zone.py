import numba as nb
import numpy as np
import math as m
from typing import List, Tuple

from swarm_rescue.solutions.assets.mapping.entity import add_entity, Entity
from swarm_rescue.solutions.assets.mapping.mapping_constants import INV_TILE_SIZE, DRONE_RADIUS

# region local constants
CLOSE_DRONE_DISTANCE = DRONE_RADIUS * 2.3
DEAD_THRESHOLD = 8
KILL_RADIUS = 3
# endregion


def detect_kills(detected_drones, alive_received, silent_drones, dead_drones, entity_map, tile_map_size):
    for drone_coord in detected_drones:
        x, y = drone_coord
        x_tile = max(0, min(int(x * INV_TILE_SIZE), tile_map_size[0] - 1))
        y_tile = max(0, min(int(y * INV_TILE_SIZE), tile_map_size[1] - 1))
        if entity_map[x_tile, y_tile] == Entity.NOCOM.value:
            continue
        drone_in_alive = any(m.dist(drone_coord, alive_coord[1:]) <= CLOSE_DRONE_DISTANCE for alive_coord in alive_received)
        closest_silent = next((silent_coord for silent_coord in list(silent_drones.keys()) if m.dist(drone_coord, silent_coord) <= CLOSE_DRONE_DISTANCE), None)
        if drone_in_alive:
            if closest_silent in silent_drones:
                del silent_drones[closest_silent]
        else:
            key = (int(x), int(y))
            if closest_silent is not None:
                silent_drones[closest_silent] += 1
                if silent_drones[closest_silent] == DEAD_THRESHOLD:
                    add_entity(x_tile, y_tile, Entity.KILL.value, entity_map, tile_map_size, KILL_RADIUS)
                    if (x_tile, y_tile) not in dead_drones:
                        dead_drones.append(key)
            else:
                silent_drones[key] = 1








