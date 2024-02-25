import numba as nb
import numpy as np
import math as m
from typing import List, Tuple

from swarm_rescue.solutions.assets.mapping.entity import add_entity, Entity
from swarm_rescue.solutions.assets.mapping.mapping_constants import INV_TILE_SIZE, DRONE_RADIUS

# region local constants
SILENT_FORGET_FACTOR = 4  # the higher, the longer it takes to forget. Must be > 1
CLOSE_DRONE_DISTANCE = DRONE_RADIUS * 2.2
DEAD_THRESHOLD = 10
DEAD_THRESHOLD *= SILENT_FORGET_FACTOR - 1
KILL_RADIUS = 3
NO_DEAD_RADIUS = 3
# endregion


def detect_kills(detected_drones, alive_received, silent_drones, dead_drones, entity_map, tile_map_size):

    new_silent_drones = dict()
    for (x, y), count in silent_drones.items():
        new_count = count - 1
        new_silent_drones[(x, y)] = new_count
        if new_count == 0 and (x, y) in dead_drones:
            dead_drones.remove((x, y))
    silent_drones.clear()
    silent_drones.update(new_silent_drones)

    for drone_coord in detected_drones:
        x, y = drone_coord
        x_tile = max(0, min(int(x * INV_TILE_SIZE), tile_map_size[0] - 1))
        y_tile = max(0, min(int(y * INV_TILE_SIZE), tile_map_size[1] - 1))
        if entity_map[x_tile, y_tile] == Entity.NOCOM.value or entity_map[x_tile, y_tile] == Entity.NOGPS.value:
            continue

        drone_in_dead = any(m.dist(drone_coord, dead) <= CLOSE_DRONE_DISTANCE for dead in dead_drones)
        drone_in_alive = any(m.dist(drone_coord, alive[1:]) <= CLOSE_DRONE_DISTANCE for alive in alive_received)
        if drone_in_dead and not drone_in_alive:
            continue

        closest_silent = next((silent for silent in list(silent_drones.keys()) if m.dist(drone_coord, silent) <= CLOSE_DRONE_DISTANCE), None)
        if drone_in_alive:
            if closest_silent is not None:
                silent_drones.pop(closest_silent, None)
        else:
            if closest_silent is not None:
                silent_drones[closest_silent] += SILENT_FORGET_FACTOR
                if silent_drones[closest_silent] >= DEAD_THRESHOLD:
                    add_entity(x_tile, y_tile, Entity.KILL.value, entity_map, tile_map_size, KILL_RADIUS)
                    dead_drones.append(closest_silent)
            else:
                key = (int(x), int(y))
                silent_drones[key] = SILENT_FORGET_FACTOR
