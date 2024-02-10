import math as m

import numpy as np

from swarm_rescue.solutions.assets.behavior.think import VICTIM_WAITING_NB, VICTIM_RESCUED_NB
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE, VICTIM_RADIUS
from swarm_rescue.solutions.assets.movement.pathfinding import BASE_WEIGHT, WALL_WEIGHT
from swarm_rescue.solutions.assets.mapping.entity import Entity, add_entity
from swarm_rescue.solutions.assets.behavior.state import State
from swarm_rescue.spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor

# region local constants
VICTIM_DETECTION_MARGIN = m.ceil(3 * VICTIM_RADIUS / TILE_SIZE) + 1
BASE_DETECTION_MARGIN = 1
MIN_SEMANTIC_DISTANCE = 5
DRONE_ENTITY_RADIUS = 1


# endregion


def process_semantic(semantic_values, pos, tile_map_size, victims, state, bases, entity_map, path_map, occupancy_map, victim_angle):
    distance_from_closest_victim = m.inf
    victim_angle = victim_angle
    distance_from_closest_base = m.inf
    distance_from_closest_drone = m.inf

    if semantic_values is not None and len(semantic_values) != 0 and not m.isnan(semantic_values[0].distance):

        for data in semantic_values:
            ray_angle = data.angle + pos[2]
            target_x = pos[0] + m.cos(ray_angle) * data.distance
            target_y = pos[1] + m.sin(ray_angle) * data.distance
            tile_target_x = max(0, min(int(round(target_x / TILE_SIZE, 0)), tile_map_size[0] - 1))
            tile_target_y = max(0, min(int(round(target_y / TILE_SIZE, 0)), tile_map_size[1] - 1))

            if data.entity_type.value == DroneSemanticSensor.TypeEntity.WOUNDED_PERSON.value and data.grasped is False:
                new_victim = True
                for victim in victims:
                    if m.dist(victim[:2], [tile_target_x, tile_target_y]) < VICTIM_DETECTION_MARGIN:
                        new_victim = False
                        victim[0] = int((victim[0] + tile_target_x) / 2)
                        victim[1] = int((victim[1] + tile_target_y) / 2)
                        if victim[2] == VICTIM_RESCUED_NB:  # possible if another victim is pushed into a former one position
                            victim[2] = VICTIM_WAITING_NB
                if new_victim:
                    victims.append([tile_target_x, tile_target_y, VICTIM_WAITING_NB])
                if state == State.RESCUE.value and data.distance < distance_from_closest_victim:
                    distance_from_closest_victim = data.distance
                    victim_angle = data.angle

                occupancy_map[tile_target_x, tile_target_y] = 1  # WIP
                clear_view(pos, tile_target_x, tile_target_y, entity_map, occupancy_map, tile_map_size)

            elif data.entity_type.value == DroneSemanticSensor.TypeEntity.RESCUE_CENTER.value:
                if min([m.dist(base, [tile_target_x, tile_target_y]) for base in bases] + [m.inf]) > BASE_DETECTION_MARGIN:
                    bases.append([tile_target_x, tile_target_y])
                    add_entity(tile_target_x, tile_target_y, Entity.BASE.value, entity_map, tile_map_size)
                if data.distance < distance_from_closest_base:
                    distance_from_closest_base = data.distance

            elif data.entity_type.value == DroneSemanticSensor.TypeEntity.DRONE.value:
                if data.distance < distance_from_closest_drone:
                    distance_from_closest_drone = data.distance
                occupancy_map[tile_target_x, tile_target_y] = 1  # WIP
                add_entity(tile_target_x, tile_target_y, Entity.DRONE.value, entity_map, tile_map_size, DRONE_ENTITY_RADIUS)
                clear_view(pos, tile_target_x, tile_target_y, entity_map, occupancy_map, tile_map_size)

    return distance_from_closest_victim, victim_angle, distance_from_closest_base, distance_from_closest_drone


def clear_view(pos, tile_target_x, tile_target_y, entity_map, occupancy_map, tile_map_size):
    """
    TODO: utiliser fonction de Louis pour faire plus propre
    """
    vector = np.array(pos[:2]) - np.array((tile_target_x, tile_target_y)) * TILE_SIZE
    distance = np.linalg.norm(vector)

    if distance < MIN_SEMANTIC_DISTANCE:
        return  # Skip updating maps if distance is below the minimum semantic distance

    vector /= distance
    for t in np.arange(0, VICTIM_DETECTION_MARGIN * TILE_SIZE / 2, TILE_SIZE // 4):
        new_x = tile_target_x + int(t * vector[0]) // TILE_SIZE
        new_y = tile_target_y + int(t * vector[1]) // TILE_SIZE
        p = (new_x, new_y)

        if entity_map[p] not in [Entity.NOGPS.value, Entity.NOCOM.value, Entity.KILL.value, Entity.BASE.value]:
            add_entity(tile_target_x, tile_target_y, Entity.VOID.value, entity_map, tile_map_size)
        occupancy_map[p] = -1  # WIP
