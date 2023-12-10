import math as m

import numpy as np

from swarm_rescue.solutions.assets.behavior.think import VICTIM_WAITING_NB
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE, VICTIM_RADIUS
from swarm_rescue.solutions.assets.movement.pathfinding import BASE_WEIGHT
from swarm_rescue.solutions.assets.mapping.entity import Entity
from swarm_rescue.solutions.assets.behavior.state import State
from swarm_rescue.spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor


def process_semantic(semantic_values, pos, tile_map_size, victims, state, bases, entity_map, path_map, occupancy_map, victim_angle):
    distance_from_closest_victim = m.inf
    victim_angle = victim_angle
    distance_from_closest_base = m.inf

    if len(semantic_values) != 0 and not m.isnan(semantic_values[0].distance):

        for data in semantic_values:
            ray_angle = data.angle + pos[2]
            target_x = pos[0] + m.cos(ray_angle) * data.distance
            target_y = pos[1] + m.sin(ray_angle) * data.distance
            tile_target_x = max(0, min(int(round(target_x / TILE_SIZE, 0)), tile_map_size[0] - 1))
            tile_target_y = max(0, min(int(round(target_y / TILE_SIZE, 0)), tile_map_size[1] - 1))

            if data.entity_type.value == DroneSemanticSensor.TypeEntity.WOUNDED_PERSON.value:
                margin = m.ceil(4 * VICTIM_RADIUS / TILE_SIZE) + 1
                if data.grasped is False :
                    new_victim = True
                    for victim in victims:
                        if m.dist(victim[:2], [tile_target_x, tile_target_y]) < margin:
                            new_victim = False
                            victim[0] = (victim[0] + tile_target_x) / 2
                            victim[1] = (victim[1] + tile_target_y) / 2
                    if new_victim:
                        victims.append([tile_target_x, tile_target_y, VICTIM_WAITING_NB])
                    if state == State.RESCUE.value and data.distance < distance_from_closest_victim:
                        distance_from_closest_victim = data.distance
                        victim_angle = data.angle

                vector = np.array(pos[:2]) - np.array((tile_target_x, tile_target_y)) * TILE_SIZE
                vector /= np.linalg.norm(vector)
                for p in set([(tile_target_x + int(t * vector[0]) // TILE_SIZE, tile_target_y + int(t * vector[1]) // TILE_SIZE)
                              for t in np.arange(0, margin * TILE_SIZE / 2, TILE_SIZE // 4)]):
                    entity_map[p] = Entity.VOID.value
                    occupancy_map[p] = -1  # to change
                    path_map[p] = BASE_WEIGHT

            elif data.entity_type.value == DroneSemanticSensor.TypeEntity.RESCUE_CENTER.value:
                margin = 2
                if min([m.dist(base, [tile_target_x, tile_target_y]) for base in bases] + [m.inf]) > margin:
                    bases.append([tile_target_x, tile_target_y])
                    entity_map[tile_target_x, tile_target_y] = Entity.BASE.value
                if data.distance < distance_from_closest_base:
                    distance_from_closest_base = data.distance

    return distance_from_closest_victim, victim_angle, distance_from_closest_base
