import math as m
from typing import Tuple

from swarm_rescue.solutions.assets.behavior.anti_kill_zone import CLOSE_DRONE_DISTANCE
from swarm_rescue.solutions.assets.behavior.think import VICTIM_WAITING_NB, VICTIM_RESCUED_NB
from swarm_rescue.solutions.assets.mapping.gridFunctions import add_value_along_line
from swarm_rescue.solutions.assets.mapping.lidarMapping import THRESHOLD_MAX
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE, VICTIM_RADIUS
from swarm_rescue.solutions.assets.mapping.entity import Entity, add_entity
from swarm_rescue.solutions.assets.behavior.state import State
from swarm_rescue.spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor

# region local constants
VICTIM_DETECTION_MARGIN = m.ceil(3 * VICTIM_RADIUS / TILE_SIZE) + 1
BASE_DETECTION_MARGIN = 0.5
MIN_SEMANTIC_DISTANCE = 50
DRONE_DETECTION_MARGIN = CLOSE_DRONE_DISTANCE * 1.1
DRONE_TRACE_DETECTION_MARGIN = CLOSE_DRONE_DISTANCE / 5
CLEAR_VALUE = - THRESHOLD_MAX / 3
DRONE_IN_MEMORY_CYCLES = 8


# endregion


def process_semantic(semantic_values, pos, tile_map_size, victims, state, bases, entity_map, map_size, occupancy_map, victim_angle, detected_drones, detected_drones_traces,
                     in_noCOMzone, dead_drones):
    distance_from_closest_victim = m.inf
    distance_from_closest_base = m.inf
    distance_from_closest_alive_drone = m.inf
    detected_drones[:] = list()
    new_detected_drones_traces = {(x, y): count - 1 for (x, y), count in detected_drones_traces.items() if count > 1}
    detected_drones_traces.clear()
    detected_drones_traces.update(new_detected_drones_traces)

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
                    victim_angle[0] = data.angle
                # clear_view(pos, target_x, target_y, occupancy_map, tile_map_size, map_size)
                occupancy_map[tile_target_x, tile_target_y] = 1  # WIP

            elif data.entity_type.value == DroneSemanticSensor.TypeEntity.RESCUE_CENTER.value:
                if min([m.dist(base, [tile_target_x, tile_target_y]) for base in bases] + [m.inf]) > BASE_DETECTION_MARGIN:
                    bases.append((tile_target_x, tile_target_y))
                    add_entity(tile_target_x, tile_target_y, Entity.BASE.value, entity_map, tile_map_size)
                if data.distance < distance_from_closest_base:
                    distance_from_closest_base = data.distance
                # if data.distance > MIN_SEMANTIC_DISTANCE:
                #     clear_view(pos, target_x, target_y, occupancy_map, tile_map_size, map_size)

            elif data.entity_type.value == DroneSemanticSensor.TypeEntity.DRONE.value:
                closest_detected = next((drone for drone in list(detected_drones_traces.keys()) if m.dist((target_x, target_y), drone) <= DRONE_TRACE_DETECTION_MARGIN),
                                        (target_x, target_y))
                detected_drones_traces[closest_detected] = DRONE_IN_MEMORY_CYCLES * (1 + in_noCOMzone * 2)
                if min([m.dist(drone, [target_x, target_y]) for drone in dead_drones] + [m.inf]) > DRONE_DETECTION_MARGIN:
                    if min([m.dist(drone, [target_x, target_y]) for drone in detected_drones] + [m.inf]) > DRONE_DETECTION_MARGIN:
                        detected_drones.append((target_x, target_y))
                    if data.distance < distance_from_closest_alive_drone and entity_map[tile_target_x, tile_target_y] != Entity.NOCOM.value:
                        distance_from_closest_alive_drone = data.distance
            clear_view(pos, target_x, target_y, occupancy_map, tile_map_size, map_size)

    return distance_from_closest_victim, distance_from_closest_base, distance_from_closest_alive_drone


def clear_view(pos: Tuple, target_x, target_y, occupancy_map, tile_map_size, map_size):
    add_value_along_line(occupancy_map, map_size, TILE_SIZE, tile_map_size, pos[0], pos[1], target_x, target_y, CLEAR_VALUE)
    # vector = np.array(pos[:2]) - np.array((tile_target_x, tile_target_y)) * TILE_SIZE
    # distance = np.linalg.norm(vector)
    #
    # if distance < MIN_SEMANTIC_DISTANCE:
    #     return  # Skip updating maps if distance is below the minimum semantic distance
    #
    # vector /= distance
    # for t in np.arange(0, VICTIM_DETECTION_MARGIN * TILE_SIZE / 2, TILE_SIZE // 4):
    #     new_x = tile_target_x + int(t * vector[0]) // TILE_SIZE
    #     new_y = tile_target_y + int(t * vector[1]) // TILE_SIZE
    #     p = (new_x, new_y)
    #
    #     if entity_map[p] not in [Entity.NOGPS.value, Entity.NOCOM.value, Entity.KILL.value, Entity.BASE.value]:
    #         add_entity(tile_target_x, tile_target_y, Entity.VOID.value, entity_map, tile_map_size)
    #     occupancy_map[p] = -1  # WIP
