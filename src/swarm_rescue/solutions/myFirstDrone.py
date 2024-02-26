import math as m
import os
import sys
from typing import Optional

from time import time  # to test

import arcade  # for drawing
import numpy as np
from scipy.spatial.distance import cdist

from swarm_rescue.solutions.assets.behavior.anti_kill_zone import detect_kills, CLOSE_DRONE_DISTANCE
from swarm_rescue.solutions.assets.communication.comm_declarations import MsgType
from swarm_rescue.solutions.assets.communication.comm_manager import compute_received_com, create_msg, DONE_DRONE_ID
from swarm_rescue.solutions.assets.mapping.entity import Entity, add_entity
from swarm_rescue.solutions.assets.movement.avoid import slow_down
from swarm_rescue.solutions.assets.movement.stuck import stuck_manager

# This line add, to sys.path, the path to parent path of this file
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from swarm_rescue.spg_overlay.entities.drone_abstract import DroneAbstract
from swarm_rescue.spg_overlay.utils.misc_data import MiscData
from swarm_rescue.solutions.assets.movement.pathfinding import find_path, compute_path_map, anticipate_pos, DRONE_ZONE
from swarm_rescue.solutions.assets.movement.control import compute_command
from swarm_rescue.solutions.assets.mapping.lidarMapping import process_lidar
from swarm_rescue.solutions.assets.mapping.semanticMapping import process_semantic, DRONE_IN_MEMORY_CYCLES
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE
from swarm_rescue.solutions.assets.behavior.state import State
from swarm_rescue.solutions.assets.behavior.map_split import zone_split, waypoint_pos, init_zone_split
from swarm_rescue.solutions.assets.movement.pathfinding import BASIC_WEIGHT, WALL_WEIGHT
from swarm_rescue.solutions.assets.behavior.think import compute_behavior, is_defined, undefined_index, undefined_target, undefined_waypoint, target_is_defined, NOGPS_WAYPOINT

# region local constants
SHARE_WAYPOINTS_WAIT = 64
SHARE_BASES_WAIT = 16
SHARE_OCCUPANCY_WAIT = 16
SHARE_ENTITY_WAIT = 16
SHARE_VICTIMS_WAIT = 4
CONFIDENCE_RADIUS = 3
SAFE_CONFIDENCE_RADIUS = 3

NO_SAFE_ON_BASE_DISTANCE = TILE_SIZE * 4

noGPS_time_limit = 80


# endregion


class MyFirstDrone(DroneAbstract):
    def __init__(self,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        # region basic init
        super().__init__(identifier=identifier,
                         misc_data=misc_data,
                         **kwargs)
        self.drone_id = identifier
        # endregion
        # region pathfinding init
        self.pos = np.zeros(3, dtype=np.float64)
        """
        The position vector of the drone, third term being its angle.
        """
        self.speed = np.zeros(3, dtype=np.float32)
        """
        The speed vector of the drone, third term being its angle.
        """
        self.target = undefined_target.copy()
        """
        The current drone's target, where is wants to move.
        """
        self.map_size = tuple(misc_data.size_area)
        """
        The map size, in pixels.
        """
        self.tile_map_size = tuple([np.int64(size / TILE_SIZE) for size in self.map_size])
        """
        The map size, in tiles.
        
        :type: tuple[int32, int32]
        """
        self.occupancy_map = np.zeros(self.tile_map_size, dtype=np.float32)
        """
        The array on which the drone's LIDAR acts. A positive value on a tile means there is a wall there, a negative value means there isn't.
        
        :type: ndarray of float32
        """
        self.path_map = np.ones(self.tile_map_size, dtype=np.int64)
        """
        The array, mainly computed from :py:attr:`occupancy_map`, that the drone uses to find its path. A number n on a tile means that this tile costs n to go 
        through.
        
        :type: ndarray of int32
        """
        self.entity_map = np.zeros(self.tile_map_size, dtype=np.int32)
        """
        The array corresponding to the entities present on each tile.
        
        :type: ndarray of int32
        """
        # self.trust_map = np.zeros(self.tile_map_size, dtype=np.int32)
        # """
        # WIP
        # """
        self.path = list()
        """
        The current path that the drone is following.
        
        :type: list of points
        """
        self.slowdown = False
        # endregion
        # region mapping init
        self.victims = list()
        """
        The list of the detected victims. Each one of this elements is of the form [tile_x, tile_y, savior]. The savior corresponds to the id of the drone 
        currently rescuing it, :py:const:`~swarm_rescue.solutions.assets.behavior.think.VICTIM_WAITING_NB` if it is an available victim, 
        and :py:const:`~swarm_rescue.solutions.assets.behavior.think.VICTIM_WAITING_NB` if it is an already rescued victim

        :type: list of 3-elements lists
        """
        self.bases = list()
        """
        The list of the detected base tiles. Each one of the list's elements is of the form [tile_x, tile_y].

        :type: list of 2-elements lists
        """
        self.detected_drones = list()
        self.detected_drones_traces = dict()
        self.silent_drones = dict()
        self.dead_drones = list()
        # endregion
        # region behavior init
        self.state = State.BOOT.value
        # if self.id not in [1, 2]:
        #     self.state = State.DONE.value
        """
        The int value corresponding to the state of the drone, among :py:class:`~swarm_rescue.solutions.assets.behavior.state.State`.
        
        :type: State
        """
        self.nb_drones = misc_data.number_drones
        self.waypoints_dims = np.empty(2, dtype=np.int32)
        self.waypoints = init_zone_split(self.tile_map_size, self.waypoints_dims)
        self.target_indication = {"victim": undefined_index, "waypoint": undefined_waypoint, "base": False}
        self.victims = list()
        self.distance_from_closest_victim = m.inf
        self.distance_from_closest_base = m.inf
        self.distance_from_closest_alive_drone = m.inf
        self.victim_angle = np.array([0], dtype=np.float32)
        self.timers = {"waypoints_scan": 0, "victim_wait": 0, "believe_wait": 0, "base_scan": 0, "noGPS": 0, "stuck": 0}
        # endregion
        # region communication
        self.msg_sent = np.zeros(1, dtype=np.int32)
        """
        The number of messages sent by this drone.
        
        :type: int
        """
        self.to_send = list()
        """
        The stack of messages that the drone need to send at the end of a cycle
        
        :type: list of messages, see *WIP*
        """
        self.alive_received = list()
        """
        The list of the alive ids and positions that the other drones sent during a cycle. Each one of the list's elements is of the form [id, tile_x, tile_y].
 
        :type: list of 3-elements list
        """
        self.processed_msg = [[]] * self.nb_drones
        """
        A list you can find the list of messages sent by other drones and processed by this drone.
        
        :type: List[List[int]]
        """
        self.comm_timers = {"share_waypoints": 0, "share_bases": 0, "share_occupancy": 0, "share_entity": 0, "share_victims": 0}
        # we add a small difference in the transmission timers so that every drone does not share its map at the same time, allowing their info to spread
        self.comm_timers["share_waypoints"] = (self.comm_timers["share_waypoints"] + self.drone_id) % SHARE_WAYPOINTS_WAIT
        self.comm_timers["share_bases"] = (self.comm_timers["share_bases"] + self.drone_id) % SHARE_BASES_WAIT
        self.comm_timers["share_occupancy"] = (self.comm_timers["share_occupancy"] + self.drone_id) % SHARE_OCCUPANCY_WAIT
        self.comm_timers["share_entity"] = (self.comm_timers["share_entity"] + self.drone_id) % SHARE_ENTITY_WAIT
        self.comm_timers["share_victims"] = (self.comm_timers["share_victims"] + self.drone_id) % SHARE_VICTIMS_WAIT
        self.abandon_victim = np.zeros(1, dtype=bool)
        """
        Is true if other drones told him to fuck off, they are already rescuing his victim
        """
        # endregion
        # region drawing (to comment out for eval)
        self.draw_id = False
        self.draw_path = False
        self.draw_path_map = False
        self.draw_entity_map = False
        self.draw_grid = False
        self.draw_waypoints = False
        # endregion

        # region testing (to comment out for eval)
        self.cycle = 0
        # endregion

    @property
    def in_noGPSzone(self):
        return self.measured_gps_position() is None

    @property
    def in_noCOMzone(self):
        return not m.isinf(self.distance_from_closest_alive_drone) and len(self.alive_received) == 0

    @property
    def in_SAFEzone(self):
        return not self.in_noGPSzone and not self.in_noCOMzone

    @property
    def got_victim(self):
        return len(self.grasped_entities()) != 0

    @property
    def is_dead(self):
        return self.odometer_values() is None

    @property
    def tile_pos(self):
        return (self.pos[0] / TILE_SIZE + 0.5).astype(np.int32), (self.pos[1] / TILE_SIZE + 0.5).astype(np.int32), self.pos[2]

    def update_position(self):
        if self.odometer_values() is not None:
            self.speed[0] = self.odometer_values()[0] * m.cos(self.pos[2] + self.odometer_values()[1])
            self.speed[1] = self.odometer_values()[0] * m.sin(self.pos[2] + self.odometer_values()[1])
            self.speed[2] = self.odometer_values()[2]
        else:
            self.speed[0] = 0
            self.speed[1] = 0
            self.speed[2] = 0
        if not self.in_noGPSzone:
            if not m.isnan(self.measured_gps_position()[0]):
                self.pos[0] = self.measured_gps_position()[0] + self.map_size[0] / 2
                self.pos[1] = self.measured_gps_position()[1] + self.map_size[1] / 2
                self.pos[2] = self.measured_compass_angle()
        else:
            self.pos[0] += self.speed[0]
            self.pos[1] += self.speed[1]
            self.pos[2] += self.speed[2]

    def define_message_for_all(self):

        self.update_position()
        self.to_send = []
        self.alive_received = []

        if self.is_dead:
            return None
        elif self.state == State.DONE.value:
            self.to_send.append(create_msg(MsgType.ALIVE.value, 'all', [DONE_DRONE_ID, self.pos[0], self.pos[1]], self.drone_id, self.msg_sent))
        else:
            if self.in_noGPSzone:
                self.timers['noGPS'] += 1
            else:
                self.timers['noGPS'] = 0
            if self.timers['noGPS'] < noGPS_time_limit:
                self.comm_timers["share_waypoints"] += 1
                if "share_bases" in self.comm_timers:
                    self.comm_timers["share_bases"] += 1
                self.comm_timers["share_occupancy"] += 1
                self.comm_timers["share_entity"] += 1
                if len(self.victims) != 0:
                    self.comm_timers["share_victims"] += 1
            else:
                self.comm_timers["share_waypoints"] -= SHARE_WAYPOINTS_WAIT
                self.comm_timers["share_occupancy"] -= SHARE_OCCUPANCY_WAIT
                self.comm_timers["share_entity"] -= SHARE_ENTITY_WAIT

            self.to_send.append(create_msg(MsgType.ALIVE.value, 'all', [self.drone_id, self.pos[0], self.pos[1]], self.drone_id, self.msg_sent))
            if self.comm_timers["share_waypoints"] > SHARE_WAYPOINTS_WAIT:
                self.to_send.append(create_msg(MsgType.SHARE_WAYPOINTS.value, 'all', self.waypoints, self.drone_id, self.msg_sent))
                self.comm_timers["share_waypoints"] = 0
            if "share_bases" in self.comm_timers and self.comm_timers["share_bases"] > SHARE_BASES_WAIT:
                self.to_send.append(create_msg(MsgType.SHARE_BASES.value, 'all', self.bases, self.drone_id, self.msg_sent))
                del self.comm_timers["share_bases"]
            if self.comm_timers["share_occupancy"] > SHARE_OCCUPANCY_WAIT:
                self.to_send.append(create_msg(MsgType.SHARE_OCCUPANCY_MAP.value, 'all', self.occupancy_map, self.drone_id, self.msg_sent))
                self.comm_timers["share_occupancy"] = 0
            if self.comm_timers["share_entity"] > SHARE_ENTITY_WAIT:
                self.to_send.append(create_msg(MsgType.SHARE_ENTITY_MAP.value, 'all', self.entity_map, self.drone_id, self.msg_sent))
                self.comm_timers["share_entity"] = 0
            if self.comm_timers["share_victims"] > SHARE_VICTIMS_WAIT:
                self.to_send.append(create_msg(MsgType.SHARE_VICTIMS.value, 'all', self.victims, self.drone_id, self.msg_sent))
                self.comm_timers["share_victims"] = 0

            compute_received_com(self.communicator.received_messages, self.to_send, self.alive_received, self.processed_msg, self.abandon_victim, self.drone_id, self.waypoints,
                                 self.bases, self.occupancy_map, self.entity_map, self.tile_map_size, self.victims, self.in_noGPSzone, self.dead_drones)

        return self.to_send

    def control(self):
        """
        How the drone moves
        """
        if self.is_dead or self.state == State.DONE.value:
            if self.is_dead:
                add_entity(self.tile_pos[0], self.tile_pos[1], Entity.KILL.value, self.entity_map, self.tile_map_size, SAFE_CONFIDENCE_RADIUS)
            return None

        self.slowdown = False

        self.occupancy_map = process_lidar(self.occupancy_map, self.map_size, TILE_SIZE, self.tile_map_size, self.lidar_values(), self.lidar_rays_angles(),
                                           self.pos[:2], self.pos[2])

        (self.distance_from_closest_victim, self.distance_from_closest_base,
         self.distance_from_closest_alive_drone) = process_semantic(self.semantic_values(), self.pos, self.tile_map_size, self.victims, self.state, self.bases, self.entity_map,
                                                                    self.map_size, self.occupancy_map, self.victim_angle, self.detected_drones, self.detected_drones_traces,
                                                                    self.in_noCOMzone, self.dead_drones)

        if self.in_SAFEzone:
            if self.distance_from_closest_base > NO_SAFE_ON_BASE_DISTANCE:
                add_entity(self.tile_pos[0], self.tile_pos[1], Entity.SAFE.value, self.entity_map, self.tile_map_size, SAFE_CONFIDENCE_RADIUS)
            else:
                add_entity(self.tile_pos[0], self.tile_pos[1], Entity.SAFE.value, self.entity_map, self.tile_map_size, 0)
        elif self.state != State.BOOT.value:
            add_entity(self.tile_pos[0], self.tile_pos[1], Entity.VOID.value, self.entity_map, self.tile_map_size, SAFE_CONFIDENCE_RADIUS)
            if self.in_noGPSzone:
                add_entity(self.tile_pos[0], self.tile_pos[1], Entity.NOGPS.value, self.entity_map, self.tile_map_size, CONFIDENCE_RADIUS)
            if self.in_noCOMzone:
                add_entity(self.tile_pos[0], self.tile_pos[1], Entity.NOCOM.value, self.entity_map, self.tile_map_size, CONFIDENCE_RADIUS)

        stuck_manager(self.timers, self.speed, self.entity_map, self.path)

        if self.timers['noGPS'] < noGPS_time_limit:
            detect_kills(self.detected_drones, self.alive_received, self.silent_drones, self.dead_drones, self.entity_map, self.tile_map_size, self.pos[:2])

        detected_drones_traces_array = np.asarray(list(self.detected_drones_traces.keys()), dtype=np.float32)
        detected_drones_traces_array = detected_drones_traces_array.reshape(-1, 2)
        self.path_map = compute_path_map(self.tile_map_size, self.occupancy_map, self.entity_map, self.state, self.target, detected_drones_traces_array)

        self.state = compute_behavior(self.drone_id, self.target, self.target_indication, self.tile_pos, self.path, self.speed, self.state, self.victims,
                                      self.distance_from_closest_victim, self.bases, self.distance_from_closest_base, self.got_victim, self.waypoints, self.waypoints_dims,
                                      self.tile_map_size, self.entity_map, self.path_map, self.timers, self.abandon_victim, self.nb_drones, self.in_noGPSzone)
        if self.abandon_victim[0]:
            # print(f"{self.drone_id}: ohoh, there is an issue with self.abandon_victim")
            self.abandon_victim[0] = False

        if target_is_defined(self.target):
            self.path = find_path(self.path_map, self.tile_pos[:2], self.target)
            self.slowdown = slow_down(self.pos[:2], self.path, self.detected_drones, self.dead_drones)

        # region testing
        if self.state == State.DONE.value:
            if self.cycle != -1:
                print("Cycles écoulés:", self.cycle, "cycles")
                self.cycle = -1
        else:
            self.cycle += 1

        # endregion

        command = compute_command(self.path, self.path_map, self.tile_pos, self.state, self.victim_angle, self.distance_from_closest_base, self.slowdown)
        return command

    def draw_top_layer(self):  # (to comment out for eval)

        x, y = self.true_position()
        x = x + self.map_size[0] / 2
        y = y + self.map_size[1] / 2

        wall_color = (218, 68, 231)
        gradation_color = (255, 155, 5)
        base_color = (148, 228, 18)
        void_color = (118, 183, 223)
        safe_color = (118, 183, 170)
        cloud_color = (220, 220, 220)
        path_color = (45, 140, 251)
        drone_color = (0, 148, 66)
        waypoint_color = (0, 208, 61)
        waypoint_color_2 = (0, 108, 61)
        waypoint_color_nogps = (10, 10, 10)
        kill_color = (255, 0, 0)
        nocom_color = (255, 255, 0)
        nogps_color = (200, 200, 200)

        if self.draw_entity_map:  # displays the map
            for i in range(0, self.tile_map_size[0]):
                for j in range(0, self.tile_map_size[1]):
                    match self.entity_map[i, j]:
                        case Entity.CLOUD.value:
                            color = cloud_color
                        case Entity.VOID.value:
                            color = void_color
                        case Entity.BASE.value:
                            color = base_color
                        case Entity.WALL.value:
                            color = wall_color
                        case Entity.NOCOM.value:
                            color = nocom_color
                        case Entity.KILL.value:
                            color = kill_color
                        case Entity.NOGPS.value:
                            color = nogps_color
                        case Entity.SAFE.value:
                            color = safe_color
                    if (i, j) in self.detected_drones:
                        color = drone_color
                    arcade.draw_rectangle_filled(i * TILE_SIZE + TILE_SIZE / 2, j * TILE_SIZE + TILE_SIZE / 2, TILE_SIZE, TILE_SIZE, color)

        if self.draw_path_map:  # displays the path_map
            maxi = np.amax(self.path_map)
            for i in range(0, self.tile_map_size[0]):
                for j in range(0, self.tile_map_size[1]):
                    if self.path_map[i, j] != BASIC_WEIGHT:
                        if self.path_map[i, j] == 0:
                            color = wall_color
                        else:
                            t = 1 - (self.path_map[i, j] - BASIC_WEIGHT) / maxi
                            color = (1 - t) * np.array(gradation_color) + t * np.array(void_color)  # gradation
                        arcade.draw_rectangle_filled(i * TILE_SIZE + TILE_SIZE / 2, j * TILE_SIZE + TILE_SIZE / 2, TILE_SIZE, TILE_SIZE, color)
            # draw_map(self.occupancy_map, TILE_SIZE, self.map_size, self.pos[:2], self.pos[2])

        if self.draw_waypoints:  # displays the waypoints
            for i in range(self.waypoints_dims[0]):
                for j in range(self.waypoints_dims[1]):
                    if self.waypoints[i, j] != 0:
                        if self.waypoints[i, j] == 1:
                            color = waypoint_color
                        elif self.waypoints[i, j] == NOGPS_WAYPOINT:
                            color = waypoint_color_nogps
                        else:
                            color = waypoint_color_2
                        pos = waypoint_pos(i, j)
                        arcade.draw_rectangle_filled((pos[0]) * TILE_SIZE + TILE_SIZE / 2, pos[1] * TILE_SIZE + TILE_SIZE / 2, TILE_SIZE, TILE_SIZE, color)

        if self.draw_path:  # displays the path
            for p in self.path:
                color = path_color
                arcade.draw_rectangle_filled(p[0] * TILE_SIZE + TILE_SIZE / 2, p[1] * TILE_SIZE + TILE_SIZE / 2, TILE_SIZE, TILE_SIZE, color)

        if self.draw_grid:  # displays the grid
            for i in range(0, self.tile_map_size[0]):
                arcade.draw_line(i * TILE_SIZE, 0, i * TILE_SIZE, self.map_size[1], arcade.color.BLACK, 1)
            for i in range(0, self.tile_map_size[1]):
                arcade.draw_line(0, i * TILE_SIZE, self.map_size[0], i * TILE_SIZE, arcade.color.BLACK, 1)

        if self.draw_id:
            xa, ya = anticipate_pos(self.tile_pos, self.speed, self.tile_map_size, self.entity_map)
            xa = xa * TILE_SIZE
            ya = ya * TILE_SIZE
            arcade.draw_text("o", xa, ya, arcade.color.RED, 15)

            arcade.draw_text(str(self.drone_id), x + 15, y - 15, arcade.color.BLACK, 15)  # draw id
