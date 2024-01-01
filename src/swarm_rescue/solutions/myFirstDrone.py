import math
import random
from typing import Optional
import math as m
import numpy as np
import arcade  # for drawing
import time

import os, sys

from swarm_rescue.solutions.assets.mapping.entity import Entity

# This line add, to sys.path, the path to parent path of this file
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from swarm_rescue.spg_overlay.entities.drone_abstract import DroneAbstract
from swarm_rescue.spg_overlay.utils.misc_data import MiscData
from swarm_rescue.maps.map_intermediate_01 import MyMapIntermediate01
from swarm_rescue.spg_overlay.gui_map.gui_sr import GuiSR
from swarm_rescue.solutions.assets.movement.pathfinding import find_path, compute_path_map
from swarm_rescue.solutions.assets.movement.control import compute_command
from swarm_rescue.solutions.assets.mapping.lidarMapping import process_lidar
from swarm_rescue.solutions.assets.mapping.semanticMapping import process_semantic
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE
from swarm_rescue.solutions.assets.mapping.draw_grid import draw_map
from swarm_rescue.solutions.assets.behavior.state import State
from swarm_rescue.solutions.assets.behavior.map_split import zone_split, waypoint_pos, ZONE_SIZE
from swarm_rescue.solutions.assets.movement.pathfinding import BASE_WEIGHT, CLOUD_BONUS, WALL_WEIGHT
from swarm_rescue.solutions.assets.behavior.think import compute_behavior, is_defined


# region local constants

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
        self.id = identifier
        # endregion
        # region pathfinding init
        self.pos = np.zeros(3, dtype=np.float16)
        """
        The position vector of the drone, third term being its angle.
        """
        self.speed = np.zeros(3, dtype=np.float16)
        """
        The speed vector of the drone, third term being its angle.
        """
        self.target = (0, 0)
        """
        The current drone's target, where is wants to move.
        """
        self.map_size = tuple(misc_data.size_area)
        """
        The map size, in pixels.
        """
        self.tile_map_size = tuple([np.int8(size / TILE_SIZE) for size in self.map_size])
        """
        The map size, in tiles.
        :type: tuple[int8, int8]
        """
        self.occupancy_map = np.zeros(self.tile_map_size, dtype=np.float32)
        """
        The array on which the drone's LIDAR acts. A positive value on a tile means there is a wall there, a negative value means there isn't.
        
        :type: ndarray of float32
        """
        self.path_map = np.ones(self.tile_map_size, dtype=np.uint8)
        """
        The array, mainly computed from :py:attr:`occupancy_map`, that the drone uses to find its path. A number n on a tile means that this tile costs n to go 
        through.
        
        :type: ndarray of uint8
        """
        self.entity_map = np.zeros(self.tile_map_size, dtype=np.uint8)
        """
        The array corresponding to the entities present on each tile.
        
        :type: ndarray of uint8
        """
        self.trust_map = np.zeros(self.tile_map_size, dtype=np.uint8)
        """
        WIP
        """
        self.path = list()
        """
        The current path that the drone is following.
        
        :type: list of points
        """
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
        The list of the detected base tiles. Each one of this elements is of the form [tile_x, tile_y].

        :type: list of 2-elements lists
        """
        # endregion
        # region behavior init
        self.state = State.BOOT.value
        """
        The int value corresponding to the state of the drone, among :py:class:`~swarm_rescue.solutions.assets.behavior.state.State`.
        
        :type: State
        """
        self.waypoints, self.n_width, self.n_height = zone_split(self.tile_map_size, self.tile_pos, 1, 1)  # WIP
        self.target_waypoint = (0, 0)
        self.victims = list()
        self.distance_from_closest_victim = m.inf
        self.distance_from_closest_base = m.inf
        self.victim_angle = 0
        self.timers = {"waypoints_scan": 0, "victim_wait": 0}
        # endregion

        # region drawing (to comment out for eval)
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
    def got_victim(self):
        return len(self.grasped_entities()) != 0

    @property
    def tile_pos(self):
        return [(self.pos[0] / TILE_SIZE + 0.5).astype(int), (self.pos[1] / TILE_SIZE + 0.5).astype(int), self.pos[2]]

    def update_position(self):
        self.speed[0] = self.odometer_values()[0] * m.cos(self.pos[2] + self.odometer_values()[1])
        self.speed[1] = self.odometer_values()[0] * m.sin(self.pos[2] + self.odometer_values()[1])
        self.speed[2] = self.odometer_values()[2]
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
        """
        Here, we don't need communication...
        """
        pass

    def control(self):
        """
        How the drone moves
        """
        self.update_position()
        self.occupancy_map = process_lidar(self.occupancy_map, self.map_size, TILE_SIZE, self.tile_map_size, self.lidar_values(), self.lidar_rays_angles(),
                                           self.pos, self.pos[2])
        self.distance_from_closest_victim, self.victim_angle, self.distance_from_closest_base = process_semantic(self.semantic_values(), self.pos,
                                                                                                                 self.tile_map_size, self.victims, self.state,
                                                                                                                 self.bases, self.entity_map, self.path_map,
                                                                                                                 self.occupancy_map, self.victim_angle)
        self.path_map = compute_path_map(self.tile_map_size, self.occupancy_map, self.entity_map, self.state, self.target)
        self.state, self.target, self.target_waypoint = compute_behavior(self.id, self.target, self.target_waypoint, self.tile_pos, self.path, self.speed,
                                                                         self.state, self.victims, self.distance_from_closest_victim, self.bases,
                                                                         self.distance_from_closest_base, self.got_victim, self.waypoints, self.n_width,
                                                                         self.n_height, self.tile_map_size, self.entity_map, self.path_map, self.timers)

        if is_defined(self.target):
            self.path = find_path(self.tile_map_size, self.path_map, self.tile_pos, self.target, self.speed)

        # region testing
        if self.state == State.DONE.value:
            if self.cycle != -1:
                print("Cycles écoulés:", self.cycle, "cycles")
                self.cycle = -1
        else:
            self.cycle += 1

        # endregion

        command = compute_command(self.path, self.path_map, self.tile_pos, self.state, self.victim_angle, self.distance_from_closest_base)
        return command

    def draw_bottom_layer(self):  # (to comment out for eval)

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
        back_color = (0, 148, 66)
        waypoint_color = (0, 208, 61)
        waypoint_color_2 = (0, 108, 61)
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
                    arcade.draw_rectangle_filled(i * TILE_SIZE + TILE_SIZE / 2, j * TILE_SIZE + TILE_SIZE / 2, TILE_SIZE, TILE_SIZE, color)

        if self.draw_path_map:  # displays the path_map

            for i in range(0, self.tile_map_size[0]):
                for j in range(0, self.tile_map_size[1]):
                    if self.path_map[i, j] != BASE_WEIGHT:
                        if self.path_map[i, j] == 0:
                            color = wall_color
                        else:
                            t = 1 - (self.path_map[i, j] - BASE_WEIGHT) / WALL_WEIGHT
                            color = (1 - t) * np.array(gradation_color) + t * np.array(void_color)  # gradation
                        arcade.draw_rectangle_filled(i * TILE_SIZE + TILE_SIZE / 2, j * TILE_SIZE + TILE_SIZE / 2, TILE_SIZE, TILE_SIZE, color)
            # draw_map(self.occupancy_map, TILE_SIZE, self.map_size, self.pos[:2], self.pos[2])

        if self.draw_waypoints:  # displays the waypoints
            for i in range(self.n_width):
                for j in range(self.n_height):
                    if self.waypoints[i, j] != 0:
                        if self.waypoints[i, j] == 1:
                            color = waypoint_color
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