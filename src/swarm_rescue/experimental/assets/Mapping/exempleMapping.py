import numpy as np
from typing import Type

import sys, os
# This line add, to sys.path, the path to parent path of this file
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

from maps.walls_medium_02 import add_walls, add_boxes
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.rescue_center import RescueCenter, wounded_rescue_center_collision
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.gui_map.map_abstract import MapAbstract

from spg.playground import Playground
from spg.utils.definitions import CollisionTypes

import lidarMapping
from draw_grid import draw_map


class MyDroneMapping(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.iteration: int = 0

        self.resolution = 8
        self.size_area_world=np.array(self.size_area)

        self.tile_map_size = (self.size_area_world / self.resolution + 0.5).astype(int)

        self.occupancy_map = np.zeros(self.tile_map_size)

        self.iteration = 0

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self):
        """
        We only send a command to do nothing
        """
        command = {"forward": 0.0,
                   "lateral": 0.0,
                   "rotation": 0.0,
                   "grasper": 0}

        # increment the iteration counter
        self.iteration += 1

        gpsPos = self.measured_gps_position()
        compasAngle = self.measured_compass_angle()
        lidar_values = self.lidar().get_sensor_values()
        lidar_ray_angles = self.lidar().ray_angles

        self.occupancy_map = lidarMapping.update_grid(self.occupancy_map, self.size_area_world, self.resolution, self.tile_map_size, lidar_values, lidar_ray_angles, gpsPos, compasAngle)
        # if self.iteration % 5 == 0:
        if True:
            draw_map(self.occupancy_map, self.tile_map_size, self.resolution, self.size_area_world,
                     self.measured_gps_position(), self.measured_compass_angle())
        #     self.grid.display(self.grid.zoomed_grid, self.estimated_pose, title="zoomed occupancy grid")
        #     # pass

        print(np.sum(self.occupancy_map >= 0))

        self.iteration += 1
        return command

    # def draw_bottom_layer(self):
    #     if self.iteration % 10 == 1:
    #         draw_map(self.occupancy_map, self.tile_map_size, self.resolution, self.size_area_world, self.measured_gps_position(), self.measured_compass_angle())


class MyMapMapping(MapAbstract):

    def __init__(self):
        super().__init__()

        # PARAMETERS MAP
        self._size_area = (1113, 750)

        self._rescue_center = RescueCenter(size=(210, 90))
        self._rescue_center_pos = ((440, 315), 0)

        self._number_drones = 1
        self._drones_pos = [((-50, 0), 0)]
        self._drones = []

    def construct_playground(self, drone_type: Type[DroneAbstract]) -> Playground:
        playground = ClosedPlayground(size=self._size_area)

        # RESCUE CENTER
        playground.add_interaction(CollisionTypes.GEM,
                                   CollisionTypes.ACTIVABLE_BY_GEM,
                                   wounded_rescue_center_collision)

        playground.add(self._rescue_center, self._rescue_center_pos)

        add_walls(playground)
        add_boxes(playground)

        # POSITIONS OF THE DRONES
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones)
        for i in range(self._number_drones):
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            playground.add(drone, self._drones_pos[i])

        return playground


def main():
    my_map = MyMapMapping()
    playground = my_map.construct_playground(drone_type=MyDroneMapping)

    gui = GuiSR(playground=playground,
                the_map=my_map,
                use_keyboard=True,
                )
    gui.run()


if __name__ == '__main__':
    main()