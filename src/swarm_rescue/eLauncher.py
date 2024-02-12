import sys, os

# # This line add, to sys.path, the path to parent path of this file
# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from spg_overlay.entities.sensor_disablers import ZoneType
from spg_overlay.gui_map.gui_sr import GuiSR

from maps.map_intermediate_01 import MyMapIntermediate01
from maps.map_intermediate_02 import MyMapIntermediate02
from maps.map_final_2023 import MyMapFinal
from maps.map_medium_01 import MyMapMedium01
from maps.map_medium_02 import MyMapMedium02

from solutions.myFirstDrone import MyFirstDrone


# Drone selection
class MyDrone(MyFirstDrone):
    pass


# Map selection
class MyMap(MyMapIntermediate01):
    pass


def main():
    my_map = MyMap(zones_config=[ZoneType.KILL_ZONE, ZoneType.NO_GPS_ZONE, ZoneType.NO_COM_ZONE])
    my_map._real_time_limit = 100000000

    # my_map._number_drones = 2
    playground = my_map.construct_playground(drone_type=MyDrone)

    gui = GuiSR(playground=playground,
                the_map=my_map,
                use_keyboard=False,
                use_mouse_measure=True,
                enable_visu_noises=False
                )

    # this function below is a blocking function until the round is finished
    gui.run()


if __name__ == "__main__":
    main()
