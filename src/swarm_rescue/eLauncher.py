import gc
from typing import Tuple

import sys, os
# This line add, to sys.path, the path to parent path of this file
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from spg_overlay.entities.sensor_disablers import ZoneType
from spg_overlay.utils.constants import DRONE_INITIAL_HEALTH
from spg_overlay.reporting.evaluation import EvalConfig, EvalPlan, ZonesConfig
from spg_overlay.reporting.score_manager import ScoreManager
from spg_overlay.reporting.data_saver import DataSaver
from spg_overlay.reporting.screen_recorder import ScreenRecorder
from spg_overlay.reporting.team_info import TeamInfo
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
    my_map = MyMap()

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
