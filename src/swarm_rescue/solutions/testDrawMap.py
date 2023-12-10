import math
import random
from typing import Optional

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle


class MyDroneTestDrawMap(DroneAbstract):

    def draw_top_layer(self):
        self.draw_identifier()

    def control(self):
        pass

    def define_message_for_all(self):
        pass