from typing import List
import numpy as np

from comm_declarations import VICTIM_MIN_DIST



def share_map():
    """
    Use last years function :
    @nb.njit
    def map_intersect(presence_map1, entity_map1, trust_map1, presence_map2, entity_map2, trust_map2, map_size):
        #TODO remove trust factor by Fabien
        new_presence_map = np.zeros(map_size, dtype=np.float32)
        new_entity_map = np.zeros(map_size, dtype=np.int8)
        new_trust_map = -np.ones(map_size, dtype=np.int8)

        for i in range(map_size[0]):
            for j in range(map_size[1]):
                if (2+trust_map1[i,j]+trust_map2[i,j] != 0):
                    new_presence_map[i,j] = (presence_map1[i,j] * (trust_map1[i,j]+1) + presence_map2[i,j] * (trust_map2[i,j]+1)) / (2+trust_map1[i,
                    j]+trust_map2[i,j])
                    new_trust_map[i,j] = 2 + trust_map1[i,j] + trust_map2[i,j]
                if (entity_map1[i,j] == Entity.BASE.value or entity_map2[i,j] == Entity.BASE.value):
                    new_entity_map[i,j] = Entity.BASE.value
                elif (entity_map1[i,j] == Entity.KILL.value or entity_map2[i,j] == Entity.KILL.value):
                    new_entity_map[i,j] = Entity.KILL.value
                elif (entity_map1[i,j] == Entity.NOCOM.value or entity_map2[i,j] == Entity.NOCOM.value):
                    new_entity_map[i,j] = Entity.NOCOM.value
                elif (entity_map1[i,j] == Entity.NOGPS.value or entity_map2[i,j] == Entity.NOGPS.value):
                    new_entity_map[i,j] = Entity.NOGPS.value
                elif (trust_map1[i,j] >= trust_map2[i,j]):
                    new_entity_map[i,j] = entity_map1[i,j]
                else:
                    new_entity_map[i, j] = entity_map2[i, j]

        return new_presence_map, new_entity_map, new_trust_map
    """
    pass


def share_victims(my_victims:List, other_victims:List):
    """
    By Louis
    Returns
    -------
    """

    for tile_x, tile_y, savior in my_victims:
        for other_tile_x, other_tile_y, other_savior in other_victims:
            # If the distance between the two victims is less than VICTIM_MIN_DIST, we consider them as the same victim
            if np.sqrt((tile_x - other_tile_x)**2 + (tile_y - other_tile_y)**2) < VICTIM_MIN_DIST:
                # We keep the savior with the highest id
                # TODO : check if using a trust factor is relevant
                if savior > other_savior:
                    other_victims.remove((other_tile_x, other_tile_y, other_savior))
                else:
                    my_victims.remove((tile_x, tile_y, savior))

    return my_victims + other_victims







