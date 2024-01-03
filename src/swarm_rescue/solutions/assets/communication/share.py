from typing import List

import numpy as np





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


def share_victims(my_victims:List, others_victims:List[List]):
    """
    By Louis
    Returns
    -------

    """
    pass







