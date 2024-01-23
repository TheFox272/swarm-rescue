import math

import numpy as np
import swarm_rescue.solutions.assets.mapping.gridFunctions as gridFun

from swarm_rescue.spg_overlay.utils.constants import MAX_RANGE_LIDAR_SENSOR

def process_lidar(occupancy_map, size_area_world, resolution, tile_map_size, lidar_values, lidar_ray_angles, pos, ori):
    """
    Bayesian map update with new observation
    occupancy_map : obstacles map
    size_area_world : size of the area in world coordinates
    tile_map_size : size of the map in tiles
    lidar_values : lidar distances data
    lidar_ray_angles : lidar angles data
    pos : gps position of the drone
    ori : compas angle of the drone
    """

    EVERY_N = 3
    LIDAR_DIST_CLIP = 40.0
    EMPTY_ZONE_VALUE = -0.602
    OBSTACLE_ZONE_VALUE = 2.0
    FREE_ZONE_VALUE = -4.0
    THRESHOLD_MIN = -40
    THRESHOLD_MAX = 40

    lidar_dist = lidar_values[::EVERY_N].copy()
    lidar_angles = lidar_ray_angles[::EVERY_N].copy()

    # Compute cos and sin of the absolute angle of the lidar
    cos_rays = np.cos(lidar_angles + ori)
    sin_rays = np.sin(lidar_angles + ori)

    max_range = MAX_RANGE_LIDAR_SENSOR * 0.9

    # For empty zones
    # points_x and point_y contains the border of detected empty zone
    # We use a value a little bit less than LIDAR_DIST_CLIP because of the noise in lidar
    lidar_dist_empty = np.maximum(lidar_dist - LIDAR_DIST_CLIP, 0.0)
    # All values of lidar_dist_empty_clip are now <= max_range
    lidar_dist_empty_clip = np.minimum(lidar_dist_empty, max_range)
    points_x = pos[0] + np.multiply(lidar_dist_empty_clip, cos_rays)
    points_y = pos[1] + np.multiply(lidar_dist_empty_clip, sin_rays)

    try:
        for pt_x, pt_y in zip(points_x, points_y):
            gridFun.add_value_along_line(occupancy_map, size_area_world, resolution, tile_map_size, pos[0], pos[1], pt_x, pt_y, EMPTY_ZONE_VALUE)
    except OverflowError:
        pass

    # For obstacle zones, all values of lidar_dist are < max_range
    select_collision = lidar_dist < max_range

    points_x = pos[0] + np.multiply(lidar_dist, cos_rays)
    points_y = pos[1] + np.multiply(lidar_dist, sin_rays)

    points_x = points_x[select_collision]
    points_y = points_y[select_collision]

    gridFun.add_points(occupancy_map, size_area_world, resolution, tile_map_size, points_x, points_y, OBSTACLE_ZONE_VALUE)

    # the current position of the drone is free !
    gridFun.add_points(occupancy_map, size_area_world, resolution, tile_map_size, pos[0], pos[1],
                       FREE_ZONE_VALUE)

    # threshold values
    occupancy_map = np.clip(occupancy_map, THRESHOLD_MIN, THRESHOLD_MAX)

    return occupancy_map
