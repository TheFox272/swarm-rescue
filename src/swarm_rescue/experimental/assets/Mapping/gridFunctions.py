# Functions copied from utils/grid.py

import math

import numpy as np

from spg_overlay.utils.pose import Pose

def _conv_world_to_grid(x_world, y_world, size_area_world, resolution):
    """
    Convert from world coordinates to map coordinates (i.e. cell index in the grid map)
    x_world, y_world : list of x and y coordinates in m
    """

    x_grid = (x_world + size_area_world[0] / 2) / resolution
    y_grid = (-y_world + size_area_world[1] / 2) / resolution

    if isinstance(x_grid, float):
        x_grid = int(x_grid)
        y_grid = int(y_grid)
    elif isinstance(x_grid, np.ndarray):
        x_grid = x_grid.astype(int)
        y_grid = y_grid.astype(int)

    return x_grid, y_grid

def _conv_grid_to_world(x_grid, y_grid, size_area_world, resolution):
    """
    Convert from map coordinates to world coordinates
    x_grid, y_grid : list of x and y coordinates in cell numbers (~pixels)
    """
    if isinstance(x_grid, int):
        x_grid = float(x_grid)
        y_grid = float(y_grid)
    elif isinstance(x_grid, np.ndarray):
        x_grid = x_grid.astype(float)
        y_grid = y_grid.astype(float)

    x_world = -size_area_world[0] / 2 + x_grid * resolution
    y_world = size_area_world[1] / 2 - y_grid * resolution

    if isinstance(x_world, np.ndarray):
        x_world = x_world.astype(float)
        y_world = y_world.astype(float)

    return x_world, y_world

def add_value_along_line(grid, size_area_world, resolution, tile_map_size, x_0: float, y_0: float, x_1: float, y_1: float, val):
    """
    Add a value to a line of points using Bresenham algorithm, input in world coordinates
    x_0, y_0 : starting point coordinates in m
    x_1, y_1 : end point coordinates in m
    val : value to add to each cell of the line
    """

    if math.isnan(x_0) or math.isnan(y_0) or math.isnan(x_1) or math.isnan(y_1):
        return

    # convert to pixels
    x_start, y_start = _conv_world_to_grid(x_0, y_0, size_area_world, resolution)
    x_end, y_end = _conv_world_to_grid(x_1, y_1, size_area_world, resolution)

    if x_start < 0 or x_start >= tile_map_size[0] or y_start < 0 or y_start >= tile_map_size[1]:
        # print("add_value_along_line: warning ray exits 1")
        return

    if x_end < 0 or x_end >= tile_map_size[0] or y_end < 0 or y_end >= tile_map_size[1]:
        # print("add_value_along_line: warning ray exits 2")
        return

    # Bresenham line drawing
    dx = x_end - x_start
    dy = y_end - y_start
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x_start, y_start = y_start, x_start
        x_end, y_end = y_end, x_end
    # swap start and end points if necessary and store swap state
    if x_start > x_end:
        x_start, x_end = x_end, x_start
        y_start, y_end = y_end, y_start
    dx = x_end - x_start  # recalculate differentials
    dy = y_end - y_start  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y_start < y_end else -1
    # iterate over bounding box generating points between start and end
    y = y_start
    points = []
    for x in range(x_start, x_end + 1):
        coord = [y, x] if is_steep else [x, y]
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    points = np.array(points).T

    # add value to the points
    grid[points[0], points[1]] += val

def add_points(grid, size_area_world, resolution, tile_map_size, points_x, points_y, val):
    """
    Add a value to an array of points, input coordinates in meters
    points_x, points_y :  list of x and y coordinates in m
    val :  value to add to the cells of the points
    """

    x_px, y_px = _conv_world_to_grid(points_x, points_y, size_area_world, resolution)

    if isinstance(points_x, int):
        if 0 <= x_px < tile_map_size[0] and 0 <= y_px < tile_map_size[1]:
            grid[x_px, y_px] += val
    elif isinstance(points_x, np.ndarray):
        select = np.logical_and(np.logical_and(x_px >= 0, x_px < tile_map_size[0]),
                                np.logical_and(y_px >= 0, y_px < tile_map_size[1]))
        x_px = x_px[select]
        y_px = y_px[select]

        grid[x_px, y_px] += val
