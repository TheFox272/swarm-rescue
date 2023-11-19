import arcade
import numpy as np

import gridFunctions as gridFunc

from spg_overlay.utils.grid import Grid

import cv2

def draw_map(map, map_size, resolution, size_area_world, pos, ori):
    img = map.T
    img = img - img.min()
    img = img / img.max() * 255
    img = np.uint8(img)
    img_color = cv2.applyColorMap(src=img, colormap=cv2.COLORMAP_JET)

    pt2_x = pos[0] + np.cos(ori) * 20
    pt2_y = pos[1] + np.sin(ori) * 20
    pt2_x, pt2_y = gridFunc._conv_world_to_grid(pt2_x, pt2_y, size_area_world, resolution)

    pt1_x, pt1_y = gridFunc._conv_world_to_grid(pos[0], pos[1], size_area_world, resolution)

    pt1 = (int(pt1_x), int(pt1_y))
    pt2 = (int(pt2_x), int(pt2_y))
    cv2.arrowedLine(img=img_color, pt1=pt1, pt2=pt2,
                    color=(0, 0, 255), thickness=2)
    display_img = cv2.resize(img_color, (960, 540), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("title", display_img)
    cv2.waitKey(1)