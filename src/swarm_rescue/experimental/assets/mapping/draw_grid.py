import numpy as np

import swarm_rescue.experimental.assets.Mapping.gridFunctions as gridFunc

import cv2

def draw_map(map, resolution, size_area_world, pos, ori):
    img = map.T
    img = img - img.min()
    img = img / img.max() * 255
    img = np.uint8(img)
    img_color = cv2.applyColorMap(src=img, colormap=cv2.COLORMAP_JET)

    pt2_x = pos[0] + np.cos(ori) * 20
    pt2_y = pos[1] + np.sin(ori) * 20
    # pt2_x, pt2_y = gridFunc._conv_world_to_grid(pt2_x, pt2_y, size_area_world, resolution)

    pt1_x, pt1_y = pos[0], pos[1]

    display_img = cv2.resize(img_color, (1050, 700), interpolation=cv2.INTER_NEAREST)

    pt1 = (int(pt1_x* (1050/size_area_world[0])), int(pt1_y* (700/size_area_world[1])))
    pt2 = (int(pt2_x* (1050/size_area_world[0])), int(pt2_y* (700/size_area_world[1])))
    cv2.arrowedLine(img=display_img, pt1=pt1, pt2=pt2,
                    color=(0, 0, 255), thickness=3)


    display_img = cv2.flip(display_img, 0)
    cv2.imshow("title", display_img)
    cv2.waitKey(1)