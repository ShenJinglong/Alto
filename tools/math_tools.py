# -*- coding: utf-8 -*-
import numpy as np

import global_params

def make_point_safe(point):
    __x = point[0]
    __y = point[1]
    if __x > global_params.IMAGE_WIDTH:
        __x = global_params.IMAGE_WIDTH
    elif __x < 0:
        __x = 0

    if __y > global_params.IMAGE_HEIGHT:
        __y = global_params.IMAGE_HEIGHT
    elif __y < 0:
        __y = 0

    return (__x, __y)

def get_average_point(point_buff):
    if len(point_buff) > 0:
        return (np.mean([point[0] for point in point_buff]), np.mean([point[1] for point in point_buff]))
    else:
        return (global_params.IMAGE_WIDTH / 2, global_params.IMAGE_HEIGHT / 2)

def get_average_number(number_buff):
    return np.mean(number_buff)
