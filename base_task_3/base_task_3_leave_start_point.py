# -*- coding: utf-8 -*-
import cv2
import numpy as np

import global_params
import macro

from base_task_3 import base_task_3_params as params

from BaseFSM import base_fsm

if global_params.RASPBERRY_MODE:
    import V_Display as vd

class base_task_3_leave_start_point_fsm(base_fsm):
    def enter_state(self, flight):
        flight.median_filter.refresh()

    def exec_state(self, flight):
        base_task_3_leave_start_point(flight)

    def exit_state(self, flight):
        pass

base_task_3_leave_start_point_counter = 0

def base_task_3_leave_start_point(flight):
    # start_time = cv2.getTickCount()
    global base_task_3_leave_start_point_counter
    
    ret, frame = flight.read_camera()
    if not ret:
        return

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    reduce_noise_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0)
    circles = cv2.HoughCircles(reduce_noise_img, cv2.HOUGH_GRADIENT,
                                params.DP, params.MIN_DIST,
                                param1 = params.PARAM1, param2 = params.PARAM2,
                                minRadius = params.MIN_RADIUS, maxRadius = params.MAX_RADIUS)
    
    try:
        circles = np.uint16(np.around(circles))
        for i in circles[:, 0, :]:
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 0, 255), 2)
        if len(circles[:, 0, :]) == 1 and circles[0, 0, 0] < global_params.IMAGE_WIDTH / 2:
            base_task_3_leave_start_point_counter += 1
        else:
            base_task_3_leave_start_point_counter = 0
    except AttributeError:
        base_task_3_leave_start_point_counter = 0

    angle = global_params.FLY_ANGLE_P
    dst_point_y = int(global_params.IMAGE_HEIGHT / 2)
    path_angle = 100
    speed_x, speed_y = flight.get_speed()
    speed_x, speed_y = int(speed_x), int(speed_y)
    uart_buff = bytearray([0x55,               0xAA,                  0x62,           angle & 0xff,
                           dst_point_y & 0xff, (speed_x >> 8) & 0xff, speed_x & 0xff, (speed_y >> 8) & 0xff,
                           speed_y & 0xff,     path_angle & 0xff,     0x00,           0xAA                  ])
    flight.send(uart_buff)

    if global_params.RASPBERRY_MODE:
        vd.show(frame)
    else:
        cv2.imshow('frame', frame)

    if base_task_3_leave_start_point_counter == params.BASE_TASK_3_LEAVE_START_POINT_NUM:
        base_task_3_leave_start_point_counter = 0
        flight.next_state = macro.BASE_TASK_3_DETECT_CAR
        print('>>> [!] BASE_TASK_3_LEAVE_START_POINT -> BASE_TASK_3_DETECT_CAR')
    # end_time = cv2.getTickCount()
    # time_cost = (end_time - start_time) / cv2.getTickFrequency()
    # print('>>> [Hz] %f' % (1 / time_cost))
