# -*- coding: utf-8 -*-

import cv2
import numpy as np

import global_params
import macro
from base_task_3 import base_task_3_params as params

from BaseFSM import base_fsm

if global_params.RASPBERRY_MODE:
    import V_Display as vd

class base_task_3_brake_fsm(base_fsm):
    def enter_state(self, flight):
        flight.median_filter.refresh()

    def exec_state(self, flight):
        base_task_3_brake(flight)

    def exit_state(self, flight):
        pass

base_task_3_brake_counter = 0
delay_counter = 0

def base_task_3_brake(flight):
    global base_task_3_brake_counter
    global delay_counter
    
    ret, frame = flight.read_camera()
    if not ret:
        return

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    reduce_noise_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0)
    circles = cv2.HoughCircles(reduce_noise_img, cv2.HOUGH_GRADIENT,
                                params.DP_2, params.MIN_DIST_2,
                                param1 = params.PARAM1_2, param2 = params.PARAM2_2,
                                minRadius = params.MIN_RADIUS_2, maxRadius = params.MAX_RADIUS_2)
    
    try:
        circles = np.uint16(np.around(circles))
        for i in circles[:, 0, :]:
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 0, 255), 2)
        if len(circles[:, 0, :]) == 1:
            flight.median_filter.add_point_c1((circles[0, 0, 0], circles[0, 0, 1]))
        else:
            flight.median_filter.add_point_c1((global_params.IMAGE_WIDTH / 2, global_params.IMAGE_HEIGHT / 2))
    except AttributeError:
        flight.median_filter.add_point_c1((global_params.IMAGE_WIDTH / 2, global_params.IMAGE_HEIGHT / 2))


    if delay_counter >= params.BASE_TASK_3_BRAKE_DELAY_NUM:
        dst_point_x, dst_point_y = flight.median_filter.get_result_point_c1()
        dst_point_x, dst_point_y = int(dst_point_x), int(dst_point_y)
        speed_x, speed_y = flight.get_speed()
        speed_x, speed_y = int(speed_x), int(speed_y)
        uart_buff = bytearray([0x55,               0xAA,                  0x61,           dst_point_x & 0xff,
                               dst_point_y & 0xff, (speed_x >> 8) & 0xff, speed_x & 0xff, (speed_y >> 8) & 0xff,
                               speed_y & 0xff,     0x00,                  0x00,           0xAA                  ])
        flight.send(uart_buff)
        base_task_3_brake_counter += 1
    else:
        angle = global_params.FLY_ANGLE_P
        dst_point_y = int(global_params.IMAGE_HEIGHT / 2)
        path_angle = 100
        speed_x, speed_y = flight.get_speed()
        speed_x, speed_y = int(speed_x), int(speed_y)
        uart_buff = bytearray([0x55,               0xAA,                  0x62,           angle & 0xff,
                               dst_point_y & 0xff, (speed_x >> 8) & 0xff, speed_x & 0xff, (speed_y >> 8) & 0xff,
                               speed_y & 0xff,     path_angle & 0xff,     0x00,           0xAA                  ])
        flight.send(uart_buff)
        delay_counter += 1

    if global_params.RASPBERRY_MODE:
        vd.show(frame)
    else:
        cv2.imshow('frame', frame)

    if base_task_3_brake_counter == params.BASE_TASK_3_BRAKE_NUM:
        base_task_3_brake_counter = 0
        delay_counter = 0
        flight.next_state = macro.BASE_TASK_3_STOP_TP_CAR
        print('>>> [!] BASE_TASK_3_BRAKE -> BASE_TASK_3_STOP_TP_CAR')