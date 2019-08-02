# -*- coding: utf-8 -*-
import cv2
import numpy as np

import global_params
import macro

from advance_task_1 import advance_task_1_params as params

from BaseFSM import base_fsm

if global_params.RASPBERRY_MODE:
    import V_Display as vd

class advance_task_1_take_off_fsm(base_fsm):
    def enter_state(self, flight):
        flight.median_filter.refresh()

    def exec_state(self, flight):
        advance_task_1_take_off(flight)

    def exit_state(self, flight):
        pass

advance_task_1_take_off_counter = 0
lose_target_counter = 0

def advance_task_1_take_off(flight):
    # start_time = cv2.getTickCount()
    global advance_task_1_take_off_counter
    global lose_target_counter
    
    ret, frame = flight.read_camera()
    if not ret:
        return

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    reduce_noise_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0)
    circles = cv2.HoughCircles(reduce_noise_img, cv2.HOUGH_GRADIENT,
                                params.MAP_DP, params.MAP_MIN_DIST,
                                param1 = params.MAP_PARAM1, param2 = params.MAP_PARAM2,
                                minRadius = params.MAP_MIN_RADIUS, maxRadius = params.MAP_MAX_RADIUS)
    
    try:
        circles = np.uint16(np.around(circles))
        for i in circles[:, 0, :]:
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 0, 255), 2)
        if len(circles[:, 0, :]) == 1:
            advance_task_1_take_off_counter += 1
            lose_target_counter = 0
            flight.median_filter.add_point_c1((circles[0, 0, 0], circles[0, 0, 1]))
        else:
            lose_target_counter += 1
            flight.median_filter.add_point_c1((global_params.IMAGE_WIDTH / 2, global_params.IMAGE_HEIGHT / 2))
    except AttributeError:
        lose_target_counter += 1
        flight.median_filter.add_point_c1((global_params.IMAGE_WIDTH / 2, global_params.IMAGE_HEIGHT / 2))

    dst_point_x, dst_point_y = flight.median_filter.get_result_point_c1()
    dst_point_x, dst_point_y = int(dst_point_x), int(dst_point_y)
    speed_x, speed_y = flight.get_speed()
    speed_x, speed_y = int(speed_x), int(speed_y)
    uart_buff = bytearray([0x55,               0xAA,                  0x80,           dst_point_x & 0xff,
                           dst_point_y & 0xff, (speed_x >> 8) & 0xff, speed_x & 0xff, (speed_y >> 8) & 0xff,
                           speed_y & 0xff,     0x00,                  0x00,           0xAA                  ])
    flight.send(uart_buff)
    # print('dst_point_x:', dst_point_x, 'dst_point_y:', dst_point_y, 'speed_x:', speed_x, 'speed_y:', speed_y)

    if global_params.RASPBERRY_MODE:
        vd.show(frame)
    else:
        cv2.imshow('frame', frame)

    if lose_target_counter == params.LOSE_TARGET_TH:
        advance_task_1_take_off_counter = 0
        lose_target_counter = 0

    if advance_task_1_take_off_counter == params.ADVANCE_TASK_1_TAKE_OFF_NUM:
        advance_task_1_take_off_counter = 0
        flight.next_state = macro.ADVANCE_TASK_1_LEAVE_START_POINT
        print('>>> [!] ADVANCE_TASK_1_TAKE_OFF -> ADVANCE_TASK_1_LEAVE_START_POINT')
    # end_time = cv2.getTickCount()
    # time_cost = (end_time - start_time) / cv2.getTickFrequency()
    # print('>>> [Hz] %f' % (1 / time_cost))
