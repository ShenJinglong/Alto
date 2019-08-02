# coding: utf-8 -*-

from BaseFSM import base_fsm
import global_params
import base_task_3.base_task_3_params as params

class base_task_3_land_fsm(base_fsm):
    def enter_state(self, flight):
        flight.median_filter.refresh()

    def exec_state(self, flight):
        base_task_3_land(flight)

    def exit_state(self, flight):
        pass

base_task_3_land_counter = 0

def base_task_3_land(flight):
    global base_task_3_land_counter
    if base_task_3_land_counter < params.BASE_TASK_3_LAND_NUM:
        angle = 120
        dst_point_y = int(global_params.IMAGE_HEIGHT / 2)
        path_angle = 100
        speed_x, speed_y = flight.get_speed()
        speed_x, speed_y = int(speed_x), int(speed_y)
        uart_buff = bytearray([0x55,               0xAA,                  0x62,           angle & 0xff,
                               dst_point_y & 0xff, (speed_x >> 8) & 0xff, speed_x & 0xff, (speed_y >> 8) & 0xff,
                               speed_y & 0xff,     path_angle & 0xff,     0x00,           0xAA                  ])
        flight.send(uart_buff)
        base_task_3_land_counter += 1
    else:
        dst_point_x, dst_point_y = 80, 60
        dst_point_x, dst_point_y = int(dst_point_x), int(dst_point_y)
        speed_x, speed_y = flight.get_speed()
        speed_x, speed_y = int(speed_x), int(speed_y)
        uart_buff = bytearray([0x55,               0xAA,                  0x60,           dst_point_x & 0xff,
                               dst_point_y & 0xff, (speed_x >> 8) & 0xff, speed_x & 0xff, (speed_y >> 8) & 0xff,
                               speed_y & 0xff,     0x00,                  0x02,           0xAA                  ])
        flight.send(uart_buff)

