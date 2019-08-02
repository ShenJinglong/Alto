# -*- coding: utf-8 -*-

import macro

from ModeSelector import mode_selector_fsm

from base_task_1.base_task_1_take_off import base_task_1_take_off_fsm
from base_task_1.base_task_1_stop_tp_start import base_task_1_stop_tp_start_fsm
from base_task_1.base_task_1_land import base_task_1_land_fsm

from base_task_3.base_task_3_take_off import base_task_3_take_off_fsm
from base_task_3.base_task_3_leave_start_point import base_task_3_leave_start_point_fsm
from base_task_3.base_task_3_detect_car import base_task_3_detect_car_fsm
from base_task_3.base_task_3_brake import base_task_3_brake_fsm
from base_task_3.base_task_3_stop_tp_car import base_task_3_stop_tp_car_fsm
from base_task_3.base_task_3_land import base_task_3_land_fsm

from advance_task_1.advance_task_1_take_off import advance_task_1_take_off_fsm
from advance_task_1.advance_task_1_leave_start_point import advance_task_1_leave_start_point_fsm
from advance_task_1.advance_task_1_detect_car import advance_task_1_detect_car_fsm
from advance_task_1.advance_task_1_brake import advance_task_1_brake_fsm
from advance_task_1.advance_task_1_stop_tp_car import advance_task_1_stop_tp_car_fsm
from advance_task_1.advance_task_1_land import advance_task_1_land_fsm

class fsm_mgr(object):
    def __init__(self):
        self._fsms = {}

        self._fsms[macro.MODE_SELECTOR] = mode_selector_fsm()

        self._fsms[macro.BASE_TASK_1_TAKE_OFF] = base_task_1_take_off_fsm()
        self._fsms[macro.BASE_TASK_1_STOP_TP_START] = base_task_1_stop_tp_start_fsm()
        self._fsms[macro.BASE_TASK_1_LAND] = base_task_1_land_fsm()

        self._fsms[macro.BASE_TASK_3_TAKE_OFF] = base_task_3_take_off_fsm()
        self._fsms[macro.BASE_TASK_3_LEAVE_START_POINT] =base_task_3_leave_start_point_fsm()
        self._fsms[macro.BASE_TASK_3_DETECT_CAR] = base_task_3_detect_car_fsm()
        self._fsms[macro.BASE_TASK_3_BRAKE] = base_task_3_brake_fsm()
        self._fsms[macro.BASE_TASK_3_STOP_TP_CAR] = base_task_3_stop_tp_car_fsm()
        self._fsms[macro.BASE_TASK_3_LAND] = base_task_3_land_fsm()

        self._fsms[macro.ADVANCE_TASK_1_TAKE_OFF] = advance_task_1_take_off_fsm()
        self._fsms[macro.ADVANCE_TASK_1_LEAVE_START_POINT] =advance_task_1_leave_start_point_fsm()
        self._fsms[macro.ADVANCE_TASK_1_DETECT_CAR] = advance_task_1_detect_car_fsm()
        self._fsms[macro.ADVANCE_TASK_1_BRAKE] = advance_task_1_brake_fsm()
        self._fsms[macro.ADVANCE_TASK_1_STOP_TP_CAR] = advance_task_1_stop_tp_car_fsm()
        self._fsms[macro.ADVANCE_TASK_1_LAND] = advance_task_1_land_fsm()

    def get_fsm(self, state):
        return self._fsms[state]

    def frame(self, flight):
        if flight.next_state == flight.curr_state:
            flight.keep_state()
        else:
            flight.change_state(flight.next_state, self._fsms[flight.next_state])