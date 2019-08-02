# -*- coding: utf-8 -*-

import cv2

import macro

from FSM import fsm_mgr
from Flight import Flight

class World(object):
    def __init__(self):
        self._flight = Flight()
        self._fsm_mgr = fsm_mgr()
        self._flight.attach_fsm(macro.MODE_SELECTOR, self._fsm_mgr.get_fsm(macro.MODE_SELECTOR))

    def __destroy_flight(self):
        self._flight.destroy()

    def __frame(self):
        self._fsm_mgr.frame(self._flight)

    def run(self):
        while True:
            # start_time = cv2.getTickCount()
            
            self.__frame()
            k = cv2.waitKey(1) & 0xff
            if k == 27:
                break

            # end_time = cv2.getTickCount()
            # time_cost = (end_time - start_time) / cv2.getTickFrequency()
            # print('>>> [Hz] %f' % (1 / time_cost))
        
        cv2.destroyAllWindows()
        self.__destroy_flight()
