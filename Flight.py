# -*- coding: utf-8 -*-
import cv2
import time
import multiprocessing

import global_params

from Filter import MedianFilter
from OpticalFlow import OpticalFlow

if global_params.RASPBERRY_MODE:
    import V_UCom as com
    import V_Display as vd

class Flight(object):
    def __init__(self):
        self.__cap = cv2.VideoCapture(0)
        self.median_filter = MedianFilter()

        if global_params.RASPBERRY_MODE:
            com.init(mode = 2)
            vd.init()

        if global_params.RECORD_VIDEO:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.__video_recorder = cv2.VideoWriter('../../Videos/' + time.strftime('%Y-%m-%d %H-%M-%S', time.localtime(time.time())) + '.avi', fourcc, 20.0, (160, 120))

        self.__frame_queue = multiprocessing.Queue(maxsize = 1)
        self.__flow_xy = multiprocessing.Queue(maxsize = 1)
        self.__optical_flow = OpticalFlow(self.__frame_queue, self.__flow_xy)
        self.__optical_flow.start()

    def read_camera(self):
        ret, frame = self.__cap.read()
        if ret:
            frame = cv2.resize(frame, (global_params.IMAGE_WIDTH, global_params.IMAGE_HEIGHT))
            try:
                self.__frame_queue.put(frame, False)
            except:
                pass
            if global_params.RECORD_VIDEO:
                self.__video_recorder.write(frame)
        return ret, frame

    def destroy(self):
        self.__cap.release()
        if self.__optical_flow.is_alive():
            self.__optical_flow.terminate()
            self.__optical_flow.join()

    def send(self, uart_buff):
        if global_params.RASPBERRY_MODE:
            com.send(uart_buff)

    def get_speed(self):
        x_speed, y_speed = 0, 0
        if not self.__flow_xy.empty():
            x_speed, y_speed = self.__flow_xy.get()
        return (x_speed, y_speed)

    def attach_fsm(self, state, fsm):
        self.__fsm = fsm
        self.curr_state = state
        self.next_state = state

    def change_state(self, new_state, new_fsm):
        self.curr_state = new_state
        self.__fsm.exit_state(self)
        self.__fsm = new_fsm
        self.__fsm.enter_state(self)
        self.__fsm.exec_state(self)

    def keep_state(self):
        self.__fsm.exec_state(self)