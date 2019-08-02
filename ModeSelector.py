# -*- coding: utf-8 -*-

import time

import global_params
import macro

from BaseFSM import base_fsm

if global_params.RASPBERRY_MODE:
    import RPi.GPIO as GPIO

class mode_selector_fsm(base_fsm):
    def enter_state(self, flight):
        flight.median_filter.refresh()

    def exec_state(self, flight):
        mode_selector(flight)

    def exit_state(self, flight):
        pass

if global_params.RASPBERRY_MODE:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(global_params.MODE_PIN_1, GPIO.IN)
    GPIO.setup(global_params.MODE_PIN_2, GPIO.IN)
    GPIO.setup(global_params.MODE_PIN_3, GPIO.IN)
    GPIO.setup(global_params.MODE_PIN_4, GPIO.IN)
    GPIO.setup(12, GPIO.OUT, initial = GPIO.HIGH)
    GPIO.setup(17, GPIO.OUT, initial = GPIO.HIGH)

def mode_selector(flight):
    if global_params.RASPBERRY_MODE:
        v1 = GPIO.input(global_params.MODE_PIN_1)
        v2 = GPIO.input(global_params.MODE_PIN_2)
        v3 = GPIO.input(global_params.MODE_PIN_3)
        v4 = GPIO.input(global_params.MODE_PIN_4)
        if v1 and v2 and v3 and v4:
            flight.next_state = macro.BASE_TASK_1_TAKE_OFF
            print('>>> [!] BASE_TASK_1_TAKE_OFF')
            GPIO.output(17, 0)
            time.sleep(0.5)
            GPIO.output(17, 1)
            time.sleep(0.5)
        elif v1 and v2 and v3 and not v4:
            flight.next_state = macro.BASE_TASK_3_TAKE_OFF
            print('>>> [!] BASE_TASK_3_TAKE_OFF')
            GPIO.output(17, 0)
            time.sleep(0.5)
            GPIO.output(17, 1)
            time.sleep(0.5)
        elif v1 and v2 and not v3 and v4:
            flight.next_state = macro.ADVANCE_TASK_1_TAKE_OFF
            print('>>> [!] ADVANCE_TASK_1_TAKE_OFF')
            GPIO.output(17, 0)
            time.sleep(0.5)
            GPIO.output(17, 1)
            time.sleep(0.5)
        else:
            GPIO.output(12, 0)
            time.sleep(0.5)
            GPIO.output(12, 1)
            time.sleep(0.5)
    else:
        flight.next_state = macro.BASE_TASK_1_TAKE_OFF
        print('>>> [!] BASE_TASK_1_TAKE_OFF')