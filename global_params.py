# -*- coding: utf-8 -*-
import sys

RECORD_VIDEO = False

IMAGE_WIDTH = 160
IMAGE_HEIGHT = 120

FLY_ANGLE_P = 110
FLY_ANGLE_N = 90

POINT_RECORD_NUM = 5
NUMBER_RECORD_NUM = 5

RASPBERRY_MODE = False if sys.platform == 'win32' else True

MODE_PIN_1 = 7
MODE_PIN_2 = 8
MODE_PIN_3 = 9
MODE_PIN_4 = 10