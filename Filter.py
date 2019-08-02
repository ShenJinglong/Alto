# -*- coding: utf-8 -*-

import global_params

from tools import math_tools

class MedianFilter(object):
    def __init__(self):
        self.__point_buff_c1 = []
        self.__number_buff_c1 = []
        self.__number_buff_c2 = []

    def add_point_c1(self, point):
        if len(self.__point_buff_c1) < global_params.POINT_RECORD_NUM:
            self.__point_buff_c1.append(point)
        else:
            del self.__point_buff_c1[0]
            self.__point_buff_c1.append(point)

    def get_result_point_c1(self):
        return math_tools.get_average_point(self.__point_buff_c1)

    def add_number_c1(self, number):
        if len(self.__number_buff_c1) < global_params.NUMBER_RECORD_NUM:
            self.__number_buff_c1.append(number)
        else:
            del self.__number_buff_c1[0]
            self.__number_buff_c1.append(number)

    def get_result_number_c1(self):
        return math_tools.get_average_number(self.__number_buff_c1)

    def add_number_c2(self, number):
        if len(self.__number_buff_c2) < global_params.NUMBER_RECORD_NUM:
            self.__number_buff_c2.append(number)
        else:
            del self.__number_buff_c2[0]
            self.__number_buff_c2.append(number)

    def get_result_number_c2(self):
        return math_tools.get_average_number(self.__number_buff_c2)

    def refresh(self):
        self.__point_buff_c1 = []
        self.__number_buff_c1 = []
        self.__number_buff_c2 = []