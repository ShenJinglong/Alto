# -*- coding: utf-8 -*-

class base_fsm(object):
    def enter_state(self):
        raise NotImplementedError

    def exec_state(self):
        raise NotImplementedError

    def exit_state(self):
        raise NotImplementedError