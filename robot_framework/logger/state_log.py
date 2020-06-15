#! /usr/bin/env python

from robot_framework.utils.state_utils import copy_all_states


class StateLog:
    def __init__(self, params):
        self.states_log = []
        self.params = params

    def reinit(self, params):
        self.__init__(params)

    def update(self, time, system_state):
        self.states_log.append((time, copy_all_states(system_state)))
