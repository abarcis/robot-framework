#! /usr/bin/env python


class BaseFilter:
    def __init__(self, system_state, params={}):
        self.system_state = system_state
        self.params = params

    def check(self):
        raise NotImplementedError()
