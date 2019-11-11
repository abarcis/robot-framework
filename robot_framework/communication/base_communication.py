#! /usr/bin/env python


class BaseCommunication:
    def __init__(self, system_state):
        self.system_state = system_state

    def send_state(self, own_ident, state):
        raise NotImplementedError()
