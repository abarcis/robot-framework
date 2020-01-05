#! /usr/bin/env python


class BaseCommunication:
    def __init__(self, system_state, send_filters=[], receive_filters=[]):
        self.system_state = system_state
        self.send_filters = send_filters
        self.receive_filters = receive_filters

    def send_state(self, own_ident, state, recipient_ident=None, filters=[]):
        if self.check_send_filters(own_ident, recipient_ident, state):
            self.send_state_unconditionally(own_ident, state, recipient_ident)

    def receive_state(self, own_ident, state, sender_ident):
        raise NotImplementedError()

    def send_state_unconditionally(self, own_ident, state, recipient_ident):
        raise NotImplementedError()

    def check_send_filters(self, own_ident, recipient_ident, state):
        results = [
            f.check(own_ident, recipient_ident, state)
            for f in self.send_filters
        ]
        return all(results)

    def check_receive_filters(self, own_ident, sender_ident, state):
        results = [
            f.check(own_ident, sender_ident, state)
            for f in self.receive_filters
        ]
        return all(results)
