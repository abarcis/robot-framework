#! /usr/bin/env python


class BaseCommunication:
    def __init__(self, system_state):
        self.system_state = system_state

    def send_state(self, own_ident, state):
        raise NotImplementedError()


class OfflineCommunication(BaseCommunication):
    def send_state(self, sender, state):
        self.system_state.knowledge.update_state(
            own_ident=None,
            other_ident=sender,
            new_state=state
        )
