#! /usr/bin/env python

from base_communication import BaseCommunication


class OfflineCommunication(BaseCommunication):
    def send_state(self, sender, state):
        self.system_state.knowledge.update_state(
            own_ident=None,
            other_ident=sender,
            new_state=state
        )
