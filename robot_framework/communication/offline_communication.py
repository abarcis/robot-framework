#! /usr/bin/env python

from .base_communication import BaseCommunication
from robot_framework.state import State


class OfflineCommunication(BaseCommunication):
    def send_state_unconditionally(self, sender, state, recipient_ident=None):
        self.system_state.knowledge.update_state(
            own_ident=recipient_ident,
            other_ident=sender,
            new_state=State(state=state)
        )
