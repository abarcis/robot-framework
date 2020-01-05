#! /usr/bin/env python

from datetime import datetime

from .base_communication import BaseCommunication
from state import State


class OfflineDistributedCommunication(BaseCommunication):
    def send_state_unconditionally(self, sender, state, recipient_ident):
        self.receive_state(recipient_ident, state, sender)

    def receive_state(self, own_ident, state, sender_ident):
        if own_ident:
            idents = [own_ident]
        else:
            idents = list(self.system_state.knowledge.get_all_agents())

        for ident in idents:
            if self.check_receive_filters(ident, sender_ident, state):
                self.receive_state_unconditionally(
                    ident, state, sender_ident
                )

    def receive_state_unconditionally(self, own_ident, state, sender_ident):
        self.system_state.knowledge.update_state(
            own_ident=own_ident,
            other_ident=sender_ident,
            new_state=State(state=state, received_timestamp=datetime.now())
        )
