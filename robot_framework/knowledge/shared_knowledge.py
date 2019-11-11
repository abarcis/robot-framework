#! /usr/bin/env python

from base_knowledge import BaseKnowledge


class SharedKnowledge(BaseKnowledge):
    def __init__(self, ids):
        self.knowledge = {ident: None for ident in ids}

    def update_state(self, own_ident, other_ident, new_state):
        self.knowledge[other_ident] = new_state

    def get_state(self, own_ident, other_ident):
        return self.knowledge[other_ident]

    def get_all_states(self, own_ident):
        return self.knowledge

    def get_states_except_own(self, own_ident):
        states_except_own = (
            (k, v)
            for k, v in self.knowledge.items()
            if k != own_ident
        )
        return states_except_own

    def __str__(self):
        return str(self.knowledge)
