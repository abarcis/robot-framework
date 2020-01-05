#! /usr/bin/env python

from .base_knowledge import BaseKnowledge


class SeparateKnowledge(BaseKnowledge):
    def __init__(self, ids, max_age=None):
        self.max_age = max_age
        self.knowledge = {
            ident: {ident2: None for ident2 in ids}
            for ident in ids
        }

    def update_state(self, own_ident, other_ident, new_state):
        self.knowledge[own_ident][other_ident] = new_state

    def get_state(self, own_ident, other_ident):
        return self.knowledge[own_ident][other_ident]

    def get_all_states(self, own_ident):
        return self.filter_states_by_age(self.knowledge[own_ident])

    def get_states_except_own(self, own_ident):
        states_except_own = (
            (k, v)
            for k, v in self.get_all_states(own_ident).items()
            if k != own_ident
        )
        return states_except_own
