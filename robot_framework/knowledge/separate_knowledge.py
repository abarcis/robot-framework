#! /usr/bin/env python

from base_knowledge import BaseKnowledge


class SeparateKnowledge(BaseKnowledge):
    def __init__(self, ids):
        self.knowledge = {
            ident: {ident2: None for ident2 in ids}
            for ident in ids
        }

    def update_state(self, own_ident, other_ident, new_state):
        self.knowledge[own_ident][other_ident] = new_state

    def get_state(self, own_ident, other_ident):
        return self.knowledge[own_ident][other_ident]

    def get_all_states(self, own_ident):
        return self.knowledge[own_ident]
