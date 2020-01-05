#! /usr/bin/env python

from datetime import datetime


class BaseKnowledge:
    def __init__(self, ids, max_age=None):
        raise NotImplementedError()

    def update_state(self, own_ident, other_ident, new_state):
        raise NotImplementedError()

    def get_state(self, own_ident, other_ident):
        raise NotImplementedError()

    def get_own_state(self, own_ident):
        self.get_state(self, own_ident, own_ident)

    def get_states_except_own(self, own_ident):
        raise NotImplementedError

    def get_all_states(self, own_ident):
        raise NotImplementedError()

    def get_all_agents(self):
        return self.knowledge.keys()

    def filter_states_by_age(self, states):
        if self.max_age is None:
            return states
        result = {}
        for ident, state in states.items():
            if state:
                delta = datetime.now() - state.timestamp
                if delta.total_seconds() <= self.max_age:
                    result[ident] = state
        return result
