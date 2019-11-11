#! /usr/bin/env python


class BaseKnowledge:
    def __init__(self, ids):
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
