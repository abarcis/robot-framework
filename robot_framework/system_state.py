#! /usr/bin/env python
from state import State


class SystemState:
    def __init__(self, ids, knowledge, states=None):
        self.ids = ids
        self.knowledge = knowledge
        if states is not None:
            self.states = states
        else:
            self.states = {}
            for ident in self.ids:
                self.states[ident] = State()
