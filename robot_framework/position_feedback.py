#! /usr/bin/env python


class PositionFeedback:
    def __init__(self, states, time_delta):
        self.states = states
        self.time_delta = time_delta

    def get_new_position(self, ident):
        state = self.states[ident]
        return state.position + state.velocity * self.time_delta
