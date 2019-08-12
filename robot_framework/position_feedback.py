#! /usr/bin/env python


class PositionFeedback:
    def get_new_position(self, state, velocity, time_delta):
        return state.position + velocity * time_delta
