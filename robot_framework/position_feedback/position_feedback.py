#! /usr/bin/env python

from pyquaternion import Quaternion


class PositionFeedback:
    def __init__(self, system_state, time_delta):
        self.system_state = system_state
        self.time_delta = time_delta

    def get_new_position(self, ident):
        state = self.system_state.states[ident]
        if not state.constraint_mode:
            return state.position + state.velocity * self.time_delta
        else:
            orient = Quaternion(state.orientation)
            vel = orient.rotate(state.velocity)
            return state.position + vel * self.time_delta

    def get_new_orientation(self, ident):
        state = self.system_state.states[ident]
        q = Quaternion(
            axis=[0, 0, 1],
            angle=state.angular_speed * self.time_delta
        )
        return state.orientation * q

    def get_node(self):
        return None
