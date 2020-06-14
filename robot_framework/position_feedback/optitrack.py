#! /usr/bin/env python

from pyquaternion import Quaternion


class Optitrack:
    def __init__(self, states, time_delta):
        self.states = states
        self.time_delta = time_delta
        self.poses = {ident: {'position': None, 'orientation': None}
                      for ident in self.states.keys()}

    def get_new_position(self, ident):
        # state = self.states[ident]
        # if not state.constraint_mode:
        #     return state.position + state.velocity * self.time_delta
        # else:
        #     orient = Quaternion(state.orientation)
        #     vel = orient.rotate(state.velocity)
        #     return state.position + vel * self.time_delta
        return self.poses[ident]['position']

    def get_new_orientation(self, ident):
        # state = self.states[ident]
        # q = Quaternion(
        #     axis=[0, 0, 1],
        #     angle=state.angular_speed * self.time_delta
        # )
        # return state.orientation * q
        return self.poses[ident]['orientation']
