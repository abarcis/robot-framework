#! /usr/bin/env python

import numpy as np
from pyquaternion import Quaternion
from random import uniform

from .orientation import OrientationOpts


class State:
    def __init__(self, position=None, orientation=None, phase=None):
        """
        @arg position position, `None` represents unknown position
        @arg orientation orientation, `None` represnts unknown orientation
        @arg phase phase, `None` represents random phase
        """
        self.position = position
        self.reset_phase(phase)
        self.orientation = orientation

    def reset_phase(self, phase=None):
        self.phase = phase
        if phase is None:
            self.phase = uniform(0, 2*np.pi)

    def __repr__(self):
        return "<State({}, {}, {})>".format(
            self.position, self.orientation, self.phase
        )

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation = value
        if value is not None:
            self._angle_xy = self.compute_angle_xy(value)

    @property
    def angle_xy(self):
        return self._angle_xy

    @staticmethod
    def compute_angle_xy(quaternion):
        vec = quaternion.rotate([1, 0, 0])
        vec_xy = vec
        vec_xy[2] = 0
        angle = np.arctan2(vec_xy[1], vec_xy[0])
        return angle


def create_state_random(dim=2, orient=OrientationOpts.ENABLED):
    if dim > 2:
        z = np.random.uniform(-1, 1)
    else:
        z = 0

    if orient == OrientationOpts.DISABLED:
        orientation = np.zeros(4)
    else:
        orientation = np.zeros(4)
        orientation[0] = np.random.uniform(-1, 1)
        orientation[3] = np.random.uniform(-1, 1)
        '''if dim == 3:
            orientation[0] = np.random.uniform(-1, 1)
            orientation[1] = np.random.uniform(-1, 1)
        orientation = orientation / np.linalg.norm(orientation)
        '''
    orientation = Quaternion(orientation)
    position = np.array(
        [np.random.uniform(-1, 1), np.random.uniform(-1, 1), z])
    phase = uniform(0, 2*np.pi)
    return State(position, orientation, phase)


def serialize_state_to_dict(state):
    return {
        'pos_x': state.position[0],
        'pos_y': state.position[1],
        'pos_z': state.position[2],
        'phase': state.phase,
        'orientation_x': state.orientation[1],
        'orientation_y': state.orientation[2],
        'orientation_z': state.orientation[3],
        'orientation_w': state.orientation[0],
        'angle_xy': state.angle_xy,
    }
