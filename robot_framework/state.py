#! /usr/bin/env python

from __future__ import division

import numpy as np
from pyquaternion import Quaternion
from datetime import datetime


class State:
    def __init__(self, phase=None, position=None, velocity=None,
                 orientation=None, angular_speed=None, state=None,
                 received_timestamp=None, sent_timestamp=None,
                 params={}, initialize=True):
        self.area_size = 10
        if received_timestamp:
            self.received_timestamp = received_timestamp
        elif state:
            self.received_timestamp = state.received_timestamp
        else:
            self.received_timestamp = None
        if state:
            self.phase = state.phase
            self.position = state.position
            self.timestamp = state.timestamp
            self.phase_level = state.phase_level
            self.phase_levels_number = state.phase_levels_number
            self.small_phase = state.small_phase
            self.phase_correction = state.phase_correction
            self.velocity = state.velocity
            self.is_teleoperated = state.is_teleoperated
            self.orientation = state.orientation
            self.angular_speed = state.angular_speed
            self.orientation_mode = state.orientation_mode
            self.constraint_mode = state.constraint_mode
        else:
            self.orientation_mode = params.get('orientation_mode', False)
            self.constraint_mode = params.get('constraint_mode', False)
            if not sent_timestamp:
                self.timestamp = datetime.now()
            else:
                self.timestamp = sent_timestamp
            self.phase_levels_number = params.get('phase_levels_number', 1)
            if phase is not None:
                self.phase = phase
            else:
                # self.phase = 0
                self.phase = np.random.random()
            small_period = 1. / self.phase_levels_number
            self.phase_level = int(self.phase / small_period)
            self.small_phase = 0  # small phases synchronized at the beginning
            # self.small_phase = (
            #     phase - self.phase_level * small_period
            # ) / small_period
            self.phase_correction = 0

            self.orientation = orientation

            self.angular_speed = 0

            if (
                orientation is None and
                initialize and
                self.orientation_mode
            ):
                orient = Quaternion(
                    axis=[0, 0, 1],
                    angle=np.random.uniform(0, 2 * np.pi)
                )
                self.orientation = orient

            if position is None and initialize:
                self.position = (np.random.random(3) - 0.5) * self.area_size
                # self.position[1] = 0
                self.position[2] = 0
            else:
                self.position = None

            if velocity is not None:
                self.velocity = velocity
            else:
                self.velocity = np.zeros(3)

            self.is_teleoperated = False

        if position is not None:
            self.position = position

        if angular_speed is not None:
            self.angular_speed = angular_speed

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

    def update(self, ident, state_update=None, position_feedback=None):
        self.timestamp = datetime.now()
        if state_update:
            if state_update.phase_update is not None:
                self.phase = state_update.phase_update

            if state_update.small_phase_update is not None:
                self.small_phase = state_update.small_phase_update

            if state_update.phase_level_update is not None:
                self.phase_level = state_update.phase_level_update
                small_period = 1. / self.phase_levels_number
                self.phase = self.phase_level * small_period

            if state_update.phase_correction_update is not None:
                self.phase_correction = (
                    state_update.phase_correction_update
                )

            if state_update.velocity_update is not None:
                self.velocity = state_update.velocity_update

            if state_update.angular_speed_update is not None:
                self.angular_speed = state_update.angular_speed_update

            if state_update.position_update is not None:
                self.position = state_update.position_update

            if state_update.orientation_update is not None:
                self.orientation = state_update.orientation_update

            if state_update.teleop_update:
                self.is_teleoperated = True
            else:
                self.is_teleoperated = False

        if position_feedback is not None:
            new_position = position_feedback.get_new_position(
                ident
            )
            if self.orientation_mode:
                new_orientation = position_feedback.get_new_orientation(
                    ident
                )
                self.orientation = new_orientation
            self.position = new_position

    def predict(self, time_delta=0):
        if not self.constraint_mode:
            future_state = State(
                state=self,
                position=self.position + self.velocity * time_delta,
            )
        else:
            orient = self.orientation
            vel = orient.rotate(self.velocity)
            q = Quaternion(
                axis=[0, 0, 1],
                angle=self.angular_speed * time_delta
            )
            future_state = State(
                state=self,
                position=self.position + vel * time_delta,
                orientation=self.orientation * q,
            )

        return future_state

    def __str__(self):
        return "phase: {:.3f}, small_phase: {}, phase level: {}, \
            position: {}, velocity: {}".format(
                self.phase,
                self.small_phase,
                self.phase_level,
                self.position,
                self.velocity
            )

    def set_received_time(self):
        self.received_timestamp = datetime.now()

    def __repr__(self):
        return "<State: {}>".format(self.__str__())
