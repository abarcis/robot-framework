#! /usr/bin/env python

import numpy as np
from datetime import datetime


class State:
    def __init__(self, phase=None, position=None, velocity=None, state=None,
                 received_timestamp=None, sent_timestamp=None,
                 phase_levels_number=1):
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
            self.phase_levels_number = state.phase_levels_number
            self.phase_level = state.phase_level
            self.velocity = state.velocity
            self.is_teleoperated = state.is_teleoperated
        else:
            if not sent_timestamp:
                self.timestamp = datetime.now()
            else:
                self.timestamp = sent_timestamp
            self.phase_levels_number = phase_levels_number
            if phase is not None:
                self.phase = phase
                # small_period = 1 / phase_levels_number
                # self.phase_level = int(phase / small_period)
                # self.phase = (
                #     phase - self.phase_level * small_period
                # ) / small_period
            else:
                self.phase = 0
                # self.phase = np.random.random()
            self.phase_level = 0

            if position is not None:
                self.position = position
            else:
                self.position = (np.random.random(3) - 0.5) * self.area_size
                #self.position[1] = 0
                self.position[2] = 0

            if velocity is not None:
                self.velocity = velocity
            else:
                self.velocity = np.zeros(3)

            self.is_teleoperated = False

    def update(self, ident, state_update=None, position_feedback=None):
        self.timestamp = datetime.now()
        if state_update:
            if state_update.phase_update is not None:
                self.phase = state_update.phase_update

            if state_update.velocity_update is not None:
                self.velocity = state_update.velocity_update

            if state_update.position_update is not None:
                self.position = state_update.position_update

            if state_update.teleop_update:
                self.is_teleoperated = True
            else:
                self.is_teleoperated = False

        if position_feedback is not None:
            new_position = position_feedback.get_new_position(
                ident
            )
            self.position = new_position

    def predict(self, time_delta=0):
        future_state = State(
            self.phase,
            self.position + self.velocity * time_delta,
            self.velocity
        )

        return future_state

    def __str__(self):
        return "phase: {:.3f}, phase level: {}, \
            position: {}, velocity: {}".format(
                self.phase,
                self.phase_level,
                self.position,
                self.velocity
            )

    def set_received_time(self):
        self.received_timestamp = datetime.now()

    def __repr__(self):
        return "<State: {}>".format(self.__str__())
