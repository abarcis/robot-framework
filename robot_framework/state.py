#! /usr/bin/env python

import numpy as np
from datetime import datetime


class State:
    def __init__(self, phase=None, position=None, velocity=None, state=None,
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
        else:
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
            state=self,
            position=self.position + self.velocity * time_delta,
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
