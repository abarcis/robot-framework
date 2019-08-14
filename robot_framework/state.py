#! /usr/bin/env python

import numpy as np


class StateUpdate:
    def __init__(
        self,
        position_update=None,
        phase_update=None,
        velocity_update=None
    ):
        self.position_update = position_update
        self.phase_update = phase_update
        self.velocity_update = velocity_update

    def __str__(self):
        return "phase update: {}, \
            position update: {}, \
            velocity update: {}".format(
                self.phase_update,
                self.position_update,
                self.velocity_update
            )


class State:
    def __init__(self, phase=None, position=None):
        if phase is not None:
            self.phase = phase
        else:
            self.phase = np.random.random()

        if position is not None:
            self.position = position
        else:
            self.position = np.random.random(3)
            self.position[2] = 0

        self.velocity = np.zeros(3)

    def update(self, ident, state_update=None, position_feedback=None):
        if state_update:
            if state_update.phase_update is not None:
                self.phase = state_update.phase_update

            if state_update.velocity_update is not None:
                self.velocity = state_update.velocity_update

            if state_update.position_update is not None:
                self.position = state_update.position_update

        if position_feedback is not None:
            new_position = position_feedback.get_new_position(
                ident
            )
            state_update.position_update = new_position

    def __str__(self):
        return "phase: {:.3f}, position: {}, velocity: {}".format(
            self.phase,
            self.position,
            self.velocity
        )

    def __repr__(self):
        return "<State: {}>".format(self.__str__())
