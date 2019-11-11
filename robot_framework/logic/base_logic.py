#! /usr/bin/env python

import numpy as np
from state_update import StateUpdate


class BaseLogic(object):
    def __init__(self, position_logic_cls, phase_logic_cls, params={}):
        self.position_logic = position_logic_cls(params)
        self.phase_logic = phase_logic_cls(params)

    def update_params(self, params):
        self.position_logic.update_params(params)
        self.phase_logic.update_params(params)

    def update_state(self, state, states):
        positions_and_phases = [
            (s.position, s.phase)
            for ident, s in states
            if s.position is not None
        ]
        positions, phases = zip(*positions_and_phases)
        positions = np.array(positions)
        phases = np.array(phases) * 2 * np.pi
        state_update = StateUpdate(
            velocity_update=self.position_logic.update_position(
                state, positions, phases
            ),
            phase_update=self.phase_logic.update_phase(
                state, positions, phases
            )
        )
        return state_update
