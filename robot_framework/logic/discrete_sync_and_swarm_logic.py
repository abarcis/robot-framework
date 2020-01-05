#! /usr/bin/env python

import numpy as np

from .base_logic import BaseLogic
from .state_update import StateUpdate


class DiscretePhaseLogic:
    def __init__(self, params={}):
        return

    def update_phase(self, state, positions=None, phases=None):
        if positions is not None and phases is not None:
            return 0
        phase = state.phase + 0.1
        if phase >= 1:
            phase = 0
        return phase


class DiscretePositionLogic:
    def __init__(self, params={}):
        self.attraction_factor = 0.1
        self.repulsion_factor = 0.2
        self.agent_radius = 0.01

    def update_position(self, state, positions, phases):
        # positions = np.array([s.position for ident, s in states])
        position = state.position
        N = len(positions)
        pos_diffs = positions - position
        norm = np.linalg.norm(pos_diffs, axis=1)
        attr = 3 * np.array([
            pos_diffs[j]/norm[j] for j in range(len(norm))
        ]) * self.attraction_factor
        rep = np.array([
            pos_diffs[j]/max(norm[j] - self.agent_radius, 0.000001)**2
            for j in range(len(norm))
        ]) * self.repulsion_factor
        vel = 1./N * np.sum(attr - rep, axis=0)
        vel[2] = 0  # controlling only XY

        return vel


class DiscreteLogic(BaseLogic):
    def __init__(self, params={}):
        super(DiscreteLogic, self).__init__(
            DiscretePositionLogic, DiscretePhaseLogic
        )
        self.velocity_updates = {}

    def update_state(self, state, states, ident=None):
        if state.phase == 0.5:
            positions_and_phases = [
                (s.position, s.phase)
                for ident, s in states
                if s.position is not None
            ]
            state = state.predict(0.5)
            positions, phases = zip(*positions_and_phases)
            positions = np.array(positions)
            phases = np.array(phases) * 2 * np.pi
            self.velocity_updates[ident] = self.position_logic.update_position(
                state, positions, phases
            )
            self.phase_update = self.phase_logic.update_phase(
                state, positions, phases
            )

        velocity_update = None
        phase_update = self.phase_logic.update_phase(state)

        if state.phase == 0:
            velocity_update = self.velocity_updates.pop(ident, None)

        state_update = StateUpdate(
            velocity_update=velocity_update,
            phase_update=phase_update
        )

        return state_update
