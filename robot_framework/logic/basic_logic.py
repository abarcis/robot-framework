#! /usr/bin/env python

import numpy as np

from base_logic import BaseLogic


class BasicPhaseLogic:
    def __init__(self, params={}):
        return

    def update_phase(self, state, positions, phases):
        # TODO put some basic logic
        phase = state.phase + 0.01
        if phase > 1:
            phase = 0
        return phase


class BasicPositionLogic:
    def __init__(self, params={}):
        self.attraction_factor = 0.2
        self.repulsion_factor = 0.4
        self.agent_radius = 0.1

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


class BasicLogic(BaseLogic):
    def __init__(self, params={}):
        super(BasicLogic, self).__init__(BasicPositionLogic, BasicPhaseLogic)
