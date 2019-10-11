#! /usr/bin/env python

import numpy as np
from state import StateUpdate


class BasicPositionLogic:
    def __init__(self, params={}):
        self.attraction_factor = 0.2
        self.repulsion_factor = 0.4
        self.agent_radius = 0.1

    def update_position(self, state, states):
        positions = np.array([s.position for ident, s in states])
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


class BasicPhaseLogic:
    def __init__(self, params={}):
        return

    def update_phase(self, state, states):
        # TODO put some basic logic
        phase = state.phase + 0.01
        if phase > 1:
            phase = 0
        return phase


class SyncAndSwarmPhaseLogic:
    def __init__(self, params={}):
        self.K = params['K']

    def update_phase(self, state, positions, phases):
        position = state.position
        phase = state.phase * 2 * np.pi
        N = len(phases)
        pos_diffs = positions - position
        phase_diffs = phases - phase
        norm = np.linalg.norm(pos_diffs, axis=1)
        return (phase + float(self.K) / N * np.sum(
            np.array([
                np.sin(phase_diffs[j]) / norm[j]
                for j in range(N)
            ])
        )) % (2 * np.pi) / (2 * np.pi)


class SyncAndSwarmPositionLogic:
    def __init__(self, params={}):
        self.J = params['J']
        self.scale = 0.7
        self.agent_radius = 0.25
        self.max_speed = 0.2
        self.rep_coeff = 1.

    def update_position(self, state, positions, phases):
        position = state.position
        phase = state.phase * 2 * np.pi
        N = len(positions)
        pos_diffs = positions - position
        phase_diffs = phases - phase
        norm = np.linalg.norm(pos_diffs, axis=1)
        attr = np.array([
            pos_diffs[j]/norm[j] * (1 + self.J * np.cos(phase_diffs[j]))
            for j in range(N)
        ])
        rep = np.array([
            pos_diffs[j]/max(norm[j] - self.agent_radius, 0.000001)**2
            for j in range(N)
        ])
        vel = self.scale * 1./N * np.sum(attr - self.rep_coeff * rep, axis=0)
        vel[2] = 0  # controlling only XY
        speed = np.linalg.norm(vel)
        if speed > self.max_speed:
            vel = vel/speed * self.max_speed

        return vel


class BaseLogic(object):
    def __init__(self, position_logic_cls, phase_logic_cls, params={}):
        self.position_logic = position_logic_cls(params)
        self.phase_logic = phase_logic_cls(params)

    def update_state(self, state, states):
        positions_and_phases = [(s.position, s.phase) for ident, s in states]
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


class BasicLogic(BaseLogic):
    def __init__(self, params={}):
        super(BasicLogic, self).__init__(BasicPositionLogic, BasicPhaseLogic)


class SyncAndSwarmLogic(BaseLogic):
    def __init__(self, params={}):
        if 'J' not in params.keys():
            params['J'] = 0.1
        if 'K' not in params.keys():
            params['K'] = -1
        super(SyncAndSwarmLogic, self).__init__(
            SyncAndSwarmPositionLogic,
            SyncAndSwarmPhaseLogic,
            params
        )
