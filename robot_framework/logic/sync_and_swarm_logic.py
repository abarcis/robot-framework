#! /usr/bin/env python

import numpy as np
import logging

from base_logic import BaseLogic


class SyncAndSwarmPhaseLogic:
    def __init__(self, params={}):
        self.K = params['K']
        self.natural_frequency = params['natural_frequency']
        self.oscillations_on = True

    def toggle_oscillations(self):
        self.oscillations_on = not self.oscillations_on
        logging.info(
            "Switching oscillations to: {}".format(
                self.oscillations_on
            )
        )

    def update_params(self, params):
        if 'natural_frequency' in params.keys():
            self.natural_frequency = params['natural_frequency']
        if 'K' not in params.keys():
            return
        self.K = params['K']

    def update_phase(self, state, positions, phases):
        position = state.position
        phase = state.phase * 2 * np.pi
        N = len(phases)
        pos_diffs = positions - position
        phase_diffs = phases - phase
        norm = np.linalg.norm(pos_diffs, axis=1)
        # TODO add dependency of time_delta
        if self.oscillations_on:
            natural_increase = self.natural_frequency * 2 * np.pi
        else:
            natural_increase = 0
        return (natural_increase + phase + float(self.K) / N * np.sum(
            np.array([
                np.sin(phase_diffs[j]) / norm[j]
                for j in range(N)
            ])
        )) % (2 * np.pi) / (2 * np.pi)


class SyncAndSwarmPositionLogic:
    def __init__(self, params={}):
        self.J = params['J']
        self.align_center = params.get('align_center', False)
        self.scale = 0.4
        self.agent_radius = 0.25
        self.max_speed = 0.1
        self.min_distance = 0.9
        if self.min_distance > 0.75:
            self.rep_coeff = 1.5
        else:
            self.rep_coeff = 1
        self.align_coeff = 0.05

    def update_params(self, params):
        self.align_center = params.get('align_center', False)
        if 'J' not in params.keys():
            return
        self.J = params['J']

    def update_position(self, state, positions, phases):
        position = state.position
        phase = state.phase * 2 * np.pi
        N = len(positions)
        pos_diffs = positions - position
        phase_diffs = phases - phase
        norm = np.linalg.norm(pos_diffs, axis=1)
        too_close = [n for n in norm if n < self.min_distance]
        if too_close:
            print("Warning: TOO CLOSE! {}".format(too_close))
        attr = np.array([
            pos_diffs[j]/norm[j] * (1 + self.J * np.cos(phase_diffs[j]))
            for j in range(N)
        ])
        rep = np.array([
            pos_diffs[j]/max(norm[j] - self.agent_radius, 0.000001)**2
            for j in range(N)
        ])
        vel = self.scale * 1./N * np.sum(attr - self.rep_coeff * rep, axis=0)
        if self.align_center:
            vel -= self.align_coeff * position
        vel[2] = 0  # controlling only XY
        speed = np.linalg.norm(vel)
        if speed > self.max_speed:
            vel = vel/speed * self.max_speed

        return vel


class SyncAndSwarmLogic(BaseLogic):
    def __init__(self, params={}):
        if 'J' not in params.keys():
            params['J'] = 0.1
        if 'K' not in params.keys():
            params['K'] = -1
        if 'natural_frequency' not in params.keys():
            params['natural_frequency'] = 0
        if 'time_delta' not in params.keys():
            params['time_delta'] = 1
        super(SyncAndSwarmLogic, self).__init__(
            SyncAndSwarmPositionLogic,
            SyncAndSwarmPhaseLogic,
            params
        )
