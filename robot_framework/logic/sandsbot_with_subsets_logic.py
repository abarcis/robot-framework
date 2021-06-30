#! /usr/bin/env python

import numpy as np

from robot_framework.logic.discrete_sync_and_swarm_logic import (
    DiscretePhaseLogic
)
from robot_framework.logic.base_logic import BaseLogic
from robot_framework.logic.discrete_sync_and_swarm_logic import (
    DiscreteLogic,
    DiscretePhaseLogic,
    DiscretePositionLogic
)
from robot_framework.utils.order_parameters import potential_M_N
from .state_update import StateUpdate
from pyquaternion import Quaternion


class SandsbotsPositionWithSubsetsLogic:
    def __init__(self, params={}):
        self.collaborators = params.get('collaborators', None)
        self.goal = params.get('goal', None)
        self.max_speed = params.get('max_speed', 0.2)
        self.max_angular_speed = 1
        self.agent_radius = params.get('agent_radius', 0.1)
        self.min_distance = params.get('min_distance', 0.1)
        self.min_speed = 0.01
        self.attraction_factor = params.get('attraction_factor', 0.5)
        self.repulsion_factor = params.get('repulsion_factor', 2)

        self.J = params.get('J', 0.1)
        self.params = params
        self.time_step = params['time_delta'] * params['small_phase_steps']

        self.sync_interaction = params.get('sync_interaction', True)

        self.speed_limit = params.get('speed_limit', True)
        self.repulsion_range = params.get('repulsion_range', None)
        self.started = False

    def update_params(self, params={}):
        self.started = False
        self.collaborators = params.get('collaborators', None)
        new_goal = params.get('goal', None)
        if new_goal is not None:
            new_goal = np.array(new_goal + [0])
        self.goal = new_goal

    def update_position(self, state, states):
        position = state.position

        positions_collaborators = np.array([
            s.position for ident, s in states.items()
            if s.position is not None and (
                self.collaborators is not None
                and ident in self.collaborators
            )
        ])
        N = len(positions_collaborators) + 1

        phases_collaborators = np.array([
            s.phase for ident, s in states.items()
            if s.position is not None and (
                self.collaborators is not None
                and ident in self.collaborators
            )
        ])

        positions_others = np.array([
            s.position for ident, s in states.items()
            if s.position is not None
            and (
                self.collaborators is None
                or ident not in self.collaborators
            )
        ])

        rep_others = np.zeros(3)
        attr = np.zeros(3)
        rep = np.zeros(3)
        step_size = 1
        max_speed = self.max_speed

        if len(positions_collaborators):
            self.started = True
            pos_diffs_collaborators = positions_collaborators - position
            norm_collaborators = np.linalg.norm(pos_diffs_collaborators, axis=1)
            min_distance = np.min(norm_collaborators)
            max_speed = min(
                self.max_speed,
                max(
                    min_distance - self.agent_radius * 2 - self.min_distance,
                    0.0001
                ) / (4 * self.time_step)
            )
            step_size = 1
            if self.speed_limit:
                worst_case_distances = (
                    norm_collaborators - 2 * max_speed * self.time_step
                )

                max_gradient = 1./N * sum(
                    [np.abs(
                        self.attraction_factor * (1 + self.J) +
                        self.repulsion_factor/(dist - 2 * self.agent_radius) ** 2
                    ) for dist in worst_case_distances]
                )
                step_size = 1. / 2. / max_gradient / self.time_step

            attr = np.array([
                norm_collaborators[j] *
                pos_diffs_collaborators[j]/norm_collaborators[j]
                for j in range(len(norm_collaborators))
            ]) * self.attraction_factor
            if self.sync_interaction:
                phase = float(state.phase_level)/state.phase_levels_number
                phase_potential = potential_M_N(
                    self.params['K'],
                    self.params['M'],
                    states=None,
                    phases=[
                        float(p)/state.phase_levels_number
                        for p in phases_collaborators
                    ] + [phase],
                )

                if self.speed_limit and self.sync_interaction:
                    step_size *= (1 - phase_potential**(1./self.params['M']))
                phase_diffs = [
                    (
                        (float(p)/state.phase_levels_number) -
                        phase
                    ) * 2 * np.pi
                    for p in phases_collaborators
                ]
                F = np.transpose((1 + self.J * np.cos(phase_diffs))[np.newaxis])

                attr = np.multiply(attr, F)
            rep = np.array([
                pos_diffs_collaborators[j]/(
                    max(
                        norm_collaborators[j] - self.agent_radius * 2,
                        self.min_distance
                    ) * norm_collaborators[j]
                )
                for j in range(len(norm_collaborators))
            ]) * self.repulsion_factor
        if len(positions_others):
            pos_diffs_others = positions_others - position
            norm_others = np.linalg.norm(pos_diffs_others, axis=1)
            norm_others_filtered = norm_others[norm_others < self.repulsion_range]
            pos_diffs_others_filtered = pos_diffs_others[
                norm_others < self.repulsion_range
            ]
            rep_others = np.sum(np.array([
                pos_diffs_others_filtered[j]/(
                    max(
                        norm_others_filtered[j] - self.agent_radius * 2, self.min_distance
                    ) * norm_others_filtered[j]
                )
                for j in range(len(norm_others_filtered))
            ]) * self.repulsion_factor, axis=0)
        goal_attr = 0
        if self.goal is not None:
            goal_diff = self.goal - position
            goal_dist = np.linalg.norm(goal_diff)
            # print(goal_dist)
            goal_attr = goal_diff * (0.5 - 0.1 / goal_dist**2)
        vel = 1./N * np.sum(attr - rep, axis=0) - rep_others + goal_attr
        vel_norm = np.linalg.norm(vel)
        if vel_norm > 0:
            speed = min((
                vel_norm,
                step_size * vel_norm,
                max_speed,
                step_size * max_speed)
            )

            vel = vel/vel_norm * speed
        else:
            vel = np.zeros(3)
        vel[2] = 0  # controlling only XY

        return vel

    def update_orientation(self, state):
        if self.goal is not None:
            goal_diff = self.goal - state.position
            angle = np.arctan2(goal_diff[1], goal_diff[0])
            # angle_diff = np.sin(angle-state.angle_xy)
            # angular_speed = angle_diff / self.time_step * 0.5
            quaternion = Quaternion(axis=[0, 0, 1], angle=angle)
            return quaternion
        return Quaternion(axis=[0, 0, 1], angle=3.14)


class SandsbotWithSubsetsLogic(BaseLogic):
    def __init__(self, params={}):
        super().__init__(
            SandsbotsPositionWithSubsetsLogic,
            DiscretePhaseLogic,
            params
        )
        self.velocity_updates = {}
        self.orientation_updates = {}
        self.phase_level_deltas = {}
        self.phase_levels_number = params.get('phase_levels_number', 1)
        self.small_phase_steps = params.get('small_phase_steps', 10)
        self.time_delta = params.get('time_delta', 0.1)
        self.params = params
        self.update_params(params)

    def update_params(self, params):
        self.collaborators = params.get('collaborators', None)
        self.params = params
        self.position_logic.update_params(params)
        self.phase_logic.update_params(params)

        # parse params

    def update_state(self, state, states, ident=None):
        velocity_update = None
        orientation_update = None
        phase_updates = self.phase_logic.update_phase(
            state,
        )
        small_phase = phase_updates["small_phase"]

        if small_phase == np.floor(0.5 * self.small_phase_steps):
            positions_and_phases = [
                (s.position, s.phase_level)
                for ident2, s in states.items()
                if s and s.position is not None
                and (
                    self.collaborators is not None and
                    ident2 in self.collaborators
                )
            ]
            state = state.predict(
               (
                   self.small_phase_steps
                   - small_phase  # np.floor(0.5 * self.small_phase_steps)
               ) * self.time_delta
            )
            if np.array([s.position for s in states.values()]).any():
                self.velocity_updates[ident] = self.position_logic.update_position(
                    state, states
                )
            self.orientation_updates[ident] = (
                self.position_logic.update_orientation(
                    state,
                )
            )
            if len(positions_and_phases) > 0:
                positions, phases = zip(*positions_and_phases)
                positions = np.array(positions)
                phases = np.array(phases)
                phase_correction_update = self.phase_logic.update_discrete_phase(
                    state, positions, phases
                )
                self.phase_level_deltas[ident] = (
                    phase_correction_update["level_delta"]
                )
                phase_updates["phase_correction"] = (
                    phase_correction_update["phase_correction"]
                )

        if small_phase == 0:
            velocity_update = self.velocity_updates.pop(ident, None)
            orientation_update = self.orientation_updates.pop(ident, None)
            phase_updates["phase_level"] = (
                phase_updates["phase_level"] +
                self.phase_level_deltas.pop(ident, 0)
            ) % state.phase_levels_number

        state_update = StateUpdate(
            velocity_update=velocity_update,
            orientation_update=orientation_update,
            phase_level_update=phase_updates["phase_level"],
            small_phase_update=phase_updates["small_phase"],
            phase_correction_update=phase_updates.pop("phase_correction", None)
        )

        return state_update


