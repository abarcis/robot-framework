#! /usr/bin/env python

import numpy as np

from .base_logic import BaseLogic
from .state_update import StateUpdate
from utils.order_parameters import potential_M_N


class DiscretePhaseLogic:
    def __init__(self, params={}):
        self.K = params.get('K', 1)
        self.M = params.get('M', 1)
        self.small_phase_steps = params.get('small_phase_steps', 10)
        self.phase_levels_number = params.get('phase_levels_number', 1)
        self.swarming_interaction = True
        return

    def update_params(self, params):
        self.K = params.get('K', self.K)
        self.M = params.get('M', self.M)

    def phase_attraction(self, phase_diff, distance=0.3):
        K = self.K
        M = self.M
        min_distance = 0.3
        distance_coefficient = min(1/min_distance, 1/distance)  # * (-0.3)

        Kms = [K] * M
        # Kms = [self.phase_levels_number / 2 / np.pi * K] * M
        Kms = [Km - K * 0.1 * distance_coefficient for Km in Kms[:-1]] + [Kms[-1]]
        # Kms = [Km * max(0.75, 1 - 1/distance) for Km in Kms[:-1]] + [Kms[-1]]
        Kms[-1] = -0.1 * Kms[-1] - 0.1 * K * distance_coefficient

        return sum([
            Kms[m-1]/m * np.sin(m * phase_diff * 2 * np.pi)
            for m in range(1, M+1)
        ])

    def noise(self, N):
        n = 2 * (np.random.sample() - 0.5)  # random number from range [-1, 1]
        margin = 0.1 * min(
            np.abs(self.phase_attraction(1/self.phase_levels_number)),
            np.abs(self.phase_attraction(0.5 - 1/self.phase_levels_number))
        ) * 1/2 * 0.99  # slightly smaller than the smallest attraction
        return n * margin * 0.99

    def phase_energy(self, N0, N):
        M = self.M
        margin = 0.1 * min(
            np.abs(self.phase_attraction(1/self.phase_levels_number)),
            np.abs(self.phase_attraction(0.5 - 1/self.phase_levels_number))
        ) * 1/2 * 0.99  # slightly smaller than the smallest attraction
        a = - margin / (N/M - N/(M+1))
        step = 0.004
        if N0 <= N/M and N0 > N/(M+1):
            energy = 0.5 * a * (N0 - N/(M+1))
        elif N0 <= N/(M+1):
            energy = step * (int(N/(M+1)) - N0 + 1)
        else:
            energy = step * (N0 - N/M)
        return (1 + 3 * energy)

    def update_discrete_phase(self, state, positions, phases):
        phase_correction = state.phase_correction
        tmp_phase = state.phase_level/self.phase_levels_number
        phase_diffs = [
            phase / self.phase_levels_number - tmp_phase for phase in phases
        ]
        if self.swarming_interaction:
            position = state.position
            pos_diffs = positions - position
            distances = np.linalg.norm(pos_diffs, axis=1)

        else:
            distances = 1 * len(phase_diffs)

        all_diffs = zip(phase_diffs, distances)
        phase_corr = [
            self.phase_attraction(phase_diff, distance)
            for phase_diff, distance in all_diffs
        ]
        phase_corr_squared = [np.sign(c) * c ** 2 for c in phase_corr]

        N0 = phase_diffs.count(0) + 1
        N = len(phase_diffs) + 1
        phase_correction = self.phase_energy(N0, N) * phase_correction

        # TODO why?
        if sum(phase_corr_squared) == 0:
            corr_delta = 0
        else:
            corr_delta = np.mean(phase_corr)

        # if self.swarming_interaction:
        #     corr_delta = np.average(np.divide(phase_corr, norm))

        phase_correction += (
            - corr_delta +
            self.noise(N)
        )

        # print("energy", self.phase_energy(N0, N))
        # print("correction", phase_correction)

        if np.abs(phase_correction) >= 1:
            discrete_phase_update = {
                "level_delta": np.sign(phase_correction),
                "phase_correction": 0
            }
        else:
            discrete_phase_update = {
                "level_delta": 0,
                "phase_correction": phase_correction
            }
        return discrete_phase_update

    def update_phase(self, state, positions=None, phases=None):
        small_phase = state.small_phase + 1
        phase_level = state.phase_level
        if small_phase >= self.small_phase_steps:
            small_phase = 0
            # phase_level = (phase_level + 1) % state.phase_levels_number
        return {"small_phase": small_phase, "phase_level": phase_level}


class DiscretePositionLogic:
    def __init__(self, params={}):
        self.max_speed = 0.2
        self.max_angular_speed = 1
        self.agent_radius = params.get('agent_radius', 0.1)
        self.min_distance = 0.1
        self.min_speed = 0.01
        # self.step_size = 1/np.abs(1 - 1/self.min_distance)
        self.attraction_factor = params.get('attraction_factor', 0.5)
        self.repulsion_factor = params.get('repulsion_factor', 2)
        self.momentum_param = 0

        self.J = params.get('J', 0.1)
        self.params = params
        self.time_step = params['time_delta'] * params['small_phase_steps']

        self.sync_interaction = True

    def update_params(self, params):
        self.J = params.get('J', self.J)
        self.params = params

    def update_position(self, state, positions, phases):
        # positions = np.array([s.position for ident, s in states])
        last_vel = state.velocity
        position = state.position
        N = len(positions)
        pos_diffs = positions - position
        norm = np.linalg.norm(pos_diffs, axis=1)
        # print('norm', norm)
        min_distance = np.min(norm)
        max_distance = np.max(norm)
        max_speed = min(
            self.max_speed,
            max(
                min_distance - self.agent_radius * 2 - self.min_distance,
                0.0001
            ) / (4 * self.time_step)
        )
        worst_case_distances = norm - 2 * max_speed * self.time_step

        max_gradient = 1/N * sum(
            [np.abs(
                self.attraction_factor * (1 + self.J) +
                self.repulsion_factor/(dist - 2 * self.agent_radius) ** 2
            ) for dist in worst_case_distances]
            # [np.abs(
            #     self.attraction_factor * (1 + self.J) -
            #     self.repulsion_factor/(dist - 2 * self.agent_radius)
            # ) for dist in distance_range]
        )
        step_size = 1 / 2 / max_gradient / self.time_step

        attr = np.array([
            norm[j] *
            pos_diffs[j]/norm[j] for j in range(len(norm))
        ]) * self.attraction_factor
        if self.sync_interaction:
            phase = state.phase_level/state.phase_levels_number
            phase_potential = potential_M_N(
                self.params['K'],
                self.params['M'],
                states=None,
                phases=[p/state.phase_levels_number for p in phases] + [phase],
            )
            # print('step size', step_size, step_size * (1 - phase_potential**(1/10)))
            step_size *= (1 - phase_potential**(1/10))
            phase_diffs = [
                (
                    (p/state.phase_levels_number) -
                    phase
                ) * 2 * np.pi
                for p in phases
            ]
            F = np.transpose((1 + self.J * np.cos(phase_diffs))[np.newaxis])

            attr = np.multiply(attr, F)
        rep = np.array([
            pos_diffs[j]/(
                max(norm[j] - self.agent_radius * 2, self.min_distance) *
                norm[j]
            )
            for j in range(len(norm))
        ]) * self.repulsion_factor
        new_vel = 1./N * np.sum(attr - rep, axis=0)
        vel = (
            self.momentum_param * last_vel +
            (1 - self.momentum_param) * new_vel
        )
        vel[2] = 0  # controlling only XY
        vel_norm = np.linalg.norm(vel)
        # critical_distance = (
        #     4 * self.agent_radius * self.attraction_factor +
        #     self.repulsion_factor +
        #     np.sqrt(
        #         8 * self.agent_radius *
        #         self.attraction_factor *
        #         self.repulsion_factor +
        #         self.repulsion_factor ** 2
        #     )
        # )
        # max_speed = min(
        #     self.max_speed,
        #     max((closest_distance - critical_distance) / 2, self.min_speed)
        # )
        # print(step_size)

        # print((vel_norm, step_size * vel_norm, max_speed))
        speed = min((vel_norm, step_size * vel_norm, max_speed, step_size * max_speed))

        vel = vel/vel_norm * speed

        return vel

    def update_orientation(self, state, velocity):
        # TODO add max ang_vel
        vel_angle = np.arctan2(velocity[1], velocity[0])
        vel_angle_diff = np.sin(
            (vel_angle - state.angle_xy)
        )
        ang_speed = vel_angle_diff / self.time_step
        return max(
            -self.max_angular_speed,
            min(self.max_angular_speed, ang_speed)
        )


class DiscreteLogic(BaseLogic):
    def __init__(self, params={}):
        super(DiscreteLogic, self).__init__(
            DiscretePositionLogic, DiscretePhaseLogic, params
        )
        self.velocity_updates = {}
        self.angular_speed_updates = {}
        self.phase_level_deltas = {}
        self.phase_levels_number = params.get('phase_levels_number', 1)
        self.small_phase_steps = params.get('small_phase_steps', 10)
        self.time_delta = params.get('time_delta', 0.1)
        self.params = params

    def update_params(self, params):
        self.params = params
        self.position_logic.update_params(params)
        self.phase_logic.update_params(params)

    def update_state(self, state, states, ident=None):
        velocity_update = None
        angular_speed_update = None
        phase_updates = self.phase_logic.update_phase(
            state,
        )

        if state.small_phase == np.floor(0.5 * self.small_phase_steps):
            positions_and_phases = [
                (s.position, s.phase_level)
                for ident, s in states
                if s.position is not None
            ]
            state = state.predict(
               (
                   self.small_phase_steps
                   - np.floor(0.5 * self.small_phase_steps)
               ) * self.time_delta
            )
            if len(positions_and_phases) > 0:
            # print('pos predict', state.position)
                positions, phases = zip(*positions_and_phases)
                # print("ID", ident, state.phase_level, phases)
                positions = np.array(positions)
                phases = np.array(phases)
                self.velocity_updates[ident] = self.position_logic.update_position(
                    state, positions, phases
                )
                if state.constraint_mode:
                    vel = self.velocity_updates[ident]
                    self.angular_speed_updates[ident] = (
                        self.position_logic.update_orientation(
                            state, vel
                        )
                    )
                    vel_angle = np.arctan2(vel[1], vel[0])
                    angle_diff = state.angle_xy - vel_angle
                    v_value = np.linalg.norm(vel) * np.cos(angle_diff)
                    self.velocity_updates[ident] = np.array([v_value, 0, 0])

                phase_correction_update = self.phase_logic.update_discrete_phase(
                    state, positions, phases
                )
                self.phase_level_deltas[ident] = (
                    phase_correction_update["level_delta"]
                )
                phase_updates["phase_correction"] = (
                    phase_correction_update["phase_correction"]
                )

        if state.small_phase == 0:
            velocity_update = self.velocity_updates.pop(ident, None)
            angular_speed_update = self.angular_speed_updates.pop(ident, None)
            phase_updates["phase_level"] = (
                phase_updates["phase_level"] +
                self.phase_level_deltas.pop(ident, 0)
            ) % state.phase_levels_number

        state_update = StateUpdate(
            velocity_update=velocity_update,
            angular_speed_update=angular_speed_update,
            phase_level_update=phase_updates["phase_level"],
            small_phase_update=phase_updates["small_phase"],
            phase_correction_update=phase_updates.pop("phase_correction", None)
        )

        return state_update
