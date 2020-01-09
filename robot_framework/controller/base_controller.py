#! /usr/bin/env python

import numpy as np
import random


class BaseController(object):
    PHASE_ORDERING = ('RANDOM', 'LEXICOGRAPHIC', 'ANGLE', 'MODIFY')

    def __init__(
        self,
        agents_num,
        logic,
        position_feedback,
        communication,
        system_state,
        visualization,
        params_list=[{}],
        sleep_fcn=None,
        time_delta=0.1,
    ):
        self.agents_num = agents_num
        self.logic = logic
        self.position_feedback = position_feedback
        self.time_delta = time_delta
        self.communication = communication
        self.system_state = system_state
        self.visualization = visualization
        self.sleep_fcn = sleep_fcn
        self.params_list = params_list

    @staticmethod
    def gen_fixed_permutation(l, seed=3):
        s = list(range(len(l)))
        i = 0
        for _ in range(len(l)):
            i += seed
            i %= len(s)
            yield l[s.pop(i)]
            i -= 1

    def reassign_phases(self, phase_ordering='RANDOM'):
        if phase_ordering == 'MODIFY':
            for ident, state in self.system_state.states.items():
                state.phase += random.uniform(-0.01, 0.01)
            return

        ids = [
            i for i in self.system_state.ids
            if self.system_state.states[i].position is not None
        ]
        uniform_phases = [1. / len(ids) * i  # + random.uniform(-0.01, 0.01)
                          for i in range(len(ids))]
        if (
            phase_ordering in self.PHASE_ORDERING
            and phase_ordering != 'RANDOM'
        ):
            pos = [tuple(self.system_state.states[i].position) for i in ids]
            agents = list(zip(ids, pos))
            if phase_ordering == 'LEXICOGRAPHIC':

                def key(x):
                    return x[1]
            elif phase_ordering == 'ANGLE':
                middle = (
                    np.average([p[0] for p in pos]),
                    np.average([p[1] for p in pos])
                )

                avg_dist = 1.5

                def key(p):
                    x = p[1][0]
                    y = p[1][1]
                    dist = np.sqrt((x - middle[0]) ** 2 + (y - middle[1]) ** 2)
                    angle = np.arctan2(x - middle[0], y - middle[1])
                    return (dist > avg_dist * 0.5, angle)

            agents.sort(key=key)
            phases = list(self.gen_fixed_permutation(uniform_phases, seed=2))
            agents_with_phases = zip(agents, phases)
            for agent in agents_with_phases:
                self.system_state.states[agent[0][0]].phase = agent[1]
            self.send_states()
        else:
            random.shuffle(uniform_phases)
            for i, ident in enumerate(ids):
                self.system_state.states[ident].phase = uniform_phases[i]
            self.send_states()

    def send_states(self):
        for ident in self.system_state.ids:
            self.communication.send_state(
                ident,
                self.system_state.states[ident]
            )

    def run(self):
        raise NotImplementedError()
