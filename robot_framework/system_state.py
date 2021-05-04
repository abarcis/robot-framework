#! /usr/bin/env python
from robot_framework.state import State
import numpy as np


def check_if_position_correct(state, states, params):
    if not states:
        return True
    position = state.position
    other_positions = np.array([s.position for s in states.values()])
    distances = np.linalg.norm(other_positions - position, axis=1)
    return not any(
        distances < params['agent_radius'] * 2 + params['min_distance'] + 2  #0.1
    )


class SystemState:
    def __init__(
        self,
        ids,
        knowledge,
        states=None,
        positions=None,
        phases=None,
        params={},
    ):
        self.ids = ids
        self.knowledge = knowledge
        if states is not None:
            self.states = states
        else:
            self.states = {}
            for ident in self.ids:
                position = None
                phase = None
                if positions:
                    position = positions[ident]
                if phases:
                    phase = phases[ident]
                state = State(
                    position=position,
                    phase=phase,
                    params=params
                )
                if position is None:
                    correct = check_if_position_correct(
                        state, self.states, params
                    )
                    while not correct:
                        state = State(
                            position=position,
                            phase=phase,
                            params=params
                        )
                        correct = check_if_position_correct(
                            state, self.states, params
                        )
                self.states[ident] = state

    def reinit(self, params):
        self.__init__(ids=self.ids, knowledge=self.knowledge, params=params)
