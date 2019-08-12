#! /usr/bin/env python

import time


class BaseController:
    def __init__(
        self,
        agents_num,
        logic,
        position_feedback,
        communication,
        system_state,
        visualization,
        time_delta=0.1,
    ):
        self.agents_num = agents_num
        self.logic = logic
        self.position_feedback = position_feedback
        self.time_delta = time_delta
        self.communication = communication
        self.system_state = system_state
        self.visualization = visualization

    def run(self):
        raise NotImplementedError()


class OfflineController(BaseController):
    def run(self):
        for ident in self.system_state.ids:
            self.system_state.knowledge.update_state(
                ident,
                ident,
                self.system_state.states[ident]
            )

        while True:
            for ident in self.system_state.ids:
                state_update = self.logic.update_state(
                    self.system_state.states[ident],
                    self.system_state.knowledge.get_states_except_own(ident)
                )

                self.system_state.states[ident].update(
                    state_update,
                    self.position_feedback,
                    self.time_delta
                )

                self.communication.send_state(
                    ident,
                    self.system_state.states[ident]
                )

            self.visualization.update(self.system_state.states)
            time.sleep(0.1)
