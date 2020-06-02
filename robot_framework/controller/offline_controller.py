#! /usr/bin/env python

from .base_controller import BaseController


class OfflineController(BaseController):
    def __init__(self, *args, **kwargs):
        super(OfflineController, self).__init__(*args, **kwargs)
        for ident in self.system_state.ids:
            self.system_state.knowledge.update_state(
                ident,
                ident,
                self.system_state.states[ident]
            )

    def update(self, *args):
        for ident in self.system_state.ids:
            self.system_state.states[ident].update(
                ident,
                position_feedback=self.position_feedback,
            )

            state_update = self.logic.update_state(
                self.system_state.states[ident],
                self.system_state.knowledge.get_states_except_own(ident)
            )

            self.system_state.states[ident].update(
                ident,
                state_update,
                self.position_feedback,
            )

        for ident in self.system_state.ids:
            self.communication.send_state(
                ident,
                self.system_state.states[ident]
            )
        t = self.current_time
        if abs(t % 1) < 0.001 or 1 - abs(t % 1) < 0.001:
            for visualization in self.visualizations:
                visualization.update(
                    self.system_state.states, self.current_time
                )
        self.current_time += self.time_delta

    def run(self):
        while True:
            self.update()
            if self.mission_end_callback:
                if self.mission_end_callback(
                    self.current_time,
                    self.system_state.states,
                    params=self.logic.params,
                ):
                    return
            if self.sleep_fcn:
                self.sleep_fcn(self.time_delta)
