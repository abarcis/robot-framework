#! /usr/bin/env python

from .base_controller import BaseController


class SynchronizedOfflineController(BaseController):
    def __init__(self, *args, **kwargs):
        super(SynchronizedOfflineController, self).__init__(*args, **kwargs)
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
                self.system_state.knowledge.get_states_except_own(ident),
                ident
            )

            self.system_state.states[ident].update(
                ident,
                state_update,
            )

            if self.system_state.states[ident].phase == 0:
                self.communication.send_state(
                    ident,
                    self.system_state.states[ident].predict(1)
                )

        self.visualization.update(self.system_state.states)

    def run(self):
        while True:
            self.update()
            if self.sleep_fcn:
                self.sleep_fcn(self.time_delta)
