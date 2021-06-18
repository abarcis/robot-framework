#! /usr/bin/env python

from __future__ import division

from .base_controller import BaseController
from .teleoperation import Teleoperation

from datetime import datetime


class SynchronizedOfflineController(BaseController, Teleoperation):
    def __init__(self, *args, **kwargs):
        self.teleop_on = kwargs.pop('teleop_on', False)
        self.key_poller = kwargs.pop('key_poller')
        self.real_time_vis = kwargs.pop('real_time_vis', True)
        self.was_pressed = 0
        self.teleop_velocity = None
        self.keyboard_callbacks = kwargs.pop('keyboard_callbacks', {})
        self.last_iter = datetime.now()
        super(SynchronizedOfflineController, self).__init__(*args, **kwargs)
        for ident in self.system_state.ids:
            self.system_state.knowledge.update_state(
                ident,
                ident,
                self.system_state.states[ident]
            )

    def update(self, *args):
        self.last_iter = datetime.now()
        if self.teleop_on:
            self.teleoperate()
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

            # print(ident, self.system_state.states[ident].small_phase)

            if self.system_state.states[ident].small_phase == 0:
                # print(ident, 'send!')
                self.communication.send_state(
                    ident,
                    # self.system_state.states[ident],
                    self.system_state.states[ident].predict(
                        self.small_phase_steps
                        * self.time_delta
                    )
                )

        # self.logger.update(self.current_time, self.system_state)
        t = self.current_time
        self.logger.update(self.current_time, self.system_state)
        if abs(t % 1) < 0.001 or 1 - abs(t % 1) < 0.001:
            print('t =', t)
            for visualization in self.visualizations:
                visualization.update(
                    self.system_state.states, self.current_time
                )

        self.current_time += self.time_delta

    def run(self):
        try:
            while True:
                self.update()
                if self.mission_end_callback:
                    if self.mission_end_callback(
                        self.current_time,
                        self.system_state.states,
                        self.logic.params,
                    ):
                        for visualization in self.visualizations:
                            visualization.update(
                                self.system_state.states, self.current_time
                            )
                        return
                if self.sleep_fcn:
                    self.sleep_fcn(self.time_delta)
        finally:
            self.logger.save()
