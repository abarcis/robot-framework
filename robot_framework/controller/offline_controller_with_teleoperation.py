#! /usr/bin/env python

import logging
import numpy as np

from .base_controller import BaseController


class OfflineControllerWithTeleoperation(BaseController, Teleoperation):
    def __init__(self, *args, **kwargs):
        self.teleoperated_id = kwargs.pop('teleoperated_id')
        self.teleop_speed = 0.1
        self.max_speed = 0.2
        self.teleop_on = True
        self.teleop_velocity = None
        self.remember_teleop = 3
        """
        For how many iterations the key press should persist.
        """
        self.was_pressed = 0
        self.teleop_blinking = kwargs.pop('teleop_blinking')
        self.teleop_led_on = True
        self.keyboard_callbacks = kwargs.pop('keyboard_callbacks', {})

        self.key_poller = kwargs.pop('key_poller')

        self.active = kwargs.pop('is_active', True)

        super(OfflineControllerWithTeleoperation, self).__init__(
            *args, **kwargs
        )
        for ident in self.system_state.ids:
            self.system_state.knowledge.update_state(
                ident,
                ident,
                self.system_state.states[ident]
            )

    def toggle_active(self):
        self.active = not self.active

    def update(self, *args):
        if self.teleop_on:
            self.teleoperate()
        for ident in self.system_state.ids:
            self.system_state.states[ident].update(
                ident,
                position_feedback=self.position_feedback,
            )
            self.communication.send_state(
                ident,
                self.system_state.states[ident]
            )

        if self.active:
            for ident in self.system_state.ids:
                if self.system_state.states[ident].position is None:
                    continue

                state_update = self.logic.update_state(
                    self.system_state.states[ident],
                    self.system_state.knowledge.get_states_except_own(ident)
                )

                if (
                    self.teleoperated_id == ident and
                    self.teleop_velocity is not None
                ):
                    state_update.velocity_update *= 0.5
                    state_update.velocity_update += self.teleop_velocity

                    state_update.teleop_update = True

                self.system_state.states[ident].update(
                    ident,
                    state_update,
                    self.position_feedback,
                )

        self.visualization.update(self.system_state.states)

    def run(self):
        while True:
            self.update()
            if self.sleep_fcn:
                self.sleep_fcn(self.time_delta)
