#! /usr/bin/env python

import logging
import numpy as np


class BaseController(object):
    def __init__(
        self,
        agents_num,
        logic,
        position_feedback,
        communication,
        system_state,
        visualization,
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

    def run(self):
        raise NotImplementedError()


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

            self.communication.send_state(
                ident,
                self.system_state.states[ident]
            )

        self.visualization.update(self.system_state.states)

    def run(self):
        while True:
            self.update()
            if self.sleep_fcn:
                self.sleep_fcn(self.time_delta)


class OfflineControllerWithTeleoperation(BaseController):
    def __init__(self, *args, **kwargs):
        self.teleoperated_id = kwargs.pop('teleoperated_id')
        self.teleop_speed = 0.1
        self.teleop_on = True
        self.teleop_velocity = None

        self.key_poller = kwargs.pop('key_poller')

        super(OfflineControllerWithTeleoperation, self).__init__(
            *args, **kwargs
        )
        for ident in self.system_state.ids:
            self.system_state.knowledge.update_state(
                ident,
                ident,
                self.system_state.states[ident]
            )

    def update(self, *args):
        if self.teleop_on:
            pressed_key = self.key_poller.poll()
            while(self.key_poller.poll()):
                pass  # clear input stream

            if pressed_key is not None or self.teleop_velocity is not None:
                self.teleop_velocity = None
                if pressed_key == 'w':
                    logging.debug("teleop: up")
                    self.teleop_velocity = np.array([0, self.teleop_speed, 0])
                if pressed_key == 's':
                    logging.debug("teleop: down")
                    self.teleop_velocity = np.array([0, -self.teleop_speed, 0])
                if pressed_key == 'd':
                    logging.debug("teleop: right")
                    self.teleop_velocity = np.array([self.teleop_speed, 0, 0])
                if pressed_key == 'a':
                    logging.debug("teleop: left")
                    self.teleop_velocity = np.array([-self.teleop_speed, 0, 0])
        for ident in self.system_state.ids:
            self.system_state.states[ident].update(
                ident,
                position_feedback=self.position_feedback,
            )

            state_update = self.logic.update_state(
                self.system_state.states[ident],
                self.system_state.knowledge.get_states_except_own(ident)
            )

            if (self.teleoperated_id == ident and
                self.teleop_velocity is not None):
                state_update.velocity_update = self.teleop_velocity

            self.system_state.states[ident].update(
                ident,
                state_update,
                self.position_feedback,
            )

            self.communication.send_state(
                ident,
                self.system_state.states[ident]
            )

        self.visualization.update(self.system_state.states)

    def run(self):
        while True:
            self.update()
            if self.sleep_fcn:
                self.sleep_fcn(self.time_delta)
