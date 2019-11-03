#! /usr/bin/env python

import logging
import numpy as np
import random


class BaseController(object):
    PHASE_ORDERING = ('RANDOM', 'LEXICOGRAPHIC', 'ANGLE')
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
        ids = self.system_state.ids
        uniform_phases = [1. / len(ids) * i #+ random.uniform(-0.01, 0.01)
                          for i in range(len(ids))]
        if (
            phase_ordering in self.PHASE_ORDERING
            and phase_ordering != 'RANDOM'
        ):
            pos = [tuple(self.system_state.states[i].position) for i in ids]
            agents = zip(ids, pos)
            if phase_ordering == 'LEXICOGRAPHIC':

                def key(x):
                    return x[1]
            elif phase_ordering == 'ANGLE':
                middle = (
                    np.average([p[0] for p in pos]),
                    np.average([p[1] for p in pos])
                )

                def key(x):
                    return np.arctan2(x[1][0] - middle[0], x[1][1] - middle[1])
                # x_from_middle = [p[0] - middle[0] for p in pos]
                # y_from_middle = [p[1] - middle[1] for p in pos]
                # angle = tuple(np.arctan2(x_from_middle, y_from_middle))
            agents.sort(key=key)
            phases = list(self.gen_fixed_permutation(uniform_phases, seed=1))
            agents_with_phases = zip(agents, phases)
            for agent in agents_with_phases:
                self.system_state.states[agent[0][0]].phase = agent[1]
                self.communication.send_state(
                    agent[0][0],
                    self.system_state.states[agent[0][0]]
                )
        else:
            random.shuffle(uniform_phases)
            for i, ident in enumerate(ids):
                self.system_state.states[ident].phase = uniform_phases[i]

                self.communication.send_state(
                    ident,
                    self.system_state.states[ident]
                )

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
            pressed_key = None
            new_key = True
            while(new_key):
                new_key = self.key_poller.poll()
                if new_key:
                    pressed_key = new_key

            if self.was_pressed > 0:
                self.was_pressed -= 1
                if self.was_pressed == 0:
                    self.teleop_velocity = None

            if pressed_key is not None:
                if pressed_key == 'w':
                    logging.debug("teleop: up")
                    self.teleop_velocity = np.array([0, self.teleop_speed, 0])
                    self.was_pressed = self.remember_teleop
                if pressed_key == 's':
                    logging.debug("teleop: down")
                    self.teleop_velocity = np.array([0, -self.teleop_speed, 0])
                    self.was_pressed = self.remember_teleop
                if pressed_key == 'd':
                    logging.debug("teleop: right")
                    self.teleop_velocity = np.array([self.teleop_speed, 0, 0])
                    self.was_pressed = self.remember_teleop
                if pressed_key == 'a':
                    logging.debug("teleop: left")
                    self.teleop_velocity = np.array([-self.teleop_speed, 0, 0])
                    self.was_pressed = self.remember_teleop
                if pressed_key in [
                    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
                ]:
                    try:
                        new_params = self.params_list[int(pressed_key)]
                        print("Changing parameters to: {}".format(new_params))
                        self.logic.update_params(new_params)
                        self.reassign_phases(phase_ordering='ANGLE')
                    except IndexError:
                        print("No parameter set with this index")
                if pressed_key in self.keyboard_callbacks.keys():
                    self.keyboard_callbacks[pressed_key]()

        for ident in self.system_state.ids:
            self.system_state.states[ident].update(
                ident,
                position_feedback=self.position_feedback,
            )

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
