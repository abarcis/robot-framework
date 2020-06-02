#! /usr/bin/env python

import logging
import numpy as np


class Teleoperation:

    def teleoperate(self):
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
            if pressed_key == 'r':
                self.reassign_phases(phase_ordering='RANDOM')
            if pressed_key == ' ':
                self.toggle_active()
            if pressed_key in [
                '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
            ]:
                try:
                    new_params = self.params_list[int(pressed_key)]
                    print(
                        "Changing parameters to: {}".format(new_params)
                    )
                    self.logic.update_params(new_params)
                    self.current_time = 0
                    for vis in self.visualizations:
                        vis.reinit(new_params)
                    if self.logger:
                        self.logger.reinit(new_params)
                    if new_params['name'] == 'STATIC PHASE WAVE':
                        self.reassign_phases(phase_ordering='ANGLE')
                    else:
                        self.reassign_phases(phase_ordering='MODIFY')
                except IndexError:
                    logging.error("No parameter set with this index")
            if pressed_key in self.keyboard_callbacks.keys():
                self.keyboard_callbacks[pressed_key]()
