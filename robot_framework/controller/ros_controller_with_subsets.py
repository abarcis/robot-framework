#! /usr/bin/env python

import numpy as np

from robot_framework.controller.ros_controller import ROSController
from robot_framework.utils.state_utils import convert_states_to_local


class ROSControllerWithSubsets(ROSController):
    require_logic = False

    def __init__(self, *args, **kwargs):
        logic_class = kwargs.pop('logic_class')
        initial_params = kwargs.pop('initial_params')
        self.task_execution_time = initial_params.get('task_execution_time', 5)
        self.camera_interface = kwargs.pop('camera_interface', None)
        super().__init__(*args, **kwargs)

        self.logics = {
            ident: logic_class(initial_params)
            for ident in self.system_state.states.keys()
        }
        self.execution_timers = {}

    def update_params(self, params):
        collaborators = params.get('collaborators', None)
        for ident, logic in self.logics.items():
            if collaborators is None or ident in collaborators:
                if self.execution_timers.get(ident):
                    self.node.destroy_timer(self.execution_timers[ident])
                    self.execution_timers[ident] = None
                new_params = params.copy()
                new_params['collaborators'] = params['collaborators'].copy()
                if collaborators:
                    new_params['collaborators'].remove(ident)
                logic.update_params(new_params)

    def pattern_formed(self, ident):
        if self.logics[ident].collaborators is None or not self.logics[ident].position_logic.started:
            return False
        if self.logics[ident].collaborators is not None:
            positions = np.array(
                [self.system_state.states[ident].position] +
                [self.system_state.states[i].position for i in self.logics[ident].collaborators]
            )
            centroid = np.mean(positions, axis=0)
            dist_from_goal = np.linalg.norm(
                self.logics[ident].position_logic.goal - centroid
            )
            vel = self.system_state.states[ident].velocity
            speed = np.linalg.norm(vel)
            minval = 0.1 * self.logics[ident].params.get('agent_radius', 0.1)
            if (
                (
                    dist_from_goal < minval or
                    len(self.logics[ident].collaborators) == 0
                 ) and
                (speed < minval or self.logics[ident].position_logic.rotate)
            ):
                return True
            # vel = self.system_state.states[ident].velocity
            # speed = np.linalg.norm(vel)
            # if (
            #     self.logics[ident].position_logic.rotate or
            #     speed < 0.1 * self.logics[ident].params.get('agent_radius', 0.1)
            # ):
            #     return True
        return False

    def task_finished(self, ident):
        def task_finished_callback():
            self.logics[ident].update_params({
                'collaborators': None,
                'goal': None,
            })
            self.node.destroy_timer(self.execution_timers[ident])
            self.execution_timers[ident] = None
            self.rf_executor.report_progress(f"{ident}:done")
        return task_finished_callback

    def take_picture_callback(self):
        print("Taking picture")
        if self.camera_interface:
            image = self.camera_interface.take_picture()
            self.camera_interface.save_picture(image)

    def update(self, *args):
        for ident in self.system_state.ids:
            self.system_state.states[ident].update(
                ident,
                position_feedback=self.position_feedback,
            )

        for ident in self.system_state.ids:
            if self.system_state.states[ident].position is None:
                continue

            own_state = self.system_state.states[ident]
            other_states = (
                self.system_state.knowledge.get_states_except_own(ident)
            )
            if self.pos_from_gps:
                own_state, other_states = convert_states_to_local(
                    own_state=own_state,
                    other_states=other_states
                )

            other_states_dict = {
                ident: state for ident, state in other_states
                if state
            }

            state_update = self.logics[ident].update_state(
                own_state,
                other_states_dict,
                ident
            )

            self.system_state.states[ident].update(
                ident,
                state_update,
                self.position_feedback,
            )

            if self.system_state.states[ident].small_phase == 0:
                # print(self.communication.received, 'received messages,',
                #       self.communication.delayed, 'delayed')
                #self.communication.received = 0
                #self.communication.delayed = 0
                predicted_state = self.system_state.states[ident].predict(
                    self.small_phase_steps * self.time_delta,
                    pos_from_gps=self.pos_from_gps
                )
                self.communication.send_state(
                    ident,
                    predicted_state
                )
                if self.pattern_formed(ident):
                    if self.execution_timers.get(ident) is None:
                        print(ident, "pattern formed")
                        self.logics[ident].position_logic.rotate = True
                        self.execution_timers[ident] = self.node.create_timer(
                            self.task_execution_time,
                            self.task_finished(ident)
                        )
                    self.take_picture_callback()

        for visualization in self.visualizations:
            visualization.update(
                self.system_state.states, self.current_time
            )
        self.logger.update(self.current_time, self.system_state)

        self.current_time += self.time_delta
