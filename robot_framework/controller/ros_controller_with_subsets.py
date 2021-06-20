#! /usr/bin/env python

from robot_framework.controller.ros_controller import ROSController
from robot_framework.utils.state_utils import convert_states_to_local


class ROSControllerWithSubsets(ROSController):
    require_logic = False

    def __init__(self, *args, **kwargs):
        logic_class = kwargs.pop('logic_class')
        initial_params = kwargs.pop('initial_params')
        super().__init__(*args, **kwargs)

        # testing only, remove once done!!!!!
        import numpy as np
        collaborators_and_goals = {
            '1': {'collaborators': ['2', '3'], 'goal': np.array([0, 0, 0])},
            '2': {'collaborators': ['1', '3'], 'goal': np.array([0, 0, 0])},
            '3': {'collaborators': ['2', '1'], 'goal': np.array([0, 0, 0])},
            '4': {'collaborators': ['5'], 'goal': np.array([3, 3, 0])},
            '5': {'collaborators': ['4'], 'goal': np.array([3, 3, 0])},
        }

        self.logics = {
            ident: logic_class({**initial_params, **collaborators_and_goals[ident]})
            for ident in self.system_state.states.keys()
        }

    def update_params(self, params):
        collaborators = params.get('collaborators', None)
        for ident, logic in self.logics.items():
            if not collaborators or ident in collaborators:
                new_params = params.copy()
                new_params['collaborators'] = params['collaborators'].copy()
                if collaborators:
                    new_params['collaborators'].remove(ident)
                print(ident, new_params)
                logic.update_params(new_params)

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

        for visualization in self.visualizations:
            visualization.update(
                self.system_state.states, self.current_time
            )
        self.logger.update(self.current_time, self.system_state)

        self.current_time += self.time_delta
