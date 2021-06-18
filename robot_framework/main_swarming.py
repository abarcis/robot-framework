#! /usr/bin/env python

import time

from robot_framework.controller.synchronized_offline_controller import (
    SynchronizedOfflineController
)
from robot_framework.logic.discrete_sync_and_swarm_logic import DiscreteLogic
from robot_framework.position_feedback.position_feedback import PositionFeedback
from robot_framework.knowledge.shared_knowledge import SharedKnowledge
from robot_framework.communication.offline_communication import OfflineCommunication
from robot_framework.system_state import SystemState
from robot_framework.visualization.live_visualization import LiveVisualization
from robot_framework.visualization.order_params_visualization import OrderParamsVisualization
from robot_framework.logger.state_log import StateLog

from robot_framework.utils.get_properties import estimate_radius

import robot_framework.keyboard as keyboard
import numpy as np

DEFAULT_IDENT = "a{}"


def main():
    with keyboard.KeyPoller() as key_poller:
        agents_num = 12
        ids = [
            DEFAULT_IDENT.format(i) for i in range(agents_num)
        ]
        period = 0.2
        small_phase_steps = 5
        time_delta = period / small_phase_steps
        initial_params = {
            'phase_levels_number': 24,
            'agent_radius': 0.1,
            'min_distance': 0.1,
            'time_delta': time_delta,
            'small_phase_steps': small_phase_steps,
            'orientation_mode': False,
            'constraint_mode': False,
            'reinit': True,
            'attraction_factor': 0.75,
            'repulsion_factor': 2,
            'speed_limit': True,
            'sync_interaction': False,
            'area_size': 10,
            'max_speed': 0.2,
            'synchronized': True,
        }

        params_presets = [
            {'J': 0, 'K': 0, 'M': 1, 'name': "STATIC SYNC", **initial_params},
        ]

        # initial_params.update(params_presets[0])
        initial_params = params_presets[0]
        logic = DiscreteLogic(initial_params)
        knowledge = SharedKnowledge(ids)
        system_state = SystemState(ids, knowledge, params=initial_params)
        position_feedback = PositionFeedback(system_state, time_delta)
        communication = OfflineCommunication(system_state)
        visualizations = [
            # OrderParamsVisualization(params=initial_params),
            LiveVisualization(
                agent_radius=initial_params['agent_radius'],
                params=initial_params,
            ),
        ]
        logger = StateLog(initial_params, path='robot_framework/results/log/swarm/final')
        controller = SynchronizedOfflineController(
            agents_num,
            logic=logic,
            position_feedback=position_feedback,
            communication=communication,
            system_state=system_state,
            visualizations=visualizations,
            logger=logger,
            # sleep_fcn=time.sleep,
            key_poller=key_poller,
            teleop_on=True,
            params_list=params_presets,
            time_delta=time_delta,
            small_phase_steps=small_phase_steps,
        )

        controller.run()


if __name__ == "__main__":
    np.random.seed(0)
    main()
