#! /usr/bin/env python

import time

from controller.synchronized_offline_controller import (
    SynchronizedOfflineController
)
from logic.discrete_sync_and_swarm_logic import DiscreteLogic
from position_feedback.position_feedback import PositionFeedback
from knowledge.shared_knowledge import SharedKnowledge
from communication.offline_communication import OfflineCommunication
from system_state import SystemState
from visualization.live_visualization import LiveVisualization
from visualization.order_params_visualization import OrderParamsVisualization
from logger.state_log import StateLog

from utils.get_properties import estimate_radius

import keyboard

DEFAULT_IDENT = "a{}"


def main():
    with keyboard.KeyPoller() as key_poller:
        agents_num = 12
        ids = [
            DEFAULT_IDENT.format(i) for i in range(agents_num)
        ]
        time_delta = 0.125
        small_phase_steps = 4
        initial_params = {
            'phase_levels_number': 24,
            'agent_radius': 0.1,
            'min_distance': 0.1,
            'time_delta': time_delta,
            'small_phase_steps': small_phase_steps,
            'orientation_mode': True,
            'constraint_mode': True,
            'reinit': True,
        }

        params_presets = [
            {'J': 0.1, 'K': 1, 'M': 1, 'name': "STATIC SYNC", **initial_params},
            {'J': -1, 'K': 0.2, 'M': 4, 'name': "STATIC ASYNC", **initial_params},
            {'J': -1, 'K': 0.2, 'M': 3, 'name': "STATIC ASYNC", **initial_params},
            {'J': 1.4, 'K': 1, 'M': agents_num, 'name': "NEW STATIC PHASE WAVE", **initial_params},
            {'J': 1.5, 'K': 1, 'M': 6, 'name': "SPLINTERED PHASE WAVE", **initial_params},
            {'J': 1.5, 'K': 1, 'M': 4, 'name': "SPLINTERED PHASE WAVE", **initial_params},
            {'J': 1, 'K': -1, 'M': 1, 'name': "ACTIVE PHASE WAVE", **initial_params},
        ]

        # initial_params.update(params_presets[0])
        initial_params = params_presets[0]
        print(estimate_radius(agents_num, initial_params['J'],
                              d=initial_params['agent_radius']))
        logic = DiscreteLogic(initial_params)
        knowledge = SharedKnowledge(ids)
        system_state = SystemState(ids, knowledge, params=initial_params)
        position_feedback = PositionFeedback(system_state, time_delta)
        communication = OfflineCommunication(system_state)
        visualizations = [
            OrderParamsVisualization(params=initial_params),
            LiveVisualization(
                agent_radius=initial_params['agent_radius'],
                params=initial_params,
            ),
        ]
        logger = StateLog(initial_params, path='results/log')
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
            time_delta=time_delta
        )

        controller.run()


if __name__ == "__main__":
    main()
