#! /usr/bin/env python

import time

from controller.offline_controller import OfflineController
from logic.sync_and_swarm_logic import SyncAndSwarmLogic
from position_feedback import PositionFeedback
from knowledge.shared_knowledge import SharedKnowledge
from communication.offline_communication import OfflineCommunication
from system_state import SystemState
from visualization.live_visualization import LiveVisualization
from visualization.order_params_visualization import OrderParamsVisualization
from logger.state_log import StateLog


DEFAULT_IDENT = "a{}"


def main():
    agents_num = 12
    ids = [
        DEFAULT_IDENT.format(i) for i in range(agents_num)
    ]
    time_delta = 0.1
    small_phase_steps = 10
    params_presets = [
        {'J': 0.1, 'K': 1, 'M': 1, 'name': "STATIC SYNC"},
        {'J': 0.1, 'K': -1, 'M': 1, 'name': "STATIC ASYNC"},
    ]
    initial_params = {
        'phase_levels_number': 18,
        'agent_radius': 0.1,
        'min_distance': 0.1,
        'small_phase_steps': small_phase_steps,
        'attraction_factor': 0.5,
        'repulsion_factor': 2,
        'time_delta': time_delta,
    }
    initial_params.update(params_presets[1])
    visualizations = [
        OrderParamsVisualization(params=initial_params),
        LiveVisualization(
            agent_radius=initial_params['agent_radius']
        ),
    ]

    logic = SyncAndSwarmLogic(initial_params)
    knowledge = SharedKnowledge(ids)
    system_state = SystemState(ids, knowledge, params=initial_params)
    position_feedback = PositionFeedback(system_state.states, time_delta)
    communication = OfflineCommunication(system_state)
    logger = StateLog(initial_params)
    controller = OfflineController(
        agents_num,
        logic=logic,
        position_feedback=position_feedback,
        communication=communication,
        system_state=system_state,
        visualizations=visualizations,
        logger=logger,
        # sleep_fcn=time.sleep,
        params_list=params_presets,
        # mission_end_callback=original_pattern_formed_callback,
        time_delta=time_delta,
        small_phase_steps=small_phase_steps,
    )

    controller.run()


if __name__ == "__main__":
    main()
