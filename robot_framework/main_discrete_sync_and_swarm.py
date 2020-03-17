#! /usr/bin/env python

import time

from controller.synchronized_offline_controller import (
    SynchronizedOfflineController
)
from logic.discrete_sync_and_swarm_logic import DiscreteLogic
from position_feedback import PositionFeedback
from knowledge.shared_knowledge import SharedKnowledge
from communication.offline_communication import OfflineCommunication
from system_state import SystemState
from visualization.live_visualization import LiveVisualization


DEFAULT_IDENT = "a{}"


def main():
    agents_num = 12
    ids = [
        DEFAULT_IDENT.format(i) for i in range(agents_num)
    ]

    params_presets = [
        {'J': 0.1, 'K': 1, 'M': 1, 'name': "STATIC SYNC"},
        {'J': 0.1, 'K': -1, 'M': 1, 'name': "STATIC ASYNC"},
        {'J': 1, 'K': 0, 'M': 1, 'name': "STATIC PHASE WAVE"},
        {'J': 1.4, 'K': 0.25, 'M': agents_num, 'name': "NEW STATIC PHASE WAVE"},
        {'J': 1.5, 'K': 0.5, 'M': 4, 'name': "SPLINTERED PHASE WAVE"},
        {'J': 1, 'K': -1, 'M': 1, 'name': "ACTIVE PHASE WAVE"},
        {'J': -1, 'K': 0.5, 'M': 2, 'name': "TEST"},
    ]

    initial_params = {
        'phase_levels_number': 24,
        'agent_radius': 0.1,
    }
    initial_params.update(params_presets[4])

    time_delta = 0.1
    logic = DiscreteLogic(initial_params)
    knowledge = SharedKnowledge(ids)
    system_state = SystemState(ids, knowledge, params=initial_params)
    position_feedback = PositionFeedback(system_state.states, time_delta)
    communication = OfflineCommunication(system_state)
    visualization = LiveVisualization(
        agent_radius=initial_params['agent_radius']
    )
    controller = SynchronizedOfflineController(
        agents_num,
        logic,
        position_feedback,
        communication,
        system_state,
        visualization,
        # sleep_fcn=time.sleep,
        time_delta=time_delta
    )

    controller.run()


if __name__ == "__main__":
    main()
