#! /usr/bin/env python

import time

from rclpy.node import Node

from controller.bs_controller import (
    BSController
)
from logic.discrete_sync_and_swarm_logic import DiscreteLogic
from position_feedback.position_feedback import PositionFeedback
from knowledge.separate_knowledge import SeparateKnowledge
from communication.ros_communication import ROSCommunication
from system_state import SystemState
from visualization.live_visualization import LiveVisualization
from logger.state_log import StateLog

import rclpy

DEFAULT_IDENT = "a{}"


def main():
    rclpy.init()
    ids = [
        'base_station',
    ]
    time_delta = 1
    small_phase_steps = 10

    params_presets = [
        {'J': 0.1, 'K': 1, 'M': 1, 'name': "STATIC SYNC"},
        {'J': -1, 'K': 0.25, 'M': 3, 'name': "STATIC ASYNC"},
        {'J': 1, 'K': 0, 'M': 1, 'name': "STATIC PHASE WAVE"},
        {'J': 1.4, 'K': 1, 'M': 8, 'name': "NEW STATIC PHASE WAVE"},
        {'J': 1.5, 'K': 1, 'M': 6, 'name': "SPLINTERED PHASE WAVE"},
        {'J': 1, 'K': -1, 'M': 1, 'name': "ACTIVE PHASE WAVE"},
        {'J': 1, 'K': 0.25, 'M': 8, 'name': "TEST"},
    ]

    initial_params = {
        'phase_levels_number': 16,
        'agent_radius': 0.1,
        'min_distance': 0.1,
        'time_delta': time_delta,
        'small_phase_steps': small_phase_steps,
        'orientation_mode': True,
        'constraint_mode': True,
        'agents_num': 8,
    }
    initial_params.update(params_presets[0])

    logic = DiscreteLogic(initial_params)
    knowledge = SeparateKnowledge(ids)
    system_state = SystemState(ids, knowledge, params=initial_params)
    position_feedback = PositionFeedback(system_state.states, time_delta)
    node = Node('controller')
    communication = ROSCommunication(
        system_state,
        node=node,
        agent_id=ids[0],
        params=initial_params,
    )
    visualizations = [
        LiveVisualization(
            agent_radius=initial_params['agent_radius'],
            params=initial_params,
        ),
    ]
    logger = StateLog(initial_params)
    controller = BSController(
        0,
        logic=logic,
        position_feedback=position_feedback,
        communication=communication,
        system_state=system_state,
        visualizations=visualizations,
        logger=logger,
        sleep_fcn=time.sleep,
        params_list=params_presets,
        time_delta=time_delta,
        node=node,
    )

    controller.run()


if __name__ == "__main__":
    main()
