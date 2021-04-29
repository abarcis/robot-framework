#! /usr/bin/env python

import time

from rclpy.node import Node

from .controller.ros_controller import (
    ROSController
)
from robot_framework.logic.discrete_sync_and_swarm_logic import DiscreteLogic
from robot_framework.position_feedback.position_feedback import PositionFeedback
from robot_framework.knowledge.separate_knowledge import SeparateKnowledge
from robot_framework.communication.ros_communication import ROSCommunication
from robot_framework.system_state import SystemState
from robot_framework.visualization.px4_visualization import (
    PX4Visualization
)
from robot_framework.logger.state_log import StateLog

import rclpy
import sys

import socket

DEFAULT_IDENT = "a{}"


def main():
    rclpy.init()

    if len(sys.argv) == 1:
        ids = [
            socket.gethostname()[-2:],
        ]
    else:
        ids = [
            sys.argv[1]
        ]
    time_delta = 0.125
    small_phase_steps = 4

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
        'phase_levels_number': 24,
        'agent_radius': 0.2,
        'min_distance': 0.1,
        'time_delta': time_delta,
        'small_phase_steps': small_phase_steps,
        'orientation_mode': False,
        'constraint_mode': False,
        'attraction_factor': 0.75,
        'pos_from_gps': True,
    }
    initial_params.update(params_presets[0])

    logic = DiscreteLogic(initial_params)
    knowledge = SeparateKnowledge(ids)
    system_state = SystemState(ids, knowledge, params=initial_params)
    position_feedback = PositionFeedback(system_state, time_delta)
    node = Node('controller')
    communication = ROSCommunication(
        system_state,
        node=node,
        agent_id=ids[0],
        params=initial_params,
    )
    visualizations = [
        PX4Visualization(f"d{ident}")
        for ident in ids
    ]
    logger = StateLog(initial_params)
    controller = ROSController(
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
        small_phase_steps=small_phase_steps,
        node=node,
    )

    controller.run(
        additional_nodes=[v.node for v in visualizations]
    )


if __name__ == "__main__":
    main()
