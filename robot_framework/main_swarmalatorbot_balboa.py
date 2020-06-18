#! /usr/bin/env python

import time

from rclpy.node import Node

from .controller.ros_controller import (
    ROSController
)
from robot_framework.logic.discrete_sync_and_swarm_logic import DiscreteLogic
from robot_framework.position_feedback.optitrack import Optitrack
from robot_framework.knowledge.separate_knowledge import SeparateKnowledge
from robot_framework.communication.ros_communication import ROSCommunication
from robot_framework.system_state import SystemState
from robot_framework.visualization.balboa_visualization import (
    BalboaVisualization
)
from robot_framework.logger.state_log import StateLog

import rclpy

import socket

DEFAULT_IDENT = "a{}"


def main():
    rclpy.init()
    ids = [
        socket.gethostname()[-2:],
    ]
    time_delta = 0.05
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
    }
    initial_params.update(params_presets[0])

    logic = DiscreteLogic(initial_params)
    knowledge = SeparateKnowledge(ids)
    system_state = SystemState(ids, knowledge, params=initial_params)
    position_feedback = Optitrack(system_state.states, time_delta)
    node = Node('controller')
    communication = ROSCommunication(
        system_state,
        node=node,
        agent_id=ids[0],
        params=initial_params,
    )
    visualizations = [
        BalboaVisualization(),
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
        node=node,
    )

    controller.run()


if __name__ == "__main__":
    main()
