#! /usr/bin/env python

import time

from rclpy.node import Node

from robot_framework.controller.ros_controller_with_subsets import (
    ROSControllerWithSubsets
)
from robot_framework.logic.sandsbot_with_subsets_logic import (
    SandsbotWithSubsetsLogic
)
from robot_framework.position_feedback.position_feedback import (
    PositionFeedback
)
from robot_framework.knowledge.separate_knowledge import SeparateKnowledge
from robot_framework.communication.ros_communication import (
    ROSCommunication
)
from robot_framework.system_state import SystemState
from robot_framework.visualization.live_visualization import (
    LiveVisualization
)
from robot_framework.logger.state_log import StateLog

from robot_framework.camera import Camera

import rclpy
import sys
import numpy as np

import socket

DEFAULT_IDENT = "a{}"
CAMERA_ON = True

def main():
    rclpy.init()
    # np.random.seed(1)
    agents_num = 1
    ids = [
        socket.gethostname()[-2:],
    ]
    time_delta = 0.25
    small_phase_steps = 4

    params_presets = [
        {'J': 0, 'K': 0, 'M': 1, 'name': "SWARMING"},
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
        'agent_radius': 0.1,
        'min_distance': 0.1,
        'time_delta': time_delta,
        'small_phase_steps': small_phase_steps,
        'orientation_mode': True,
        'constraint_mode': False,
        'attraction_factor': 0.75,
        'repulsion_factor': 2,
        'pos_from_gps': False,
        'phase_interaction': False,
        'max_speed': 0.2,
        'repulsion_range': 1,
        'task_execution_time': 5,
        'task_execution_speed': 0.1,
    }
    initial_params.update(params_presets[0])

    knowledge = SeparateKnowledge(ids)
    system_state = SystemState(ids, knowledge, params=initial_params)
    position_feedback = PositionFeedback(system_state, time_delta)
    node = Node('controller')
    communication = ROSCommunication(
        system_state,
        agent_id=ids[0],
        node=node,
        params=initial_params,
    )
    visualizations = [
        # LiveVisualization(agent_radius=initial_params['agent_radius'], params=initial_params)
    ]
    if CAMERA_ON:
        camera_interface = Camera()
    else:
        camera_interface = None
    logger = StateLog(initial_params, path=None)
    controller = ROSControllerWithSubsets(
        0,
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
        logic_class=SandsbotWithSubsetsLogic,
        initial_params=initial_params,
        camera_interface=camera_interface,
    )

    controller.run()


if __name__ == "__main__":
    main()
