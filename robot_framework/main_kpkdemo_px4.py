#! /usr/bin/env python

import time

from rclpy.node import Node

from robot_framework.controller.ros_controller_with_subsets import (
    ROSControllerWithSubsets
)
from robot_framework.logic.sandsbot_with_subsets_logic import (
    SandsbotWithSubsetsLogic
)
from robot_framework.position_feedback.px4 import PX4PositionFeedback
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
CAMERA_ON = False
if CAMERA_ON:
    from robot_framework.camera import Camera



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
    time_delta = 0.5
    small_phase_steps = 2

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
        'agent_radius': 2.5,
        'min_distance': 1,
        'time_delta': time_delta,
        'small_phase_steps': small_phase_steps,
        'orientation_mode': True,
        'constraint_mode': False,
        'attraction_factor': 0.1,
        'attraction_range': 30,
        'repulsion_factor': 30,
        'pos_from_gps': True,
        'phase_interaction': False,
        'max_speed': 2.5,
        'repulsion_range': 15,
        'task_execution_time': 30,
        'task_execution_speed': 1,
        'goal_min_distance': 2,
        'goal_min_speed': 0.5,
        'goal_attraction': 0.2,
        'goal_repulsion': 2.5,
        'goal_const_speed': 2.5,
        'goal_attr_dist': 8,
        'create_file': '/tmp/poi_found',
    }
    initial_params.update(params_presets[0])

    knowledge = SeparateKnowledge(ids)
    system_state = SystemState(ids, knowledge, params=initial_params)
    position_feedback = PX4PositionFeedback(system_state, time_delta)
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
    if CAMERA_ON:
        camera_interface = Camera()
    else:
        camera_interface = None
    logger = StateLog(initial_params, path=None)  # '/home/pi/log')
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
        pos_from_gps=True,
        camera_interface=camera_interface,
    )

    controller.run(
        additional_nodes=[v.node for v in visualizations]
    )


if __name__ == "__main__":
    main()
