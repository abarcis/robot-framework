#! /usr/bin/env python

import random

from rclpy.node import Node
import rclpy

from controller.ros_controller import ROSController
from logic.sync_and_swarm_logic import SyncAndSwarmLogic
from position_feedback import PositionFeedback
from knowledge.separate_knowledge import SeparateKnowledge
from msg_filters.range_filter import RangeFilter
from communication.ros_communication import (
    ROSCommunication
)
from system_state import SystemState
from visualization.live_visualization import LiveVisualization
import keyboard

import matplotlib
matplotlib.use('TkAgg')

DEFAULT_IDENT = "{}"


def main():
    with keyboard.KeyPoller() as key_poller:
        rclpy.init()
        agents_num = 30
        ids = [
            DEFAULT_IDENT.format(i) for i in range(agents_num)
        ]
        time_delta = 0.5
        initial_params = {
            'natural_frequency': 0,
        }
        params_presets = [
            {'J': 0.1, 'K': 0.7, 'align_center': False, 'name': "STATIC SYNC"},
            {'J': 0.1, 'K': -1, 'align_center': False, 'name': "STATIC ASYNC"},
            {'J': 1, 'K': 0, 'align_center': False,
             'name': "STATIC PHASE WAVE"},
            {'J': 1, 'K': -0.1, 'align_center': False, 'name':
             "SPLINTERED PHASE WAVE"},
            {'J': 1, 'K': -0.25, 'align_center': False, 'name':
             "ACTIVE PHASE WAVE"},
            {'J': 0.1, 'K': 1, 'align_center': True, 'name': "STATIC SYNC"},
            {'J': 0.1, 'K': -1, 'align_center': True, 'name': "STATIC ASYNC"},
            {'J': 1, 'K': 0, 'align_center': True,
             'name': "STATIC PHASE WAVE"},
            {'J': 1, 'K': -0.1, 'align_center': True, 'name':
             "SPLINTERED PHASE WAVE"},
            {'J': 1, 'K': -0.75, 'align_center': True, 'name':
             "ACTIVE PHASE WAVE"},
        ]
        initial_params.update(params_presets[4])
        uniform_phases = [1. / len(ids) * i for i in range(len(ids))]
        random.shuffle(uniform_phases)
        phases = {
            ident: uniform_phases[i]
            for i, ident
            in enumerate(ids)
        }
        logic = SyncAndSwarmLogic(initial_params)
        knowledge = SeparateKnowledge(ids, max_age=2)
        system_state = SystemState(ids, knowledge, phases=phases)
        position_feedback = PositionFeedback(system_state.states, time_delta)
        receive_filters = [RangeFilter(system_state, params={'range': 2.5})]
        node = Node('controller')
        communication = {
            ident:
            ROSCommunication(
                system_state,
                receive_filters=receive_filters,
                node=node,
                agent_id=ident,
            )
            for ident in ids
        }
        visualization = LiveVisualization()
        controller = ROSController(
            agents_num,
            logic,
            position_feedback,
            communication,
            system_state,
            visualization,
            time_delta=time_delta,
            node=node,
            teleoperated_id=ids[0],
            teleop_blinking=True,
            key_poller=key_poller,
            params_list=params_presets,
        )

        controller.run()


if __name__ == "__main__":
    main()
