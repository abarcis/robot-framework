#! /usr/bin/env python

import time

from rclpy.node import Node

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


DEFAULT_IDENT = "a{}"


def main():
    agents_num = 10
    ids = [
        DEFAULT_IDENT.format(i) for i in range(agents_num)
    ]
    time_delta = 0.1
    logic = SyncAndSwarmLogic()
    knowledge = SeparateKnowledge(ids, max_age=2)
    system_state = SystemState(ids, knowledge)
    position_feedback = PositionFeedback(system_state.states, time_delta)
    receive_filters = [RangeFilter(system_state, params={'range': 1})]
    node = Node('controller')
    communication = ROSCommunication(
        system_state,
        receive_filters=receive_filters,
        node=node,
    )
    visualization = LiveVisualization()
    controller = ROSController(
        agents_num,
        logic,
        position_feedback,
        communication,
        system_state,
        visualization,
        time_delta=time_delta,
        node=node
    )

    controller.run()


if __name__ == "__main__":
    main()
