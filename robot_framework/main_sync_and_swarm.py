#! /usr/bin/env python

import time

from controller import OfflineController
from logic import SyncAndSwarmLogic
from position_feedback import PositionFeedback
from knowledge import SharedKnowledge
from communication import OfflineCommunication
from system_state import SystemState
from visualization import LiveVisualization


DEFAULT_IDENT = "a{}"


def main():
    agents_num = 30
    ids = [
        DEFAULT_IDENT.format(i) for i in range(agents_num)
    ]
    time_delta = 0.1
    logic = SyncAndSwarmLogic()
    knowledge = SharedKnowledge(ids)
    system_state = SystemState(ids, knowledge)
    position_feedback = PositionFeedback(system_state.states, time_delta)
    communication = OfflineCommunication(system_state)
    visualization = LiveVisualization()
    controller = OfflineController(
        agents_num,
        logic,
        position_feedback,
        communication,
        system_state,
        visualization,
        sleep_fcn=time.sleep,
        time_delta=time_delta
    )

    controller.run()


if __name__ == "__main__":
    main()
