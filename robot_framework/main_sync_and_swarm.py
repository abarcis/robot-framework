#! /usr/bin/env python

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
    logic = SyncAndSwarmLogic()
    position_feedback = PositionFeedback()
    knowledge = SharedKnowledge(ids)
    system_state = SystemState(ids, knowledge)
    communication = OfflineCommunication(system_state)
    visualization = LiveVisualization()
    controller = OfflineController(
        agents_num,
        logic,
        position_feedback,
        communication,
        system_state,
        visualization
    )

    controller.run()


if __name__ == "__main__":
    main()
