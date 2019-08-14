#! /usr/bin/env python

from controller import OfflineController
from logic import SyncAndSwarmLogic
from knowledge import SharedKnowledge
from communication import OfflineCommunication
from system_state import SystemState
from crazyflies import CrazySwarmInterface


DEFAULT_IDENT = "{}"


def main():
    agents_num = 30
    ids = [
        DEFAULT_IDENT.format(i) for i in range(agents_num)
    ]
    crazyswarm_interface = CrazySwarmInterface()
    ids = list(crazyswarm_interface.swarm.allcfs.crazyfliesById.keys())
    positions = {
        cf.id: cf.initialPosition
        for cf in crazyswarm_interface.swarm.allcfs.crazyflies
    }
    logic = SyncAndSwarmLogic()
    knowledge = SharedKnowledge(ids)
    system_state = SystemState(ids, knowledge, positions)
    communication = OfflineCommunication(system_state)
    controller = OfflineController(
        agents_num,
        logic,
        crazyswarm_interface,
        communication,
        system_state,
        crazyswarm_interface
    )

    controller.run()


if __name__ == "__main__":
    main()
