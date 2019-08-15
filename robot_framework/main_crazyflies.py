#! /usr/bin/env python

from controller import OfflineControllerWithTeleoperation
from logic import SyncAndSwarmLogic
from knowledge import SharedKnowledge
from communication import OfflineCommunication
from system_state import SystemState
from crazyflies import CrazySwarmInterface
import keyboard


DEFAULT_IDENT = "{}"


def main():
    with keyboard.KeyPoller() as key_poller:
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
        system_state = SystemState(ids, knowledge, positions=positions)
        communication = OfflineCommunication(system_state)
        controller = OfflineControllerWithTeleoperation(
            agents_num,
            logic,
            crazyswarm_interface,
            communication,
            system_state,
            crazyswarm_interface,
            time_delta=0.1,
            teleoperated_id=ids[0],
            key_poller=key_poller,
        )

        crazyswarm_interface.swarm.run_looped(
            controller.update,
            update_interval=0.1,
            takeoff_height=0.5,
            keep_height=True
        )


if __name__ == "__main__":
    main()
