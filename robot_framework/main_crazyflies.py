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
        params = {'J': 0.1, 'K': -1}
        update_interval = 0.5
        agents_num = 30
        crazyswarm_interface = CrazySwarmInterface()
        ids = list(crazyswarm_interface.swarm.allcfs.crazyfliesById.keys())
        positions = {
            cf.id: cf.initialPosition
            for cf in crazyswarm_interface.swarm.allcfs.crazyflies
        }
        logic = SyncAndSwarmLogic(params)
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
            time_delta=update_interval,
            teleoperated_id=ids[0],
            key_poller=key_poller,
        )

        crazyswarm_interface.swarm.run_looped(
            controller.update,
            update_interval=update_interval,
            takeoff_height=1.5,
            keep_height=True,
            interactive_takeoff=True,
        )


if __name__ == "__main__":
    main()
