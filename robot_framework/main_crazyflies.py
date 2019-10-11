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
        params = {'J': 1, 'K': -0.25}
        update_interval = 0.25
        agents_num = 30
        crazyswarm_interface = CrazySwarmInterface()
        ids = list(crazyswarm_interface.swarm.allcfs.crazyfliesById.keys())
        positions = {
            cf.id: cf.initialPosition
            for cf in crazyswarm_interface.swarm.allcfs.crazyflies
        }
        phases = {
            cf.id: 1. / len(ids) * i
            for i, cf
            in enumerate(crazyswarm_interface.swarm.allcfs.crazyflies)
        }
        print(phases)
        logic = SyncAndSwarmLogic(params)
        knowledge = SharedKnowledge(ids)
        system_state = SystemState(
            ids,
            knowledge,
            positions=positions,
            phases=phases
        )
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
            teleop_blinking=True,
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
