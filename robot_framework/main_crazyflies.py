#! /usr/bin/env python

from controller.offline_controller_with_teleoperation import OfflineControllerWithTeleoperation # NOQA
from logic.sync_and_swarm_logic import SyncAndSwarmLogic
from knowledge.shared_knowledge import SharedKnowledge
from communication.offline_communication import OfflineCommunication
from system_state import SystemState
from crazyflies import CrazySwarmInterface
import keyboard
import random


DEFAULT_IDENT = "{}"


def main():
    with keyboard.KeyPoller() as key_poller:
        params_presets = [
            {'J': 0.1, 'K': 0.4, 'align_center': True, 'name': "STATIC SYNC"},
            {'J': 0.1, 'K': -1, 'align_center': True, 'name': "STATIC ASYNC"},
            {'J': 1, 'K': 0, 'align_center': True,
             'name': "STATIC PHASE WAVE"},
            {'J': 1, 'K': -0.25, 'align_center': True, 'name':
             "ACTIVE PHASE WAVE"},
        ]
        update_interval = 0.5
        agents_num = 30
        flying_altitude = 5
        initial_params = {
            'natural_frequency': 0.1,
        }
        initial_params.update(params_presets[0])

        crazyswarm_interface = CrazySwarmInterface(
            flying_altitude=flying_altitude
        )
        ids = list(crazyswarm_interface.swarm.allcfs.crazyfliesById.keys())
        positions = {
            cf.id: cf.initialPosition
            for cf in crazyswarm_interface.swarm.allcfs.crazyflies
        }
        uniform_phases = [1. / len(ids) * i for i in range(len(ids))]
        random.shuffle(uniform_phases)
        phases = {
            cf.id: uniform_phases[i]
            for i, cf
            in enumerate(crazyswarm_interface.swarm.allcfs.crazyflies)
        }
        logic = SyncAndSwarmLogic(initial_params)
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
            params_list=params_presets,
            keyboard_callbacks={
                'k': crazyswarm_interface.kill_random_drone,
                'v': crazyswarm_interface.toggle_waving,
                'f': logic.phase_logic.toggle_oscillations,
            },
            is_active=False
        )

        crazyswarm_interface.swarm.run_looped(
            controller.update,
            update_interval=update_interval,
            takeoff_height=flying_altitude,
            takeoff_time=flying_altitude/0.3,
            keep_height=flying_altitude,
            interactive_takeoff=True,
        )


if __name__ == "__main__":
    main()
