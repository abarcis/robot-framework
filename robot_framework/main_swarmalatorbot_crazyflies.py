#! /usr/bin/env python

from controller.synchronized_offline_controller import (
    SynchronizedOfflineController
)
from logic.discrete_sync_and_swarm_logic import DiscreteSyncAndSwarmLogic
from knowledge.shared_knowledge import SharedKnowledge
from communication.offline_communication import OfflineCommunication
from system_state import SystemState
from crazyflies import CrazySwarmInterface
import keyboard
import random


DEFAULT_IDENT = "{}"


def main():
    with keyboard.KeyPoller() as key_poller:
        agents_num = 8
        params_presets = [
            {'J': 0.1, 'K': 1, 'M': 1, 'name': "STATIC SYNC"},
            {'J': -1.5, 'K': 0.1, 'M': 4, 'name': "STATIC ASYNC"},
            {'J': 1.4, 'K': 1, 'M': agents_num, 'name': "NEW STATIC PHASE WAVE"},
            {'J': 1.5, 'K': 1, 'M': 4, 'name': "SPLINTERED PHASE WAVE"},
            # {'J': 1, 'K': -1, 'M': 1, 'name': "ACTIVE PHASE WAVE"},
        ]
        time_delta = 0.1
        small_phase_steps = 10

        initial_params = {
            'phase_levels_number': 24,
            'agent_radius': 0.1,
            'min_distance': 0.4,
            'time_delta': time_delta,
            'small_phase_steps': small_phase_steps,
            'orientation_mode': True,
            'constraint_mode': True,
        }
        flying_altitude = 2
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
        logic = DiscreteSyncAndSwarmLogic(initial_params)
        knowledge = SharedKnowledge(ids)
        system_state = SystemState(
            ids,
            knowledge,
            positions=positions,
            phases=phases
        )
        communication = OfflineCommunication(system_state)

        crazyswarm_interface.system_state = system_state

        controller = SynchronizedOfflineController(
            agents_num,
            logic,
            crazyswarm_interface,
            communication,
            system_state,
            crazyswarm_interface,
            time_delta=time_delta,
            teleoperated_id=ids[0],
            key_poller=key_poller,
            teleop_blinking=True,
            params_list=params_presets,
            keyboard_callbacks={
                'k': crazyswarm_interface.kill_random_drone,
                'v': crazyswarm_interface.toggle_waving,
                'o': logic.phase_logic.toggle_oscillations,
                'l': crazyswarm_interface.swarm.finish,
                'f': crazyswarm_interface.swarm.finish_in_formation,
            },
            is_active=False
        )

        crazyswarm_interface.swarm.run_looped(
            controller.update,
            update_interval=time_delta,
            takeoff_height=flying_altitude,
            takeoff_time=flying_altitude/0.3,
            keep_height=flying_altitude,
            interactive_takeoff=True,
        )


if __name__ == "__main__":
    main()
