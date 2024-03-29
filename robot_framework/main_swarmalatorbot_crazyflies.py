#! /usr/bin/env python

from robot_framework.controller.synchronized_offline_controller import (
    SynchronizedOfflineController
)
from robot_framework.logic.discrete_sync_and_swarm_logic import DiscreteLogic
from robot_framework.knowledge.shared_knowledge import SharedKnowledge
from robot_framework.communication.offline_communication import OfflineCommunication
from robot_framework.system_state import SystemState
from robot_framework.crazyflies import CrazySwarmInterface
from robot_framework import keyboard
from robot_framework.logger.state_log import StateLog
import random

import numpy as np


DEFAULT_IDENT = "{}"


def main():
    with keyboard.KeyPoller() as key_poller:
        agents_num = 8
        params_presets = [
            {'J': 0.1, 'K': 1, 'M': 1, 'name': "STATIC SYNC"},
            {'J': -1.5, 'K': 0.1, 'M': 2, 'name': "STATIC ASYNC"},
            {'J': 1.4, 'K': 1, 'M': agents_num, 'name': "NEW STATIC PHASE WAVE"},
            {'J': 1.5, 'K': 1, 'M': 3, 'name': "SPLINTERED PHASE WAVE"},
            # {'J': 1, 'K': -1, 'M': 1, 'name': "ACTIVE PHASE WAVE"},
        ]
        time_delta = 0.05
        small_phase_steps = 10
        flying_altitude = 2

        initial_params = {
            'phase_levels_number': 24,
            'agent_radius': 0.1,
            'min_distance': 0.3,
            'attraction_factor': 0.75,
            'time_delta': time_delta,
            'small_phase_steps': small_phase_steps,
            'orientation_mode': False,
            'constraint_mode': False,
            'goal': np.array([0, 0, flying_altitude])
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
        uniform_phases = [random.random() for i in range(len(ids))] # 1. / len(ids) * i
        random.shuffle(uniform_phases)
        phases = {
            cf.id: uniform_phases[i]
            for i, cf
            in enumerate(crazyswarm_interface.swarm.allcfs.crazyflies)
        }
        logic = DiscreteLogic(initial_params)
        knowledge = SharedKnowledge(ids)
        system_state = SystemState(
            ids,
            knowledge,
            positions=positions,
            phases=phases,
            params=initial_params
        )
        communication = OfflineCommunication(system_state)

        crazyswarm_interface.system_state = system_state

        controller = SynchronizedOfflineController(
            agents_num,
            logic=logic,
            position_feedback=crazyswarm_interface,
            communication=communication,
            system_state=system_state,
            visualizations=[crazyswarm_interface],
            time_delta=time_delta,
            # teleoperated_id=ids[0],
            key_poller=key_poller,
            teleop_on=True,
            logger=StateLog(initial_params),
            # teleop_blinking=True,
            params_list=params_presets,
            keyboard_callbacks={
                'k': crazyswarm_interface.kill_random_drone,
                'v': crazyswarm_interface.toggle_waving,
                #'o': logic.phase_logic.toggle_oscillations,
                'l': crazyswarm_interface.swarm.finish,
                'f': crazyswarm_interface.swarm.finish_in_formation,
            },
            # is_active=False
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
