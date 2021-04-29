#! /usr/bin/env python

import time

from controller.synchronized_offline_controller import (
    SynchronizedOfflineController
)
from controller.offline_controller import (
    OfflineController
)
from logic.discrete_sync_and_swarm_logic import DiscreteLogic
from logic.sync_and_swarm_logic import SyncAndSwarmLogic
from position_feedback.position_feedback import PositionFeedback
from knowledge.shared_knowledge import SharedKnowledge
from communication.offline_communication import OfflineCommunication
from system_state import SystemState
from visualization.live_visualization import LiveVisualization
from visualization.order_params_visualization import OrderParamsVisualization
from logger.state_log import StateLog

from utils.get_properties import estimate_radius
from utils.pattern_formed import (
    is_discrete_pattern_formed,
    is_original_pattern_formed,
)
from utils.order_parameters import calculate_var_r
import datetime
import os
import numpy as np

import keyboard

DEFAULT_IDENT = "a{}"
SIM_NUMBER = 1


def gen_pattern_formed_callback(vis, dirname, model='discrete'):
    def pattern_formed_callback(t, states, params):
        if model == 'discrete':
            is_pattern_formed = is_discrete_pattern_formed
            time_step = params['small_phase_steps'] * params['time_delta']
        if model == 'original':
            is_pattern_formed = is_original_pattern_formed
            time_step = params['time_delta']
        if is_pattern_formed(
            states, params
        ):
            vd = calculate_var_r(states.values())
            filename = (
                f"{model}:"
                f"N={params['agents_num']}:"
                f"Vd={vd}"
                f".pdf"
            )
            vis.fig.savefig(f'{dirname}/{filename}')
            print(t, "!!!!!FORMED!!!!")
            return True
        if t > 10000:
            print(f"{model}:{time_step} NOT FORMED")
            return True
        return False
    return pattern_formed_callback


def main():
    dirname = f'results/vd/{datetime.datetime.now()}'
    os.mkdir(dirname)
    with keyboard.KeyPoller() as key_poller:
        time_delta = 0.1
        params_presets = [
            {'J': 0.1, 'K': 1, 'M': 1, 'name': "STATIC SYNC"},
        ]
        scale_factors = [(0.5, 2)]  # [(1, 1), (0.5, 2), (0.5, 1), (0.5, 4)]

        small_phase_steps = 2
        initial_params = {
            'phase_levels_number': 18,
            'agent_radius': 0.1,
            'min_distance': 0.1,
            'small_phase_steps': small_phase_steps,
            'attraction_factor': 0.5,
            'repulsion_factor': 2
        }
        initial_params.update(params_presets[0])
        visualizations = [
            OrderParamsVisualization(params=initial_params),
            LiveVisualization(
                agent_radius=initial_params['agent_radius']
            ),
        ]
        discrete_pattern_formed_callback = gen_pattern_formed_callback(
            visualizations[-1],
            dirname,
            'discrete'
        )
        original_pattern_formed_callback = gen_pattern_formed_callback(
            visualizations[-1],
            dirname,
            'original'
        )

        # discrete modelA
        print("DISCRETE")
        for agents_num in range(4, 100, 2):
            print(agents_num)
            ids = [
                DEFAULT_IDENT.format(i) for i in range(agents_num)
            ]
            for attraction_factor, repulsion_factor in scale_factors:
                for i in range(SIM_NUMBER):
                    #for i in range(10):
                    initial_params.update({
                        'agents_num': agents_num,
                        'time_delta': time_delta,
                        'attraction_factor': attraction_factor,
                        'repulsion_factor': repulsion_factor,
                    })

                    logic = DiscreteLogic(initial_params)
                    knowledge = SharedKnowledge(ids)
                    system_state = SystemState(ids, knowledge, params=initial_params)
                    position_feedback = PositionFeedback(system_state, time_delta)
                    communication = OfflineCommunication(system_state)
                    logger = StateLog(initial_params, path=dirname)
                    controller = SynchronizedOfflineController(
                        agents_num,
                        logic=logic,
                        position_feedback=position_feedback,
                        communication=communication,
                        system_state=system_state,
                        visualizations=visualizations,
                        logger=logger,
                        # sleep_fcn=time.sleep,
                        key_poller=key_poller,
                        teleop_on=True,
                        params_list=params_presets,
                        mission_end_callback=discrete_pattern_formed_callback,
                        time_delta=time_delta,
                        small_phase_steps=small_phase_steps,
                    )

                    controller.run()

if __name__ == "__main__":
    main()
