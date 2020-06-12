#! /usr/bin/env python

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .base_controller import BaseController
from .rf_mission_executor import RFMissionExecutor
from mission_executor import MissionClient


class ROSController(BaseController):
    def __init__(self, *args, **kwargs):
        self.node = kwargs.pop('node')
        self.max_speed = 0.2

        super(ROSController, self).__init__(
            *args, **kwargs
        )

    def start(self):
        self.timer = self.node.create_timer(
            timer_period_sec=self.time_delta,
            callback=self.update
        )

    def stop(self):
        self.node.destroy_timer(self.timer)

    def update_params(self, params):
        self.logic.update_params(params)

    def update(self, *args):
        for ident in self.system_state.ids:
            self.system_state.states[ident].update(
                ident,
                position_feedback=self.position_feedback,
            )

        for ident in self.system_state.ids:
            if self.system_state.states[ident].position is None:
                continue

            state_update = self.logic.update_state(
                self.system_state.states[ident],
                self.system_state.knowledge.get_states_except_own(ident)
            )

            self.system_state.states[ident].update(
                ident,
                state_update,
                self.position_feedback,
            )

        if self.system_state.states[ident].small_phase == 0:
            self.send_state(ident)

            for visualization in self.visualizations:
                visualization.update(
                    self.system_state.states, self.current_time
                )

        self.current_time += self.time_delta

    def send_state(self, ident):
        self.communication[ident].send_state(
            ident,
            self.system_state.states[ident]
        )

    def run(self):
        try:
            rf_executor = RFMissionExecutor('mission_executor')
            mission_client = MissionClient()
            mission_client.add_mission_executor(rf_executor)
            executor = SingleThreadedExecutor()
            executor.add_node(rf_executor)
            executor.add_node(mission_client)
            executor.add_node(self.node)
            mission_client.get_logger().info(
                'Node initialized, waiting for events.'
            )
            executor.spin()

        finally:
            mission_client.destroy_node()
            rf_executor.destroy_node()
            self.node.destroy_node()
            rclpy.shutdown()
