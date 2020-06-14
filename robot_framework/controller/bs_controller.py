#! /usr/bin/env python

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .base_controller import BaseController


class BSController(BaseController):
    def __init__(self, *args, **kwargs):
        self.node = kwargs.pop('node')

        super(BSController, self).__init__(
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
        states = [
            s[1] for s in
            self.system_state.knowledge.get_states_except_own(
                self.system_state.ids[0]
            )
        ]
        for visualization in self.visualizations:
            visualization.update(states, self.current_time)
        self.current_time += self.time_delta

    def send_state(self, ident):
        self.communication[ident].send_state(
            ident,
            self.system_state.states[ident]
        )

    def run(self):
        try:
            executor = SingleThreadedExecutor()
            executor.add_node(self.node)
            self.start()
            executor.spin()

        finally:
            self.node.destroy_node()
            rclpy.shutdown()
