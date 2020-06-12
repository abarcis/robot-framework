#! /usr/bin/env python

from rclpy.node import Node
from mission_manager.client import MissionExecutor


class RFMissionExecutor(Node, MissionExecutor):
    def __init__(self, controller):
        super().__init__('executor')
        self.controller = controller

    def start_mission(self, timestamp):
        self.get_logger().info(
            'Starting mission'
        )
        self.controller.start()

    def end_mission(self, timestamp):
        self.get_logger().info(
            'Ending mission'
        )
        self.controller.stop()

    def change_params(self, params, timestamp):
        self.get_logger().info(
            'Changing parameters to {}'.format(params)
        )
        self.controller.update_params(params)
