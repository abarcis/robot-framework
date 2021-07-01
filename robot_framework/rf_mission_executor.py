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
        for k, p in params.items():
            if k == 'M':
                params[k] = int(p)
            elif k == 'collaborators':
                params[k] = list(p.split(':'))
            elif k == 'goal':
                params[k] = [float(coord) for coord in p.split(':')]
            else:
                params[k] = float(p)
        self.get_logger().info(
            'Changing parameters to {}'.format(params)
        )
        self.controller.update_params(params)

    def progress_reported(self, progress):
        print("PROGRESS: ", progress)
