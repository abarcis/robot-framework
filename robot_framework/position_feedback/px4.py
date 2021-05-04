#! /usr/bin/env python

from pyquaternion import Quaternion
import numpy as np

from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition


class PX4PositionFeedback:
    def __init__(self, system_state, time_delta):
        self.node = Node('position_feedback')
        self.system_state = system_state
        self.time_delta = time_delta
        self.poses = {ident: {'position': None, 'orientation': None}
                      for ident in self.system_state.states.keys()}
        self.subscribers = [
            self.node.create_subscription(
                VehicleGlobalPosition,
                f'/d{ident}/VehicleGlobalPosition_PubSubTopic',
                self.set_new_pose_callback(ident),
                1,
            )
            for ident in self.system_state.states.keys()
        ]

    def set_new_pose_callback(self, ident):
        def _set_new_pose_callback(msg):
            self.poses[ident]['position'] = np.array([
                msg.lat, msg.lon, 0
            ])
            # self.poses[ident]['orientation'] = Quaternion([
            #     msg.pose.orientation.w, msg.pose.orientation.x,
            #     msg.pose.orientation.y, msg.pose.orientation.z
            # ])
            # vec = self.poses[ident]['orientation'].rotate([1, 0, 0])
            # vec_xy = vec
            # vec_xy[2] = 0
        return _set_new_pose_callback

    def get_new_position(self, ident):
        return self.poses[ident]['position']

    def get_new_orientation(self, ident):
        return self.poses[ident]['orientation']

    def get_node(self):
        return self.node
