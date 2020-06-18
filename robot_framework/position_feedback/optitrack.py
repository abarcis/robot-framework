#! /usr/bin/env python

from pyquaternion import Quaternion
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class Optitrack:
    def __init__(self, states, time_delta):
        self.node = Node('position_feedback')
        self.states = states
        self.time_delta = time_delta
        self.poses = {ident: {'position': None, 'orientation': None}
                      for ident in self.states.keys()}
        self.subscribers = [
            self.node.create_subscription(
                PoseStamped,
                f'/b{ident}/pose',
                self.set_new_pose_callback(ident),
                1,
            )
            for ident in self.states.keys()
        ]

    def set_new_pose_callback(self, ident):
        def _set_new_pose_callback(msg):
            self.poses[ident]['position'] = np.array([
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            ])
            self.poses[ident]['orientation'] = Quaternion([
                msg.pose.orientation.w, msg.pose.orientation.x,
                msg.pose.orientation.y, msg.pose.orientation.z
            ])
            vec = self.poses[ident]['orientation'].rotate([1, 0, 0])
            vec_xy = vec
            vec_xy[2] = 0
        return _set_new_pose_callback

    def get_new_position(self, ident):
        return self.poses[ident]['position']

    def get_new_orientation(self, ident):
        return self.poses[ident]['orientation']

    def get_node(self):
        return self.node