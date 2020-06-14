#! /usr/bin/env python

import colorsys
import time
import random

from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from rclpy.node import Node
from std_msgs.msg import Char, ColorRGBA, Float32, String
from geometry_msgs.msg import TwistStamped

from .base_visualization import BaseVisualization

CUSTOM_QOS = QoSProfile(
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    reliability=QoSReliabilityPolicy.
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    # RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    depth=1,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
)


class BalboaVisualization(BaseVisualization):
    def __init__(self, agent_radius=None):
        self.agent_radius = agent_radius

        self.node = Node('balboa_visualization')

        self.vel_pub = self.node.create_publisher(
            TwistStamped,
            "vel_swing",
            qos_profile=CUSTOM_QOS,
        )
        self.color_pub = self.node.create_publisher(
            ColorRGBA,
            "color",
            qos_profile=CUSTOM_QOS,
        )
        self.command_pub = self.node.create_publisher(
            Char,
            "command",
            qos_profile=CUSTOM_QOS,
        )

    def update(self, states, t=None):
        print(states, t)
        self.node.get_logger().info(
            'state received'
        )

        for state in states.values():
            rgb = colorsys.hsv_to_rgb(state.phase, 1, 1)

            color = ColorRGBA()
            color.r = float(rgb[0])
            color.g = float(rgb[1])
            color.b = float(rgb[2])
            time.sleep(random.random()/2)
            self.color_pub.publish(color)

            # x, y, z = state.position
            # xs.append(x)
            # ys.append(y)
            # zs.append(z)
            # positions.append(state.position)
            # phases.append(state.phase)
            # if state.orientation_mode:
            #     vec = state.orientation.rotate([0.3, 0, 0])
            #     begin = state.position
            #     end = begin + vec
            #     orientation_line = [
            #         [begin[0], end[0]],
            #         [begin[1], end[1]],
            #         'k',
            #     ]
            #     orientations.append(orientation_line)
