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
from std_msgs.msg import Header, Char, ColorRGBA, Float32, String
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from builtin_interfaces.msg import Time

from .base_visualization import BaseVisualization

CUSTOM_QOS = QoSProfile(
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    reliability=QoSReliabilityPolicy.
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    # RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    depth=1,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
)

# TODO adjust for PX4, for now only copied version for Balboas


class PX4Visualization(BaseVisualization):
    def __init__(self, agent_radius=None):
        self.agent_radius = agent_radius

        self.node = Node('balboa_visualization')

        self.vel_pub = self.node.create_publisher(
            TwistStamped,
            "vel",
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
        # self.node.get_logger().info(
        #     'state received'
        # )

        for state in states.values():
            rgb = colorsys.hsv_to_rgb(state.phase, 1, 1)

            color = ColorRGBA()
            color.r = float(rgb[0])
            color.g = float(rgb[1])
            color.b = float(rgb[2])
            self.color_pub.publish(color)

            now = self.node.get_clock().now()
            nanosec, sec = map(
                int,
                (now.nanoseconds % 10**9, now.nanoseconds/10**9)
            )
            header = Header(
                frame_id="",
                stamp=Time(sec=sec, nanosec=nanosec)
            )
            x, y, z = state.velocity
            twist = Twist(
                linear=Vector3(x=float(x), y=float(y), z=float(z)),
                angular=Vector3(x=0., y=0., z=float(state.angular_speed))
            )
            twist_msg = TwistStamped(header=header, twist=twist)
            self.vel_pub.publish(twist_msg)
