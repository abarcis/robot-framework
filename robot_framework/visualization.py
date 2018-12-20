#! /usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion
import colorsys
import numpy as np
from builtin_interfaces.msg import Time, Duration
import math
import time
import sys

from sync_and_swarm_msgs.msg import State as StateMsg
import rclpy
from rclpy.node import Node

from .ros_communication import CUSTOM_QOS


class SimVisulization(Node):
    def __init__(self):
        super().__init__('sim_visualization')
        state_topic = sys.argv[1]
        print(state_topic)
        self.state_sub = self.create_subscription(
            StateMsg, state_topic, self.add_marker,
            qos_profile=CUSTOM_QOS,
        )
        self.vis_pub = self.create_publisher(
            MarkerArray, 'sync_and_swarm/visualization',
        )
        self.marker_pub = self.create_publisher(
            Marker, 'sync_and_swarm/markers',
        )
        update_rate = 20
        self.markers = {}
        self.markers_no_color = {}
        timer_period = 1/update_rate
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def test(self, msg):
        num = msg.id % 100
        text = str(time.time()) + ' ' * num + '0\n'
        print(text)
        # sys.stderr.write(text)
        # self.get_logger().info(text)

    def add_marker(self, msg):
        hue = msg.phase/(2*np.pi)
        rgb = colorsys.hsv_to_rgb(hue, 1, 1)
        modf = math.modf(time.time())
        nanosec, sec = map(int, (modf[0]*10**9, modf[1]))
        color = ColorRGBA(r=float(rgb[0]), g=float(rgb[1]),
                          b=float(rgb[2]), a=float(1))

        if msg.pose.orientation != Quaternion(x=0.0, y=0.0, z=0.0, w=0.0):
            arrow = Marker(
                type=Marker.ARROW,
                id=msg.id+10000,
                lifetime=Duration(sec=4),
                pose=msg.pose,
                scale=Vector3(x=0.07, y=0.02, z=0.02),
                header=Header(frame_id='base_link'),
                color=color
            )
            self.markers['{}_arr'.format(msg.id)] = arrow
        else:
            msg.pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        marker = Marker(
            type=Marker.SPHERE,
            id=msg.id,
            lifetime=Duration(nanosec=500000000),
            pose=msg.pose,
            scale=Vector3(x=0.05, y=0.05, z=0.05),
            header=Header(
                frame_id='base_link',
                stamp=Time(sec=sec, nanosec=nanosec)
            ),
            color=color
        )
        marker_no_color = Marker(
            type=Marker.SPHERE,
            id=msg.id+100000,
            lifetime=Duration(sec=6),
            pose=msg.pose,
            scale=Vector3(x=0.05, y=0.05, z=0.05),
            header=Header(
                frame_id='base_link',
                stamp=Time(sec=sec, nanosec=nanosec)
            ),
            color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.2)
        )

        self.markers[msg.id] = marker
        self.markers['{}_no_col'.format(msg.id)] = marker_no_color
        # marker.color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
        # marker.lifetime = Duration(sec=2)
        # self.markers['{}_no_color'.format(msg.id)] = marker
        # self.marker_pub.publish(marker)
        # self.marker_pub.publish(marker_no_color)

    def timer_callback(self):
        print(len(self.markers))
        # sys.stdout.flush()
        markers_arr = MarkerArray()
        markers_arr.markers = list(self.markers.values())
        self.vis_pub.publish(markers_arr)
        self.markers = {}

    def run(self):
        rclpy.spin(self)

        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    vis = SimVisulization()
    vis.run()


if __name__ == '__main__':
    main()
