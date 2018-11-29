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

from sync_and_swarm_msgs.msg import State as StateMsg
import rclpy
from rclpy.node import Node

from .ros_communication import CUSTOM_QOS


class SimVisulization(Node):
    def __init__(self):
        super().__init__('sim_visualization')
        self.state_sub = self.create_subscription(
            StateMsg, '/global/sync_and_swarm/state', self.add_marker,
            qos_profile=CUSTOM_QOS,
        )
        self.vis_pub = self.create_publisher(
            MarkerArray, 'sync_and_swarm/visualization',
        )
        self.marker_pub = self.create_publisher(
            Marker, 'sync_and_swarm/markers',
        )
        update_rate = 5
        self.markers = {}
        timer_period = 1/update_rate
        self.tmr = self.create_timer(timer_period, self.timer_callback)

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
            lifetime=Duration(sec=4),
            pose=msg.pose,
            scale=Vector3(x=0.05, y=0.05, z=0.05),
            header=Header(
                frame_id='base_link',
                stamp=Time(sec=sec, nanosec=nanosec)
            ),
            color=color
        )
        self.markers[msg.id] = marker

    def timer_callback(self):
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
