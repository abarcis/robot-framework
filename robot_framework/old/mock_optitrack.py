#! /usr/bin/env python

from geometry_msgs.msg import (
    Pose, PoseStamped, Point, Quaternion, TwistStamped
)
from std_msgs.msg import Header
from pyquaternion import Quaternion as QuaternionType

import numpy as np
from builtin_interfaces.msg import Time

import math
import time
import datetime
import sys

import rclpy
from rclpy.node import Node

from .movement_mechanics import update_position
from .ros_communication import POSE_QOS, CUSTOM_QOS
from .utils import UpdateRate


class SwarmalatorState():
    def __init__(self, pose=None):
        if pose is None:
            self.position = np.array([np.random.uniform(-1, 1),
                                      np.random.uniform(-1, 1), 0])
            self.orientation = np.zeros(4)
            self.orientation[0] = np.random.uniform(-1, 1)
            self.orientation[3] = np.random.uniform(-1, 1)
            self.orientation = QuaternionType(self.orientation)
        else:
            self.position = np.array(pose[0])
            self.orientation = QuaternionType(angle=pose[1], axis=[0, 0, 1])
        self.velocity = np.zeros(3)
        self.angular_velocity = 0
        self.frame = "map"


class SwarmalatorStateWrapper():
    def __init__(self, pub, pose=None):
        self.state = SwarmalatorState(pose)
        self.pose_pub = pub
        self.last_update = datetime.datetime.now()

    def update_velocity(self, msg):
        self.state.frame = msg.header.frame_id
        self.state.velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ])
        self.state.angular_velocity = msg.twist.angular.z


class MockOptitrack(Node):
    def __init__(self, swarmalators):
        super().__init__('mock_optitrack')
        update_rate = 20
        self.swarmalators = {}
        for s in swarmalators:
            params = s.split('@')
            name = params[0]
            if len(params) > 1:
                pose = self.parse_pose(s)
            else:
                pose = None
            pose_pub = self.create_publisher(
                PoseStamped, '/{}/sync_and_swarm/pose'.format(name),
                qos_profile=POSE_QOS,
            )
            st = SwarmalatorStateWrapper(pose_pub, pose)
            self.create_subscription(
                TwistStamped, '/{}/sync_and_swarm/vel'.format(name),
                st.update_velocity,
                qos_profile=CUSTOM_QOS,
            )
            self.swarmalators[name] = st
        timer_period = 1/update_rate
        self.send_tmr = self.create_timer(
            timer_period, self.send_states_callback)
        self.update_tmr = self.create_timer(0.01, self.update_states_callback)
        self.states_update_rate = UpdateRate()
        self.update_rate_timer = self.create_timer(
            1, self.update_rate_callback)

    def update_rate_callback(self):
        print("MockOptitrack updates: {}".format(
            self.states_update_rate.pop_rate()))

    def parse_pose(self, s):
        pose = s.split('@')[1]
        try:
            pose = pose.split(',')
            x = float(pose[0])
            y = float(pose[1])
            if len(pose) == 3:
                angle = float(pose[2])
            else:
                angle = 0
                pose = ([x, y, 0], angle)
        except IndexError:
            print("{} is not a valid parameter. "
                  "Possible formats: 'name', 'name@pos_x,pos_y', "
                  "name@pos_x,pos_y,angle.".format(s))
            pose = None
        return pose

    def update_states_callback(self):
        for name, s in self.swarmalators.items():
            time_now = datetime.datetime.now()
            t = (time_now - s.last_update).total_seconds()
            position, orientation = update_position(
                s.state,
                s.state.velocity,
                s.state.frame == "map",
                s.state.angular_velocity,
                t
            )
            s.state.position = position
            s.state.orientation = orientation

            s.last_update = time_now

    def send_states_callback(self):
        self.states_update_rate.update()
        for name, swarmalator in self.swarmalators.items():
            modf = math.modf(time.time())
            nanosec, sec = map(int, (modf[0]*10**9, modf[1]))
            header = Header(
                frame_id="map",
                stamp=Time(sec=sec, nanosec=nanosec)
            )
            pose = PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(
                        x=swarmalator.state.position[0],
                        y=swarmalator.state.position[1],
                        z=swarmalator.state.position[2]
                    ),
                    orientation=Quaternion(
                        x=swarmalator.state.orientation.elements[1],
                        y=swarmalator.state.orientation.elements[2],
                        z=swarmalator.state.orientation.elements[3],
                        w=swarmalator.state.orientation.elements[0]
                    )
                )
            )

            swarmalator.pose_pub.publish(pose)

    def run(self):
        rclpy.spin(self)

    def destroy(self):
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    mock_optitrack = MockOptitrack(sys.argv[1:])
    mock_optitrack.run()


if __name__ == '__main__':
    main()
