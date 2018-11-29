#! /usr/bin/env python

import math
import numpy as np
import threading
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion
from pyquaternion import Quaternion as QuaternionType
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Vector3

from sync_and_swarm_msgs.msg import State as StateMsg

from .state import State
from .utils import UpdateRate


def create_state_from_msg(msg):
    pos = msg.pose.position
    orient = msg.pose.orientation
    position = np.array([pos.x, pos.y, pos.z])
    phase = msg.phase
    orientation = QuaternionType([orient.w, orient.x, orient.y, orient.z])
    return State(position, orientation, phase)


def create_pose_from_msg(msg):
    pos = msg.pose.position
    orient = msg.pose.orientation
    position = np.array([pos.x, pos.y, pos.z])
    orientation = QuaternionType([orient.w, orient.x, orient.y, orient.z])
    return (position, orientation)


CUSTOM_QOS = QoSProfile(
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    reliability=QoSReliabilityPolicy.
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    # RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    depth=5,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
)


POSE_QOS = QoSProfile(
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    reliability=QoSReliabilityPolicy.
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    # RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    depth=5,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
)


class ROSCommunication():
    def __init__(self, node, namespace):
        self.node = node
        self.namespace = namespace

        self.state_callbacks = []
        self.pose_callbacks = []

        self.state_pub = node.create_publisher(
            StateMsg, '/global/sync_and_swarm/state',
            qos_profile=CUSTOM_QOS,
        )
        self.state_sub = node.create_subscription(
            StateMsg, '/global/sync_and_swarm/state',
            self.run_state_callbacks,
            qos_profile=CUSTOM_QOS,
        )
        self.pose_sub = node.create_subscription(
            PoseStamped, '/{}/sync_and_swarm/pose'.format(namespace),
            self.run_pose_callbacks,
            qos_profile=POSE_QOS,
        )
        self.vel_pub = node.create_publisher(
            TwistStamped, '/{}/sync_and_swarm/vel'.format(namespace),
            # qos_profile=CUSTOM_QOS,
        )
        self.phase_pub = node.create_publisher(
            Float32, '/{}/sync_and_swarm/phase'.format(namespace),
            # qos_profile=CUSTOM_QOS,
        )
        self.pose_updates = UpdateRate()
        self.state_updates = UpdateRate()
        timer_period = 1
        self.tmr = self.node.create_timer(timer_period, self.timer_callback)

    def send_state(self, ident, state, frame=""):
        # TODO: get current time from ROS, after the clock implementation
        # is finished ( https://github.com/ros2/ros2/issues/178)
        modf = math.modf(time.time())
        nanosec, sec = map(int, (modf[0]*10**9, modf[1]))
        header = Header(
            frame_id=frame,
            stamp=Time(sec=sec, nanosec=nanosec)
        )
        msg = StateMsg(
            header=header,
            id=ident,
            phase=state.phase,
            pose=Pose(
                position=Point(
                    x=state.position[0],
                    y=state.position[1],
                    z=state.position[2]
                ),
                orientation=Quaternion(
                    x=state.orientation.elements[1],
                    y=state.orientation.elements[2],
                    z=state.orientation.elements[3],
                    w=state.orientation.elements[0]
                )
            )
        )
        self.state_pub.publish(msg)

    def timer_callback(self):
        self.node.get_logger().debug(
            "{} updates stats --- pose: {} state: {}".format(
                self.namespace,
                self.pose_updates.pop_rate(),
                self.state_updates.pop_rate(),
            )
        )

    def send_phase(self, phase):
        msg = Float32(data=phase)
        self.phase_pub.publish(msg)

    def send_vel(self, vel, ang_vel, frame=""):
        # TODO: get current time from ROS, after the clock implementation
        # is finished ( https://github.com/ros2/ros2/issues/178)
        modf = math.modf(time.time())
        nanosec, sec = map(int, (modf[0]*10**9, modf[1]))
        header = Header(
            frame_id=frame,
            stamp=Time(sec=sec, nanosec=nanosec)
        )
        msg = TwistStamped(
            header=header,
            twist=Twist(
                linear=Vector3(
                    x=float(vel[0]),
                    y=float(vel[1]),
                    z=float(vel[2])
                ),
                angular=Vector3(
                    x=float(0),
                    y=float(0),
                    z=float(ang_vel)
                )
            )
        )
        # TODO phase
        self.vel_pub.publish(msg)

    def run_pose_callbacks(self, msg):
        self.pose_updates.update()
        position, orientation = create_pose_from_msg(msg)
        for callback in self.pose_callbacks:
            callback(position, orientation)

    def add_pose_callback(self, callback):
        self.pose_callbacks.append(callback)

    def run_state_callbacks(self, msg):
        self.state_updates.update()
        ident = msg.id
        state = create_state_from_msg(msg)
        for callback in self.state_callbacks:
            callback(ident, state)

    def add_state_callback(self, callback):
        self.state_callbacks.append(callback)

    def is_alive(self):
        return rclpy.ok()


class ROSCommunicationManager(Node):
    def __init__(self):
        super().__init__('communication_manager')
        self.comms = []
        self.thread = threading.Thread(target=self.run)

    def create_communication_node(self, *args, **kwargs):
        c = ROSCommunication(self, *args, **kwargs)
        self.comms.append(c)
        return c

    def start_communication_thread(self):
        self.thread.start()

    def run(self):
        rclpy.spin(self)

    def destroy(self):
        self.destroy_node()
        self.thread.join()
        rclpy.shutdown()
