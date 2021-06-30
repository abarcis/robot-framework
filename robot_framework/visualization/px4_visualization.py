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
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import Cpuload

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
    ALTITUDE = 5

    def __init__(self, namespace):
        self.node = Node('px4_visualization', namespace=namespace)
        self.timestamp_ = 0
        self.vx, self.vy, self.vz = (0, 0, 0)
        self.yaw = 3.14

        self.offboard_control_mode_publisher_ = self.node.create_publisher(
            OffboardControlMode, 'OffboardControlMode_PubSubTopic', 10
        )
        self.trajectory_setpoint_publisher_ = self.node.create_publisher(
            TrajectorySetpoint, 'TrajectorySetpoint_PubSubTopic', 10
        )
        self.vehicle_command_publisher_ = self.node.create_publisher(
            VehicleCommand, 'VehicleCommand_PubSubTopic', 10
        )
        self.offboard_setpoint_counter_ = 0

        def timesync_callback(msg):
            self.timestamp_ = msg.timestamp

        def timer_callback():
            # if self.offboard_setpoint_counter_ == 10:
            #     # Change to Offboard mode after 10 setpoints
            #     self._publish_vehicle_command(
            #         VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
            #     )
            #     # Arm the vehicle
            #     self._arm()

            # Offboard_control_mode needs to be paired with trajectory_setpoint
            self._publish_offboard_control_mode()
            self._publish_trajectory_setpoint()

            # Stop the counter after reaching 11
            if self.offboard_setpoint_counter_ < 11:
                self.offboard_setpoint_counter_ += 1

        self.timesync_sub_ = self.node.create_subscription(
            Timesync, 'Timesync_PubSubTopic', timesync_callback, 10
        )
        self.log_ = self.node.get_logger()
        self.timer_ = self.node.create_timer(0.1, timer_callback)


    def update(self, states, t=None):
        # self.node.get_logger().info(
        #     'state received'
        # )

        assert len(states) == 1
        for state in states.values():
            self.vx, self.vy, self.vz = state.velocity
            self.yaw = state.angle_xy

    '''
    Publish the offboard control mode.
    For this example, only position and altitude controls are active.
    '''
    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher_.publish(msg)

    '''
    '''
    def _publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_
        msg.x = float("NaN")
        msg.y = float("NaN")
        msg.z = -float(self.ALTITUDE)
        msg.vx = float(self.vx)
        msg.vy = float(self.vy)
        msg.vz = 0.0
        msg.yaw = self.yaw

        self.trajectory_setpoint_publisher_.publish(msg)

    '''
    Publish vehicle commands
    command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
    param1    Command parameter 1
    param2    Command parameter 2
    '''
    def _publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_publisher_.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0
        )
        self.log_.info("Arm command send")

    def _disarm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0
        )
        self.log_.info("Disarm command send")
