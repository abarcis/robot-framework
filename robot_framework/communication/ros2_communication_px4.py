#! /usr/bin/env python

from datetime import datetime
import numpy as np

from pyquaternion import Quaternion as QuaternionType

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from builtin_interfaces.msg import Time

from .base_communication import BaseCommunication
from robot_framework.state import State
from sync_and_swarm_msgs.msg import State as StateMsg


class ROSCommunication(BaseCommunication):
    def __init__(self, *args, **kwargs):
        self.ident = kwargs.pop('agent_id')
        self.node = kwargs.pop('node')
        self.params = kwargs.pop('params')
        self.received = 0
        self.delayed = 0
        if self.ident != 'base_station':
            self.state_publisher = self.node.create_publisher(
                StateMsg,
                '/state',
                100
            )
        self.state_subscription = self.node.create_subscription(
            StateMsg,
            '/state',
            self.receive_state_callback,
            100
        )
        super(ROSCommunication, self).__init__(
            *args, **kwargs
        )

    def create_state_from_msg(self, msg):
        pos = msg.pose.position
        orient = msg.pose.orientation
        if self.params['orientation_mode']:
            orientation = QuaternionType(
                [orient.w, orient.x, orient.y, orient.z]
            )
        else:
            orientation = None
        position = np.array([pos.x, pos.y, pos.z])
        phase = msg.phase
        return State(
            position=position, phase=phase, orientation=orientation,
            params=self.params,
            sent_timestamp=datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec / 10**9),
            received_timestamp=datetime.now(),
        )

    def create_state_msg(self, sender, state, recipient_ident):
        now = self.node.get_clock().now()
        nanosec, sec = map(
            int,
            (now.nanoseconds % 10**9, now.nanoseconds/10**9)
        )
        header = Header(
            frame_id="",
            stamp=Time(sec=sec, nanosec=nanosec)
        )
        if state.orientation_mode:
            orientation = Quaternion(
                x=state.orientation.elements[1],
                y=state.orientation.elements[2],
                z=state.orientation.elements[3],
                w=state.orientation.elements[0]
            )
        else:
            orientation = None

        sender_ident = int(sender)
        phase = float(state.phase)
        msg = StateMsg(
            header=header,
            id=sender_ident,
            phase=phase,
            pose=Pose(
                position=Point(
                    x=state.position[0],
                    y=state.position[1],
                    z=state.position[2]
                ),
                orientation=orientation,
            )
        )
        return msg

    def send_state_unconditionally(self, sender, state, recipient_ident):
        msg = self.create_state_msg(sender, state, recipient_ident)
        self.state_publisher.publish(msg)

    def receive_state_callback(self, msg):
        state = self.create_state_from_msg(msg)
        self.receive_state(self.ident, state, str(msg.id).zfill(2))

    def receive_state(self, own_ident, state, sender_ident):
        if sender_ident == own_ident:
            return
        self.received += 1
        if (state.received_timestamp - state.timestamp).total_seconds() > 0.2:
            self.delayed += 1
        if self.check_receive_filters(own_ident, sender_ident, state):
            self.receive_state_unconditionally(
                own_ident, state, sender_ident
            )

    def receive_state_unconditionally(self, own_ident, state, sender_ident):
        self.system_state.knowledge.update_state(
            own_ident=own_ident,
            other_ident=sender_ident,
            new_state=State(state=state, received_timestamp=datetime.now()),
        )