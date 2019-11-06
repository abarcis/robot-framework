#! /usr/bin/env python
import rclpy
from rclpy.executors import SingleThreadedExecutor
import time

from .orientation import OrientationOpts
from .ros_communication import ROSCommunicationManager
from .mock_optitrack import MockOptitrack
from .system_state import (
    SystemStateManager,
    SystemState,
    LoggableSystemStateManagerMixin,
    LoggableSystemStateMixin,
)
from .utils import time_delta, time_throttled


class ROSCommunicationMixin:
    def __init__(self, communication_interface, *args, **kwargs):
        self.com = communication_interface
        super().__init__(*args, **kwargs)


class ROSCommunicationStateSubscriberMixin:
    def __init__(self, *args, **kwargs):
        self.states = {}
        self.com.add_state_callback(self._update_state)
        super().__init__(*args, **kwargs)

    def _update_state(self, ident, state):
        self.states[ident] = state


class ROSCommunicationPoseSubscriberMixin:
    def __init__(self, *args, **kwargs):
        @time_delta(0)
        def update_pose(position, orientation, time_delta):
            if not self.is_running:
                return
            state, vel, ang_vel = self.agent.update_state(
                position, orientation, self.states.items(), time_delta
            )
            self.send_state(state)
            self.send_vel(vel, ang_vel)
            self.send_phase(state.phase)

        self.com.add_pose_callback(update_pose)
        super().__init__(*args, **kwargs)


class ROSCommunicationStatePublisherMixin:
    def __init__(self, *args, **kwargs):
        _super = super()

        @time_throttled(1)
        def send_state_throttled(state):
            self.com.send_state(
                self.agent.ident,
                state,
                "map" if self.agent.orient_mode != OrientationOpts.CONSTRAINED
                else "local"
            )

        def send_state(state):
            _super.send_state(state)
            send_state_throttled(state)

        self.send_state = send_state

        super().__init__(*args, **kwargs)


class ROSCommunicationVelocityPublisherMixin:
    @time_throttled(0.05)
    def send_vel(self, vel, ang_vel):
        self.com.send_vel(
            vel,
            ang_vel,
            "map" if self.agent.orient_mode != OrientationOpts.CONSTRAINED
            else "local"
        )


class ROSCommunicationPhasePublisherMixin:
    @time_throttled(0.05)
    def send_phase(self, phase):
        self.com.send_phase(phase)


class ROSSystemState(
    ROSCommunicationMixin,
    ROSCommunicationStatePublisherMixin,
    LoggableSystemStateMixin,
    ROSCommunicationVelocityPublisherMixin,
    ROSCommunicationPhasePublisherMixin,
    ROSCommunicationStateSubscriberMixin,
    ROSCommunicationPoseSubscriberMixin,
    SystemState,
):
    pass


class ROSSpinManagerMixin:
    def __init__(self, *args, **kwargs):
        rclpy.init()
        self.nodes = []
        super().__init__(*args, **kwargs)

    def spin(self):
        executor = SingleThreadedExecutor()
        for node in self.nodes:
            executor.add_node(node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            for node in self.nodes:
                node.destroy_node()
            self.destroy()


class ROSCommunicationManagerMixin:
    def __init__(self, name_template, vis_topic, *args, **kwargs):
        self.name_template = name_template
        self.comm_manager = ROSCommunicationManager()
        self.nodes.append(self.comm_manager)
        self.vis_topic = vis_topic
        self.send_prob = kwargs['send_prob']
        self.recv_prob = kwargs['recv_prob']
        self.name_list = []

        super().__init__(*args, **kwargs)

    def get_system_state_kwargs(self, ident):
        name = self.name_template.format(ident)

        kwargs = super().get_system_state_kwargs(ident)
        kwargs.update({
            'communication_interface': (
                self.comm_manager.create_communication_node(
                    name,
                    self.vis_topic,
                    self.send_prob,
                    self.recv_prob,
                )
            )
        })
        self.name_list.append(name)
        return kwargs


class MockOptitrackManagerMixin:
    def spin(self, *args, **kwargs):
        self.nodes.append(MockOptitrack(self.name_list))
        super().spin()


class ROSSystemStateMixin:
    def get_system_state_class(self):
        return ROSSystemState


class ROSStaticPoseUpdateMixin:
    def __init__(self, *args, **kwargs):
        time_step = 0.05

        def update_agents_pose():
            if not self.is_running:
                return

            start = time.time()

            states = self.common_state.states.copy()
            self.common_state.states.clear()

            for agent in self.agents:
                agent.update_pose(time_step, states)

            self.timer_node.get_logger().info("Usage: {}%%".format(
                int(((time.time() - start)/time_step)*100)
            ))

        self.timer_node = rclpy.create_node("timer_pose_updater")
        self.timer_node.create_timer(time_step, update_agents_pose)

        self.nodes.append(self.timer_node)

        super().__init__(*args, **kwargs)


class ROSSystemStateManager(
    LoggableSystemStateManagerMixin,
    ROSSpinManagerMixin,
    ROSCommunicationManagerMixin,
    ROSSystemStateMixin,
    SystemStateManager,
):
    pass


class ROSSystemStateWithOptitrackManager(
    MockOptitrackManagerMixin,
    ROSSystemStateManager,
):
    pass
