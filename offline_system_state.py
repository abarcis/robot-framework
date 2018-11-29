#! /usr/bin/env python

import numpy as np

from .movement_mechanics import update_position
from .ros_system_state import (
    ROSSpinManagerMixin,
    ROSStaticPoseUpdateMixin,
    ROSCommunicationManagerMixin,
    ROSCommunicationMixin,
    ROSCommunicationStatePublisherMixin,
)
from .state import create_state_random
from .system_state import (
    SystemStateManager,
    SystemState,
    # LoggableSystemStateManagerMixin,
    # LoggableSystemStateMixin,
)
from .orientation import OrientationOpts


class CommonOfflineState:
    def __init__(self):
        self.states = {}

    def set_agent_state(self, ident, state):
        self.states[ident] = state

    def __str__(self):
        return str(self.states)


class StaticPoseUpdateMixin:
    def __init__(self, initial_state=None, *args, **kwargs):
        self.vel = np.zeros(3)
        self.ang_vel = 0

        super().__init__(*args, **kwargs)
        if initial_state is None:
            initial_state = create_state_random(
                orient=self.agent.orient_mode
            )
        self.state = initial_state

    def update_pose(self, TIME_STEP):
        position, orientation = update_position(
            self.state, self.vel,
            self.agent.orient_mode != OrientationOpts.CONSTRAINED,
            self.ang_vel, TIME_STEP
        )
        self.state.position = position
        self.state.orientation = orientation

        state, vel, ang_vel = self.agent.update_state(
            position, orientation, self.common_state.states.items(),
            TIME_STEP
        )

        self.send_state(state)
        self.send_vel(vel, ang_vel)

    def send_vel(self, vel, ang_vel):
        self.vel = vel
        self.ang_vel = ang_vel
        super().send_vel(vel, ang_vel)


class CommonStateMixin:
    def __init__(self, common_state, *args, **kwargs):
        self.common_state = common_state
        super().__init__(*args, **kwargs)

    def send_state(self, state):
        self.common_state.set_agent_state(self.agent.ident, state)
        super().send_state(state)


class OfflineSystemState(
    ROSCommunicationMixin,
    ROSCommunicationStatePublisherMixin,
    # LoggableSystemStateMixin,
    StaticPoseUpdateMixin,
    CommonStateMixin,
    SystemState,
):
    pass


class CommonOfflineStateManagerMixin:
    def __init__(self, *args, **kwargs):
        self.common_state = CommonOfflineState()
        super().__init__(*args, **kwargs)

    def get_system_state_kwargs(self, ident):
        kwargs = super().get_system_state_kwargs(ident)
        kwargs.update({
            'common_state': self.common_state,
        })
        return kwargs


class OfflineStateMixin:
    def get_system_state_class(self):
        return OfflineSystemState


class OfflineSystemStateManager(
    # LoggableSystemStateManagerMixin,
    ROSSpinManagerMixin,
    ROSCommunicationManagerMixin,
    ROSStaticPoseUpdateMixin,
    CommonOfflineStateManagerMixin,
    OfflineStateMixin,
    SystemStateManager,
):
    pass
