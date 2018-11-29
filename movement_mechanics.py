#! /usr/bin/env python
import numpy as np
from pyquaternion import Quaternion


def update_position(state, velocity, velocity_in_global_frame,
                    angular_velocity, time):
    if not velocity_in_global_frame:
        dp = state.orientation.rotate(np.array([1, 0, 0])) * velocity[0]

    else:
        dp = velocity

    position = (
        state.position
        + dp * time
    )
    if angular_velocity == 0:
        orientation = state.orientation
    else:
        orientation = Quaternion(
            axis=[0, 0, 1],
            angle=(
                (state.orientation.angle + angular_velocity * time)
                % (2 * np.pi)
            )
        )
    return position, orientation
