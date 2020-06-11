#! /usr/bin/env python


class StateUpdate:
    def __init__(
        self,
        position_update=None,
        phase_update=None,
        phase_level_update=None,
        small_phase_update=None,
        phase_correction_update=None,
        velocity_update=None,
        orientation_update=None,
        angular_speed_update=None,
        teleop_update=None
    ):
        self.position_update = position_update
        self.phase_update = phase_update
        self.phase_level_update = phase_level_update
        self.small_phase_update = small_phase_update
        self.phase_correction_update = phase_correction_update
        self.velocity_update = velocity_update
        self.orientation_update = orientation_update
        self.angular_speed_update = angular_speed_update
        self.teleop_update = teleop_update

    def __str__(self):
        return "phase update: {}, \
            phase_level_update: {}, \
            small_phase_update: {}, \
            phase_correction_update: {}, \
            position update: {}, \
            orientation update: {}, \
            velocity update: {}, \
            angular speed update: {}".format(
                self.phase_update,
                self.phase_level_update,
                self.small_phase_update,
                self.phase_correction_update,
                self.position_update,
                self.orientation_update,
                self.velocity_update,
                self.angular_speed_update,
            )
