#! /usr/bin/env python


class StateUpdate:
    def __init__(
        self,
        position_update=None,
        phase_update=None,
        phase_level_update=None,
        velocity_update=None,
        teleop_update=None
    ):
        self.position_update = position_update
        self.phase_update = phase_update
        self.phase_level_update = phase_level_update
        self.velocity_update = velocity_update
        self.teleop_update = teleop_update

    def __str__(self):
        return "phase update: {}, \
            phase_level_update: {}, \
            position update: {}, \
            velocity update: {}".format(
                self.phase_update,
                self.phase_level_update,
                self.position_update,
                self.velocity_update
            )
