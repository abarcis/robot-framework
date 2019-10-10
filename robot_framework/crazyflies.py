#! /usr/bin/env python

from pykpkswarm.kpkswarm import KPKCrazySwarm
import colorsys


class CrazySwarmInterface:
    def __init__(self):
        self.swarm = KPKCrazySwarm()
        self.last_states = None
        for cf in self.swarm.allcfs.crazyflies:
            cf.set_bounding_box(2.2, 3.5, 4)

    def get_new_position(self, ident):
        return self.swarm.allcfs.crazyfliesById[ident].position()

    def update(self, states):
        for cf in self.swarm.allcfs.crazyflies:
            cf.vel(states[cf.id].velocity)
            r, g, b = colorsys.hsv_to_rgb(states[cf.id].phase, 1, 1)
            cf.led(r, g, b)
            if (
                states[cf.id].is_teleoperated and
                (
                    not self.last_states or
                    not self.last_states[cf.id].is_teleoperated
                )
            ):
                cf.tun_headlight_on()
            if (
                not states[cf.id].is_teleoperated and
                (
                    self.last_states and
                    self.last_states[cf.id].is_teleoperated
                )
            ):
                cf.tun_headlight_on(0)
        self.last_states = states
