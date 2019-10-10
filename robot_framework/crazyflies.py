#! /usr/bin/env python

from pykpkswarm.kpkswarm import KPKCrazySwarm
import colorsys


class CrazySwarmInterface:
    def __init__(self):
        self.swarm = KPKCrazySwarm()
        self.was_teleoperated = False
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
                not self.was_teleoperated
            ):
                self.was_teleoperated = True
                # cf.turn_headlight_on()
            elif (
                not states[cf.id].is_teleoperated and
                self.was_teleoperated
            ):
                self.was_teleoperated = False
                # cf.turn_headlight_on(0)
