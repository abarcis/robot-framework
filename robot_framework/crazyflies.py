#! /usr/bin/env python

from pykpkswarm.kpkswarm import KPKCrazySwarm
import colorsys
import random


class CrazySwarmInterface:
    def __init__(self):
        self.swarm = KPKCrazySwarm()
        self.was_on = False
        for cf in self.swarm.allcfs.crazyflies:
            cf.set_bounding_box(2.2, 3.5, 4)

    def get_new_position(self, ident):
        cf = self.swarm.allcfs.crazyfliesById[ident]
        if cf.broken:
            return None
        else:
            return cf.position()

    def kill_random_drone(self):
        still_alive = [
            d for d in self.swarm.allcfs.crazyflies
            if not d.broken
        ]
        to_kill = random.choice(still_alive)
        print("Killing drone #{}".format(to_kill.id))
        to_kill.broken = True
        to_kill.land(
            targetHeight=0.2,
            duration=3
        )

    def update(self, states):
        for cf in self.swarm.allcfs.crazyflies:
            r, g, b = colorsys.hsv_to_rgb(states[cf.id].phase, 1, 1)
            cf.vel(states[cf.id].velocity)
            if (
                states[cf.id].is_teleoperated and
                not self.was_on == cf.id
            ):
                self.was_on = cf.id
                cf.led(0, 0, 0)
            elif (
                states[cf.id].is_teleoperated and
                self.was_on == cf.id
            ):
                self.was_on = False
                cf.led(r, g, b)
            else:
                cf.led(r, g, b)
