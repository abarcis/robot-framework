#! /usr/bin/env python

from pykpkswarm.kpkswarm import KPKCrazySwarm
import colorsys
import logging
import math
import random

LANDING_VEL_VERT = 0.3  # [m/s]
LANDING_HEIGHT = 0.00  # [m]


class CrazySwarmInterface:
    def __init__(self, flying_altitude, system_state=None):
        self.swarm = KPKCrazySwarm(landing_fcn=self.raindrop_land)
        self.flying_altitude = flying_altitude
        self.was_on = False
        self.system_state = system_state
        for cf in self.swarm.allcfs.crazyflies:
            cf.set_bounding_box(2.0, 3.5, 10)
        self.waving = False

    def toggle_waving(self):
        self.waving = not self.waving
        logging.info("Waving set to {}".format(self.waving))

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
        logging.info("Killing drone #{}".format(to_kill.id))
        to_kill.broken = True
        to_kill.land(
            targetHeight=0.2,
            duration=3
        )

    def raindrop_land(
        self,
        landing_velocity=LANDING_VEL_VERT,
        offset=0.5,
    ):
        time_extra = 0.1
        if self.system_state is not None:
            alive_crazyflies = [
                (cf, self.system_state.states[cf.id].phase)
                for cf in self.swarm.allcfs.crazyflies
                if not cf.broken
            ]
        else:
            alive_crazyflies = [
                (cf, None)
                for cf in self.swarm.allcfs.crazyflies
                if not cf.broken
            ]
        alive_crazyflies = sorted(alive_crazyflies, key=lambda k: (k[1], k[0]))
        height = alive_crazyflies[0][0].position()[2]
        duration = (height - LANDING_HEIGHT) / landing_velocity
        for cf, phase in alive_crazyflies:
            cf.land(targetHeight=LANDING_HEIGHT, duration=duration)
            self.swarm.timeHelper.sleep(offset)
        self.swarm.timeHelper.sleep(duration + time_extra)

    def update(self, states):
        for cf in self.swarm.allcfs.crazyflies:
            r, g, b = colorsys.hsv_to_rgb(states[cf.id].phase, 1, 1)

            if self.waving:
                waving_height = math.sin(states[cf.id].phase*math.pi*2)/20
                cf.keep_height(self.flying_altitude + waving_height)
            else:
                cf.keep_height(self.flying_altitude)
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
