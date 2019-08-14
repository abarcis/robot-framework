#! /usr/bin/env python

from pykpkswarm.kpkswarm import KPKCrazySwarm


class CrazySwarmInterface:
    def __init__(self):
        self.swarm = KPKCrazySwarm()
        for cf in self.swarm.allcfs.crazyflies:
            cf.set_bounding_box(1, 1, 1)

    def get_new_position(self, ident):
        return self.swarm.allcfs.crazyflies[ident].position()

    def update(self, states):
        for cf in self.swarm.allcfs.crazyflies:
            cf.vel(states[cf.id].velocity)
