#! /usr/bin/env python

import numpy as np
from .order_parameters import potential_M_N, centroid_m


def is_discrete_pattern_formed(states, params):
    K = params['K']
    M = params['M']
    potential = potential_M_N(K, M, states.values())
    velocities = np.array([s.velocity for s in states.values()])
    speeds = np.linalg.norm(velocities, axis=1)
    # print(max(speeds))
    return potential < 1e-15 and max(speeds) < 0.001


def is_original_pattern_formed(states, params):
    phases = [s.phase for s in states.values()]
    centroid = centroid_m(1, phases)
    velocities = np.array([s.velocity for s in states.values()])
    speeds = np.linalg.norm(velocities, axis=1)
    return abs(1 - centroid) < 1e-15 and max(speeds) < 0.001
