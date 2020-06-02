#! /usr/bin/env python

import numpy as np


def estimate_radius(N, J1, J2=0, d=0.1):
    attraction_factor = 0.5
    repulsion_factor = 2
    radius_estimate = np.sqrt((N - 1 + J2) / (N * (2 - J1))
                              * repulsion_factor / attraction_factor)
    new_radius = 0

    sines = np.array([np.sin(2 * np.pi * j / (2 * N)) for j in range(1, N)])
    while abs(new_radius - radius_estimate) > 0.01:
        if new_radius:
            radius_estimate = new_radius
        s = sum(2 * np.divide(sines, sines - d / radius_estimate))
        new_radius = np.sqrt(s / (4 * N * (1 - 0.5 * J1))
                             * repulsion_factor / attraction_factor)

    return new_radius
