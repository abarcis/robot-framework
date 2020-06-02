#! /usr/bin/env python

import matplotlib.pyplot as plt

from .base_visualization import BaseVisualization
from utils.order_parameters import (
    calculate_var_r,
    potential_M_N,
    angular_distance,
    calculate_S,
    centroid_m
)


class OrderParamsVisualization(BaseVisualization):
    def __init__(self, params):
        self.params = params

        self.ts = []
        self.var_rs = []
        self.time_coord_potentials = []
        self.angular_distances = []
        self.Ss = []
        self.centroids = []

        self.fig, (
            (self.ax_var_rs, self.ax_S),
            (self.ax_ang_dist, self.ax_time_coord),
            (self.ax_centroid, self.ax_spare)
        ) = plt.subplots(3, 2, sharex=True)
        self.reinit(params)
        self.fig.show()
        # self.ax = self.fig.add_subplot(111)

    def reinit(self, params):
        self.params = params
        self.ts = []
        self.var_rs = []
        self.time_coord_potentials = []
        self.angular_distances = []
        self.Ss = []
        self.centroids = []

        self.ax_var_rs.cla()
        self.ax_S.cla()
        self.ax_ang_dist.cla()
        self.ax_time_coord.cla()
        self.ax_centroid.cla()

        self.ax_var_rs.set_ylim(ymin=0)
        self.ax_S.set_ylim(ymin=0)
        self.ax_ang_dist.set_ylim(ymin=0)
        self.ax_time_coord.set_ylim(ymin=0)
        self.ax_centroid.set_ylim(ymin=0)

        self.ax_var_rs.set_title("Distance from middle diff")
        self.ax_S.set_title("S")
        self.ax_ang_dist.set_title("Angular distance diff")
        self.ax_time_coord.set_title("Time coordination potential")
        self.ax_centroid.set_title("Synchronization centroid")

    def update(self, states, t):
        if abs(t % 1) < 0.001 or 1 - abs(t % 1) < 0.001:
            self.ts.append(t)

            # calculate new parameters
            self.var_rs.append(calculate_var_r(list(states.values())))
            # self.time_coord_potentials.append(
            #     potential_M_N(self.params['K'], self.params['M'],
            #                   list(states.values()))
            # )
            self.angular_distances.append(angular_distance(
                self.params['M'], states.values())
            )
            # self.Ss.append(calculate_S(states.values()))
            # phases = [
            #     s.phase_level / s.phase_levels_number for s in states.values()
            # ]
            # self.centroids.append(centroid_m(1, phases))

            # # update plots
            # self.ax_var_rs.scatter(self.ts, self.var_rs, s=20, c='b')
            # self.ax_time_coord.scatter(self.ts, self.time_coord_potentials,
            #                            s=20, c='r')
            # self.ax_ang_dist.scatter(self.ts, self.angular_distances,
            #                          s=20, c='g')
            # self.ax_S.scatter(self.ts, self.Ss, s=20, c='k')
            # self.ax_centroid.scatter(self.ts, self.centroids, s=20, c='m')

        plt.pause(0.001)
