#! /usr/bin/env python

import matplotlib.pyplot as plt
import mpl_toolkits  # NOQA
from mpl_toolkits.mplot3d import Axes3D  # NOQA
import colorsys

from .base_visualization import BaseVisualization


class LiveVisualization(BaseVisualization):
    def __init__(self, agent_radius=None):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])
        self.ax.set_zlim([0, 10])
        self.ax.view_init(azim=0, elev=90)
        self.plot = None
        self.agent_radius = agent_radius
        self.fig.show()

    def update(self, states, t=None):
        xs = []
        ys = []
        zs = []
        positions = []
        colors = []
        phases = []
        vmin = 0
        vmax = 1

        for state in states.values():
            x, y, z = state.position
            xs.append(x)
            ys.append(y)
            zs.append(z)
            positions.append(state.position)
            phases.append(state.phase)
            colors.append(colorsys.hsv_to_rgb(state.phase, 1, 1))

        if self.plot is None:
            s = None
            if self.agent_radius:
                ppd = 0.5 * 72./self.ax.figure.dpi
                trans = self.ax.transData.transform
                # set size of marker to size of robot
                s = (
                    (trans((0, 0.5 * self.agent_radius)) - trans((0, 0))) * ppd
                )[1]
            self.plot = self.ax.scatter(
                xs,
                ys,
                zs,
                c=colors,
                vmin=vmin,
                vmax=vmax,
                s=s
            )
        else:
            # import numpy as np
            self.plot._offsets3d = (xs, ys, zs)
            self.plot.set_facecolors(colors)
            self.plot.set_edgecolors(colors)
            self.plot._facecolor3d = self.plot.get_facecolor()
            self.plot._edgecolor3d = self.plot.get_edgecolor()
            # self.plot.set_array(np.array(phases))
            # self.plot.set_3d_properties(zs, 'z')

        self.fig.canvas.flush_events()
        # plt.pause(0.001)
