#! /usr/bin/env python


from mpl_toolkits.mplot3d import Axes3D  # NOQA
import matplotlib.pyplot as plt
import colorsys

from base_visualization import BaseVisualization


class LiveVisualization(BaseVisualization):
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([0, 3])

        self.plot = None

    def update(self, states):
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
            self.plot = self.ax.scatter(
                xs,
                ys,
                zs,
                c=colors,
                vmin=vmin,
                vmax=vmax
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

        plt.pause(0.001)
