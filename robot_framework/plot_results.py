#! /usr/bin/env python

import os
import matplotlib.pyplot as plt
import numpy as np


def parse_convergence_time_data(filenames):
    data = {'original': {}, 'discrete': {}}
    for f in filenames:
        d = f.split(':')
        model = d[0]
        dt = d[1]
        t = float(d[2].split('=')[1])
        scale = d[3]
        if scale not in data[model]:
            data[model][scale] = {}
        if dt not in data[model][scale]:
            data[model][scale][dt] = []
        data[model][scale][dt].append(t)
    return data


def plot_convergence_time(data):
    f = plt.figure()
    for scale, times in data['discrete'].items():
        xs = []
        ys = []
        for k, t in times.items():
            dt = float(k.split('=')[1])
            xs.append(dt)
            ys.append(sum(t)/len(t))

        s = plt.scatter(xs, ys, label='swarmalatorbot')

    for scale, times in data['original'].items():
        for dt, val in times.items():
            xs = np.arange(0, 5, 0.1)
            avg_t = sum(val)/len(val)
            ys = [avg_t for x in xs]
            plt.plot(xs, ys, '--', color='grey', label='baseline')
    plt.legend()
    plt.xlabel('time step $\Delta T\, [\mathrm{s}]$')
    plt.ylabel('convergence time $[\mathrm{s}]$')
    plt.show()


def main():
    convergence_results_dir = 'results/'
    all_result_dirs = sorted(os.listdir(convergence_results_dir))
    convergence_results_filenames = os.listdir(
        f'{convergence_results_dir}{all_result_dirs[-1]}'
    )
    convergence_time_data = parse_convergence_time_data(
        convergence_results_filenames
    )
    plot_convergence_time(convergence_time_data)


if __name__ == "__main__":
    main()
