#! /usr/bin/env python

import os
import matplotlib.pyplot as plt
import matplotlib
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
    f, ax1 = plt.subplots()
    for scale, times in data['discrete'].items():
        xs = []
        ys = []
        msg_num = []
        for k, t in times.items():
            dt = float(k.split('=')[1])
            xs.append(dt)
            ys.append(sum(t)/len(t))
            msg_num.append(ys[-1] / xs[-1])

        ax1.scatter(xs, ys, label='convergence time', color='blue')
        ax2 = ax1.twinx()
        ax2.scatter(xs, msg_num, label='number of messages', color='red', marker='x')
    f.tight_layout()

    for scale, times in data['original'].items():
        for dt, val in times.items():
            xs = np.arange(0, 5.2, 0.1)
            avg_t = sum(val)/len(val)
            ys = [avg_t for x in xs]
            ax1.plot(xs, ys, '--', color='grey', label='conv. time baseline')

    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines + lines2, labels + labels2, loc='upper center')
    ax1.set_xlabel('period length $T\, [\mathrm{s}]$')
    ax1.set_ylabel('convergence time $[\mathrm{s}]$')
    ax2.set_ylabel('number of messages')
    ax1.set_xlim([-0.01, 5.1])
    ax1.set_ylim([-0.01, 2250])
    ax2.set_ylim([-0.01, 900])
    l = ax1.get_ylim()
    l2 = ax2.get_ylim()
    fun = lambda x : l2[0]+(x-l[0])/(l[1]-l[0])*(l2[1]-l2[0])
    ticks = fun(ax1.get_yticks())
    ax2.yaxis.set_major_locator(matplotlib.ticker.FixedLocator(ticks))
    f.savefig(f'/home/agniewek/repos/papers/2020-journal/data/convergence_time.pdf', transparent=True, bbox_inches="tight")
    plt.show()


def main():
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.alpha'] = 0.5
    plt.rcParams['legend.fancybox'] = True
    #print(plt.rcParams.find_all('axes'))

    plt.rcParams['xtick.bottom'] = False
    plt.rcParams['ytick.left'] = False

    plt.rcParams['figure.frameon'] = True
    plt.rcParams['axes.edgecolor'] = '#b3b3b3'
    convergence_results_dir = 'results/convergence/'
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
