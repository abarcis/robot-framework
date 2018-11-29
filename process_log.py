import colorsys
import csv
import sys
import os

import numpy as np
import matplotlib.pyplot as plt
from pprint import pprint
import re
import itertools


# Available keys:
# ['id', 'timestamp', 'pos_x', 'pos_y', 'pos_z', 'phase', 'orientation_x',
# 'orientation_y', 'orientation_z', 'orientation_w', 'angle_xy'])


def process_directory(directory):
    hostnames = [f for f in os.listdir(directory)
                 if os.path.isdir(os.path.join(directory, f))]

    result = {}
    for h in hostnames:
        log_file_path = os.path.join(directory, h, 'log')
        if not os.path.isdir(log_file_path):
            print(("WARN: folder '{}' does not have a 'log' directory in it"
                  ", skipping").format(h))
            continue
        result[h] = [f for f in os.listdir(log_file_path)
                     if os.path.isfile(os.path.join(log_file_path, f))]
    response = ".*"
    while response != "":
        result = {
            h: sorted([r for r in fs
                       if re.search(response, r)])
            for h, fs in result.items()
        }
        pprint(result)
        print("Stats:")
        pprint({h: len(fs) for h, fs in result.items()})
        print("Type in a regexp to filter the results or press [enter].")
        response = input()
    process_files(
        sum([[os.path.join(directory, h, 'log', f) for f in fs]
             for h, fs in result.items()], [])
    )


def parse_file(filepath):
    print("Starting processing file \n{}".format(filepath))
    with open(filepath, newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        csv_data = []
        try:
            for l in reader:
                csv_data.append(l)
        except csv.Error:
            print("WARN: corrupted file")

        header = csv_data[0]
        data = zip(*csv_data[1:])
        data_parsed = []
        for column in data:
            try:
                data_parsed.append([float(e) for e in column])
            except ValueError:
                data_parsed.append(list(column))
        # ALl columns should have the same length:
        assert all([len(data_parsed[0]) == len(d) for d in data_parsed])

        together = dict(zip(header, data_parsed))
    if len(data_parsed) == 0:
        print("WARN: No data in file")
    print("Read {} records from CSV file.".format(len(together.get('id', []))))
    return together


def process_files(filepaths):
    datas = [parse_file(f) for f in filepaths]

    assert all(set(datas[0].keys()) == set(d.keys()) or d == {} for d in datas)
    start = 0
    end = -1

    data = {
        key: list(itertools.chain(
            *zip(*[d.get(key, [])[start:end] for d in datas])
        ))
        for key in datas[0].keys()
    }
    start = 0

    val = 1

    phases = data['phase']

    colors_rgb = [
        colorsys.hsv_to_rgb(
            phase/(2*np.pi),
            i/len(phases),
            val
        )
        for i, phase in enumerate(phases)
    ]
    # colors_rgba = [c + (i/len(colors_rgb), )
    #                for i, c in enumerate(colors_rgb)]
    colors = colors_rgb

    x = data['pos_x']
    y = data['pos_y']

    while True:
        plt.scatter(x[start:end], y[start:end], c=colors[start:end])
        plt.show()
        plt.clf()
        plt.cla()
        plt.close()
        limits = input("Change limits (format: start:stop)")
        start, end = map(int, limits.split(":"))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("First argument should be a filepath")
        sys.exit(1)
    if os.path.isdir(sys.argv[1]):
        process_directory(sys.argv[1])
    else:
        process_files(sys.argv[1:])
