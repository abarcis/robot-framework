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


def get_files_from_directory(directory):
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
    return result


def interactively_process_directory(directory):
    files = get_files_from_directory(directory)
    response = ".*"
    while response != "":
        files = {
            h: sorted([r for r in fs
                       if re.search(response, r)])
            for h, fs in files.items()
        }
        pprint(files)
        print("Stats:")
        pprint({h: len(fs) for h, fs in files.items()})
        print("Type in a regexp to filter the filess or press [enter].")
        response = input()
    process_files(
        sum([[os.path.join(directory, h, 'log', f) for f in fs]
             for h, fs in files.items()], [])
    )


def generate_preview(directory, output_directory):
    files = get_files_from_directory(directory)
    all_filenames = itertools.chain(*files.values())
    all_dates = set(n[:19] for n in all_filenames)
    dates_in_2018 = [n for n in all_dates
                     if n.startswith("2018")]

    for d in dates_in_2018:
        filepaths = itertools.chain(*[
            [
                os.path.join(directory, h, 'log', f) for f in fs
                if f.startswith(d)
            ]
            for h, fs in files.items()
        ])
        process_files(
            filepaths,
            os.path.join(output_directory, "{}.png".format(d))
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


def process_files(filepaths, output_file=None):
    datas = [parse_file(f) for f in filepaths]
    datas = [d for d in datas
             if d != {}]

    if not all(set(datas[0].keys()) == set(d.keys()) for d in datas):
        import ipdb
        ipdb.set_trace()
    start = 0
    end = -1

    if datas == []:
        print("WARN: no data given")
        return

    def merge_data(datas):
        if True:  # sort by date
            all_sorted = sorted(itertools.chain(*datas))
            for e in all_sorted:
                yield e[1]

        else:
            da = list(datas)
            longest_length = max([len(a) for a in da])
            for i in range(longest_length):
                for d in da:
                    idx = len(d) - longest_length + i
                    if idx >= 0:
                        yield d[idx][1]
    data = {
        key: list(merge_data(
            tuple(zip(d.get('timestamp', []), d.get(key, []))) for d in datas)
        )
        for key in datas[0].keys()
    }
    phases = data['phase']

    x = data['pos_x']
    y = data['pos_y']

    while True:
        plt.clf()
        plt.cla()
        plt.close()
        plt.figure(num=None, figsize=(12, 12), dpi=100, facecolor='w',
                   edgecolor='k')
        plt.axis('off')
        ax = plt.gca()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-0.9, 1.1)

        hues = [phase/(2*np.pi) for phase in phases[start:end]]
        colors = [
            colorsys.hsv_to_rgb(
                hue,
                i/len(hues),
                1
            )
            for i, hue in enumerate(hues)
        ]
        plt.scatter(x[start:end], y[start:end], c=colors)
        if output_file is not None:
            plt.savefig(output_file, bbox_inches='tight')
            return
        plt.savefig("output.png")
        plt.show()
        limits = input("Change limits (format: start:stop)")
        start, end = map(int, limits.split(":"))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("First argument should be a filepath")
        sys.exit(1)
    if os.path.isdir(sys.argv[1]):
        if len(sys.argv) > 2:
            generate_preview(sys.argv[1], sys.argv[2])
        else:
            interactively_process_directory(sys.argv[1])
    else:
        process_files(sys.argv[1:])
