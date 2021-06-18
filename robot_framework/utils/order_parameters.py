#! /usr/bin/env python

import numpy as np


def calculate_var_r(states):
    positions = np.array([s.position for s in states])
    center = np.average(positions, axis=0)
    dists_from_center = [np.linalg.norm(p - center) for p in positions]
    R = max(dists_from_center)
    normalized_dists = dists_from_center/R
    var_r = np.var(normalized_dists)
    var_r_unit_disk = 1/18
    normalized_var_r = var_r/var_r_unit_disk
    # var_r = max(normalized_dists) - min(normalized_dists)
    return normalized_var_r


def calculate_avg_r(states):
    positions = np.array([s.position for s in states])
    center = np.average(positions, axis=0)
    dists_from_center = [np.linalg.norm(p - center) for p in positions]
    R = max(dists_from_center)
    normalized_dists = dists_from_center/R
    avg_r = np.average(normalized_dists)
    avg_r_unit_disk = 2/3
    normalized_avg_r = avg_r/avg_r_unit_disk
    # var_r = max(normalized_dists) - min(normalized_dists)
    return normalized_avg_r


def calculate_avg_dist(states):
    positions = np.array([s.position for s in states])
    center = np.average(positions, axis=0)
    dists_from_center = [np.linalg.norm(p - center) for p in positions]
    dists = [min([np.linalg.norm(p1 - p2) for p1 in positions if any(p1-p2)]) for p2 in positions]
    # print(dists)
    avg_d = np.average(dists)
    R = max(dists_from_center)+0.1
    avg_d_unit_disk = 128/(45 * np.pi) * R
    normalized_avg_r = avg_d/avg_d_unit_disk
    # var_r = max(normalized_dists) - min(normalized_dists)
    return np.std(dists)  #  min(dists)/max(dists)  # normalized_avg_r


def centroid_m(m, phases):
    phases2pi = [m * p * 2 * np.pi for p in phases]
    avg = 1/m * np.mean(np.exp(1j * np.array(phases2pi)))
    return np.absolute(avg)


def potential_m(m, phases):
    return len(phases)/2 * centroid_m(m, phases) ** 2


def potential_M_N(K, M, states, phases=None):
    if phases is None:
        phases = [s.phase_level/s.phase_levels_number for s in states]
    N = len(phases)
    max_potential = N/2 * (sum([1/m**2 for m in range(1, M)]) + 0.1 * 1/M**2)
    Kms = [(m, 1) for m in range(1, M)] + [(M, -0.1)]
    potential = sum(
        [
            Km * potential_m(m, phases)
            for m, Km in Kms
        ]
    ) + N/2 * 0.1 / M ** 2
    return potential / max_potential


def get_angular_positions(states):
    positions = np.array([s.position for s in states])
    center = np.average(positions, axis=0)
    positions_respect_center = positions - center
    angular_positions = np.array(
        [np.arctan2(p[1], p[0]) for p in positions_respect_center]
    )
    return angular_positions


def angular_distance(M, states):
    angular_positions = get_angular_positions(states)
    ang_distances_diffs = []
    for i, p in enumerate(angular_positions):
        ang_distances_left = (angular_positions - p) % (2 * np.pi)
        ang_distances_left[i] = 2 * np.pi
        ang_distance_left = min(ang_distances_left)
        ang_distances_right = (- angular_positions + p) % (2 * np.pi)
        ang_distances_right[i] = 2 * np.pi
        ang_distance_right = min(ang_distances_right)
        ang_distances_diffs.append(abs(ang_distance_left - ang_distance_right))
    ang_distances_diffs = np.array(ang_distances_diffs)
    normalized_ang_distances = ang_distances_diffs/(2 * np.pi)
    distances_two_clusters = np.array(
        [np.pi] * 4 + [0] * (len(ang_distances_diffs) - 4)
    ) / (2 * np.pi)
    return np.var(normalized_ang_distances) / np.var(distances_two_clusters)


def calculate_S(states):
    angular_positions = get_angular_positions(states)
    phases = np.array([s.phase_level/s.phase_levels_number for s in states])
    phases2pi = phases * 2 * np.pi
    N = len(states)
    Splus = np.absolute(
        1/N * sum(np.exp(1j * (phases2pi + angular_positions)))
    )
    Sminus = np.absolute(
        1/N * sum(np.exp(1j * (phases2pi - angular_positions)))
    )
    return max(Splus, Sminus)
