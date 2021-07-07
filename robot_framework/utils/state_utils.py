#! /usr/bin/env python

import numpy as np
from robot_framework.state import State


def copy_all_states(system_state):
    states = {}
    for ident, state in system_state.states.items():
        states[ident] = State(state=state)
    return states


def gps_coord_to_dist(lat1, lon1, lat2, lon2):
    lat1_rad = lat1 * np.pi / 180
    lat2_rad = lat2 * np.pi / 180
    lon1_rad = lon1 * np.pi / 180
    lon2_rad = lon2 * np.pi / 180
    R = 6378137  # Radius of earth in m
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    a = (
        np.sin(dlat/2)**2 +
        np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon/2)**2
    )
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    d = R * c
    return d


def lat_dist(lat1, lat2):
    dist = gps_coord_to_dist(lat1, 0, lat2, 0)
    if lat1 < lat2:
        return dist
    else:
        return -dist


def lon_dist(lon1, lon2, lat):
    dist = gps_coord_to_dist(lat, lon1, lat, lon2)
    if lon1 < lon2:
        return dist
    else:
        return -dist


def convert_position_to_local(own_position, other_position):
    x = lat_dist(own_position[0], other_position[0])
    y = lon_dist(own_position[1], other_position[1], own_position[0])
    relative_position = np.array([x, y, 0])
    return relative_position


def convert_states_to_local(own_state, other_states):
    own_relative = State(state=own_state, position=np.zeros(3))
    other_relative = []
    for ident, state in other_states:
        relative_position = convert_position_to_local(own_state.position, state.position)
        other_relative.append(
            (ident, State(state=state, position=relative_position))
        )

    # print('converted states', own_relative, other_relative)
    return(own_relative, other_relative)
