#! /usr/bin/env python

from robot_framework.utils.state_utils import copy_all_states

from copy import deepcopy
import json
from datetime import datetime
import numpy as np
from pyquaternion import Quaternion


class StateLog:
    def __init__(self, params):
        self.states_log = []
        self.knowledge_log = []
        self.params = params

    def save(self):
        filename = f'/home/pi/log/{datetime.now()}:J={self.params["J"]},K={self.params["K"]},M={self.params["M"]}.json'
        data = {'states': self.states_log, 'knowledge': self.knowledge_log}
        with open(filename, 'w') as f:
            json.dump(data, f, cls=MyEncoder)

    def save_and_reinit(self, params):
        # save to file
        self.save()
        # reinit
        self.reinit(params)

    def reinit(self, params):
        self.__init__(params)

    def update(self, time, system_state):
        self.states_log.append((time, copy_all_states(system_state)))
        self.knowledge_log.append(
            (time, deepcopy(system_state.knowledge.knowledge))
        )


class MyEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, datetime):
            return o.__str__()
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, Quaternion):
            return o.elements.tolist()
        return o.__dict__
