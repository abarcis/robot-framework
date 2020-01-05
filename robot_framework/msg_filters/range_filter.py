#! /usr/bin/env python

import numpy as np

from .base_filter import BaseFilter


class RangeFilter(BaseFilter):
    def check(self, recipient_ident, sender_ident, state):
        position = self.system_state.states[recipient_ident].position
        position_diff = position - state.position
        distance = np.linalg.norm(position_diff)
        return (distance <= self.params['range'])
