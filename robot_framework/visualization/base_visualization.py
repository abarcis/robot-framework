#! /usr/bin/env python


class BaseVisualization:
    def update(self, states, states_log=None):
        raise NotImplementedError()

    def reinit(self, params=None):
        pass
