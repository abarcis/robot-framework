#! /usr/bin/env python


class BaseVisualization:
    def update(self, states):
        raise NotImplementedError()
