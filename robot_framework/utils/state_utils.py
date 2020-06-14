#! /usr/bin/env python


from robot_framework.state import State


def copy_all_states(system_state):
    states = {}
    for ident, state in system_state.states.items():
        states[ident] = State(state=state)
    return states
