#!/usr/bin/env python
"""
File Description: Hospital mod problem file. Initial state and task list for the planning problem is defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.ipyhop import State

# ******************************************        Problem Definition      ****************************************** #
init_state = State('init_state')
init_state.loc = {
    'nurse1': 'nr',
    'nurse2': 'nr',
    'nurse3': 'nr',
    'nurse4': 'nr',
    'uvdrobot1': 'ds',
    'spotrobot1': 'ds',
    'uvdrobot2': 'ds',
    'spotrobot2': 'ds',
    'collector1': 'ds',
    'collector2': 'ds',
    'arm1': 'ds'
}
init_state.doors = {
    'room1': True,
    'room2': True,
    'room3': True,
    'room4': True,
    'room5': True,
    'room6': True,
    'lab': True,
    'icu': True
},
init_state.sample = {
    'room1': False,
    'room2': False,
    'room3': False,
    'room4': False,
    'room5': False,
    'room6': False,
    'nurse1': False,
    'nurse2': False,
    'nurse3': False,
    'nurse4': False,
    'uvdrobot1': False,
    'spotrobot1': False,
    'uvdrobot2': False,
    'spotrobot2': False,
    'collector1': False,
    'collector2': False,
    'arm1': False
}
init_state.cleaned = {
    'room1': True,
    'room2': True,
    'room3': True,
    'room4': True,
    'room5': True,
    'room6': True,
    'icu': True
}
init_state.disinfected = {
    'room1': True,
    'room2': True,
    'room3': True,
    'room4': True,
    'room5': True,
    'room6': True,
    'icu': True
}
init_state.low_battery = {
    'uvdrobot1': False,
    'spotrobot1': False,
    'uvdrobot2': False,
    'spotrobot2': False,
    'collector1': False,
    'collector2': False,
    'arm1': False
}
init_state.resource_at = {
    'resource1': 'stor1',
    'resource2': 'stor2',
    'resource3': 'stor3',
    'resource4': 'stor4'
} # resource -> storage
init_state.carrying = {
    'collector1': None,
    'collector2': None,
    'collector3': None
} # robot -> resource or None
init_state.requested = { } # task -> (resource, location)

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Problem isn't implemented.")