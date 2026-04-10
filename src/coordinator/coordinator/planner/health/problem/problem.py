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
init_state.connected = {
    'room1': ['intersection1'],
    'room2': ['intersection1'],
    'room3': ['intersection2'],
    'room4': ['intersection2'],
    'room5': ['intersection3'],
    'room6': ['intersection3'],
    'docking_station': ['intersection4'],
    'icu': ['intersection4'],
    'intersection1': ['room1', 'room2', 'intersection2'],
    'intersection2': ['room3', 'room4', 'intersection1', 'intersection3'],
    'intersection3': ['room5', 'room6', 'intersection2', 'intersection4'],
    'intersection4': ['docking_station', 'icu', 'intersection3'],
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

init_state.resource_ready = {
    'resource1': False,
    'resource2': False,
    'resource3': False,
    'resource4': False
} # storage -> bool
init_state.at_checkpoint = {
    'resource1': None,
    'resource2': None,
    'resource3': None,
    'resource4': None
} # resource -> checkpoint

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Problem isn't implemented.")