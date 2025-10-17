#!/usr/bin/env python
"""
File Description: Hospital mod problem file. Initial state and task list for the planning problem is defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from murosa_plan.ipyhop import State

# ******************************************        Problem Definition      ****************************************** #
init_state = State('init_state')
init_state.loc = { 
    'nurse1': 'room1',
    'nurse2': 'room2',
    'nurse3': 'room3',
    'uvdrobot': 'room4',
    'spotrobot': 'room4'
}
init_state.doors = { 
    'room1': True,
    'room2': True,
    'room3': True,
    'room4': True,
    'room5': True,
    'room6': True,
    'icu': True
}
init_state.layout = { 
    'room1': 'intersection1', 'intersection1': 'room1',
    'room2': 'intersection1', 'intersection1': 'room2',
    'intersection1': 'intersection2', 'intersection2': 'intersection1',
    'room3': 'intersection2', 'intersection2': 'room4',
    'room4': 'intersection2', 'intersection2': 'room3',
    'intersection2': 'intersection3', 'intersection3': 'intersection2',
    'room5': 'intersection3', 'intersection3': 'room6',
    'room6': 'intersection3', 'intersection3': 'room5',
    'intersection3': 'intersection4', 'intersection4': 'intersection3',
    'icu': 'intersection4', 'intersection4': 'icu',
    'docking_station': 'intersection4', 'intersection4': 'docking_station',
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
    'room1': False,
    'room2': False,
    'room3': False,
    'room4': False,
    'room5': False,
    'room6': False,
    'icu': False
}

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Problem isn't implemented.")