#!/usr/bin/env python
"""
File Description: Hospital mod problem file. Initial state and task list for the planning problem is defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from murosa_plan.ipyhop import State

# ******************************************        Problem Definition      ****************************************** #
init_state = State('init_state')
init_state.loc = { 
    'nurse_disinfected1': 'room1',
    'nurse_disinfected2': 'room2', 
    'nurse_disinfected3': 'room3',
    'nurse_disinfected4': 'room4',
    'uvdrobot1': 'room4', 
    'spotrobot1': 'room4',
    'uvdrobot2': 'room4', 
    'spotrobot2': 'room4'
}
init_state.doors = { 
    'room1': True, 
    'room2': True, 
    'room3': True, 
    'room4': True, 
    'icu': True
},
init_state.cleaned = { 
    'room1': True,
    'room2': True,
    'room3': True,
    'room4': True,
    'icu': True
}
init_state.disinfected = {
    'room1': True,
    'room2': True,
    'room3': True,
    'room4': True,
    'icu': True
}
init_state.low_battery = {
    'uvdrobot1': False,
    'spotrobot1': False,
    'uvdrobot2': False,
    'spotrobot2': False
}

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Problem isn't implemented.")