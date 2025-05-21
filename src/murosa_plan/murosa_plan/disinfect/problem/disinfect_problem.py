#!/usr/bin/env python
"""
File Description: Hospital mod problem file. Initial state and task list for the planning problem is defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from murosa_plan.ipyhop import State

# ******************************************        Problem Definition      ****************************************** #
init_state = State('init_state')
init_state.loc = { 'nurse1': 'room1','nurse2': 'room2', 'nurse3': 'room3','uvdrobot': 'room4', 'spotrobot': 'room4'}
init_state.doors = { 'room1': True, 'room2': True, 'room3': True, 'room4': True }
init_state.cleaned = { 'room1' : True,'room3' : True, 'room3' : True }
init_state.disinfected = {'room1': False,'room2': False, 'room3': False}

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Problem isn't implemented.")