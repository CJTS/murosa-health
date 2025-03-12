#!/usr/bin/env python
"""
File Description: Hospital mod problem file. Initial state and task list for the planning problem is defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from murosa_plan_health.ipyhop import State, MultiGoal

# ******************************************        Problem Definition      ****************************************** #
init_state = State('init_state')
init_state.loc = { 'nurse1': 'Room1', 'robot1': 'Room2', 'arm1': 'Room3' }
init_state.doors = { 'Room1': True, 'Room2': True, 'Room3': True }
init_state.sample = { 'nurse1': True, 'robot1': False, 'arm1': False }

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Problem isn't implemented.")
