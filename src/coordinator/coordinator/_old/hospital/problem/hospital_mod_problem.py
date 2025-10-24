#!/usr/bin/env python
"""
File Description: Hospital mod problem file. Initial state and task list for the planning problem is defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.ipyhop import State

# ******************************************        Problem Definition      ****************************************** #
init_state = State('init_state')
init_state.loc = { 'nurse1': 'room1','nurse2': 'room3', 'robot1': 'room2', 'robot2': 'room2', 'arm1': 'room4', 'arm2': 'room4' }
init_state.doors = { 'room1': False, 'room2': True, 'room3': True, 'room4': True }
init_state.sample = { 'nurse1': True, 'nurse2': True, 'robot1': False, 'robot2': False, 'arm1': False, 'arm2': False }

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Problem isn't implemented.")
