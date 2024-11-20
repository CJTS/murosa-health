#!/usr/bin/env python
"""
File Description: Hospital mod problem file. Initial state and task list for the planning problem is defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from murosa_plan_patrol.ipyhop import State
# ******************************************        Problem Definition      ****************************************** #
init_state = State('init_state')
init_state.loc = { 'r2d2': 'wp_control' }
init_state.connected = {
    'wp_control': ['wp1', 'wp2', 'wp3', 'wp4'],
    'wp1': ['wp_control'],
    'wp2': ['wp_control'],
    'wp3': ['wp_control'],
    'wp4': ['wp_control']
}
init_state.patroled = { 'wp1': False, 'wp2': False, 'wp3': False, 'wp4': False }

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Problem isn't implemented.")
