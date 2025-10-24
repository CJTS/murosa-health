#!/usr/bin/env python
"""
File Description: Hospital mod problem file. Initial state and task list for the planning problem is defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.ipyhop import State
# ******************************************        Problem Definition      ****************************************** #
init_state = State('init_state')
init_state.loc = { 'patrol1': 'wp_control', 'patrol2': 'wp_control', 'patrol3': 'wp_control' }
init_state.battery = { 'patrol1': 'high', 'patrol2': 'high', 'patrol3': 'high' }
init_state.connected = {
    'wp_control': ['wp1', 'wp2', 'wp3', 'wp4', 'wp5', 'wp6', 'wp7', 'wp8', 'wp9', 'wp10', 'wp11', 'wp12'],
    'wp1': ['wp_control'],
    'wp2': ['wp_control'],
    'wp3': ['wp_control'],
    'wp4': ['wp_control'],
    'wp5': ['wp_control'],
    'wp6': ['wp_control'],
    'wp7': ['wp_control'],
    'wp8': ['wp_control'],
    'wp9': ['wp_control'],
    'wp10': ['wp_control'],
    'wp11': ['wp_control'],
    'wp12': ['wp_control']
}
init_state.patrolled = { 'wp1': False, 'wp2': False, 'wp3': False, 'wp4': False, 'wp5': False, 'wp6': False, 'wp7': False, 'wp8': False, 'wp9': False, 'wp10': False, 'wp11': False, 'wp12': False }

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Problem isn't implemented.")
