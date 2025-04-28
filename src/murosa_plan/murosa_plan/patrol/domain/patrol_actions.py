#!/usr/bin/env python
"""
File Description: Hospital actions file. All the actions for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from murosa_plan.ipyhop import Actions

# ******************************************        Action Definitions      ****************************************** #
actions = Actions()

def patrol(state, robot_):
    return state

def charge(state, robot_):
    return state

actions.declare_actions([patrol, charge])

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Commands isn't implemented.")
