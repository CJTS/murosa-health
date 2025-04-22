#!/usr/bin/env python
"""
File Description: Hospital methods file. All the methods for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from murosa_plan_patrol.ipyhop import Methods

# ******************************************        Method Definitions      ****************************************** #
methods = Methods()

def patrol_all(state, robot_):
    return [
        ('m_patrol', robot_)
    ]
methods.declare_task_methods('m_patrol_all', [patrol_all])

def patrol(state, robot_):
    return [('patrol', robot_)]
methods.declare_task_methods('m_patrol', [patrol])

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Methods isn't implemented.")
