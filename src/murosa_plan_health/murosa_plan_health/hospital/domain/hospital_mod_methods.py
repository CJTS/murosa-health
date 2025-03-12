#!/usr/bin/env python
"""
File Description: Hospital methods file. All the methods for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from murosa_plan_health.ipyhop import Methods

# ******************************************        Method Definitions      ****************************************** #
methods = Methods()

def pickup_and_deliver_sample(state, robot_, nurse_, arm_):
    if state.sample[nurse_] == True:
        return [
            ('m_approach_nurse', robot_, nurse_),
            ('m_pick_sample', robot_, nurse_),
            ('m_approach_arm', robot_, arm_, nurse_),
            ('m_unload_sample', robot_, arm_)
        ]
methods.declare_task_methods('m_pickup_and_deliver_sample', [pickup_and_deliver_sample])

def approach_arm(state, robot_, arm_, nurse_):
    if state.doors[state.loc[arm_]] :
        return [('a_navto', robot_, state.loc[arm_]), ('a_approach_arm', robot_, arm_)]
    else:
        return [('a_open_door', nurse_, state.loc[arm_]), ('a_navto', robot_, state.loc[arm_]), ('a_approach_arm', robot_, arm_)]
methods.declare_task_methods('m_approach_arm', [approach_arm])

def approach_nurse(state, robot_, nurse_):
    if state.doors[state.loc[nurse_]] :
        return [('a_navto', robot_, state.loc[nurse_]), ('a_approach_nurse', robot_), ('a_authenticate_nurse', robot_, nurse_)]
    else:
        return [('a_open_door', nurse_, state.loc[nurse_]), ('a_navto', robot_, state.loc[nurse_]), ('a_approach_nurse', robot_), ('a_authenticate_nurse', robot_, nurse_)]

methods.declare_task_methods('m_approach_nurse', [approach_nurse])

def pick_sample(state, robot_, nurse_):
    if state.loc[robot_] == state.loc[nurse_] and state.sample[nurse_]:
        return [('a_open_drawer', robot_), ('a_deposit', nurse_, robot_),  ('a_close_drawer', robot_)]
methods.declare_task_methods('m_pick_sample', [pick_sample])

def unload_sample(state, robot_, arm_):
    if state.loc[robot_] == state.loc[arm_] and state.sample[robot_]:
        return [('a_open_drawer', robot_),  ('a_pick_up_sample', arm_, robot_),  ('a_close_drawer', robot_)]
methods.declare_task_methods('m_unload_sample', [unload_sample])


# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Methods isn't implemented.")
