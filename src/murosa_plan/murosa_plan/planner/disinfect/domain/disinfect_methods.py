#!/usr/bin/env python
"""
File Description: Hospital methods file. All the methods for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from murosa_plan.ipyhop import Methods

# ******************************************        Method Definitions      ****************************************** #
methods = Methods()

def patrol_and_disinfect(state, spotrobot_,uvdrobot_, nurse_):
    if state.disinfected[state.loc[nurse_]] == False:
        return [
            ('m_approach_nurse', spotrobot_, nurse_),
            ('m_patrol_room', spotrobot_, nurse_),
            ('m_disinfect_room', uvdrobot_, spotrobot_,nurse_)
        ]
methods.declare_task_methods('m_patrol_and_disinfect',[patrol_and_disinfect])

def approach_nurse(state, spotrobot_,nurse_):
    actions = []

    if state.low_battery[spotrobot_]:
        actions.append(('a_charge', spotrobot_))

    actions = actions + [('a_navto', spotrobot_, state.loc[nurse_])]

    if not state.doors[state.loc[nurse_]]:
        actions = actions + [('a_open_door', spotrobot_, state.loc[nurse_])]

    actions = actions + [
        ('a_approach_nurse', spotrobot_, nurse_),
        ('a_authenticate_nurse', spotrobot_, nurse_)
    ]

    return actions
methods.declare_task_methods('m_approach_nurse', [approach_nurse])

def patrol_room(state, spotrobot_, nurse_): 
    actions = []

    if not state.cleaned[state.loc[spotrobot_]]:
        actions = actions + [('a_clean_room', nurse_, state.loc[nurse_])]

    actions = actions + [
        ('a_authorize_patrol',spotrobot_,nurse_),
        ('a_patrol_room', spotrobot_, state.loc[spotrobot_])
    ]

    return actions
methods.declare_task_methods('m_patrol_room', [patrol_room])


def disinfect_room(state, uvdrobot_, spotrobot_, nurse_):
    actions = []

    if not state.cleaned[state.loc[nurse_]]:
        return actions  # cannot disinfect if not cleaned
    
    if state.low_battery[uvdrobot_]:
        actions.append(('a_charge', uvdrobot_))

    actions = actions + [
        ('a_authorize_disinfect',uvdrobot_,spotrobot_),
        ('a_navto', uvdrobot_, state.loc[nurse_]),
        ('a_disinfect_room', uvdrobot_, state.loc[nurse_])
    ]

    return actions
methods.declare_task_methods('m_disinfect_room', [disinfect_room])

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Methods isn't implemented.")

