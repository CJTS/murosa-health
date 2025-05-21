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
            ('m_disinfect_room', uvdrobot_, nurse_)
        ]
methods.declare_task_methods('m_patrol_and_disinfect',[patrol_and_disinfect])

def approach_nurse(state, spotrobot_,nurse_):
    if state.doors[state.loc[nurse_]] :
        return [('a_navto', spotrobot_, state.loc[nurse_]), ('a_approach_nurse', spotrobot_, nurse_), ('a_authenticate_nurse', spotrobot_, nurse_)]
    else:
        return [('a_navto', spotrobot_, state.loc[nurse_]), ('a_open_door', spotrobot_, state.loc[nurse_]), ('a_approach_nurse', spotrobot_, nurse_), ('a_authenticate_nurse', spotrobot_, nurse_)]

methods.declare_task_methods('m_approach_nurse', [approach_nurse])

def patrol_room(state, spotrobot_, nurse_):
    if state.cleaned[state.loc[spotrobot_]]:
        return [('a_authorize_patrol',spotrobot_,nurse_), ('a_patrol_room', spotrobot_, state.loc[spotrobot_])]
    else:
        return[('a_clean_room', nurse_, state.loc[nurse_]), ('a_authorize_patrol',spotrobot_,nurse_), ('a_patrol_room', spotrobot_, state.loc[spotrobot_])]
    
methods.declare_task_methods('m_patrol_room', [patrol_room])


def disinfect_room(state, uvdrobot_, nurse_):
    if state.cleaned[state.loc[nurse_]]:
        return [('a_navto', uvdrobot_, state.loc[nurse_]), ('a_disinfect_room', uvdrobot_, state.loc[uvdrobot_])]
methods.declare_task_methods('m_disinfect_room', [disinfect_room])

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Methods isn't implemented.")

