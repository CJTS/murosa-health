#!/usr/bin/env python
"""
File Description: Hospital methods file. All the methods for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from agents.planner.spot_planner.domain.common_method import methods

# ******************************************        Method Definitions      ****************************************** #
def patrol(state, spotrobot_,uvdrobot_, nurse_, nurse_room_):
    if state.disinfected[state.loc[nurse_]] == False:
        return [
            ('m_approach_nurse', spotrobot_, nurse_),
            ('m_patrol_room', spotrobot_, nurse_),
            ('a_authorize_disinfect', uvdrobot_, spotrobot_)
        ]
    return False
methods.declare_task_methods('m_patrol_and_disinfect',[patrol])

def approach_nurse(state, robot_, nurse_):
    if state.doors[state.loc[nurse_]]:
        return [('a_navto', robot_, state.loc[nurse_]), ('a_approach_nurse', robot_, nurse_), ('a_authenticate_nurse', robot_, nurse_)]
    else:
        return [('m_handle_door_closed', robot_, state.loc[nurse_]), ('a_navto', robot_, state.loc[nurse_]), ('a_approach_nurse', robot_, nurse_), ('a_authenticate_nurse', robot_, nurse_)]
methods.declare_task_methods('m_approach_nurse', [approach_nurse])

def m_handle_door_closed(state, spotrobot_, room_):
    return [
        ('a_detect_macanet',     spotrobot_, room_),
        ('a_open_door', spotrobot_, room_),
        
        
    ]
methods.declare_task_methods('m_handle_door_closed', [m_handle_door_closed])

def patrol_room(state, spotrobot_, nurse_):
    actions = []

    if not state.cleaned[state.loc[nurse_]]:
        return False

    return [
        ('a_authorize_patrol',spotrobot_,nurse_),
        ('a_patrol_room', spotrobot_, state.loc[nurse_])
    ]

    
methods.declare_task_methods('m_patrol_room', [patrol_room])





# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Methods isn't implemented.")
