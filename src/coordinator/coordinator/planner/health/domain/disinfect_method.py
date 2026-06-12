#!/usr/bin/env python
"""
File Description: Hospital methods file. All the methods for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.health.domain.common_method import methods

# ******************************************        Method Definitions      ****************************************** #
def patrol_and_disinfect(state, nurse_, nurse_room_, spotrobot_, uvdrobot_):
    if state.disinfected[nurse_room_] == False:
        return [
            ('m_approach_nurse', spotrobot_, nurse_),
            ('m_patrol_room', spotrobot_, nurse_, nurse_room_),
            ('m_disinfect_room', uvdrobot_, spotrobot_,nurse_, nurse_room_)
        ]
methods.declare_task_methods('m_patrol_and_disinfect',[patrol_and_disinfect])

def patrol_room(state, spotrobot_, nurse_, nurse_room_):
    if not state.cleaned[nurse_room_]:
        return [
            ('a_clean_room', nurse_, nurse_room_),
            ('a_authorize_patrol', spotrobot_, nurse_),
            ('a_patrol_room', spotrobot_, nurse_room_)
        ]
    return [
        ('a_authorize_patrol', spotrobot_, nurse_),
        ('a_patrol_room', spotrobot_, nurse_room_)
    ]
methods.declare_task_methods('m_patrol_room', [patrol_room])

def disinfect_room(state, uvdrobot_, spotrobot_, nurse_, nurse_room_):
    if not state.cleaned[nurse_room_]:
        return False
    actions = []
    if state.low_battery[uvdrobot_]:
        actions.append(('a_charge', uvdrobot_))
    actions += [
        ('a_authorize_disinfect', uvdrobot_, spotrobot_),
        ('a_navto', uvdrobot_, nurse_room_),
        ('a_disinfect_room', uvdrobot_, nurse_room_)
    ]
    return actions
methods.declare_task_methods('m_disinfect_room', [disinfect_room])

def m_handle_door_closed(state, spotrobot_, room_):
    return [
        ('a_detect_macanet',     spotrobot_, room_),
        ('a_open_door', spotrobot_, room_),
        
        
    ]
methods.declare_task_methods('m_handle_door_closed', [m_handle_door_closed])



# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Methods isn't implemented.")
