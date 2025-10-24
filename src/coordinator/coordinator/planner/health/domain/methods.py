#!/usr/bin/env python
"""
File Description: Hospital methods file. All the methods for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.ipyhop import Methods

# ******************************************        Method Definitions      ****************************************** #
methods = Methods()

def pickup_and_deliver_sample(state, nurse_, room_, robot_, arm_):
    if state.sample[room_] == True:
        return [
            ('m_collect_sample', nurse_, room_),
            ('m_approach_nurse', robot_, nurse_),
            ('m_pick_sample', robot_, nurse_),
            ('m_approach_arm', robot_, arm_, nurse_),
            ('m_unload_sample', robot_, arm_)
        ]
methods.declare_task_methods('m_pickup_and_deliver_sample', [pickup_and_deliver_sample])

def collect_sample(state, nurse_, room_):
    return [('a_navto', nurse_, room_), ('a_collect_sample', nurse_, room_)]
methods.declare_task_methods('m_collect_sample', [collect_sample])

def approach_nurse(state, robot_, nurse_):
    if state.doors[state.loc[nurse_]]:
        return [('a_navto', robot_, state.loc[nurse_]), ('a_approach_nurse', robot_, nurse_), ('a_authenticate_nurse', robot_, nurse_)]
    else:
        return [('a_open_door', nurse_, state.loc[nurse_]), ('a_navto', robot_, state.loc[nurse_]), ('a_approach_nurse', robot_, nurse_), ('a_authenticate_nurse', robot_, nurse_)]

methods.declare_task_methods('m_approach_nurse', [approach_nurse])

def pick_sample(state, robot_, nurse_):
    if state.loc[robot_] == state.loc[nurse_] and state.sample[nurse_]:
        return [('a_open_drawer', robot_), ('a_deposit', nurse_, robot_),  ('a_close_drawer', robot_)]
methods.declare_task_methods('m_pick_sample', [pick_sample])

def approach_arm(state, robot_, arm_, nurse_):
    if state.doors[state.loc[arm_]] :
        return [('a_navto', robot_, state.loc[arm_]), ('a_approach_arm', robot_, arm_)]
    else:
        return [('a_open_door', nurse_, state.loc[arm_]), ('a_navto', robot_, state.loc[arm_]), ('a_approach_arm', robot_, arm_)]
methods.declare_task_methods('m_approach_arm', [approach_arm])

def unload_sample(state, robot_, arm_):
    if state.loc[robot_] == state.loc[arm_] and state.sample[robot_]:
        return [('a_open_drawer', robot_),  ('a_pick_up_sample', arm_, robot_),  ('a_close_drawer', robot_)]
methods.declare_task_methods('m_unload_sample', [unload_sample])

def patrol_and_disinfect(state, spotrobot_,uvdrobot_, nurse_):
    if state.disinfected[state.loc[nurse_]] == False:
        return [
            ('m_approach_nurse', spotrobot_, nurse_),
            ('m_patrol_room', spotrobot_, nurse_),
            ('m_disinfect_room', uvdrobot_, spotrobot_,nurse_)
        ]
methods.declare_task_methods('m_patrol_and_disinfect',[patrol_and_disinfect])

# def approach_nurse(state, spotrobot_,nurse_):
#     actions = []

#     if state.low_battery[spotrobot_]:
#         actions.append(('a_charge', spotrobot_))

#     actions = actions + [('a_navto', spotrobot_, state.loc[nurse_])]

#     if not state.doors[state.loc[nurse_]]:
#         actions = actions + [('a_open_door', spotrobot_, state.loc[nurse_])]

#     actions = actions + [
#         ('a_approach_nurse', spotrobot_, nurse_),
#         ('a_authenticate_nurse', spotrobot_, nurse_)
#     ]

#     return actions
# methods.declare_task_methods('m_approach_nurse', [approach_nurse])

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
