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

def deliver_resource_task(state, task_):
    robot_ = state.assigned[task_]
    resource_, _ = state.requested[task_]
    storage_ = state.resource_at.get(resource_)

    if storage_:
        return [
            ('m_collect_resource', robot_, resource_, storage_, task_),
            ('m_execute_delivery', robot_, task_)
        ]
methods.declare_task_methods('m_deliver_resource_task', [deliver_resource_task])

def collect_resource_normal(state, robot_, resource_, storage_, task_):
    if state.low_battery[robot_]:
        return False

    loc_ = state.storage_loc[storage_]

    return [
        ('a_navto', robot_, loc_),
        ('a_request_resource', robot_, storage_, resource_),
        ('a_pick_resource', robot_, storage_, resource_)
    ]

def collect_resource_low_battery(state, robot_, resource_, storage_, task_):
    if not state.low_battery[robot_]:
        return False

    # naive reassignment (you can improve selection)
    for r in state.low_battery:
        if not state.low_battery[r] and r != robot_:
            return [
                ('a_reassign_task', task_, r)
            ]

methods.declare_task_methods(
    'm_collect_resource',
    [collect_resource_low_battery, collect_resource_normal]
)

def deliver_normal(state, robot_, task_):
    if state.low_battery[robot_]:
        return False

    _, loc_ = state.requested[task_]

    return [
        ('a_navto', robot_, loc_),
        ('a_deliver_resource', robot_, task_)
    ]

def deliver_low_battery(state, robot_, task_):
    if not state.low_battery[robot_]:
        return False

    resource_, _ = state.requested[task_]

    checkpoint_ = list(state.checkpoint_loc.keys())[0]
    loc_ = state.checkpoint_loc[checkpoint_]

    return [
        ('a_navto', robot_, loc_),
        ('a_drop_checkpoint', robot_, checkpoint_),
        ('m_recover_from_checkpoint', resource_, task_)
    ]

methods.declare_task_methods(
    'm_execute_delivery',
    [deliver_low_battery, deliver_normal]
)

def recover_from_checkpoint(state, resource_, task_):
    checkpoint_ = state.at_checkpoint.get(resource_)

    if not checkpoint_:
        return False

    loc_ = state.checkpoint_loc[checkpoint_]

    # find available robot
    for r in state.low_battery:
        if not state.low_battery[r]:
            return [
                ('a_navto', r, loc_),
                ('a_pick_checkpoint', r, resource_, checkpoint_),
                ('m_execute_delivery', r, task_)
            ]

methods.declare_task_methods(
    'm_recover_from_checkpoint',
    [recover_from_checkpoint]
)

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Methods isn't implemented.")
