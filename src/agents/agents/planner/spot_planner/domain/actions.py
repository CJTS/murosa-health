#!/usr/bin/env python
"""
File Description: Hospital actions file. All the actions for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from agents.planner.ipyhop import Actions

# ******************************************        Action Definitions      ****************************************** #
actions = Actions()

# robot navigate to a location
def a_navto(state, agent_, loc_):
    state.loc[agent_] = loc_
    return state

# robot opens to a location
def a_open_door(state, nurse_, room_):
    state.doors[room_] = True
    return state

# robot aproaches a nurse
def a_approach_nurse(state, robot_, nurse_):
    return state

# robot authenticate the nurse
def a_authenticate_nurse(state, robot_, _nurse):
    return state
   
# robot navigate to a location
def a_navto(state, robot_, loc_):
    state.loc[robot_] = loc_
    return state

def a_detect_macanet(state, spotrobot_, room_):
    return state

def a_authorize_patrol(state, spotrobot_, nurse_):
    return state

def a_patrol_room(state, spotrobot_, room_):
    return state

def a_authorize_disinfect(state, uvdrobot_, spotrobot_):
    return state


actions.declare_actions([
    a_navto,
    a_open_door,
    a_approach_nurse,
    a_authenticate_nurse,
    a_detect_macanet,
    a_authorize_patrol,
    a_patrol_room,
    a_authorize_disinfect,
    
])

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Commands isn't implemented.")
