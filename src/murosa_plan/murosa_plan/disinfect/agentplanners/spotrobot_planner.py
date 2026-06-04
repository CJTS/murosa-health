

from murosa_plan.ipyhop import Actions, Methods, State
from murosa_plan.agent_planner import AgentPlanner



def a_navto(state, robot_, loc_):
    state.loc[robot_] = loc_
    return state

def a_open_door(state, agent_, room_):
    state.doors[room_] = True
    return state

def a_detect_macanet(state, agent_, room_):
    return state
def a_approach_nurse(state, spotrobot_, nurse_):
    return state

def a_authenticate_nurse(state, spotrobot_, nurse_):
    return state

def a_authorize_patrol(state, spotrobot_, nurse_):
    return state

def a_patrol_room(state, spotrobot_, room_):
    return state
def a_authorize_disinfect(state, uvdrobot_, spotrobot_):
    return state

def m_patrol_room(state, spotrobot_, room_):
    if not state.cleaned[room_]:
        return False

    if state.doors[room_]:
        
        return [
            ('a_navto',       spotrobot_, room_),
            ('a_patrol_room', spotrobot_, room_),
        ]
    else:
        
        return [
            ('m_handle_door_closed', spotrobot_, room_),
            ('a_patrol_room',        spotrobot_, room_),
        ]

def m_handle_door_closed(state, spotrobot_, room_):
    return [
        ('a_detect_macanet',     spotrobot_, room_),
        ('a_open_door', spotrobot_, room_),
        ('a_navto',          spotrobot_, room_),
        
    ]

def m_handle_approach_nurse(state, spotrobot_, nurse_):
    
    room_ = state.loc[nurse_]   
    return [
        ('a_navto',              spotrobot_, room_),
        ('a_approach_nurse',     spotrobot_, nurse_),
        ('a_authenticate_nurse', spotrobot_, nurse_),
        ]


# ======================================================================
# SpotrobotPlanner
# ======================================================================

class SpotrobotPlanner(AgentPlanner):
 
    def __init__(self):
        super().__init__('spotrobot_domain')
 
    def _get_actions(self):
        actions = Actions()
        actions.declare_actions([
            a_navto, a_open_door, a_approach_nurse,
            a_authenticate_nurse, a_authorize_patrol, a_detect_macanet, 
            a_patrol_room, a_authorize_disinfect,
        ])
        return actions
    
 

    def _get_methods(self):
        methods = Methods()
        methods.declare_task_methods('m_patrol_room', [m_patrol_room])
        methods.declare_task_methods('m_handle_door_closed',    [m_handle_door_closed])
        methods.declare_task_methods('m_handle_approach_nurse', [m_handle_approach_nurse])
        return methods
    def update_state(self, actionTuple: tuple):
        
        action = actionTuple[0]
        if action == 'a_navto':
            self.state.loc[actionTuple[1]] = actionTuple[2]
        elif action == 'a_open_door':
            self.state.doors[actionTuple[2]] = True

    def get_tasks(self, error_desc, agent_name, context=None):
        if error_desc[0] == 'door_closed' and len(error_desc) >= 2:
            return [('m_patrol_room', agent_name, error_desc[1])]
        return []