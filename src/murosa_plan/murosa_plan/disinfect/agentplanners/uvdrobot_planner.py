
from murosa_plan.ipyhop import Actions, Methods
from murosa_plan.disinfect.agentplanners.agent_planner import AgentPlanner



def a_navto(state, robot_, loc_):
    state.loc[robot_] = loc_
    return state


def a_disinfect_room(state, uvdrobot_, room_):
    state.disinfected[room_] = True
    return state


# ======================================================================
# Methods
# ======================================================================

def m_handle_disinfect(state, uvdrobot_, spotrobot_, room_):
    if state.cleaned.get(room_, False):
        return [
            ('a_navto',               uvdrobot_, room_),
            ('a_disinfect_room',      uvdrobot_, room_),
        ]
    return False



class UVDRobotPlanner(AgentPlanner):

    def __init__(self):
        super().__init__('uvdrobot_domain')

    def _get_actions(self):
        actions = Actions()
        actions.declare_actions([a_navto, a_disinfect_room])
        return actions

    def _get_methods(self):
        methods = Methods()
        methods.declare_task_methods('m_handle_disinfect', [m_handle_disinfect])
        return methods

    def get_tasks(self, error_desc: list, agent_name: str) -> list:
        if not error_desc:
            return []

        if error_desc[0] == 'room_not_disinfected' and len(error_desc) >= 3:
            room      = error_desc[1]
            spotrobot = error_desc[2]
            return [('m_handle_disinfect', agent_name, spotrobot, room)]

        return []