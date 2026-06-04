
from murosa_plan.ipyhop import Actions, Methods
from murosa_plan.disinfect.agent_planner import AgentPlanner

def a_clean_room(state, nurse_, room_):
    state.cleaned[room_] = True
    return state


def m_handle_dirty_room(state, nurse_, room_):
    if not state.cleaned.get(room_, True):
        return [('a_clean_room', nurse_, room_)]
    return []  



class NursePlanner(AgentPlanner):

    def __init__(self):
        super().__init__('nurse_domain')

    def _get_actions(self):
        actions = Actions()
        actions.declare_actions([a_clean_room])
        return actions

    def _get_methods(self):
        methods = Methods()
        methods.declare_task_methods('m_handle_dirty_room', [m_handle_dirty_room])
        return methods

    def get_tasks(self, error_desc: list, agent_name: str) -> list:
        if not error_desc:
            return []

        if error_desc[0] == 'dirty_room' and len(error_desc) >= 2:
            return [('m_handle_dirty_room', agent_name, error_desc[1])]

        return []