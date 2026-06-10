from agents.helpers.agent_planner import AgentPlanner
from agents.planner.spot_planner.domain.methods import methods
from agents.planner.spot_planner.domain.actions import actions
from agents.planner.spot_planner.problem.problem import init_state
from agents.planner.ipyhop import IPyHOP

class SpotrobotPlanner(AgentPlanner):

    def __init__(self):
        super().__init__('spotrobot_domain')

    def _get_actions(self):
        return actions

    def _get_methods(self):
        return methods
    def _get_initial_state(self):
        return init_state
    def update_state(self, actionTuple: tuple):
        action = actionTuple[0]
        if action == 'a_navto':
            self.state.loc[actionTuple[1]] = actionTuple[2]
        elif action == 'a_open_door':
            self.state.doors[actionTuple[2]] = True

    def get_tasks(self, error_desc, agent_name, context=None):
        if error_desc[0] == 'door_closed' and len(error_desc) >= 2:
            nurse    = next((p for p in (context or []) if 'nurse' in p), None)
            uvdrobot = next((p for p in (context or []) if 'uvdrobot' in p), None)
            if not uvdrobot or not nurse:
                print('Missing nurse or uvdrobot in context — cannot replan')
                return []
            return [('m_patrol_and_disinfect', agent_name, uvdrobot, nurse)]
        return []