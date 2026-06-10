import copy
from agents.planner.ipyhop import IPyHOP
from agents.planner.spot_planner.problem.problem import init_state


class AgentPlanner:
    def __init__(self, domain_name: str):
        self.domain_name = domain_name
        self.state = copy.deepcopy(self._get_initial_state())
        self.actions = self._get_actions()
        self.methods = self._get_methods()

        self.planner = IPyHOP(self.methods, self.actions)

    def _get_initial_state(self):
        raise NotImplementedError

    def _get_actions(self):
        raise NotImplementedError

    def _get_methods(self):
        raise NotImplementedError

    def get_tasks(self, error_desc: list, agent_name: str, context=None) -> list:
        raise NotImplementedError

    def update_state(self, actionTuple: tuple):
        pass

    def plan(self, error_desc: list, agent_name: str, context=None):
        tasks = self.get_tasks(error_desc, agent_name, context=context)
        if not tasks:
            return None

        state_copy = copy.deepcopy(self.state)

        try:
            result = self.planner.plan(state_copy, tasks, verbose=1)
        except Exception as e:
            result = None

        return result if result else None