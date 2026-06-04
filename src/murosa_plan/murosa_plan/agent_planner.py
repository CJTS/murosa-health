import copy
from murosa_plan.ipyhop import IPyHOP
from murosa_plan.disinfect.problem.disinfect_problem import init_state

class AgentPlanner:
    def __init__(self, domain_name: str):
        self.domain_name = domain_name
        self.state = copy.deepcopy(init_state)
        self.actions = self._get_actions()
        self.methods = self._get_methods()
        # Mesma API do coordenador
        self.planner = IPyHOP(self.methods, self.actions)

    def _get_actions(self):
        raise NotImplementedError

    def _get_methods(self):
        raise NotImplementedError

    def get_tasks(self, error_desc: list, agent_name: str) -> list:
        raise NotImplementedError

    def update_state(self, actionTuple: tuple):
        pass

    def get_tasks(self, error_desc: list, agent_name: str, context: list = None) -> list:
        raise NotImplementedError

    def plan(self, error_desc: list, agent_name: str, context: list = None):
        tasks = self.get_tasks(error_desc, agent_name, context=context)
        if not tasks:
            return None
        state_copy = copy.deepcopy(self.state)
        try:
            result = self.planner.plan(state_copy, tasks, verbose=1)
        except Exception as e:
            result = None
        return result if result else None