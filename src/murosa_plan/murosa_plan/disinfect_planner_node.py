import json
import rclpy
from rclpy.node import Node
from interfaces.srv import Action
from murosa_plan.disinfect.domain.disinfect_methods import methods
from murosa_plan.disinfect.domain.disinfect_actions import actions
from murosa_plan.disinfect.problem.disinfect_problem import init_state
from murosa_plan.ipyhop import IPyHOP
from std_msgs.msg import Bool
import networkx as nx
class Planner(Node):
    def __init__(self):
        super().__init__('Planner')
        self.get_logger().info('Starting Planner server')
        self.planner_communication_sync_server = self.create_service(
            Action, 'planner_communication_sync_server', self.receive_sync_message
        )
        self.state = init_state
        self.planner = None 
        # Subscriber para indicar fim da execução
        self.end_subscription = self.create_subscription(
            Bool, '/jason/shutdown_signal', self.shutdown_callback, 10
        )
        self.end_simulation_subscription = self.create_subscription(
            Bool, '/coordinator/shutdown_signal', self.end_simulation_callback, 10
        )

        self.get_logger().info('Planner server started')

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            raise SystemExit
        
    def end_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento do coordenador, finalizando...")
            raise SystemExit

    def receive_sync_message(self, request, response):
        messageTuple = tuple(request.action.split('|'))

        if messageTuple[0] == 'need_plan':
            actionTuple = tuple(messageTuple[1].split(','))
            self.get_logger().info('Creating plan for: %s %s %s' % (
                actionTuple[0], actionTuple[1], actionTuple[2]
            ))

            self.planner = IPyHOP(methods, actions)
            plan = self.planner.plan(self.state, [(
                'm_patrol_and_disinfect', actionTuple[0], actionTuple[1], actionTuple[2]
            )], verbose=1)
            

            responsePlan = [','.join(action) for action in plan]
            response.observation = '/'.join(responsePlan)

            self.get_logger().info('Sending response')
            return response
        
        elif messageTuple[0] == 'replan':
            if self.planner is None:
                response.observation = 'replan_failed'
                return response

            failed_action_key = messageTuple[1]  
            failed_action_tuple = tuple(failed_action_key.split(','))

            fail_node_id = None
            for node_id, node in self.planner.sol_tree.nodes(data=True):
                if node.get('info') == failed_action_tuple:
                    fail_node_id = node_id
                    break

            if fail_node_id is None:
                self.get_logger().error(f'Nó não encontrado para a ação: {failed_action_key}')
                response.observation = 'replan_failed'
                return response

            
            ancestors = list(nx.ancestors(self.planner.sol_tree, fail_node_id))
            ancestors.append(fail_node_id)
            for nid in ancestors:
                node = self.planner.sol_tree.nodes[nid]
                if 'methods' in node:
                    node['available_methods'] = iter(node['methods'])

            plan_result = self.planner.replan(self.state, fail_node_id, verbose=1)

            if not plan_result:
                response.observation = 'replan_failed'
                return response

            responsePlan = [','.join(action) for action in plan_result]
            response.observation = 'replan/' + '/'.join(responsePlan)
            return response
        
        elif messageTuple[0] == 'update_state':
            state = json.loads(messageTuple[1])
            self.state.loc = state['loc']
            self.state.doors = state['doors']
            self.state.disinfected = state['disinfected']
            # self.state.cleaned = state['cleaned']
            return response
        elif messageTuple[0] == 'update_room_uncleaned': 
            room = messageTuple[1]
            self.state.cleaned[room] = False
            self.get_logger().info(f"Estado atualizado: cleaned[{room}] = False → {self.state.cleaned}")
            response.observation = 'success'
            return response

        response.observation = 'success'
        return response

def main():
    rclpy.init()
    planner = Planner()
    try:
        rclpy.spin(planner)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()