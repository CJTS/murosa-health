import json
import rclpy
import networkx as nx

from rclpy.node import Node
from interfaces.srv import Action
from std_msgs.msg import Bool

from coordinator.planner.health.domain.methods import methods
from coordinator.planner.health.domain.actions import actions
from coordinator.planner.health.problem.problem import init_state
from coordinator.planner.ipyhop import IPyHOP

class Planner(Node):
    def __init__(self):
        super().__init__('Planner')
        self.get_logger().info('Starting Planner server')
        self.planner_communication_sync_server = self.create_service(
            Action, 'planner_communication_sync_server', self.receive_sync_message
        )
        self.state = init_state
        self.planner = None

        self.end_simulation_subscription = self.create_subscription(
            Bool, '/coordinator/shutdown_signal', self.end_simulation_callback, 10
        )

        self.get_logger().info('Planner server started')

    def end_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento do coordenador, finalizando...")
            raise SystemExit

    def receive_sync_message(self, request, response):
        messageTuple = tuple(request.action.split('|'))

        if messageTuple[0] == 'need_plan':
            actionTuple = tuple(messageTuple[1].split(','))

            plan_param = []
            responsePlan = []

            if actionTuple[0] == 'CollectSampleMission':
                goal = 'm_pickup_and_deliver_sample'
                plan_param = [(goal, actionTuple[1], actionTuple[2], actionTuple[3], actionTuple[4])]
                self.get_logger().info('Creating plan for: %s %s %s %s %s' % (
                    actionTuple[0], actionTuple[1], actionTuple[2], actionTuple[3], actionTuple[4]
                ))
            elif actionTuple[0] == 'DisinfectRoomMission' or actionTuple[0] == 'DisinfectICUMission':
                goal = 'm_patrol_and_disinfect'
                plan_param = [(goal, actionTuple[1], actionTuple[2], actionTuple[3], actionTuple[4])]
                self.get_logger().info('Creating plan for: %s %s %s %s %s' % (
                    actionTuple[0], actionTuple[1], actionTuple[2], actionTuple[3], actionTuple[4]
                ))
            elif actionTuple[0] == 'DeliverSampleMission':
                goal = 'm_deliver_resource_task'
                plan_param = [(goal, actionTuple[1], actionTuple[2], actionTuple[3], actionTuple[4], actionTuple[5], actionTuple[6], actionTuple[7])]
                self.get_logger().info('Creating plan for: %s %s %s %s %s %s %s' % (
                    actionTuple[1], actionTuple[2], actionTuple[3], actionTuple[4], actionTuple[5], actionTuple[6], actionTuple[7]
                ))

            self.planner = IPyHOP(methods, actions)
            plan = self.planner.plan(self.state, plan_param, verbose=1)

            for action in plan:
                responsePlan.append(','.join(action))
                self.get_logger().info(str(action))

            response.observation = '/'.join(responsePlan)

            self.get_logger().info('Sending response')
            return response
        elif messageTuple[0] == 'repair':
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
                response.observation = 'repair_failed'
                return response

            ancestors = list(nx.ancestors(self.planner.sol_tree, fail_node_id))
            ancestors.append(fail_node_id)
            for nid in ancestors:
                node = self.planner.sol_tree.nodes[nid]
                if 'methods' in node:
                    node['available_methods'] = iter(node['methods'])

            plan_result = self.planner.replan(self.state, fail_node_id, verbose=1)

            if not plan_result:
                response.observation = 'repair_failed'
                return response

            responsePlan = [','.join(action) for action in plan_result]
            response.observation = 'repair/' + '/'.join(responsePlan)
            return response
        elif messageTuple[0] == 'update_state':
            state = json.loads(messageTuple[1])
            self.state.loc = state['loc']
            self.state.doors = state['doors']
            self.state.sample = state['sample']
            self.state.samples = state['samples']
            self.state.cleaned = state['cleaned']
            self.state.disinfected = state['disinfected']
            self.state.low_battery = state['low_battery']
            self.state.resource_at = state['resource_at']
            self.state.carrying = state['carrying']
            return response
        elif messageTuple[0] == 'update_room_uncleaned':
            self.state.cleaned[messageTuple[1]] = False

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