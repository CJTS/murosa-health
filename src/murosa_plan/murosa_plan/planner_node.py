
import json
import rclpy
from rclpy.node import Node
from interfaces.srv import Action
from murosa_plan.hospital.domain.hospital_mod_methods import methods
from murosa_plan.hospital.domain.hospital_mod_actions import actions
from murosa_plan.hospital.problem.hospital_mod_problem import init_state
from murosa_plan.ipyhop import IPyHOP
from std_msgs.msg import Bool

class Planner(Node):
    def __init__(self):
        super().__init__('Planner')
        self.get_logger().info('Starting Planner server')
        self.planner_communication_sync_server = self.create_service(
            Action, 'planner_communication_sync_server', self.receive_sync_message
        )
        self.state = init_state

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
                actionTuple[1], actionTuple[0], actionTuple[2]
            ))

            planner = IPyHOP(methods, actions)
            plan = planner.plan(self.state, [(
                'm_pickup_and_deliver_sample', actionTuple[1], actionTuple[0], actionTuple[2]
            )], verbose=1)

            responsePlan = []

            for action in plan:
                self.get_logger().info(','.join(action))
                responsePlan.append(','.join(action))

            response.observation = '/'.join(responsePlan)

            self.get_logger().info('Sending response')
            return response
        elif messageTuple[0] == 'update_state':
            state = json.loads(messageTuple[1])
            self.state.loc = state['loc']
            self.state.doors = state['doors']
            self.state.sample = state['sample']
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