from interfaces.srv import Action

import rclpy
from rclpy.node import Node
from murosa_plan_patrol.patrol.domain.patrol_methods import methods
from murosa_plan_patrol.patrol.domain.patrol_actions import actions
from murosa_plan_patrol.patrol.problem.patrol_problem import init_state
from murosa_plan_patrol.ipyhop import IPyHOP
from std_msgs.msg import String, Bool


class Planner(Node):
    def __init__(self):
        super().__init__('Planner')
        self.state = init_state

        self.get_logger().info('Starting Planner server')
        self.planner_communication_sync_server = self.create_service(
            Action, 'planner_communication_sync_server', self.receive_sync_message
        )
        self.end_subscription = self.create_subscription(
            Bool,
            '/shutdown_signal',
            self.shutdown_callback,
            10
        )
        self.get_logger().info('Planner server started')

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            raise SystemExit

    def receive_sync_message(self, request, response):
        self.get_logger().info('Receiving sync message. %s' % (request.action))
        actionTuple = tuple(request.action.split(','))

        if actionTuple[0] == 'need_plan':
            self.get_logger().info('Creating plan for: %s' % (
                actionTuple[1]
            ))

            planner = IPyHOP(methods, actions)
            plan = planner.plan(self.state, [(
                'm_patrol_all', actionTuple[1]
            )], verbose=1)

            responsePlan = []

            for action in plan:
                self.get_logger().info(','.join(action))
                responsePlan.append(','.join(action))

            response.observation = '/'.join(responsePlan)

            self.get_logger().info('Sending response')
            return response

        response.observation = 'success'
        return response


def main():
    rclpy.init()
    planner = Planner()
    planner.get_logger().info('spin')
    try:
        rclpy.spin(planner)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    planner.get_logger().info('stop spin')
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
