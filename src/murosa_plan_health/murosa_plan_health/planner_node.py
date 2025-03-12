from interfaces.srv import Action

import rclpy
from rclpy.node import Node
from murosa_plan_health.hospital.domain.hospital_mod_methods import methods
from murosa_plan_health.hospital.domain.hospital_mod_actions import actions
from murosa_plan_health.hospital.problem.hospital_mod_problem import init_state
from murosa_plan_health.ipyhop import IPyHOP


class Planner(Node):
    def __init__(self):
        super().__init__('Planner')
        self.get_logger().info('Starting Planner server')
        self.planner_communication_sync_server = self.create_service(
            Action, 'planner_communication_sync_server', self.receive_sync_message
        )
        self.state = init_state
        self.get_logger().info('Planner server started')

    def receive_sync_message(self, request, response):
        self.get_logger().info('Receiving sync message. %s' % (request.action))
        actionTuple = tuple(request.action.split(','))

        if actionTuple[0] == 'need_plan':
            self.get_logger().info('Creating plan for: %s %s %s' % (
                actionTuple[1], actionTuple[1], actionTuple[2]
            ))

            planner = IPyHOP(methods, actions)
            plan = planner.plan(self.state, [(
                'm_pickup_and_deliver_sample', actionTuple[1], actionTuple[2], actionTuple[3]
            )], verbose=1)

            responsePlan = []

            for action in plan:
                self.get_logger().info(','.join(action))
                responsePlan.append(','.join(action))

            response.observation = '/'.join(responsePlan)

            self.get_logger().info('Sending response')
            return response
        elif actionTuple[0] == 'update_state':
            if actionTuple[1] == 'door_closed':
                self.get_logger().info("door_closed")
                self.state.doors[actionTuple[2]] = False
                return response

        response.observation = 'success'
        return response


def main():
    rclpy.init()
    planner = Planner()
    rclpy.spin(planner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
