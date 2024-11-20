from threading import Thread
import time
import json
import rclpy
from rclpy.node import Node
from interfaces.srv import NeedPlan, HasSample, SendPlan, Action
from murosa_plan_patrol.helper import action_string_to_tuple, action_tuple_to_string
from std_msgs.msg import String, Bool

class Coordinator(Node):
    def __init__(self):
        super().__init__('Coordinator')
        self.agents_client = {}
        self.robots = []
        self.replan = True
        self.errors = {}
        self.last_error = None
        self.current_plan = []
        self.plans_actions = 0
        self.plans_actions_current = 0
        self.declare_parameter('replan', rclpy.Parameter.Type.BOOL)
        replan_param = self.get_parameter('replan').get_parameter_value().bool_value
        self.should_replan = replan_param
        self.terminate = False
        self.start_server()
        self.start_clients()
        self.start = time.time()
        self.start_sim()

    def start_server(self):
        self.get_logger().info('Starting Coordinator server')
        self.coordinator_server = self.create_service(
            HasSample, 'coordinator_server', self.receive_message
        )
        self.coordinator_communication_sync_server = self.create_service(
            Action, 'coordinator_communication_sync_server', self.receive_sync_message
        )
        self.publisher_ = self.create_publisher(String, '/coordinator/jason/plan', 10)
        self.end_subscription = self.create_subscription(
            Bool,
            '/shutdown_signal',
            self.shutdown_callback,
            10
        )
        self.end_subscription  # Prevent unused variable warning

        self.get_logger().info('Coordinator server started')

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            self.end = time.time()
            self.get_logger().info(
                "The time of execution of above program is:%s" % (
                    self.end - self.start
                )
            )
            self.get_logger().info('Achieved')
            self.get_logger().info('ENDSIM')
            raise SystemExit

    def receive_message(self, request, response):
        self.start = time.time()
        self.get_logger().info('Receiving message of %s in %s.' % (
            request.nurseid, request.roomid
        ))
        self.nurses.append(request.nurseid)
        self.replan = True
        response.ok = 'Okay!'
        self.get_logger().info('Sending response')
        return response

    def receive_sync_message(self, request, response):
        self.get_logger().info('Receiving sync message: %s' % (request.action))
        actionTuple = tuple(request.action.split(','))

        if actionTuple[0] == 'action_complete':
            self.get_logger().info("action_complete")
            self.plans_actions_current = self.plans_actions_current + 1
            self.get_logger().info(
                "plans_actions_current %i of %i" %
                (self.plans_actions_current, self.plans_actions)
            )
            response.observation = 'okay'
            if self.plan_complete():
                self.end = time.time()
                self.get_logger().info(
                    "The time of execution of above program is:%s" % (
                        self.end - self.start
                    )
                )
                self.get_logger().info('Achieved')
                self.get_logger().info('ENDSIM')
            return response
        elif actionTuple[0] == 'online':
            self.add_client(actionTuple)
            response.observation = 'okay'
            return response
        elif actionTuple[0] == 'error':
            if actionTuple[1] == 'door_closed':
                self.get_logger().info("door_closed")
                if not self.known_error(actionTuple):
                    if self.should_replan:
                        self.replan = True
                        self.get_logger().info('Reactive replan')
                    response.observation = 'making new plan'
                    return response
            if not self.should_replan:
                self.end = time.time()
                self.get_logger().info(
                    "The time of execution of above program is:%s" % (
                        self.end - self.start
                    )
                )
                self.get_logger().info('Failed')
                self.get_logger().info('ENDSIM')

        response.observation = 'success'
        return response

    def plan_complete(self):
        return self.plans_actions_current >= self.plans_actions

    def add_client(self, client):
        if client[1] == 'robot':
            self.robots.append(client[2])

        self.agents_client['%s_%s_communication_sync_server' % (client[1], client[2])] = self.create_client(
            Action, '%s_%s_communication_sync_server' % (client[1], client[2])
        )
        while not self.agents_client['%s_%s_communication_sync_server' % (client[1], client[2])].wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'client %s_%s sync service not available, waiting again...' %
                (client[1], client[2])
            )

        self.agents_client['%s_%s_server' % (client[1], client[2])] = self.create_client(
            SendPlan, '%s_%s_server' %
            (client[1], client[2])
        )
        while not self.agents_client['%s_%s_server' % (client[1], client[2])].wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'client %s_%s service not available, waiting again...' %
                (client[1], client[2])
            )

    def known_error(self, error):
        if not error[1] in self.errors:
            self.last_error = error[2]
            self.errors[error[1]] = [error[2]]
            return False
        elif error[2] in self.errors[error[1]]:
            return True
        elif not error[2] in self.errors[error[1]]:
            self.last_error = error[2]
            self.errors[error[1]].append(error[2])
            return False

    def start_clients(self):
        self.get_logger().info('Starting Coordinator clients')
        self.planner_communication_sync_client = self.create_client(
            Action, 'planner_communication_sync_server'
        )
        while not self.planner_communication_sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('planner sync service not available, waiting again...')

        self.environment_client = self.create_client(
            Action, 'environment_server'
        )
        while not self.environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('environment service not available, waiting again...')

        self.robot_client = self.create_client(
            SendPlan, 'robot_server'
        )
        while not self.robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('robot service not available, waiting again...')
        self.get_logger().info('Coordinator clients started')

    def send_need_plan_request(self, robotid):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('need_plan', robotid))
        return self.planner_communication_sync_client.call_async(self.action_request)

    def send_update_state_request(self, state):
        self.update_state_request = Action.Request()
        self.update_state_request.action = state
        return self.planner_communication_sync_client.call(self.update_state_request)

    def send_monitor_state_request(self, action):
        self.update_state_request = Action.Request()
        self.update_state_request.action = action
        return self.environment_client.call(self.update_state_request)

    def send_plans_request(self):
        self.get_logger().info('Sending plans')
        # self.send_plan_request_robot = SendPlan.Request()
        # self.send_plan_request_robot.plan = list(map(
        #     action_tuple_to_string, self.robotsActions
        # ))
        # self.robot_client.call_async(self.send_plan_request_robot)
        msg = String()
        msg.data = 'patrol'
        self.publisher_.publish(msg)
        self.get_logger().info('Plans sent')

    def find_robot(self):
        return 'r2d2'

    def split_plans(self):
        self.robotsActions = []
        tuples = map(action_string_to_tuple, self.current_plan)

        for action in tuples:
            if 'r2d2' in action:
                self.robotsActions.append(action)

        self.plans_actions = len(self.robotsActions)

    def check_env(self):
        # self.get_logger().info('Monitoring')
        response = self.send_monitor_state_request(','.join(('monitor',)))
        # self.get_logger().info(response.observation)
        state = json.loads(response.observation)

        if not state['doors']['Room1']:
            self.get_logger().info('Found a problem')
            self.replan = not self.known_error(
                ('error', 'door_closed', 'Room1'))
        elif not state['doors']['Room2']:
            self.get_logger().info('Found a problem')
            self.replan = not self.known_error(
                ('error', 'door_closed', 'Room2'))
        elif not state['doors']['Room3']:
            self.get_logger().info('Found a problem')
            self.replan = not self.known_error(
                ('error', 'door_closed', 'Room3'))

    def start_sim(self):
        self.get_logger().info('Planing')
        available_robot = self.find_robot()

        self.get_logger().info('Creating plan for: %s' % (
            available_robot
        ))
        future = self.send_need_plan_request(
            available_robot)
        rclpy.spin_until_future_complete(self, future)
        plan_response = future.result()
        self.get_logger().info('Plan received for: %s' % (
            available_robot
        ))
        # self.get_logger().info(plan_response)
        self.current_plan = plan_response.observation.split('/')

        for action in self.current_plan:
            self.get_logger().info(action)

        self.split_plans()
        self.send_plans_request()
        self.plans_actions_current = 0

def main():
    rclpy.init()
    coordinator = Coordinator()
    coordinator.get_logger().info('spin')
    try:
        rclpy.spin(coordinator)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    coordinator.get_logger().info('stop spin')
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
