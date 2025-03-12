import random
from threading import Thread
import rclpy
from rclpy.node import Node
from interfaces.srv import HasSample, SendPlan, Action
from murosa_plan_health.helper import action_string_to_tuple


class Robot(Node):
    def __init__(self):
        super().__init__('Robot')
        self.local_plan = []
        self.has_sample = False
        self.drawer = False
        self.pos = 'Room2'
        self.current_action = ''
        self.nurseId = ''
        self.current_action_tuple = ()
        self.new_plan = False

    def start_server(self):
        self.get_logger().info('Starting Robot server')
        self.robot_server = self.create_service(
            SendPlan, 'robot_server', self.receive_message
        )
        self.robot_communication_sync_server = self.create_service(
            Action, 'robot_communication_sync_server', self.receive_sync_message
        )
        self.get_logger().info('Robot server started')

    def receive_message(self, request, response):
        self.get_logger().info('Receiving message.')
        self.get_logger().info("Robot's actions:")
        self.new_plan = True
        self.local_plan = list(map(action_string_to_tuple, request.plan))
        for action in self.local_plan:
            self.get_logger().info(action[0])
        response.ok = 'Okay!'
        return response

    def receive_sync_message(self, request, response):
        self.get_logger().info('Receiving sync message. %s' % (request.action))
        actionTuple = tuple(request.action.split(','))

        if actionTuple[0] == 'a_deposit':
            if self.current_action == 'a_deposit' and self.drawer:
                self.has_sample = True
                self.get_logger().info("Deposit")
                response.observation = 'success'
                return response
            elif self.current_action == 'a_deposit' and not self.drawer:
                self.get_logger().info("drawer not opened")
                response.observation = 'drawer not opened'
            else:
                self.get_logger().info("not ready")
                response.observation = 'not ready'
                return response
        elif actionTuple[0] == 'a_authenticate_nurse':
            if self.current_action == 'a_authenticate_nurse':
                self.nurseId = actionTuple[2]
                self.get_logger().info("Authenticate nurse")
                response.observation = 'success'
                return response
            else:
                self.get_logger().info("not ready")
                response.observation = 'not ready'
                return response
        elif actionTuple[0] == 'a_pick_up_sample':
            if self.current_action == 'a_pick_up_sample':
                self.has_sample = False
                self.get_logger().info("Pick up")
                response.observation = 'success'
                return response
            else:
                self.get_logger().info("not ready")
                response.observation = 'not ready'
                return response

        response.observation = 'success'
        return response

    def start_client(self):
        self.get_logger().info('Starting Robots clients')
        self.coordinator_client = self.create_client(
            HasSample, 'coordinator_server'
        )
        while not self.coordinator_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('coordinator service not available, waiting again...')

        self.coordinator_communication_sync_client = self.create_client(
            Action, 'coordinator_communication_sync_server'
        )
        while not self.coordinator_communication_sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('coordinator sync service not available, waiting again...')

        self.environment_client = self.create_client(
            Action, 'environment_server'
        )
        while not self.environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('environment service not available, waiting again...')
        self.get_logger().info('Robots clients started')

    def notifySuccess(self):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('action_complete', self.current_action))
        return self.coordinator_communication_sync_client.call(
            self.action_request
        )

    def notifyError(self, error):
        self.action_request = Action.Request()
        self.action_request.action = error
        return self.coordinator_communication_sync_client.call(
            self.action_request
        )

    def run(self):
        while rclpy.ok():
            # local plan
            if len(self.local_plan) > 0:
                self.current_action_tuple = self.local_plan.pop(0)
                self.get_logger().info(
                    "Action: %s" % (self.current_action_tuple[0])
                )
                ok = self.choose_action(self.current_action_tuple)
                self.get_logger().info("Action result: %s" % (ok))

                if ok :
                    self.notifySuccess()

                if not ok and not self.new_plan:
                    self.local_plan.insert(0, self.current_action_tuple)
                self.new_plan = False

    # ACTIONS
    def choose_action(self, actionTuple):
        self.current_action = actionTuple[0]
        if actionTuple[0] == 'a_navto':
            self.get_logger().info('Doing a_navto')
            response = self.a_navto(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)
            if response.observation == 'success':
                self.pos = actionTuple[2]
            elif response.observation == 'door closed':
                self.notifyError(
                    ','.join(('error', 'door_closed', actionTuple[2]))
                )
                return False
        elif actionTuple[0] == 'a_approach_nurse':
            self.get_logger().info('Doing a_approach_nurse')
            response = self.a_approach_nurse(actionTuple[1])
            self.get_logger().info(response.observation)
        elif actionTuple[0] == 'a_authenticate_nurse':
            self.get_logger().info('Doing a_authenticate_nurse')
            return self.a_authenticate_nurse(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_open_drawer':
            self.get_logger().info('Doing a_open_drawer')
            response = self.a_open_drawer(actionTuple[1])
            self.get_logger().info(response)
        elif actionTuple[0] == 'a_deposit':
            self.get_logger().info('Doing a_deposit')
            return self.a_deposit(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_close_drawer':
            self.get_logger().info('Doing a_close_drawer')
            response = self.a_close_drawer(actionTuple[1])
            self.get_logger().info(response)
        elif actionTuple[0] == 'a_approach_arm':
            self.get_logger().info('Doing a_approach_arm')
            response = self.a_approach_arm(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)
        elif actionTuple[0] == 'a_pick_up_sample':
            self.get_logger().info('Doing a_pick_up_sample')
            return self.a_pick_up_sample(actionTuple[1], actionTuple[2])
        return True

    def a_navto(self, robot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_navto', robot, room))
        return self.environment_client.call(self.action_request)

    def a_approach_nurse(self, robot):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_approach_nurse', robot))
        return self.environment_client.call(self.action_request)

    def a_authenticate_nurse(self, robot, nurse):
        if not self.nurseId == nurse:
            self.get_logger().info('wating for %s to authenticate to %s' % (nurse, robot))
            return False
        return True

    def a_open_drawer(self, robot):
        possibilityChoices = [True, False]
        # self.drawer = random.choice(possibilityChoices)
        self.drawer = True
        return 'success'

    def a_close_drawer(self, robot):
        self.drawer = False
        return 'success'

    def a_approach_arm(self, robot, arm):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_approach_arm', robot, arm))
        return self.environment_client.call(self.action_request)

    def a_deposit(self, nurse, robot):
        if not self.has_sample:
            self.get_logger().info('wating for %s to deposit in %s' % (nurse, robot))
            return False
        return True

    def a_pick_up_sample(self, arm, robot):
        if self.has_sample:
            self.get_logger().info('wating for %s to pick up from %s' % (arm, robot))
            return False
        return True


def main():
    rclpy.init()
    robot = Robot()
    robot.start_server()
    robot.start_client()
    spin_thread = Thread(target=rclpy.spin, args=(robot,))
    spin_thread.start()
    robot.run()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
