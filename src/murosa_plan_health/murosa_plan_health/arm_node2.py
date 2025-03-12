from threading import Thread
import rclpy
from rclpy.node import Node
from interfaces.srv import Action, SendPlan
from murosa_plan_health.helper import action_string_to_tuple

class Arm(Node):

    def __init__(self):
        super().__init__('Arm')
        self.local_plan = []
        self.current_action = ()
        self.new_plan = False

    def start_server(self):
        self.get_logger().info('Starting Arm server')
        self.arm_server = self.create_service(
            SendPlan, 'arm_server', self.receive_message
        )
        self.get_logger().info('Arm server started')

    def receive_message(self, request, response):
        self.get_logger().info('Receiving message.')
        self.get_logger().info("Arm's actions:")
        self.new_plan = True
        self.local_plan = list(map(action_string_to_tuple, request.plan))
        for action in self.local_plan:
            self.get_logger().info(action[0])
        response.ok = 'Okay!'
        return response

    def start_client(self):
        self.get_logger().info('Starting Nurse clients')
        self.environment_client = self.create_client(
            Action, 'environment_server'
        )
        while not self.environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('environment service not available, waiting again...')
            
        self.robot_communication_sync_client = self.create_client(
            Action, 'robot_communication_sync_server'
        )
        while not self.robot_communication_sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('robot service not available, waiting again...')
        self.get_logger().info('Nurse clients started')

    def run(self):
        while rclpy.ok():
            # local plan
            if len(self.local_plan) > 0:
                self.current_action = self.local_plan.pop(0)
                self.get_logger().info("Action: %s" % (self.current_action[0]))
                ok = self.choose_action(self.current_action)
                self.get_logger().info("Action result: %s" % (ok))
                if not ok and not self.new_plan:
                    self.local_plan.insert(0, self.current_action)
                self.new_plan = False

    #ACTIONS
    def choose_action(self, actionTuple):
        if actionTuple[0] == 'a_approach_arm':
            self.get_logger().info('Doing a_approach_arm')
            response = self.a_approach_arm(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)
        elif actionTuple[0] == 'a_pick_up_sample':
            self.get_logger().info('Doing a_pick_up_sample')
            response = self.a_pick_up_sample(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)

        if not response.observation == 'success':
            return False
        return True

    def a_approach_arm(self, robot, arm):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_approach_arm', robot, arm))
        return self.environment_client.call(self.action_request)


    def a_pick_up_sample(self, arm, robot):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_pick_up_sample', arm, robot))
        return self.robot_communication_sync_client.call(self.action_request)

def main():
    rclpy.init()
    arm = Arm()
    arm.start_server()
    arm.start_client()
    spin_thread = Thread(target=rclpy.spin, args=(arm,))
    spin_thread.start()
    arm.run()
    arm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
