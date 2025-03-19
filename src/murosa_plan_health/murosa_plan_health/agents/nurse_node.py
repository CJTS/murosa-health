import rclpy
import time
from random import randrange
from murosa_plan_health.agent import Agent
from murosa_plan_health.helper import FIPAMessage
from interfaces.srv import Message
from murosa_plan_health.FIPAPerformatives import FIPAPerformative

from interfaces.srv import Action

class Nurse(Agent):
    def __init__(self, className):
        super().__init__(className)
        self.has_sample = False
        self.run()

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            if not self.has_sample:
                self.has_sample = True
                seconds = randrange(10)
                time.sleep(seconds)
                self.send_has_sample_message()

    def send_has_sample_message(self):
        message = FIPAMessage(FIPAPerformative.INFORM.value, self.className, 'Coordinator', 'HasSample').encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        return self.cli.call_async(ros_msg)

    def choose_action(self, actionTuple):
        if actionTuple[0] == 'a_authenticate_nurse':
            self.get_logger().info('Doing a_authenticate_nurse')
            response = self.a_authenticate_nurse(
                actionTuple[1], actionTuple[2]
            )
            self.get_logger().info(response.observation)
        elif actionTuple[0] == 'a_open_door':
            self.get_logger().info('Doing a_open_door')
            response = self.a_open_door(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)
        elif actionTuple[0] == 'a_deposit':
            self.get_logger().info('Doing a_deposit')
            response = self.a_deposit(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)

        if not response.observation == 'success':
            return False
        return True

    def a_authenticate_nurse(self, robot, nurse):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_authenticate_nurse', robot, nurse)
        )
        return self.robot_communication_sync_client.call(
            self.action_request
        )

    def a_open_door(self, nurse, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_open_door', nurse, room)
        )
        return self.environment_client.call_async(self.action_request)

    def a_deposit(self, nurse, robot):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_deposit', nurse, robot)
        )
        return self.robot_communication_sync_client.call(
            self.action_request
        )

def main():
    rclpy.init()
    nurse = Nurse('Nurse')
    try:
        nurse.run()
        # rclpy.spin(nurse)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    nurse.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()