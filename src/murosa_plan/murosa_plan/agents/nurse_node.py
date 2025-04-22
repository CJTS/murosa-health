import rclpy
import time
from random import randrange
from murosa_plan.agent import Agent
from murosa_plan.helper import FIPAMessage
from interfaces.srv import Message
from murosa_plan.FIPAPerformatives import FIPAPerformative
from murosa_plan.ActionResults import ActionResult

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
                future = self.a_create_sample()
                rclpy.spin_until_future_complete(self, future)
                self.send_has_sample_message()
            else:
                if not self.wating:
                    self.act()

    def send_has_sample_message(self):
        message = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Coordinator', 'HasSample').encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        return self.cli.call_async(ros_msg)

    def choose_action(self, actionTuple):
        self.get_logger().info(actionTuple[0])
        future = None
        if actionTuple[0] == 'a_authenticate_nurse':
            self.get_logger().info('Doing a_authenticate_nurse')
            self.a_authenticate_nurse(
                actionTuple[1], actionTuple[2]
            )
            return ActionResult.WAITING
        elif actionTuple[0] == 'a_open_door':
            self.get_logger().info('Doing a_open_door')
            future = self.a_open_door(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_deposit':
            self.get_logger().info('Doing a_deposit')
            self.a_deposit(actionTuple[1], actionTuple[2])
            return ActionResult.WAITING
        if future != None:
            self.get_logger().info("Wating for response")
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(response.observation)

            if not response.observation == 'success':
                self.pos = actionTuple[2]
                return ActionResult.FAILURE

        return ActionResult.SUCCESS

    def a_create_sample(self):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_create_sample', self.agentName)
        )
        return self.environment_client.call_async(self.action_request)

    def a_authenticate_nurse(self, robot, nurse):
        self.get_logger().info("a_authenticate_nurse")
        if all('a_authenticate_nurse' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for robot")
            self.ask_for_agent(robot, 'a_authenticate_nurse')
        else:
            self.get_logger().info("Robot is waiting, send action message")
            self.acting_for_agent(robot, 'a_authenticate_nurse')

    def a_open_door(self, nurse, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_open_door', nurse, room)
        )
        return self.environment_client.call_async(self.action_request)

    def a_deposit(self, nurse, robot):
        self.get_logger().info("a_deposit")
        if all('a_deposit' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for robot")
            self.ask_for_agent(robot, 'a_deposit')
        else:
            self.get_logger().info("Robot is waiting, send action message")
            self.acting_for_agent(robot, 'a_deposit')

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