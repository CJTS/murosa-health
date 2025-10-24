import rclpy
import time

from interfaces.srv import Message, Action
from agents.helpers.agent import Agent
from agents.helpers.helper import FIPAMessage
from agents.helpers.ActionResults import ActionResult
from agents.helpers.FIPAPerformatives import FIPAPerformative

class Nurse(Agent):
    def __init__(self, className):
        super().__init__(className)

    def send_has_infected_room(self):
        message = FIPAMessage(FIPAPerformative.INFORM.value, self.get_name(), 'Coordinator', 'InitialTrigger|' + self.current_room).encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        return self.cli.call_async(ros_msg)

    def choose_action(self, actionTuple):
        future = None

        if actionTuple[0] == 'a_navto':
            self.get_logger().info('Doing a_navto')
            self.a_navto(actionTuple[1], actionTuple[2])
            return ActionResult.MOVING
        elif actionTuple[0] == 'a_approach_nurse':
            self.get_logger().info('Doing a_approach_nurse')
            self.a_approach_nurse(actionTuple[1], actionTuple[2])
            return ActionResult.WAITING
        elif actionTuple[0] == 'a_authenticate_nurse':
            self.get_logger().info('Doing a_authenticate_nurse')
            self.a_authenticate_nurse(
                actionTuple[1], actionTuple[2]
            )
            return ActionResult.WAITING
        elif actionTuple[0] == 'a_authorize_patrol':
            self.get_logger().info('Doing a_authorize_patrol')
            self.a_authorize_patrol(
                actionTuple[1], actionTuple[2]
            )
            return ActionResult.WAITING
        elif actionTuple[0] == 'a_open_door':
            self.get_logger().info('Doing a_open_door')
            future = self.a_open_door(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_clean_room':
            self.get_logger().info('Doing a_clean_room')
            future = self.a_clean_room(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_authenticate_nurse':
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
            self.get_logger().info("Waiting for response")
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(response.observation)

            if not response.observation == 'success':
                self.pos = actionTuple[2]
                return ActionResult.FAILURE

        return ActionResult.SUCCESS

    def a_infected_room(self):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_infected_room', self.get_name())
        )
        return self.environment_client.call_async(self.action_request)

    def a_authorize_patrol(self, spotrobot, nurse):
        if all('a_authorize_patrol' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for robot")
            self.ask_for_agent(spotrobot, 'a_authorize_patrol')
        else:
            self.get_logger().info("Robot is waiting, send action message")
            self.acting_for_agent(spotrobot, 'a_authorize_patrol')

    def a_approach_nurse(self, spotrobot, nurse):
        if all('a_approach_nurse' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for robot")
            self.ask_for_agent(spotrobot, 'a_approach_nurse')
        else:
            self.get_logger().info("Robot is waiting, send action message")
            self.acting_for_agent(spotrobot, 'a_approach_nurse')

    def a_open_door(self, nurse, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_open_door', nurse, room)
        )
        return self.environment_client.call_async(self.action_request)

    def a_clean_room(self, nurse, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_clean_room', nurse, room)
        )
        return self.environment_client.call_async(self.action_request)

    def a_create_sample(self):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_create_sample', self.get_name())
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
    nurse = Nurse('Nurse_Disinfected')
    try:
        nurse.run()
        # rclpy.spin(nurse)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    nurse.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()