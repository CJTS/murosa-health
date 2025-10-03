import rclpy
import time
from random import randrange
from murosa_plan.agent import Agent
from murosa_plan.helper import FIPAMessage
from interfaces.srv import Message
from murosa_plan.FIPAPerformatives import FIPAPerformative
from murosa_plan.ActionResults import ActionResult
from std_msgs.msg import String, Bool

from interfaces.srv import Action

class Nurse(Agent):
    def __init__(self, className):
        super().__init__(className)
        self.counter = 10000
        self.room = None

    def run(self):
        while rclpy.ok():
            # self.counter = self.counter + 1
            rclpy.spin_once(self, timeout_sec=0.001)
            if self.room is None:
                future = self.what_room()
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response.observation != 'none':
                    self.room = response.observation
                    self.get_logger().info("I am in room: " + self.room)

            if self.counter == 10000:
                self.counter = 0
                seconds = 1
                time.sleep(seconds)
                future = self.a_infected_room()
                rclpy.spin_until_future_complete(self, future)
                self.send_has_infected_room()
            else:
                if not self.wating:
                    self.act()

    def send_has_infected_room(self):
        message = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Coordinator', 'InitialTrigger|' + self.room).encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        return self.cli.call_async(ros_msg)

    def choose_action(self, actionTuple):
        future = None
        if actionTuple[0] == 'a_authenticate_nurse':
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

        if future != None:
            self.get_logger().info("Waiting for response")
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(response.observation)

            if not response.observation == 'success':
                self.pos = actionTuple[2]
                return ActionResult.FAILURE

        return ActionResult.SUCCESS

    def what_room(self):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('what_room', self.agentName)
        )
        return self.environment_client.call_async(self.action_request)

    def a_infected_room(self):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_infected_room', self.agentName)
        )
        return self.environment_client.call_async(self.action_request)
    
    def a_authorize_patrol(self, spotrobot, nurse):
        if all('a_authorize_patrol' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for robot")
            self.ask_for_agent(spotrobot, 'a_authorize_patrol')
        else:
            self.get_logger().info("Robot is waiting, send action message")
            self.acting_for_agent(spotrobot, 'a_authorize_patrol')

    def a_authenticate_nurse(self, spotrobot, nurse):
        if all('a_authenticate_nurse' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for robot")
            self.ask_for_agent(spotrobot, 'a_authenticate_nurse')
        else:
            self.get_logger().info("Robot is waiting, send action message")
            self.acting_for_agent(spotrobot, 'a_authenticate_nurse')

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