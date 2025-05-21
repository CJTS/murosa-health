import rclpy
from murosa_plan.agent import Agent
from murosa_plan.FIPAPerformatives import FIPAPerformative
from murosa_plan.ActionResults import ActionResult
from murosa_plan.helper import FIPAMessage
from interfaces.srv import Action, Message

class Spotrobot(Agent):
    def __init__(self, className):
        super().__init__(className)

    # ACTIONS
    def choose_action(self, actionTuple):
        self.current_action = actionTuple[0]
        future = None
        if actionTuple[0] == 'a_navto':
            self.get_logger().info('Doing a_navto')
            future = self.a_navto(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_open_door':
            self.get_logger().info('Doing a_open_door')
            future = self.a_open_door(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_approach_nurse':
            self.get_logger().info('Doing a_approach_nurse')
            future = self.a_approach_nurse(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_authenticate_nurse':
            self.get_logger().info('Doing a_authenticate_nurse')
            self.a_authenticate_nurse(actionTuple[1], actionTuple[2])
            return ActionResult.WAITING
        elif actionTuple[0] == 'a_authorize_patrol':
            self.get_logger().info('Doing a_authorize_patrol')
            self.a_authenticate_nurse(actionTuple[1], actionTuple[2])
            return ActionResult.WAITING
        elif actionTuple[0]== 'a_patrol_room':
            self.get_logger().info('Doing a_patrol_room')
            future = self.a_patrol_room(actionTuple[1], actionTuple[2])


        if future != None:
            self.get_logger().info("Wating for response")
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(response.observation)

            if response.observation == 'success':
                self.pos = actionTuple[2]
            elif response.observation == 'door closed':
                self.notifyError(
                    ','.join(('door_closed', actionTuple[2]))
                )
                return ActionResult.FAILURE
            elif response.observation == 'dirty room':
                self.notifyError(
                    ','.join(('dirty_room', actionTuple[2]))
                )
                return ActionResult.FAILURE

        return ActionResult.SUCCESS
    
    def a_navto(self, spotrobot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_navto', spotrobot, room))
        return self.environment_client.call_async(self.action_request)
    
    def a_open_door(self, spotrobot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_open_door', spotrobot, room)
        )
        return self.environment_client.call_async(self.action_request)

    def a_approach_nurse(self, spotrobot, nurse):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_approach_nurse', spotrobot, nurse))
        return self.environment_client.call_async(self.action_request)

    def a_authenticate_nurse(self, spotrobot, nurse):
        self.get_logger().info("a_authenticate_nurse")
        if all('a_authenticate_nurse' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for nurse")
            self.ask_for_agent(nurse, 'a_authenticate_nurse')
        else:
            self.get_logger().info("Nurse is waiting, send action message")
            self.acting_for_agent(nurse, 'a_authenticate_nurse')
    
    def a_authorize_patrol(self, spotrobot,nurse):
        self.get_logger().info("a_authorize_patrol")
        if all('a_authorize_patrol' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for nurse")
            self.ask_for_agent(nurse, 'a_authorize_patrol')
        else:
            self.get_logger().info("Nurse is waiting, send action message")
            self.acting_for_agent(nurse, 'a_authorize_patrol')

    def a_patrol_room(self, spotrobot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_patrol_room', spotrobot, room))
        return self.environment_client.call_async(self.action_request)
    
def main():
    rclpy.init()
    robot = Spotrobot('Robot')
    try:
        # rclpy.spin(robot)
        robot.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
