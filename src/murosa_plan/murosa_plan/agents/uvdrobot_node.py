import rclpy
from murosa_plan.agent import Agent
from murosa_plan.FIPAPerformatives import FIPAPerformative
from murosa_plan.ActionResults import ActionResult
from murosa_plan.helper import FIPAMessage
from interfaces.srv import Action, Message

class Uvdrobot(Agent):
    def __init__(self, className):
        super().__init__(className)

    def choose_action(self, actionTuple):
        self.current_action = actionTuple[0]
        future = None
        if actionTuple[0] == 'a_navto':
            self.get_logger().info('Doing a_navto')
            future = self.a_navto(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_disinfect_room':
            self.get_logger().info('Doing a_disinfect_room')
            future = self.a_disinfect_room(actionTuple[1], actionTuple[2])
        
           

        if future != None:
            self.get_logger().info("Waiting for response")
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
            elif response.observation == 'dirty_room':
                self.notifyError(
                    ','.join(('dirty_room', actionTuple[2]))
                )
                return ActionResult.FAILURE
        

        return ActionResult.SUCCESS
    
    def a_navto(self, uvdrobot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_navto', uvdrobot, room))
        return self.environment_client.call_async(self.action_request)
    
    def a_disinfect_room(self, uvdrobot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_disinfect_room', uvdrobot, room))
        return self.environment_client.call_async(self.action_request)
    

def main():
    rclpy.init()
    robot = Uvdrobot('Uvdrobot')
    try:
        # rclpy.spin(robot)
        robot.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    