import rclpy
from murosa_plan.agent import Agent
from murosa_plan.ActionResults import ActionResult

from interfaces.srv import Action

class Patrol(Agent):
    def __init__(self, className):
        super().__init__(className)
        self.battery = 3

    def choose_action(self, actionTuple):
        future = None

        if actionTuple[0] == 'move':
            if(self.battery > 2):
                self.get_logger().info('Doing move')
                future = self.move(actionTuple[1], actionTuple[2])
                self.battery -= 1
            else:
                self.get_logger().info('low_battery')
                self.notifyError(
                    ','.join(('low_battery', self.agentName))
                )
                return ActionResult.FAILURE
        elif actionTuple[0] == 'charge':
            self.get_logger().info('Doing charge')
            self.charge(actionTuple[1])

        if future != None:
            self.get_logger().info("Wating for response")
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(response.observation)

            if not response.observation == 'success':
                self.pos = actionTuple[2]
                return ActionResult.FAILURE

        return ActionResult.SUCCESS

    def move(self, robot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('move', robot, room)
        )
        return self.environment_client.call_async(self.action_request)

    def charge(self, robot):
        self.battery += 5

def main():
    rclpy.init()
    patrol = Patrol('Patrol')
    try:
        patrol.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    patrol.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()