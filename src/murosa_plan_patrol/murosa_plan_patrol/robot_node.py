import rclpy
from murosa_plan_health.agent import Agent
from murosa_plan_health.ActionResults import ActionResult

from interfaces.srv import Action

class Robot(Agent):
    def __init__(self, className):
        super().__init__(className)

    def choose_action(self, actionTuple):
        future = None

        if actionTuple[0] == 'a_move':
            self.get_logger().info('Doing a_move')
            future = self.a_move(actionTuple[1], actionTuple[2])

        if future != None:
            self.get_logger().info("Wating for response")
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(response.observation)

            if not response.observation == 'success':
                self.pos = actionTuple[2]
                return ActionResult.FAILURE

        return ActionResult.SUCCESS

    def a_move(self, robot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_move', robot, room)
        )
        return self.environment_client.call_async(self.action_request)

def main():
    rclpy.init()
    robot = Robot('Robot')
    try:
        robot.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()