import rclpy

from interfaces.srv import Action
from agents.helpers.agent import Agent
from agents.helpers.ActionResults import ActionResult

class LargeDeliveryRobot(Agent):
    def __init__(self, className):
        super().__init__(className)

    # ACTIONS
    def choose_action(self, actionTuple):
        self.current_action = actionTuple[0]
        future = None
        if actionTuple[0] == 'a_navto':
            self.get_logger().info('Doing a_navto:' + actionTuple[2])
            self.a_navto(actionTuple[1], actionTuple[2])
            return ActionResult.MOVING
        elif actionTuple[0] == 'a_request_resource':
            self.get_logger().info('Doing a_request_resource')
            future = self.a_request_resource(actionTuple[1], actionTuple[2], actionTuple[3])

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
            elif response.observation == 'resource not available':
                self.notifyError(
                    ','.join(('resource_not_available', actionTuple[3])) + '|' + ','.join(actionTuple)
                )
                return ActionResult.FAILURE

        return ActionResult.SUCCESS

    def a_request_resource(self, robot, storage, resource):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_request_resource', robot, storage, resource))
        return self.environment_client.call_async(self.action_request)

def main():
    rclpy.init()
    large_delivery_robot = LargeDeliveryRobot('LargeDeliveryRobot')
    try:
        large_delivery_robot.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    large_delivery_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()