import rclpy

from agents.helpers.agent import Agent
from agents.helpers.ActionResults import ActionResult

class Arm(Agent):
    def __init__(self, className):
        super().__init__(className)

    def choose_action(self, actionTuple):
        future = None

        if actionTuple[0] == 'a_pick_up_sample':
            self.get_logger().info('Doing a_pick_up_sample')
            self.a_pick_up_sample(actionTuple[1], actionTuple[2])
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

    def a_pick_up_sample(self, arm, robot):
        self.wating_action("a_pick_up_sample", arm, robot)

def main():
    rclpy.init()
    arm = Arm('Arm')
    try:
        # rclpy.spin(arm)
        arm.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()