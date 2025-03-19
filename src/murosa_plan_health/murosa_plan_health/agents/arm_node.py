import rclpy
from murosa_plan_health.agent import Agent

from interfaces.srv import Action

class Arm(Agent):
    def __init__(self, className):
        super().__init__(className)

    def choose_action(self, actionTuple):
        if actionTuple[0] == 'a_approach_arm':
            self.get_logger().info('Doing a_approach_arm')
            response = self.a_approach_arm(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)
        elif actionTuple[0] == 'a_pick_up_sample':
            self.get_logger().info('Doing a_pick_up_sample')
            response = self.a_pick_up_sample(actionTuple[1], actionTuple[2])
            self.get_logger().info(response.observation)

        if not response.observation == 'success':
            return False
        return True

    def a_approach_arm(self, robot, arm):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_approach_arm', robot, arm))
        return self.environment_client.call_async(self.action_request)

    def a_pick_up_sample(self, arm, robot):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_pick_up_sample', arm, robot))
        return self.robot_communication_sync_client.call(self.action_request)

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