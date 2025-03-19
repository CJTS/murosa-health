import rclpy
from murosa_plan_health.agent import Agent
from murosa_plan_health.FIPAPerformatives import FIPAPerformative
from murosa_plan_health.helper import FIPAMessage
from interfaces.srv import Action

class Robot(Agent):
    def __init__(self, className):
        super().__init__(className)

    # ACTIONS
    def choose_action(self, actionTuple):
        self.current_action = actionTuple[0]
        if actionTuple[0] == 'a_navto':
            self.get_logger().info('Doing a_navto')
            future = self.a_navto(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_approach_nurse':
            self.get_logger().info('Doing a_approach_nurse')
            future = self.a_approach_nurse(actionTuple[1])
        elif actionTuple[0] == 'a_authenticate_nurse':
            self.get_logger().info('Doing a_authenticate_nurse')
            future = self.a_authenticate_nurse(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_open_drawer':
            self.get_logger().info('Doing a_open_drawer')
            future = self.a_open_drawer(actionTuple[1])
        elif actionTuple[0] == 'a_deposit':
            self.get_logger().info('Doing a_deposit')
            future = self.a_deposit(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_close_drawer':
            self.get_logger().info('Doing a_close_drawer')
            future = self.a_close_drawer(actionTuple[1])
        elif actionTuple[0] == 'a_approach_arm':
            self.get_logger().info('Doing a_approach_arm')
            future = self.a_approach_arm(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_pick_up_sample':
            self.get_logger().info('Doing a_pick_up_sample')
            future = self.a_pick_up_sample(actionTuple[1], actionTuple[2])

        self.get_logger().info("Wating for response")
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(response.observation)

        if response.observation == 'success':
            self.pos = actionTuple[2]
        elif response.observation == 'door closed':
            self.notifyError(
                ','.join(('error', 'door_closed', actionTuple[2]))
            )
            return False

        return True

    def a_navto(self, robot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_navto', robot, room))
        return self.environment_client.call_async(self.action_request)

    def a_approach_nurse(self, robot):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_approach_nurse', robot))
        return self.environment_client.call_async(self.action_request)

    def a_authenticate_nurse(self, robot, nurse):
        if not self.nurseId == nurse:
            self.get_logger().info('wating for %s to authenticate to %s' % (nurse, robot))
            return False
        return True

    def a_open_drawer(self, robot):
        possibilityChoices = [True, False]
        # self.drawer = random.choice(possibilityChoices)
        self.drawer = True
        return 'success'

    def a_close_drawer(self, robot):
        self.drawer = False
        return 'success'

    def a_approach_arm(self, robot, arm):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_approach_arm', robot, arm))
        return self.environment_client.call_async(self.action_request)

    def a_deposit(self, nurse, robot):
        if not self.has_sample:
            self.get_logger().info('wating for %s to deposit in %s' % (nurse, robot))
            return False
        return True

    def a_pick_up_sample(self, arm, robot):
        if self.has_sample:
            self.get_logger().info('wating for %s to pick up from %s' % (arm, robot))
            return False
        return True

def main():
    rclpy.init()
    robot = Robot('Robot')
    try:
        # rclpy.spin(robot)
        robot.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()