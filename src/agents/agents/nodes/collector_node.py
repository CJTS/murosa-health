import rclpy

from interfaces.srv import Action
from agents.helpers.agent import Agent
from agents.helpers.ActionResults import ActionResult

class Collector(Agent):
    def __init__(self, className):
        super().__init__(className)

    # ACTIONS
    def choose_action(self, actionTuple):
        self.current_action = actionTuple[0]
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
            self.a_authenticate_nurse(actionTuple[1], actionTuple[2])
            return ActionResult.WAITING
        elif actionTuple[0] == 'a_open_drawer':
            self.get_logger().info('Doing a_open_drawer')
            self.a_open_drawer(actionTuple[1])
            return ActionResult.SUCCESS
        elif actionTuple[0] == 'a_deposit':
            self.get_logger().info('Doing a_deposit')
            self.a_deposit(actionTuple[1], actionTuple[2])
            return ActionResult.WAITING
        elif actionTuple[0] == 'a_close_drawer':
            self.get_logger().info('Doing a_close_drawer')
            self.a_close_drawer(actionTuple[1])
            return ActionResult.SUCCESS
        elif actionTuple[0] == 'a_approach_arm':
            self.get_logger().info('Doing a_approach_arm')
            future = self.a_approach_arm(actionTuple[1], actionTuple[2])
        elif actionTuple[0] == 'a_pick_up_sample':
            self.get_logger().info('Doing a_pick_up_sample')
            self.a_pick_up_sample(actionTuple[1], actionTuple[2])
            return ActionResult.WAITING

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

        return ActionResult.SUCCESS

    def a_approach_nurse(self, robot, nurse):
        self.get_logger().info("a_approach_nurse")
        if all('a_approach_nurse' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for nurse")
            self.ask_for_agent(nurse, 'a_approach_nurse')
        else:
            self.get_logger().info("Nurse is waiting, send action message")
            self.acting_for_agent(nurse, 'a_approach_nurse')

    def a_authenticate_nurse(self, robot, nurse):
        self.get_logger().info("a_authenticate_nurse")
        if all('a_authenticate_nurse' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for nurse")
            self.ask_for_agent(nurse, 'a_authenticate_nurse')
        else:
            self.get_logger().info("Nurse is waiting, send action message")
            self.acting_for_agent(nurse, 'a_authenticate_nurse')

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
        self.get_logger().info("a_deposit")
        if all('a_deposit' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for nurse")
            self.ask_for_agent(nurse, 'a_deposit')
        else:
            self.get_logger().info("Nurse is waiting, send action message")
            self.acting_for_agent(nurse, 'a_deposit')

    def a_pick_up_sample(self, arm, robot):
        self.get_logger().info("a_pick_up_sample")
        if all('a_pick_up_sample' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for arm")
            self.ask_for_agent(arm, 'a_pick_up_sample')
        else:
            self.get_logger().info("Arm is waiting, send action message")
            self.acting_for_agent(arm, 'a_pick_up_sample')

def main():
    rclpy.init()
    collector = Collector('Collector')
    try:
        # rclpy.spin(collector)
        collector.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()