import rclpy

from interfaces.srv import Action
from agents.helpers.agent import Agent
from agents.helpers.ActionResults import ActionResult

class Uvd(Agent):
    def __init__(self, className):
        super().__init__(className)
        self.battery = 3

    def choose_action(self, actionTuple):
        self.current_action = actionTuple[0]
        future = None
        if actionTuple[0] == 'a_navto':
            if(self.battery > 2):
                self.get_logger().info('Doing a_navto')
                self.a_navto(actionTuple[1], actionTuple[2])
                self.battery -= 1
                return ActionResult.MOVING
            else:
                self.get_logger().info('low_battery')
                return ActionResult.BATTERY_FAILURE
        elif actionTuple[0] == 'a_disinfect_room':
            if(self.battery > 2):
                self.get_logger().info('Doing a_disinfect_room')
                future = self.a_disinfect_room(actionTuple[1], actionTuple[2])
                self.battery -= 1
            else:
                self.get_logger().info('low_battery')
                return ActionResult.BATTERY_FAILURE
        elif actionTuple[0] == 'a_authorize_disinfect':
            if(self.battery > 2):
                self.get_logger().info('Doing a_authorize_disinfect')
                self.a_authorize_disinfect(actionTuple[1], actionTuple[2])
                self.battery -= 1
                return ActionResult.WAITING
            else:
                self.get_logger().info('low_battery')
                return ActionResult.BATTERY_FAILURE
        elif actionTuple[0] == 'a_charge':
            self.get_logger().info('Doing charge')
            self.a_charge()


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
            elif response.observation == 'dirty room':
                self.notifyError(
                    ','.join(('dirty_room', actionTuple[2]))
                )
                return ActionResult.FAILURE

        return ActionResult.SUCCESS

    def a_disinfect_room(self, uvdrobot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('a_disinfect_room', uvdrobot, room))
        return self.environment_client.call_async(self.action_request)

    def a_authorize_disinfect(self, uvdrobot_,spotrobot_):
        if all('a_authorize_disinfect' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for spotrobot")
            self.ask_for_agent(spotrobot_, 'a_authorize_disinfect')
        else:
            self.get_logger().info("spotrobot is waiting, send action message")
            self.acting_for_agent(spotrobot_, 'a_authorize_disinfect')

    def a_charge(self):
        self.battery += 10

def main():
    rclpy.init()
    uvd = Uvd('Uvd')
    try:
        uvd.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    uvd.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    