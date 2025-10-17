import rclpy
from murosa_plan.murosa_plan.helpers.agent import Agent
from murosa_plan.ActionResults import ActionResult
from interfaces.srv import Action

class Spot(Agent):
    def __init__(self, className):
        super().__init__(className)
        self.battery = 3
        
        self.goal_room = None

    # ACTIONS
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
        elif actionTuple[0] == 'a_open_door':
            if(self.battery > 2):
                self.get_logger().info('Doing a_open_door')
                future = self.a_open_door(actionTuple[1], actionTuple[2])
                self.battery -= 1
            else:
                self.get_logger().info('low_battery')
                return ActionResult.BATTERY_FAILURE
        elif actionTuple[0] == 'a_approach_nurse':
            if(self.battery > 2):
                self.get_logger().info('Doing a_approach_nurse')
                self.a_approach_nurse(actionTuple[1], actionTuple[2])
                self.battery -= 1
                return ActionResult.WAITING
            else:
                self.get_logger().info('low_battery')
                return ActionResult.BATTERY_FAILURE
        elif actionTuple[0] == 'a_authenticate_nurse':
            if(self.battery > 2):
                self.get_logger().info('Doing a_authenticate_nurse')
                self.a_authenticate_nurse(actionTuple[1], actionTuple[2])
                self.battery -= 1
                return ActionResult.WAITING
            else:
                self.get_logger().info('low_battery')
                return ActionResult.BATTERY_FAILURE
        elif actionTuple[0] == 'a_authorize_patrol':
            if(self.battery > 2):
                self.get_logger().info('Doing a_authorize_patrol')
                self.a_authorize_patrol(actionTuple[1], actionTuple[2])
                self.battery -= 1
                return ActionResult.WAITING
            else:
                self.get_logger().info('low_battery')
                return ActionResult.BATTERY_FAILURE
        elif actionTuple[0]== 'a_patrol_room':
            if(self.battery > 2):
                self.get_logger().info('Doing a_patrol_room')
                future = self.a_patrol_room(actionTuple[1], actionTuple[2])
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
            self.get_logger().info('Doing a_charge')
            self.a_charge()

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
    
    def a_open_door(self, spotrobot, room):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(
            ('a_open_door', spotrobot, room)
        )
        return self.environment_client.call_async(self.action_request)

    def a_approach_nurse(self, spotrobot, nurse):
        if all('a_approach_nurse' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for nurse")
            self.ask_for_agent(nurse, 'a_approach_nurse')
        else:
            self.get_logger().info("Nurse is waiting, send action message")
            self.acting_for_agent(nurse, 'a_approach_nurse')

    def a_authenticate_nurse(self, spotrobot, nurse):
        if all('a_authenticate_nurse' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for nurse")
            self.ask_for_agent(nurse, 'a_authenticate_nurse')
        else:
            self.get_logger().info("Nurse is waiting, send action message")
            self.acting_for_agent(nurse, 'a_authenticate_nurse')
    
    def a_authorize_patrol(self, spotrobot,nurse):
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
    
    def a_authorize_disinfect(self, uvdrobot_,spotrobot_):
        if all('a_authorize_disinfect' not in action for action in self.wating_response):
            self.get_logger().info("Here first, lwaiting for spotrobot")
            self.ask_for_agent(uvdrobot_, 'a_authorize_disinfect')
        else:
            self.get_logger().info("Nurse is waiting, send action message")
            self.acting_for_agent(uvdrobot_, 'a_authorize_disinfect')

    def a_charge(self):
        self.battery += 10

def main():
    rclpy.init()
    spot = Spot('Spot')
    try:
        spot.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    spot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
