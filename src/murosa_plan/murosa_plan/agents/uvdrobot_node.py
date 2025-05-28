import rclpy
from murosa_plan.agent import Agent
from murosa_plan.FIPAPerformatives import FIPAPerformative
from murosa_plan.ActionResults import ActionResult
from murosa_plan.helper import FIPAMessage
from interfaces.srv import Action, Message
from std_msgs.msg import String, Bool

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
        elif actionTuple[0] == 'a_authorize_disinfect':
            self.get_logger().info('Doing a_authorize_disinfect')
            self.a_authorize_disinfect(actionTuple[1], actionTuple[2])
            return ActionResult.WAITING
        
           

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
    
    def a_authorize_disinfect(self, uvdrobot_,spotrobot_):
        self.get_logger().info("a_authorize_disinfect")
        if all('a_authorize_disinfect' not in action for action in self.wating_response):
            self.get_logger().info("Here first, waiting for spotrobot")
            self.ask_for_agent(spotrobot_, 'a_authorize_disinfect')
        else:
            self.get_logger().info("Nurse is waiting, send action message")
            self.acting_for_agent(spotrobot_, 'a_authorize_disinfect')


    def act(self):
        if(len(self.plan) > 0) :
            action = self.plan.pop(0)
            result = self.choose_action(action)
            if result == ActionResult.WAITING:
                self.wating = True
                self.plan.insert(0, action)
        elif(len(self.actions) > 0):
            self.get_logger().info('Acting')
            action = self.actions.pop(0)
            result = self.choose_action(action)
            if result == ActionResult.SUCCESS:
                self.get_logger().info("ActionResult.SUCCESS")
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'Success|' + ",".join(action)).encode()
                self.publisher.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
            elif result == ActionResult.FAILURE:
                self.get_logger().info("ActionResult.FAILURE")
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.INFORM.value, self.agentName, 'Jason', 'Failure|' + ",".join(action)).encode()
                self.publisher.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
            elif result == ActionResult.WAITING:
                self.get_logger().info("ActionResult.WAITING")
                self.wating = True
                self.actions.append(action)

    def respond_agent(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        decoded_msg = FIPAMessage.decode(msg.data)
        if not self.is_for_me(decoded_msg):
            self.get_logger().info('And it is not for me')
            return

        self.get_logger().info('And it is for me')
        if "Ready" == decoded_msg.content.split("|")[0]:
            if all(decoded_msg.content.split("|")[1] not in action for action in self.wating_response):
                self.get_logger().info('No there yet')
                self.wating_response.append((decoded_msg.sender, decoded_msg.content.split("|")[1]))
            else:
                self.get_logger().info('Ready to act')
                self.acting_for_agent(decoded_msg.sender, decoded_msg.content.split("|")[1])
        elif "Done" == decoded_msg.content.split("|")[0]:
            if any(decoded_msg.content.split("|")[1] in action for action in self.actions) or any(decoded_msg.content.split("|")[1] in action for action in self.plan):
                self.get_logger().info('Finished action')
                self.plan.pop(0)
                self.wating_response.clear()
                self.wating = False
                self.acting_for_agent(decoded_msg.sender, decoded_msg.content.split("|")[1])
            else:
                self.get_logger().info('Already finished action')
    

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
    