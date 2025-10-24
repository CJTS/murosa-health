import json
import rclpy

from rclpy.node import Node
from interfaces.srv import Action
from std_msgs.msg import Bool

from coordinator.planner.health.domain.methods import methods
from coordinator.planner.health.domain.actions import actions
from coordinator.planner.health.problem.problem import init_state
from coordinator.planner.ipyhop import IPyHOP

class Planner(Node):
    def __init__(self):
        super().__init__('Planner')
        self.get_logger().info('Starting Planner server')
        self.planner_communication_sync_server = self.create_service(
            Action, 'planner_communication_sync_server', self.receive_sync_message
        )
        self.state = init_state

        self.end_simulation_subscription = self.create_subscription(
            Bool, '/coordinator/shutdown_signal', self.end_simulation_callback, 10
        )

        self.get_logger().info('Planner server started')

    def end_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento do coordenador, finalizando...")
            raise SystemExit

    def receive_sync_message(self, request, response):
        messageTuple = tuple(request.action.split('|'))

        if messageTuple[0] == 'need_plan':
            actionTuple = tuple(messageTuple[1].split(','))
            self.get_logger().info('Creating plan for: %s %s %s %s %s' % (
                actionTuple[0], actionTuple[1], actionTuple[2], actionTuple[3], actionTuple[4]
            ))

            goal = ''

            if actionTuple[0] == 'CollectSampleMission':
                goal = 'm_pickup_and_deliver_sample'
            elif actionTuple[0] == 'DisinfectRoomMission' or actionTuple[0] == 'DisinfectICUMission':
                goal = 'm_patrol_and_disinfect'

            planner = IPyHOP(methods, actions)
            plan = planner.plan(self.state, [(
                goal, actionTuple[1], actionTuple[2], actionTuple[3], actionTuple[4]
            )], verbose=1)

            responsePlan = []

            for action in plan:
                responsePlan.append(','.join(action))
                self.get_logger().info(str(action))

            response.observation = '/'.join(responsePlan)

            self.get_logger().info('Sending response')
            return response
        elif messageTuple[0] == 'update_state':
            state = json.loads(messageTuple[1])
            self.state.loc = state['loc']
            self.state.doors = state['doors']
            self.state.disinfected = state['disinfected']
            self.state.cleaned = state['cleaned']
            self.state.low_battery = state['low_battery']
            self.state.sample = state['sample']
            return response
        elif messageTuple[0] == 'update_room_uncleaned':
            self.state.cleaned[messageTuple[1]] = False

        response.observation = 'success'
        return response

def main():
    rclpy.init()
    planner = Planner()
    try:
        rclpy.spin(planner)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()