import random
from threading import Thread
import rclpy
from rclpy.node import Node
from interfaces.srv import HasSample, SendPlan, Action
from murosa_plan_patrol.helper import action_string_to_tuple
from std_msgs.msg import String, Bool

class Robot(Node):
    def __init__(self):
        super().__init__('Robot')
        self.local_plan = []

        self.current_action_tuple = ()
        self.new_plan = False
        self.start_server()

    def start_server(self):
        self.get_logger().info('Starting Robot server')

        # Servidor para falar com o robo
        self.robot_server = self.create_service(
            SendPlan, 'robot_server', self.receive_message
        )

        # Subscriber para falar com o Jason (Ação move)
        self.subscription = self.create_subscription(
            String, '/move_base/move', self.listener_callback, 10
        )

        # Publisher para falar o resultado da ação
        self.publisher_ = self.create_publisher(String, '/move_base/result', 10)

        # Subscriber para indicar fim da execução
        self.end_subscription = self.create_subscription(
            Bool, '/shutdown_signal', self.shutdown_callback, 10
        )

        self.get_logger().info('Robot server started')

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            raise SystemExit

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        msg = String()
        msg.data = 'success'
        # Erros
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def receive_message(self, request, response):
        self.get_logger().info('Receiving message.')
        self.get_logger().info("Robot's actions:")
        self.new_plan = True
        self.local_plan = list(map(action_string_to_tuple, request.plan))
        for action in self.local_plan:
            self.get_logger().info(action[0])
        response.ok = 'Okay!'
        return response

    def notifySuccess(self):
        self.action_request = Action.Request()
        self.action_request.action = ','.join(('action_complete', self.current_action))
        return self.coordinator_communication_sync_client.call(
            self.action_request
        )

def main():
    rclpy.init()
    robot = Robot()
    robot.get_logger().info('spin')
    try:
        rclpy.spin(robot)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    robot.get_logger().info('stop spin')
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
